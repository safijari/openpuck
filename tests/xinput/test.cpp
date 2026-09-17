#include "xinput_auth.h"
#include "xinput_descriptors.h"
#include "src/libxsm3/excrypt.h"
#include "src/libxsm3/usbdsec.h"
#include "src/libxsm3/xsm3.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#define CHECK(condition) check(bool(condition), #condition, __LINE__)

static void check(bool condition, const char *expression, int line)
{
	if (!condition) {
		fprintf(stderr, "%s:%d: %s\n", __FILE__, line, expression);
		abort();
	}
}

using Bytes = std::vector<uint8_t>;

struct Fixture {
	const char *console;
	const char *nonce;
	const char *identification;
	const char *init;
	const char *init_response;
	const char *verify[3];
	const char *verify_response[3];
};

#include "fixtures.h"

static Bytes hex(const char *text)
{
	CHECK(strlen(text) % 2 == 0);
	Bytes result;
	while (*text) {
		unsigned byte;
		CHECK(sscanf(text, "%2x", &byte) == 1);
		result.push_back(byte);
		text += 2;
	}
	return result;
}

static void checksum(Bytes &packet)
{
	CHECK(packet.size() == size_t(packet[4]) + 6);
	packet.back() = 0;
	for (size_t i = 5; i + 1 < packet.size(); i++)
		packet.back() ^= packet[i];
}

static void checkPacket(const Bytes &packet, size_t length)
{
	CHECK(packet.size() == length);
	CHECK(packet[0] == 0x49 && packet[1] == 0x4C);
	CHECK(packet[2] == 0 && packet[3] == 0);
	Bytes copy = packet;
	checksum(copy);
	CHECK(copy == packet);
}

static uint32_t lockDepth;
static unsigned notifications, rngCalls, workerStarts;
static bool workerAvailable = true, rngAvailable = true;
static bool resetInRng, replaceInRng;
static uint8_t *ep0;
static uint16_t offered, transferred;
static unsigned transfers;

static tusb_control_request_t request(uint8_t command, uint16_t length,
				      uint8_t type = 0xC1,
				      uint16_t index = 0x0103)
{
	return { type, command, 0, index, length };
}

bool tud_control_xfer(uint8_t rhport, const tusb_control_request_t *req,
		      void *buffer, uint16_t length)
{
	CHECK(rhport == 0);
	CHECK(lockDepth == 0);
	CHECK(buffer != nullptr);
	CHECK(length <= 46);
	ep0 = static_cast<uint8_t *>(buffer);
	offered = length;
	transferred = std::min(length, req->wLength);
	transfers++;
	return true;
}

uint32_t xinputAuthLock()
{
	return lockDepth++;
}

void xinputAuthUnlock(uint32_t mask)
{
	CHECK(lockDepth == mask + 1);
	lockDepth = mask;
}

bool xinputAuthStartWorker()
{
	workerStarts++;
	return workerAvailable;
}

void xinputAuthNotify()
{
	CHECK(lockDepth == 0);
	notifications++;
}

void xinputAuthBoardSerial(char serial[13])
{
	memcpy(serial, "012345ABCDEF", 13);
}

static bool setup(const tusb_control_request_t &req)
{
	ep0 = nullptr;
	offered = transferred = 0;
	unsigned before = transfers;
	bool ok = xinputAuthControl(0, CONTROL_STAGE_SETUP, &req);
	CHECK(lockDepth == 0);
	CHECK(transfers == before + unsigned(ok));
	return ok;
}

static void out(uint8_t command, const Bytes &packet, uint16_t index = 0x0103,
		uint8_t type = 0x41)
{
	auto req = request(command, packet.size(), type, index);
	unsigned before = notifications;
	CHECK(setup(req));
	CHECK(offered == packet.size() && transferred == packet.size());
	memcpy(ep0, packet.data(), packet.size());
	CHECK(notifications == before);
	CHECK(xinputAuthControl(0, CONTROL_STAGE_DATA, &req));
	CHECK(notifications == before + 1);
	CHECK(xinputAuthControl(0, CONTROL_STAGE_ACK, &req));
	CHECK(notifications == before + 1);
}

bool xsm3_random_bytes(uint8_t *buffer, size_t length)
{
	CHECK(lockDepth == 0);
	CHECK(length == 16);
	rngCalls++;
	for (size_t i = 0; i < length; i++)
		buffer[i] = 0x90 + i;
	if (resetInRng) {
		resetInRng = false;
		xinputAuthReset();
	}
	if (replaceInRng) {
		replaceInRng = false;
		out(0x82, hex(fixtures[1].init));
	}
	return rngAvailable;
}

static Bytes in(uint8_t command, uint16_t length = 65535,
		uint16_t index = 0x0103, uint8_t type = 0xC1)
{
	auto req = request(command, length, type, index);
	CHECK(setup(req));
	Bytes result(ep0, ep0 + transferred);
	unsigned before = transfers;
	CHECK(xinputAuthControl(0, CONTROL_STAGE_DATA, &req));
	CHECK(xinputAuthControl(0, CONTROL_STAGE_ACK, &req));
	CHECK(transfers == before);
	return result;
}

static void state(uint8_t expected)
{
	CHECK(in(0x86) == Bytes({ expected, 0 }));
}

static void noReply()
{
	CHECK(!setup(request(0x83, 46)));
}

static void failed()
{
	noReply();
	CHECK(!setup(request(0x86, 2)));
	CHECK(!setup(request(0x87, 22, 0x41)));
	xinputAuthProcess();
	noReply();
}

/*
 * This console uses only primitives, never xsm3_do_challenge_* or its globals.
 * Every packet is also compared with the independent OpenSSL/Python fixture,
 * so matching bugs in the shared primitives cannot bless a broken exchange.
 */
class Console {
    public:
	const Fixture &fixture;
	Bytes nonce, id, key1, key2, controller, hash, salt;

	explicit Console(const Fixture &data)
		: fixture(data)
		, nonce(hex(data.nonce))
		, id(hex(data.console))
		, key1(16)
		, key2(16)
		, controller(16)
		, hash(20)
		, salt(16)
	{
		uint8_t digest[20], kv1[16], kv2[16];
		ExCryptSha(id.data(), id.size(), digest);
		auto root1 = hex("828078683a523a9810f40c127066dcba");
		auto root2 = hex("66621a78f8609c8a269a04aed85c1ec8");
		UsbdSecXSM3AuthenticationCrypt(root1.data(), digest, 16, kv1,
					       1);
		UsbdSecXSM3AuthenticationCrypt(root2.data(), digest + 4, 16,
					       kv2, 1);
		Bytes swapped(nonce.begin() + 8, nonce.end());
		swapped.insert(swapped.end(), nonce.begin(), nonce.begin() + 8);
		UsbdSecXSM3AuthenticationCrypt(kv1, nonce.data(), 16,
					       key1.data(), 1);
		UsbdSecXSM3AuthenticationCrypt(kv2, swapped.data(), 16,
					       key2.data(), 1);
	}

	Bytes init()
	{
		Bytes plain = nonce;
		plain.insert(plain.end(), id.begin(), id.end());
		Bytes packet(34);
		packet[0] = 0x49;
		packet[1] = 0x4C;
		packet[4] = 28;
		auto cryptKey = hex("e35bfb1ccdad325bf70e07fd623da7c4");
		auto macKey = hex("8f2908380b5bfe687c26462a51f2bc19");
		uint8_t mac[8];
		UsbdSecXSM3AuthenticationCrypt(cryptKey.data(), plain.data(),
					       24, packet.data() + 5, 1);
		UsbdSecXSM3AuthenticationMac(macKey.data(), nullptr,
					     packet.data() + 5, 24, mac);
		memcpy(packet.data() + 29, mac + 4, 4);
		checksum(packet);
		CHECK(packet == hex(fixture.init));
		return packet;
	}

	void acceptInit(const Bytes &response)
	{
		checkPacket(response, 46);
		CHECK(response == hex(fixture.init_response));
		uint8_t plain[32];
		UsbdSecXSM3AuthenticationCrypt(key1.data(), response.data() + 5,
					       32, plain, 0);
		CHECK(std::equal(nonce.begin(), nonce.end(), plain + 16));
		memcpy(controller.data(), plain, 16);
		ExCryptSha(plain, sizeof plain, hash.data());
		memcpy(salt.data(), controller.data() + 12, 4);
		memcpy(salt.data() + 4, nonce.data() + 12, 4);
	}

	Bytes verify(unsigned iteration)
	{
		CHECK(iteration < 3);
		for (unsigned i = 0; i < 8; i++)
			salt[8 + i] = 0x60 + iteration * 8 + i;
		Bytes packet(22);
		packet[0] = 0x49;
		packet[1] = 0x4C;
		packet[4] = 16;
		UsbdSecXSM3AuthenticationCrypt(controller.data(),
					       salt.data() + 8, 8,
					       packet.data() + 5, 1);
		UsbdSecXSM3AuthenticationMac(hash.data(), salt.data(),
					     packet.data() + 5, 8,
					     packet.data() + 13);
		checksum(packet);
		CHECK(packet == hex(fixture.verify[iteration]));
		return packet;
	}

	void acceptVerify(Bytes response, unsigned iteration)
	{
		checkPacket(response, 22);
		CHECK(response == hex(fixture.verify_response[iteration]));
		uint8_t mac[8];
		UsbdSecXSM3AuthenticationMac(key2.data(), salt.data(),
					     response.data() + 5, 8, mac);
		CHECK(memcmp(mac, response.data() + 13, 8) == 0);
	}
};

static void begin(Console &console, uint16_t index = 0x0103,
		  uint8_t type = 0x41)
{
	unsigned before = rngCalls;
	out(0x82, console.init(), index, type);
	CHECK(rngCalls == before);
	state(1);
	noReply();
	xinputAuthProcess();
	CHECK(rngCalls == before + 1);
	state(2);
	console.acceptInit(in(0x83));
	xinputAuthProcess();
	CHECK(rngCalls == before + 1);
}

static void exchange(Console &console)
{
	for (unsigned i = 0; i < 3; i++) {
		unsigned before = rngCalls;
		out(0x87, console.verify(i));
		state(1);
		noReply();
		xinputAuthProcess();
		CHECK(rngCalls == before);
		state(2);
		Bytes response = in(0x83);
		CHECK(in(0x83) == response);
		console.acceptVerify(response, i);
	}
}

static void cryptoKnownAnswers()
{
	uint8_t digest[20];
	ExCryptSha(reinterpret_cast<const uint8_t *>("abc"), 3, digest);
	CHECK(Bytes(digest, digest + 20) ==
	      hex("a9993e364706816aba3e25717850c26c9cd0d89d"));
	ExCryptSha(reinterpret_cast<const uint8_t *>(""), 0, digest);
	CHECK(Bytes(digest, digest + 20) ==
	      hex("da39a3ee5e6b4b0d3255bfef95601890afd80709"));
	// This vendor helper only supports single-block SHA-1 (at most 55 bytes).
	uint8_t message[55];
	for (unsigned i = 0; i < sizeof message; i++)
		message[i] = i;
	ExCryptSha(message, 32, digest);
	CHECK(Bytes(digest, digest + 20) ==
	      hex("ae5bd8efea5322c4d9986d06680a781392f9a642"));
	ExCryptSha(message, sizeof message, digest);
	CHECK(Bytes(digest, digest + 20) ==
	      hex("8ae2d46729cfe68ff927af5eec9c7d1b66d65ac2"));
	uint64_t key;
	auto keyBytes = hex("133457799bbcdff1");
	memcpy(&key, keyBytes.data(), 8);
	EXCRYPT_DES_STATE des;
	ExCryptDesKey(&des, &key);
	auto plain = hex("0123456789abcdef");
	uint8_t encrypted[8], recovered[8];
	ExCryptDesEcb(&des, plain.data(), encrypted, 1);
	CHECK(Bytes(encrypted, encrypted + 8) == hex("85e813540f0ab405"));
	ExCryptDesEcb(&des, encrypted, recovered, 0);
	CHECK(Bytes(recovered, recovered + 8) == plain);
	uint8_t parity[256], input[256];
	for (unsigned i = 0; i < 256; i++)
		input[i] = i;
	ExCryptDesParity(input, sizeof input, parity);
	for (unsigned i = 0; i < 256; i++)
		CHECK((parity[i] & 0xFE) == (i & 0xFE));
	// DES ignores parity bits; this helper does not normalize odd parity.
	for (unsigned i = 0; i < 256; i += 8) {
		EXCRYPT_DES_STATE original, adjusted;
		memcpy(&key, input + i, 8);
		ExCryptDesKey(&original, &key);
		memcpy(&key, parity + i, 8);
		ExCryptDesKey(&adjusted, &key);
		CHECK(memcmp(&original, &adjusted, sizeof original) == 0);
	}
	uint8_t macKey[16], data[24];
	for (unsigned i = 0; i < sizeof macKey; i++)
		macKey[i] = i;
	for (unsigned i = 0; i < sizeof data; i++)
		data[i] = i;
	auto salt = hex("ffffffffffffffff1020304050607080");
	UsbdSecXSM3AuthenticationMac(macKey, salt.data(), data, sizeof data,
				     encrypted);
	CHECK(Bytes(encrypted, encrypted + 8) == hex(mac_wrap));
	CHECK(salt == hex(salt_wrap));
	puts("PASS SHA-1/DES known answers, DES parity, salted MAC wrap");
}

static void descriptors()
{
	static_assert(sizeof(tusb_control_request_t) == 8, "SETUP size");
	CHECK(sizeof XINPUT_CONFIG_BODY == 144);
	CHECK(sizeof XINPUT_CONFIG_BODY + 9 == 153);
	const uint8_t counts[] = { 2, 4, 1, 0 };
	const uint8_t protocols[] = { 1, 3, 2, 0x13 };
	const uint8_t classLengths[] = { 17, 27, 9, 6 };
	const uint8_t endpoints[] = { 0x81, 0x02, 0x83, 0x04, 0x85, 0x06, 0x86 };
	const uint8_t intervals[] = { 1, 8, 2, 4, 0x40, 0x10, 0x10 };
	unsigned interfaces = 0, endpoint = 0, localEndpoints = 0;
	unsigned classDescriptors = 0;
	for (size_t offset = 0; offset < sizeof XINPUT_CONFIG_BODY;) {
		const uint8_t *d = XINPUT_CONFIG_BODY + offset;
		CHECK(d[0] >= 2 && offset + d[0] <= sizeof XINPUT_CONFIG_BODY);
		switch (d[1]) {
		case 4:
			if (interfaces)
				CHECK(localEndpoints == counts[interfaces - 1]);
			CHECK(interfaces < 4 && d[0] == 9);
			CHECK(d[2] == interfaces && d[3] == 0);
			CHECK(d[4] == counts[interfaces] && d[5] == 0xFF);
			CHECK(d[6] == (interfaces == 3 ? 0xFD : 0x5D));
			CHECK(d[7] == protocols[interfaces]);
			CHECK(d[8] == (interfaces == 3 ? 4 : 0));
			if (interfaces == 3)
				CHECK(offset + 8 ==
				      XINPUT_SECURITY_STRING_OFFSET);
			interfaces++;
			localEndpoints = 0;
			break;
		case 5:
			CHECK(interfaces > 0 && endpoint < 7);
			CHECK(d[0] == 7 && d[2] == endpoints[endpoint]);
			CHECK(d[3] == 3 && d[4] == 32 && d[5] == 0);
			CHECK(d[6] == intervals[endpoint]);
			endpoint++;
			localEndpoints++;
			break;
		case 0x21:
		case 0x41:
			CHECK(interfaces > 0 &&
			      classDescriptors == interfaces - 1);
			CHECK(d[0] == classLengths[classDescriptors]);
			CHECK(d[1] == (interfaces == 4 ? 0x41 : 0x21));
			classDescriptors++;
			break;
		default:
			CHECK(false);
		}
		offset += d[0];
	}
	CHECK(interfaces == 4 && endpoint == 7 && classDescriptors == 4);
	CHECK(localEndpoints == 0);
	CHECK(XINPUT_SECURITY_STRING_OFFSET == 137);
	CHECK(strcmp(XINPUT_SECURITY_STRING,
		     "Xbox Security Method 3, Version 1.00, \xc2\xa9 2005 Microsoft "
		     "Corporation. All rights reserved.") == 0);
	puts("PASS descriptor lengths, interfaces, endpoints, security string");
}

static const uint16_t lengths[] = {
	0,  1,	2,  5,	21,  22,  23,	 28,	29,    33,    34,
	35, 45, 46, 47, 255, 256, 32767, 32768, 65534, 65535,
};

static void identificationAndBounds()
{
	xinputAuthPrepare();
	CHECK(workerStarts == 1);
	CHECK(strlen(xinputAuthSerial()) == 12);
	CHECK(strcmp(xinputAuthSerial(), "012345ABCDEF") == 0);
	auto expected = hex(fixtures[0].identification);
	for (uint16_t index : { 3, 0x0103 }) {
		for (uint16_t length : lengths) {
			Bytes id = in(0x81, length, index);
			CHECK(offered == 29);
			Bytes truncated = expected;
			truncated.resize(std::min<size_t>(length, 29));
			CHECK(id == truncated);
		}
	}
	CHECK(in(0x81, 65535, 0, 0xC0) == expected);
	CHECK(in(0x81) == expected);
	CHECK(memcmp(expected.data() + 5, xinputAuthSerial(), 12) == 0);
	Bytes checked = expected;
	checksum(checked);
	CHECK(checked == expected);
	state(1);
	noReply();
	CHECK(!setup(request(0x87, 22, 0x41)));
	Console console(fixtures[0]);
	begin(console);
	for (uint8_t command : { 0x82, 0x87 }) {
		for (uint16_t length : lengths) {
			if (length == (command == 0x82 ? 34 : 22))
				continue;
			CHECK(!setup(request(command, length, 0x41)));
			CHECK(in(0x83) == hex(fixtures[0].init_response));
		}
	}
	for (uint8_t command : { 0x83, 0x84, 0x86 }) {
		Bytes full = in(command);
		for (uint16_t length : lengths) {
			Bytes reply = in(command, length);
			CHECK(offered == full.size());
			Bytes truncated = full;
			truncated.resize(std::min<size_t>(length, full.size()));
			CHECK(reply == truncated);
		}
	}
	for (uint8_t command : { 0x81, 0x82, 0x83, 0x84, 0x86, 0x87 }) {
		bool output = command == 0x82 || command == 0x87;
		uint16_t length = command == 0x82 ? 34 : 22;
		uint8_t correct = output ? 0x41 : 0xC1;
		CHECK(!setup(request(command, length, correct ^ 0x80)));
		for (uint8_t type : { 0x00, 0x01, 0x21, 0x42, 0x43, 0x80, 0x81,
				      0xA1, 0xC2, 0xC3 })
			CHECK(!setup(request(command, length, type)));
		for (uint16_t index : { 0, 1, 2, 4, 0x0100, 0x0203, 65535 })
			CHECK(!setup(request(command, length, correct, index)));
		CHECK(!setup(request(command, length, correct - 1, 3)));
	}
	for (uint8_t command : { 0x00, 0x80, 0x85, 0x88, 0xFF }) {
		CHECK(!setup(request(command, 34, 0x41)));
		CHECK(!setup(request(command, 46)));
	}
	CHECK(in(0x83) == hex(fixtures[0].init_response));
	puts("PASS identification, serial stability, lengths and request routing");
}

static void sessions()
{
	for (unsigned round = 0; round < 2; round++) {
		for (const auto &fixture : fixtures) {
			Console console(fixture);
			if (round)
				xinputAuthReset();
			begin(console, round ? 3 : 0x0103);
			exchange(console);
			for (uint16_t length : lengths) {
				auto expected = hex(fixture.verify_response[2]);
				expected.resize(std::min<size_t>(length, 22));
				CHECK(in(0x83, length) == expected);
				CHECK(offered == 22);
			}
		}
	}
	Console device(fixtures[0]);
	begin(device, 0, 0x40);
	CHECK(in(0x83, 65535, 0, 0xC0) == hex(fixtures[0].init_response));
	exchange(device);
	CHECK(in(0x81) == hex(fixtures[0].identification));
	state(1);
	noReply();
	CHECK(strcmp(xinputAuthSerial(), "012345ABCDEF") == 0);
	puts("PASS two consoles, reinit/reset, repeated verification, IN truncation");
}

static void stagesAndReset()
{
	xinputAuthReset();
	auto req = request(0x82, 34, 0x41);
	CHECK(!xinputAuthControl(0, CONTROL_STAGE_DATA, &req));
	CHECK(setup(req));
	unsigned before = notifications, randomBefore = rngCalls;
	CHECK(xinputAuthControl(0, CONTROL_STAGE_ACK, &req));
	xinputAuthProcess();
	CHECK(notifications == before && rngCalls == randomBefore);
	auto init = hex(fixtures[0].init);
	memcpy(ep0, init.data(), init.size());
	CHECK(xinputAuthControl(0, CONTROL_STAGE_DATA, &req));
	CHECK(notifications == before + 1);
	CHECK(!xinputAuthControl(0, CONTROL_STAGE_DATA, &req));
	xinputAuthReset();
	xinputAuthProcess();
	CHECK(rngCalls == randomBefore);
	state(1);
	noReply();
	CHECK(setup(req));
	xinputAuthReset();
	CHECK(!xinputAuthControl(0, CONTROL_STAGE_DATA, &req));

	Console console(fixtures[0]);
	begin(console);
	auto replyReq = request(0x83, 46);
	CHECK(setup(replyReq));
	uint8_t *owned = ep0;
	Bytes snapshot(owned, owned + offered);
	xinputAuthReset();
	CHECK(Bytes(owned, owned + snapshot.size()) == snapshot);
	CHECK(xinputAuthControl(0, CONTROL_STAGE_DATA, &replyReq));
	CHECK(xinputAuthControl(0, CONTROL_STAGE_ACK, &replyReq));
	noReply();

	out(0x82, console.init());
	CHECK(setup(request(0x86, 2)));
	owned = ep0;
	CHECK(owned[0] == 1 && owned[1] == 0);
	xinputAuthProcess();
	CHECK(owned[0] == 1 && owned[1] == 0);
	state(2);

	resetInRng = true;
	out(0x82, console.init());
	xinputAuthProcess();
	CHECK(!resetInRng);
	state(1);
	noReply();
	CHECK(!setup(request(0x87, 22, 0x41)));
	begin(console);
	exchange(console);

	replaceInRng = true;
	out(0x82, console.init());
	xinputAuthProcess();
	CHECK(!replaceInRng);
	state(1);
	noReply();
	xinputAuthProcess();
	Console replacement(fixtures[1]);
	replacement.acceptInit(in(0x83));
	exchange(replacement);
	puts("PASS EP0 stages, buffer ownership, reset/replacement during crypto");
}

static void invalidChallenges()
{
	for (bool verify : { false, true }) {
		for (unsigned corruption = 0; corruption < 4; corruption++) {
			Console console(fixtures[0]);
			begin(console);
			Bytes packet = verify ? console.verify(0) :
						console.init();
			switch (corruption) {
			case 0:
				packet.back() ^= 1;
				break;
			case 1:
				packet[verify ? 13 : 29] ^= 1;
				checksum(packet);
				break;
			case 2:
				packet[4] = 0;
				break;
			case 3:
				packet[4] = 255;
				break;
			}
			unsigned before = rngCalls;
			out(verify ? 0x87 : 0x82, packet);
			noReply();
			state(1);
			xinputAuthProcess();
			CHECK(rngCalls == before);
			failed();
			CHECK(std::all_of(xsm3_challenge_response,
					  xsm3_challenge_response + 48,
					  [](uint8_t b) { return b == 0; }));
			Console recovery(fixtures[1]);
			begin(recovery);
			exchange(recovery);
		}
	}
	Console console(fixtures[0]);
	begin(console);
	rngAvailable = false;
	out(0x82, console.init());
	xinputAuthProcess();
	failed();
	rngAvailable = true;
	begin(console);
	exchange(console);
	workerAvailable = false;
	xinputAuthPrepare();
	CHECK(!setup(request(0x81, 29)));
	CHECK(!setup(request(0x82, 34, 0x41)));
	workerAvailable = true;
	xinputAuthPrepare();
	CHECK(in(0x81) == hex(fixtures[0].identification));
	begin(console);
	exchange(console);
	puts("PASS malformed packets, checksum/MAC failures, RNG/worker failures");
}

int main()
{
	cryptoKnownAnswers();
	descriptors();
	identificationAndBounds();
	sessions();
	stagesAndReset();
	invalidChallenges();
	CHECK(lockDepth == 0);
	puts("PASS all native XInput regression tests");
	return 0;
}