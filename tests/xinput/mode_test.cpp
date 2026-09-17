#include "mode_xinput.h"
#include "config.h"
#include "gamepad_util.h"
#include "haptics.h"
#include "usb_app_drivers.h"
#include "usb_tx.h"
#include "xinput_auth.h"
#include "xinput_descriptors.h"
#include <Arduino.h>
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
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
Adafruit_USBD_Device TinyUSBDevice;
uint8_t g_usbMode = MODE_XBOX;
uint8_t g_abSwap, g_back[4], g_qamMap, g_rumble = 1;
Slot g_slot[NSLOT];
unsigned long g_connReplyMs[NSLOT];
static unsigned long now = 1000;
static uint32_t interruptMask;
static bool mounted = true;
static tusb_desc_device_t device;
static std::string manufacturer, product, serial, interfaceString;
static uint8_t attributes, interfaceCount, endpointCount[2];
static uint16_t maxPower;
static unsigned additions, registrations;
static Adafruit_USBD_Interface *registeredInterface;
static usbTxDrainFn drain;
static Bytes configuration;

struct Endpoint {
	bool opened = false, busy = false, claimed = false;
	uint8_t *buffer = nullptr;
	uint16_t length = 0;
};

static Endpoint endpoints[256];
static std::vector<uint8_t> opened, closed;
static std::vector<Bytes> reports;
static uint8_t failOpen;
static bool failTransfer;

struct Rumble {
	uint16_t low, high;
	uint8_t slot;
};

static std::vector<Rumble> rumbles;
static std::array<int16_t, 4> padCoordinates;

unsigned long millis()
{
	return now;
}

uint32_t __get_PRIMASK()
{
	return interruptMask;
}

void __disable_irq()
{
	interruptMask = 1;
}

void __set_PRIMASK(uint32_t mask)
{
	CHECK(mask <= 1);
	interruptMask = mask;
}

bool tud_mounted()
{
	return mounted;
}

const uint8_t *tud_descriptor_device_cb()
{
	return reinterpret_cast<const uint8_t *>(&device);
}

void Adafruit_USBD_Interface::setStringDescriptor(const char *str)
{
	interfaceString = str;
	_strid = 4;
}

uint8_t Adafruit_USBD_Device::allocInterface(uint8_t count)
{
	uint8_t first = interfaceCount;
	interfaceCount += count;
	return first;
}

uint8_t Adafruit_USBD_Device::allocEndpoint(uint8_t direction)
{
	CHECK(direction <= 1);
	return ++endpointCount[direction] | (direction ? 0x80 : 0);
}

bool Adafruit_USBD_Device::addInterface(Adafruit_USBD_Interface &interface)
{
	additions++;
	registeredInterface = &interface;
	uint16_t length = interface.getInterfaceDescriptor(0, nullptr, 0);
	CHECK(length == 144);
	configuration.resize(length);
	CHECK(interface.getInterfaceDescriptor(0, configuration.data(),
					       length) == length);
	return true;
}

void Adafruit_USBD_Device::setID(uint16_t vendor, uint16_t productID)
{
	device.idVendor = vendor;
	device.idProduct = productID;
}

void Adafruit_USBD_Device::setVersion(uint16_t version)
{
	device.bcdUSB = version;
}

void Adafruit_USBD_Device::setDeviceVersion(uint16_t version)
{
	device.bcdDevice = version;
}

void Adafruit_USBD_Device::setManufacturerDescriptor(const char *str)
{
	manufacturer = str;
}

void Adafruit_USBD_Device::setProductDescriptor(const char *str)
{
	product = str;
}

void Adafruit_USBD_Device::setSerialDescriptor(const char *str)
{
	serial = str;
}

void Adafruit_USBD_Device::setConfigurationAttribute(uint8_t value)
{
	attributes = value;
}

void Adafruit_USBD_Device::setConfigurationMaxPower(uint16_t value)
{
	maxPower = value;
}

void usbTxRegisterDrain(usbTxDrainFn fn)
{
	registrations++;
	drain = fn;
}

void xinputAuthYield()
{
	CHECK(interruptMask == 0);
}

bool hapticSteamRumble(uint16_t low, uint16_t high, uint8_t slot)
{
	CHECK(interruptMask == 0 && slot < NSLOT);
	rumbles.push_back({ low, high, slot });
	return true;
}

void padStickBlend(uint32_t, int16_t lpx, int16_t lpy, int16_t rpx, int16_t rpy,
		   int16_t *, int16_t *, int16_t *, int16_t *)
{
	// Record the caller's reads; shared trackpad blending is not under test.
	padCoordinates = { lpx, lpy, rpx, rpy };
}

extern "C" bool usbd_edpt_open(uint8_t rhport,
			       const tusb_desc_endpoint_t *descriptor)
{
	CHECK(rhport == 0 && descriptor != nullptr);
	uint8_t ep = descriptor->bEndpointAddress;
	CHECK(!endpoints[ep].opened);
	if (ep == failOpen)
		return false;
	CHECK(descriptor->wMaxPacketSize == 32);
	endpoints[ep].opened = true;
	opened.push_back(ep);
	return true;
}

extern "C" void usbd_edpt_close(uint8_t rhport, uint8_t ep)
{
	CHECK(rhport == 0 && endpoints[ep].opened);
	endpoints[ep] = {};
	closed.push_back(ep);
}

extern "C" bool usbd_edpt_xfer(uint8_t rhport, uint8_t ep, uint8_t *buffer,
			       uint16_t length)
{
	CHECK(rhport == 0 && buffer != nullptr);
	auto &endpoint = endpoints[ep];
	CHECK(endpoint.opened && !endpoint.busy);
	CHECK((ep == 0x81 && length == 20) || (ep == 2 && length == 32));
	if (failTransfer) {
		failTransfer = false;
		return false;
	}
	endpoint.busy = true;
	endpoint.claimed = false;
	endpoint.buffer = buffer;
	endpoint.length = length;
	if (ep == 0x81)
		reports.emplace_back(buffer, buffer + length);
	return true;
}

extern "C" bool usbd_edpt_busy(uint8_t rhport, uint8_t ep)
{
	CHECK(rhport == 0);
	return endpoints[ep].busy;
}

extern "C" bool usbd_edpt_claim(uint8_t rhport, uint8_t ep)
{
	CHECK(rhport == 0);
	auto &endpoint = endpoints[ep];
	if (endpoint.busy || endpoint.claimed)
		return false;
	endpoint.claimed = true;
	return true;
}

extern "C" bool usbd_edpt_release(uint8_t rhport, uint8_t ep)
{
	CHECK(rhport == 0 && endpoints[ep].claimed);
	endpoints[ep].claimed = false;
	return true;
}

static uint16_t openInterface(const Bytes &body, size_t offset, uint16_t length)
{
	return xinputClassDriver()->open(
		0,
		reinterpret_cast<const tusb_desc_interface_t *>(body.data() +
								offset),
		length);
}

static void resetUSB()
{
	for (auto &endpoint : endpoints)
		endpoint = {};
	xinputClassDriver()->reset(0);
	g_xboxCtl.task();
	opened.clear();
	closed.clear();
	reports.clear();
	rumbles.clear();
}

static void enumerate()
{
	static const uint16_t starts[] = { 0, 40, 104, 129, 144 };
	for (unsigned i = 0; i < 4; i++) {
		uint16_t length = starts[i + 1] - starts[i];
		CHECK(openInterface(configuration, starts[i], length) ==
		      length);
		CHECK(openInterface(configuration, starts[i], length) == 0);
	}
	CHECK(opened == Bytes({ 0x81, 0x02 }));
	CHECK(endpoints[2].busy && endpoints[2].length == 32);
}

static void completeInput(xfer_result_t result = XFER_RESULT_SUCCESS)
{
	CHECK(endpoints[0x81].busy);
	endpoints[0x81].busy = false;
	CHECK(xinputClassDriver()->xfer_cb(0, 0x81, result, 20));
}

static void output(const Bytes &packet, uint32_t actual,
		   xfer_result_t result = XFER_RESULT_SUCCESS)
{
	auto &ep = endpoints[2];
	CHECK(ep.busy);
	CHECK(packet.size() <= ep.length);
	// Keep the old tail, as DMA does, so short rumble cannot use stale bytes.
	if (!packet.empty())
		memcpy(ep.buffer, packet.data(), packet.size());
	ep.busy = false;
	CHECK(xinputClassDriver()->xfer_cb(0, 2, result, actual));
}

static Bytes neutral()
{
	Bytes report(20);
	report[1] = 20;
	return report;
}

static void enumerationTests()
{
	static_assert(sizeof(tusb_desc_interface_t) == 9, "Interface size");
	static_assert(sizeof(tusb_desc_endpoint_t) == 7, "Endpoint size");
	static_assert(sizeof(tusb_desc_device_t) == 18, "Device size");
	xinputClassDriver()->init();
	g_xboxCtl.begin();
	CHECK(!g_xboxCtl.dynamicMount());
	CHECK(additions == 1 && registrations == 1 && drain != nullptr);
	CHECK(device.idVendor == 0x045E && device.idProduct == 0x028E);
	CHECK(device.bcdUSB == 0x0200 && device.bcdDevice == 0x0114);
	CHECK(device.bDeviceClass == 0xFF && device.bDeviceSubClass == 0xFF);
	CHECK(device.bDeviceProtocol == 0xFF);
	CHECK(manufacturer == "\xc2\xa9Microsoft Corporation");
	CHECK(product == "Controller" && serial == "012345ABCDEF");
	CHECK(attributes == 0xA0 && maxPower == 500);
	CHECK(interfaceString == XINPUT_SECURITY_STRING);
	CHECK(interfaceCount == 4 && endpointCount[0] == 6 &&
	      endpointCount[1] == 6);
	CHECK(configuration ==
	      Bytes(XINPUT_CONFIG_BODY,
		    XINPUT_CONFIG_BODY + sizeof XINPUT_CONFIG_BODY));
	Bytes guard(160, 0xA5);
	for (unsigned length = 0; length < 144; length++)
		CHECK(registeredInterface->getInterfaceDescriptor(
			      0, guard.data(), length) == 0);
	CHECK(registeredInterface->getInterfaceDescriptor(1, guard.data(),
							  160) == 0);
	CHECK(std::all_of(guard.begin(), guard.end(),
			  [](uint8_t b) { return b == 0xA5; }));
	CHECK(interfaceCount == 4 && endpointCount[0] == 6 &&
	      endpointCount[1] == 6);
	CHECK(xinputClassDriver()->open(0, nullptr, 144) == 0);
	static const uint16_t starts[] = { 0, 40, 104, 129, 144 };
	for (unsigned i = 0; i < 4; i++) {
		uint16_t length = starts[i + 1] - starts[i];
		for (uint16_t shortLength = 0; shortLength < length;
		     shortLength++)
			CHECK(openInterface(configuration, starts[i],
					    shortLength) == 0);
		for (size_t byte = starts[i]; byte < starts[i + 1]; byte++) {
			if (byte == XINPUT_SECURITY_STRING_OFFSET)
				continue;
			Bytes corrupt = configuration;
			corrupt[byte] ^= 0x80;
			CHECK(openInterface(corrupt, starts[i], length) == 0);
		}
	}
	CHECK(opened.empty());
	failOpen = 0x81;
	CHECK(openInterface(configuration, 0, 40) == 0);
	CHECK(opened.empty());
	failOpen = 2;
	CHECK(openInterface(configuration, 0, 40) == 0);
	CHECK(opened == Bytes({ 0x81 }) && closed == Bytes({ 0x81 }));
	failOpen = 0;
	resetUSB();
	enumerate();
	drain();
	CHECK(reports.size() == 1 && reports.back() == neutral());
	completeInput();
	for (unsigned i = 0; i < 3; i++) {
		g_xboxCtl.task();
		drain();
	}
	CHECK(reports.size() == 1 && additions == 1);
	CHECK(!xinputClassDriver()->xfer_cb(1, 2, XFER_RESULT_SUCCESS, 8));
	CHECK(!xinputClassDriver()->xfer_cb(0, 0x83, XFER_RESULT_SUCCESS, 8));
	puts("PASS actual mode: static identity, descriptor builder/parser, "
	     "endpoint reservations, accessory claims, neutral without RF");
}

static Bytes rfReport(uint8_t id = 0x45)
{
	Bytes report(28);
	report[0] = id;
	uint32_t buttons = TB_A | TB_VIEW | TB_MENU | TB_DUP | TB_L2;
	for (unsigned i = 0; i < 4; i++)
		report[2 + i] = buttons >> (i * 8);
	le16(report.data() + 8, 0x4000);
	le16(report.data() + 10, -32768);
	le16(report.data() + 12, 32767);
	le16(report.data() + 14, -1);
	le16(report.data() + 16, 1234);
	le16(report.data() + 18, 101);
	le16(report.data() + 20, -102);
	le16(report.data() + 24, 103);
	le16(report.data() + 26, -104);
	return report;
}

static void rfTests()
{
	g_slot[0].used = g_slot[1].used = true;
	g_connReplyMs[0] = g_connReplyMs[1] = now;
	auto report = rfReport();
	for (unsigned length = 0; length < 26; length++) {
		// Report includes ID and sequence before the body. Exact allocations
		// let ASan detect reads beyond the advertised body, if enabled.
		Bytes shortReport(report.begin(), report.begin() + length + 2);
		g_xboxCtl.onReport45(0, shortReport.data(), true, length);
	}
	g_xboxCtl.onReport45(0, nullptr, true, 26);
	for (int slot : { -1, NSLOT, 255 })
		g_xboxCtl.onReport45(slot, report.data(), true, 26);
	for (uint8_t id : { 0, 0x43, 0x44, 0x47, 0xFF }) {
		auto bad = rfReport(id);
		g_xboxCtl.onReport45(0, bad.data(), true, 26);
	}
	g_xboxCtl.onReport45(0, report.data(), false, 26);
	drain();
	CHECK(reports.size() == 1);
	g_connReplyMs[0] = now - 300;
	g_xboxCtl.onReport45(0, report.data(), true, 26);
	drain();
	CHECK(reports.size() == 1);
	g_connReplyMs[0] = now - 299;
	g_xboxCtl.onReport45(0, report.data(), true, 26);
	drain();
	Bytes expected = { 0,	 20,   0x31, 0x10, 0xFF, 0x80, 0,
			   0x80, 0xFF, 0x7F, 0xFF, 0xFF, 0xD2, 4,
			   0,	 0,    0,    0,	   0,	 0 };
	CHECK(reports.size() == 2 && reports.back() == expected);
	CHECK((padCoordinates ==
	       std::array<int16_t, 4>{ 101, -102, 103, -104 }));
	completeInput();
	g_xboxCtl.onReport45(1, report.data(), true, 26);
	drain();
	CHECK(reports.size() == 2);
	Bytes synthetic(28);
	synthetic[0] = 0x45;
	g_xboxCtl.onReport45(0, synthetic.data(), false, 26);
	drain();
	CHECK(reports.size() == 3 && reports.back() == neutral());
	completeInput();
	report[0] = 0x42;
	g_xboxCtl.onReport45(0, report.data(), true, 26);
	drain();
	CHECK(reports.size() == 4 && reports.back() == expected);
	completeInput();
	g_connReplyMs[0] = now - 1200;
	g_xboxCtl.task();
	drain();
	CHECK(reports.size() == 4);
	now++;
	g_xboxCtl.task();
	drain();
	CHECK(reports.size() == 5 && reports.back() == neutral());
	CHECK(rumbles.size() == 1 && rumbles.back().slot == 0);
	CHECK(rumbles.back().low == 0 && rumbles.back().high == 0);
	completeInput();
	g_connReplyMs[1] = now;
	g_xboxCtl.onReport45(1, report.data(), false, 26);
	drain();
	CHECK(reports.size() == 5);
	g_xboxCtl.onReport45(1, report.data(), true, 26);
	drain();
	CHECK(reports.size() == 6 && reports.back() == expected);
	completeInput();
	CHECK(additions == 1 && registrations == 1 && opened.size() == 2);
	puts("PASS actual mode: bodyTlen 0..25, exact 26-byte body, RF selection, "
	     "synthetic neutral, disconnect and replacement without enumeration");
}

static void rumbleTests()
{
	rumbles.clear();
	Bytes valid = { 0, 8, 0, 0x12, 0xFE, 0, 0, 0 };
	output(valid, 8);
	g_xboxCtl.task();
	CHECK(rumbles.size() == 1 && rumbles.back().slot == 1);
	CHECK(rumbles.back().low == 0x1212 && rumbles.back().high == 0xFEFE);
	for (uint32_t length : { 0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 9u, 20u, 32u,
				 33u, 65536u, UINT32_MAX }) {
		Bytes packet = valid;
		packet.resize(std::min<uint32_t>(length, 32));
		output(packet, length);
		g_xboxCtl.task();
		CHECK(rumbles.size() == 1 && endpoints[2].busy);
	}
	for (unsigned byte : { 0, 1, 2, 5, 6, 7 }) {
		Bytes malformed = valid;
		malformed[byte] ^= 1;
		output(malformed, 8);
		g_xboxCtl.task();
		CHECK(rumbles.size() == 1);
	}
	for (auto result : { XFER_RESULT_FAILED, XFER_RESULT_STALLED,
			     XFER_RESULT_TIMEOUT, XFER_RESULT_INVALID }) {
		output(valid, 8, result);
		g_xboxCtl.task();
		CHECK(rumbles.size() == 1);
	}
	for (unsigned led = 0; led <= 255; led++) {
		output({ 1, 3, uint8_t(led) }, 3);
		g_xboxCtl.task();
		CHECK(rumbles.size() == 1);
	}
	g_rumble = 0;
	output(valid, 8);
	g_xboxCtl.task();
	CHECK(rumbles.size() == 2 && rumbles.back().low == 0 &&
	      rumbles.back().high == 0);
	g_rumble = 1;
	output(valid, 8);
	g_xboxCtl.task();
	now += 2500;
	g_connReplyMs[1] = now;
	g_xboxCtl.task();
	CHECK(rumbles.size() == 3);
	now++;
	g_connReplyMs[1] = now;
	g_xboxCtl.task();
	CHECK(rumbles.size() == 4 && rumbles.back().low == 0 &&
	      rumbles.back().high == 0);
	g_xboxCtl.task();
	CHECK(rumbles.size() == 4);
	failTransfer = true;
	output({}, 0);
	CHECK(!endpoints[2].busy);
	drain();
	CHECK(endpoints[2].busy);
	output(valid, 8);
	g_slot[1].used = false;
	g_xboxCtl.task();
	CHECK(rumbles.size() == 5 && rumbles.back().slot == 1);
	CHECK(rumbles.back().low == 0 && rumbles.back().high == 0);
	failTransfer = true;
	drain();
	CHECK(!endpoints[0x81].busy);
	drain();
	CHECK(reports.back() == neutral());
	completeInput(XFER_RESULT_FAILED);
	mounted = false;
	size_t before = reports.size();
	drain();
	CHECK(reports.size() == before);
	mounted = true;
	drain();
	CHECK(reports.size() == before + 1 && reports.back() == neutral());
	completeInput();
	output(valid, 8);
	g_xboxCtl.task();
	CHECK(rumbles.size() == 5);
	CHECK(additions == 1 && registrations == 1);
	puts("PASS actual mode: rumble routing, malformed/stale/failed OUT, LED "
	     "isolation, disable/watchdog/disconnect stops, endpoint retries");
}

void modeTests()
{
	enumerationTests();
	rfTests();
	rumbleTests();
	resetUSB();
	CHECK(xinputClassDriver()->deinit());
	CHECK(interruptMask == 0);
}