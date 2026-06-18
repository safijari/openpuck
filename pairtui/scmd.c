// scmd.c — Steam Controller / Puck feature-report command tool (macOS IOKit).
// Talks to the VENDOR HID interface (usage page 0xFF00) which opens WITHOUT
// Input Monitoring permission. Implements the 64-byte SET_REPORT(command) plus
// GET_REPORT(reply) command channel observed on the Triton copycat/real puck.
//
//   clang -framework IOKit -framework CoreFoundation -o scmd scmd.c
//   ./scmd list [pidHex]            # list Valve HID nodes and serials
//   ./scmd <pidHex> [--serial S] [--loc HEX]    # run the SAFE read battery (no writes)
//   ./scmd <pidHex> [--serial S] [--loc HEX] <cmdHex> [payloadHexBytes...]   # one custom command
//   ./scmd <pidHex> [--serial S] [--loc HEX] rid2 <cmdHex> [payloadHexBytes...] # custom command on one report id
//   ./scmd <pidHex> [--serial S] [--loc HEX] seq-noget rid2 <cmd> [bytes...] -- rid2 <cmd> [bytes...]
//   ./scmd <pidHex> [--serial S] [--loc HEX] a3watch [count] [delayMs] # watch/decode Triton A3 bond events
//   ./scmd <pidHex> [--serial S] [--loc HEX] a3variants # read-only A3 framing tests
//   ./scmd <pidHex> [--serial S] [--loc HEX] triton-bond <0|1> <keyHex16> <peerSerial>
//   ./scmd <pidHex> [--serial S] [--loc HEX] triton-transport <slot0|slot1|slot2|rawByte>
//
// SAFE reads used by default: 0xAE GET_STRING_ATTRIBUTE (serials), 0x83
// GET_ATTRIBUTES_VALUES, 0xA1 GET_DEVICE_INFO, 0xB4 DONGLE_GET_WIRELESS_STATE.
//
// Command labels below are only labels. 0xA2/0xA3 are Triton-confirmed from our
// copycat logs; the 0xAF..0xC4 names come from SDL's shared Steam constants and
// are not proof of Triton pairing semantics. Destructive opcodes (0x86 reset,
// 0x9F off, 0xAD pair, 0xB2/B3 dongle candidates, 0xAF/B0 radio-record
// candidates) are NEVER sent unless you pass them explicitly.
#include <IOKit/hid/IOHIDManager.h>
#include <IOKit/IOKitLib.h>
#include <CoreFoundation/CoreFoundation.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define VALVE_VID 0x28DE

static void dump(const char *label, const uint8_t *p, int n)
{
	printf("%s [%d]:\n  ", label, n);
	for (int i = 0; i < n; i++) {
		if (i && i % 32 == 0)
			printf("\n  ");
		printf("%02X ", p[i]);
	}
	printf("\n  ascii: ");
	for (int i = 0; i < n; i++)
		putchar((p[i] >= 32 && p[i] < 127) ? p[i] : '.');
	printf("\n");
}

static void print_payload_inline(const uint8_t *p, int n)
{
	if (n <= 0) {
		printf("-");
		return;
	}
	for (int i = 0; i < n; i++) {
		if (i)
			printf(" ");
		printf("%02X", p[i]);
	}
}

static const char *cmd_name(uint8_t cmd)
{
	switch (cmd) {
	case 0x83:
		return "GET_ATTRIBUTES_VALUES";
	case 0xA1:
		return "GET_DEVICE_INFO";
	case 0xA2:
		return "TRITON_A2_OBSERVED_PAIRING_RECORD";
	case 0xA3:
		return "TRITON_A3_BOND_EVENT_OR_STATUS";
	case 0xAD:
		return "ENABLE_PAIRING";
	case 0xAE:
		return "GET_STRING_ATTRIBUTE";
	case 0xAF:
		return "SDL_SHARED_RADIO_ERASE_RECORDS";
	case 0xB0:
		return "SDL_SHARED_RADIO_WRITE_RECORD";
	case 0xB2:
		return "SDL_SHARED_DONGLE_DISCONNECT_DEVICE";
	case 0xB3:
		return "SDL_SHARED_DONGLE_COMMIT_DEVICE_CANDIDATE";
	case 0xB4:
		return "SDL_SHARED_DONGLE_GET_WIRELESS_STATE";
	case 0xC4:
		return "SDL_SHARED_DONGLE_GET_CONNECTED_SLOTS";
	case 0xEE:
		return "TRITON_KEYED_VALUE_WRITE";
	case 0xEF:
		return "TRITON_KEYED_VALUE_COMMIT";
	default:
		return NULL;
	}
}

static int hid_long(IOHIDDeviceRef d, CFStringRef key, long *out)
{
	CFTypeRef v = IOHIDDeviceGetProperty(d, key);
	if (!v || CFGetTypeID(v) != CFNumberGetTypeID())
		return 0;
	return CFNumberGetValue((CFNumberRef)v, kCFNumberLongType, out);
}

static void hid_string(IOHIDDeviceRef d, CFStringRef key, char *out,
		       size_t outn)
{
	if (!outn)
		return;
	out[0] = 0;
	CFTypeRef v = IOHIDDeviceGetProperty(d, key);
	if (!v)
		return;
	if (CFGetTypeID(v) == CFStringGetTypeID()) {
		CFStringGetCString((CFStringRef)v, out, outn,
				   kCFStringEncodingUTF8);
	} else if (CFGetTypeID(v) == CFNumberGetTypeID()) {
		long x = 0;
		if (CFNumberGetValue((CFNumberRef)v, kCFNumberLongType, &x))
			snprintf(out, outn, "%ld", x);
	}
}

static int reg_long(io_object_t svc, const char *key, long *out)
{
	CFStringRef k = CFStringCreateWithCString(kCFAllocatorDefault, key,
						  kCFStringEncodingUTF8);
	if (!k)
		return 0;
	CFTypeRef v =
		IORegistryEntryCreateCFProperty(svc, k, kCFAllocatorDefault, 0);
	CFRelease(k);
	if (!v)
		return 0;
	int ok = 0;
	if (CFGetTypeID(v) == CFNumberGetTypeID())
		ok = CFNumberGetValue((CFNumberRef)v, kCFNumberLongType, out);
	CFRelease(v);
	return ok;
}

static void reg_string(io_object_t svc, const char *key, char *out, size_t outn)
{
	if (!outn)
		return;
	out[0] = 0;
	CFStringRef k = CFStringCreateWithCString(kCFAllocatorDefault, key,
						  kCFStringEncodingUTF8);
	if (!k)
		return;
	CFTypeRef v =
		IORegistryEntryCreateCFProperty(svc, k, kCFAllocatorDefault, 0);
	CFRelease(k);
	if (!v)
		return;
	if (CFGetTypeID(v) == CFStringGetTypeID()) {
		CFStringGetCString((CFStringRef)v, out, outn,
				   kCFStringEncodingUTF8);
	} else if (CFGetTypeID(v) == CFNumberGetTypeID()) {
		long x = 0;
		if (CFNumberGetValue((CFNumberRef)v, kCFNumberLongType, &x))
			snprintf(out, outn, "%ld", x);
	}
	CFRelease(v);
}

static void print_device_line(io_object_t svc, IOHIDDeviceRef d)
{
	long vid = -1, pid = -1, up = -1, usage = -1, loc = -1, maxF = -1;
	char serial[128], product[128], manufacturer[128];
	serial[0] = product[0] = manufacturer[0] = 0;
	reg_long(svc, "VendorID", &vid);
	if (vid < 0)
		hid_long(d, CFSTR(kIOHIDVendorIDKey), &vid);
	reg_long(svc, "ProductID", &pid);
	if (pid < 0)
		hid_long(d, CFSTR(kIOHIDProductIDKey), &pid);
	reg_long(svc, "PrimaryUsagePage", &up);
	if (up < 0)
		hid_long(d, CFSTR(kIOHIDPrimaryUsagePageKey), &up);
	reg_long(svc, "PrimaryUsage", &usage);
	if (usage < 0)
		hid_long(d, CFSTR(kIOHIDPrimaryUsageKey), &usage);
	reg_long(svc, "LocationID", &loc);
	if (loc < 0)
		hid_long(d, CFSTR(kIOHIDLocationIDKey), &loc);
	reg_long(svc, "MaxFeatureReportSize", &maxF);
	reg_string(svc, "SerialNumber", serial, sizeof serial);
	if (!serial[0])
		hid_string(d, CFSTR(kIOHIDSerialNumberKey), serial,
			   sizeof serial);
	reg_string(svc, "Product", product, sizeof product);
	if (!product[0])
		hid_string(d, CFSTR(kIOHIDProductKey), product, sizeof product);
	reg_string(svc, "Manufacturer", manufacturer, sizeof manufacturer);
	if (!manufacturer[0])
		hid_string(d, CFSTR(kIOHIDManufacturerKey), manufacturer,
			   sizeof manufacturer);
	printf("Valve %04lX:%04lX usagePage=0x%lX usage=0x%lX maxFeature=%ld loc=0x%lX serial=%s product=%s manufacturer=%s\n",
	       vid, pid, up, usage, maxF, loc, serial[0] ? serial : "-",
	       product[0] ? product : "-",
	       manufacturer[0] ? manufacturer : "-");
}

static int list_devices(long wantPid)
{
	CFMutableDictionaryRef m = IOServiceMatching("IOHIDDevice");
	io_iterator_t it;
	if (IOServiceGetMatchingServices(kIOMainPortDefault, m, &it) !=
	    KERN_SUCCESS) {
		fprintf(stderr, "match fail\n");
		return 1;
	}
	int n = 0;
	io_object_t svc;
	while ((svc = IOIteratorNext(it))) {
		IOHIDDeviceRef d = IOHIDDeviceCreate(kCFAllocatorDefault, svc);
		if (d) {
			long vid = -1, pid = -1;
			reg_long(svc, "VendorID", &vid);
			if (vid < 0)
				hid_long(d, CFSTR(kIOHIDVendorIDKey), &vid);
			reg_long(svc, "ProductID", &pid);
			if (pid < 0)
				hid_long(d, CFSTR(kIOHIDProductIDKey), &pid);
			if (vid == VALVE_VID &&
			    (wantPid < 0 || pid == wantPid)) {
				print_device_line(svc, d);
				n++;
			}
			CFRelease(d);
		}
		IOObjectRelease(svc);
	}
	IOObjectRelease(it);
	if (!n)
		printf("no Valve HID devices found%s\n",
		       wantPid < 0 ? "" : " for requested PID");
	return n ? 0 : 1;
}

// Issue one command on report id `rid`: [rid][cmd][len][payload...].
static void sc_cmd_ex(IOHIDDeviceRef d, int rid, uint8_t cmd, const uint8_t *pl,
		      int pln, int do_get)
{
	uint8_t tx[64];
	memset(tx, 0, sizeof tx);
	tx[0] = (uint8_t)rid;
	tx[1] = cmd;
	tx[2] = (uint8_t)pln;
	if (pln > 0)
		memcpy(tx + 3, pl, pln);
	IOReturn rs =
		IOHIDDeviceSetReport(d, kIOHIDReportTypeFeature, rid, tx, 64);
	const char *name = cmd_name(cmd);
	if (rs != kIOReturnSuccess) {
		printf("\n=== cmd 0x%02X%s%s (payload %d: ", cmd,
		       name ? " " : "", name ? name : "", pln);
		print_payload_inline(pl, pln);
		printf(") on report id %d  [set=0x%08X get=skipped-set-failed] ===\n",
		       rid, rs);
		return;
	}
	if (!do_get) {
		printf("\n=== cmd 0x%02X%s%s (payload %d: ", cmd,
		       name ? " " : "", name ? name : "", pln);
		print_payload_inline(pl, pln);
		printf(") on report id %d  [set=0x%08X get=skipped] ===\n", rid,
		       rs);
		return;
	}
	usleep(20000);
	uint8_t rx[64];
	memset(rx, 0, sizeof rx);
	rx[0] = (uint8_t)rid;
	CFIndex rl = 64;
	IOReturn rg =
		IOHIDDeviceGetReport(d, kIOHIDReportTypeFeature, rid, rx, &rl);
	printf("\n=== cmd 0x%02X%s%s (payload %d: ", cmd, name ? " " : "",
	       name ? name : "", pln);
	print_payload_inline(pl, pln);
	printf(") on report id %d  [set=0x%08X get=0x%08X] ===\n", rid, rs, rg);
	if (rg == kIOReturnSuccess)
		dump("reply", rx, (int)rl);
	else
		printf("  (GET failed)\n");
}

static void sc_cmd(IOHIDDeviceRef d, int rid, uint8_t cmd, const uint8_t *pl,
		   int pln)
{
	sc_cmd_ex(d, rid, cmd, pl, pln, 1);
}

static int hex_value(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}

static int parse_hex_bytes(const char *hex, uint8_t *out, int outn)
{
	int n = 0;
	int hi = -1;
	for (const char *p = hex; *p; p++) {
		if (*p == ':' || *p == '-' || *p == ' ' || *p == '_')
			continue;
		int v = hex_value(*p);
		if (v < 0)
			return -1;
		if (hi < 0) {
			hi = v;
		} else {
			if (n >= outn)
				return -1;
			out[n++] = (uint8_t)((hi << 4) | v);
			hi = -1;
		}
	}
	if (hi >= 0)
		return -1;
	return n;
}

static int make_serial16(const char *serial, uint8_t out[16])
{
	memset(out, 0, 16);
	size_t n = strlen(serial);
	if (n > 16)
		return 0;
	memcpy(out, serial, n);
	return 1;
}

static int triton_keyed_write(IOHIDDeviceRef dev, const char *key,
			      const uint8_t *payload, int payload_len)
{
	size_t key_len = strlen(key) + 1;
	if (key_len + (size_t)payload_len > 61 || key_len > 61) {
		fprintf(stderr,
			"keyed payload too long for 64-byte feature report\n");
		return 2;
	}

	uint8_t ee[61];
	memcpy(ee, key, key_len);
	if (payload_len > 0)
		memcpy(ee + key_len, payload, payload_len);

	uint8_t ef[61];
	memcpy(ef, key, key_len);

	printf("Triton keyed write key=\"%s\" payload=", key);
	print_payload_inline(payload, payload_len);
	printf("\n");
	sc_cmd_ex(dev, 2, 0xEE, ee, (int)key_len + payload_len, 0);
	usleep(20000);
	sc_cmd_ex(dev, 2, 0xEF, ef, (int)key_len, 0);
	return 0;
}

static int triton_bond_write(IOHIDDeviceRef dev, int slot, const char *key_hex,
			     const char *serial)
{
	if (slot < 0 || slot > 1) {
		fprintf(stderr, "triton-bond slot must be 0 or 1\n");
		return 2;
	}
	uint8_t payload[24];
	int keyn = parse_hex_bytes(key_hex, payload, 8);
	if (keyn != 8) {
		fprintf(stderr,
			"triton-bond key must be exactly 8 bytes / 16 hex digits\n");
		return 2;
	}
	if (!make_serial16(serial, payload + 8)) {
		fprintf(stderr,
			"controller serial must be at most 16 ASCII bytes\n");
		return 2;
	}
	const char *key = slot == 0 ? "esb/bond" : "esb/bond_2";
	printf("note: Steam uses this keyed path for controller-side bond storage; real pucks reject it.\n");
	return triton_keyed_write(dev, key, payload, sizeof payload);
}

static int parse_triton_transport(const char *arg, uint8_t *out)
{
	if (strcmp(arg, "slot0") == 0 || strcmp(arg, "0") == 0) {
		*out = 2;
		return 1;
	}
	if (strcmp(arg, "slot1") == 0 || strcmp(arg, "1") == 0) {
		*out = 3;
		return 1;
	}
	if (strcmp(arg, "slot2") == 0 || strcmp(arg, "2") == 0) {
		*out = 0;
		return 1;
	}
	char *end = NULL;
	long raw = strtol(arg, &end, 0);
	if (end && *end == 0 && raw >= 0 && raw <= 255) {
		*out = (uint8_t)raw;
		return 1;
	}
	return 0;
}

static int triton_transport_write(IOHIDDeviceRef dev, const char *arg)
{
	uint8_t value = 0;
	if (!parse_triton_transport(arg, &value)) {
		fprintf(stderr,
			"triton-transport expects slot0, slot1, slot2, or raw byte 0..255\n");
		return 2;
	}
	return triton_keyed_write(dev, "user/wireless_transport", &value, 1);
}

static void print_a3_decode(const uint8_t *rx, int rl)
{
	if (rl < 3 || rx[1] != 0xA3) {
		printf("  not an A3 reply\n");
		return;
	}
	int len = rx[2];
	if (len != 0x18 || rl < 27) {
		printf("  A3 len=0x%02X (no bond payload)\n", len);
		return;
	}
	const uint8_t *key = rx + 3;
	const uint8_t *serial = rx + 11;
	uint32_t w0 = (uint32_t)key[0] | ((uint32_t)key[1] << 8) |
		      ((uint32_t)key[2] << 16) | ((uint32_t)key[3] << 24);
	uint32_t w1 = (uint32_t)key[4] | ((uint32_t)key[5] << 8) |
		      ((uint32_t)key[6] << 16) | ((uint32_t)key[7] << 24);
	printf("  A3 bond key=");
	for (int i = 0; i < 8; i++)
		printf("%s%02X", i ? " " : "", key[i]);
	printf(" steam_words=%X_%X serial=", w0, w1);
	for (int i = 0; i < 16 && serial[i]; i++)
		putchar((serial[i] >= 32 && serial[i] < 127) ? serial[i] : '.');
	printf("\n");
}

static void print_a3_decode_any(const uint8_t *rx, int rl)
{
	if (rl >= 2 && rx[0] == 0xA3) {
		uint8_t shifted[64];
		int n = rl + 1;
		if (n > (int)sizeof shifted)
			n = (int)sizeof shifted;
		memset(shifted, 0, sizeof shifted);
		shifted[0] = 2;
		memcpy(shifted + 1, rx, n - 1);
		print_a3_decode(shifted, n);
		return;
	}
	print_a3_decode(rx, rl);
}

static int a3_watch(IOHIDDeviceRef dev, int count, int delay_ms)
{
	if (count <= 0)
		count = 20;
	if (delay_ms < 0)
		delay_ms = 100;
	for (int i = 0; i < count; i++) {
		uint8_t tx[64];
		memset(tx, 0, sizeof tx);
		tx[0] = 2;
		tx[1] = 0xA3;
		tx[2] = 0;
		IOReturn rs = IOHIDDeviceSetReport(dev, kIOHIDReportTypeFeature,
						   2, tx, 64);
		usleep(20000);
		uint8_t rx[64];
		memset(rx, 0, sizeof rx);
		rx[0] = 2;
		CFIndex rl = 64;
		IOReturn rg = kIOReturnError;
		if (rs == kIOReturnSuccess)
			rg = IOHIDDeviceGetReport(dev, kIOHIDReportTypeFeature,
						  2, rx, &rl);
		printf("[%02d] A3 set=0x%08X get=0x%08X", i + 1, rs, rg);
		if (rg == kIOReturnSuccess && rl >= 3)
			printf(" reply=%02X %02X %02X", rx[0], rx[1], rx[2]);
		printf("\n");
		if (rg == kIOReturnSuccess)
			print_a3_decode(rx, (int)rl);
		if (delay_ms > 0 && i + 1 < count)
			usleep((useconds_t)delay_ms * 1000);
	}
	return 0;
}

static void a3_variant(IOHIDDeviceRef dev, const char *label, int include_rid,
		       int set_len, int get_len, int seed_rid)
{
	uint8_t tx[64];
	memset(tx, 0, sizeof tx);
	if (include_rid) {
		tx[0] = 2;
		tx[1] = 0xA3;
		tx[2] = 0;
	} else {
		tx[0] = 0xA3;
		tx[1] = 0;
	}
	IOReturn rs = IOHIDDeviceSetReport(dev, kIOHIDReportTypeFeature, 2, tx,
					   set_len);
	usleep(20000);
	uint8_t rx[64];
	memset(rx, 0, sizeof rx);
	if (seed_rid)
		rx[0] = 2;
	CFIndex rl = get_len;
	IOReturn rg = kIOReturnError;
	if (rs == kIOReturnSuccess)
		rg = IOHIDDeviceGetReport(dev, kIOHIDReportTypeFeature, 2, rx,
					  &rl);
	printf("\n=== A3 variant %s setLen=%d getLen=%d includeRid=%d seedRid=%d set=0x%08X get=0x%08X rl=%ld ===\n",
	       label, set_len, get_len, include_rid, seed_rid, rs, rg,
	       (long)rl);
	if (rg == kIOReturnSuccess) {
		dump("reply", rx, (int)rl);
		print_a3_decode_any(rx, (int)rl);
	}
}

static int a3_variants(IOHIDDeviceRef dev)
{
	a3_variant(dev, "steam-short", 0, 2, 0x1a, 0);
	a3_variant(dev, "steam-short-seeded", 0, 2, 0x1a, 1);
	a3_variant(dev, "rid-short", 1, 3, 0x1b, 1);
	a3_variant(dev, "rid-64", 1, 64, 64, 1);
	return 0;
}

static int run_seq(IOHIDDeviceRef dev, int argc, char **argv, int start,
		   int do_get)
{
	int i = start;
	while (i < argc) {
		if (strcmp(argv[i], "--") == 0) {
			i++;
			continue;
		}
		if (strncmp(argv[i], "rid", 3) != 0) {
			fprintf(stderr,
				"seq expects ridN before each command, got: %s\n",
				argv[i]);
			return 2;
		}
		int rid = atoi(argv[i] + 3);
		if (rid < 1 || rid > 255) {
			fprintf(stderr, "bad report id: %s\n", argv[i]);
			return 2;
		}
		i++;
		if (i >= argc) {
			fprintf(stderr, "seq missing cmd after rid%d\n", rid);
			return 2;
		}
		uint8_t cmd = (uint8_t)strtol(argv[i++], NULL, 16);
		uint8_t pl[62];
		int pln = 0;
		while (i < argc && strcmp(argv[i], "--") != 0) {
			if (strncmp(argv[i], "rid", 3) == 0)
				break;
			if (pln >= 62) {
				fprintf(stderr,
					"payload too long for cmd 0x%02X\n",
					cmd);
				return 2;
			}
			pl[pln++] = (uint8_t)strtol(argv[i++], NULL, 16);
		}
		sc_cmd_ex(dev, rid, cmd, pl, pln, do_get);
		usleep(20000);
	}
	return 0;
}

int main(int argc, char **argv)
{
	if (argc < 2) {
		fprintf(stderr,
			"usage: %s list [pidHex] | <pidHex> [--serial SERIAL] [cmdHex payloadBytes...]\n",
			argv[0]);
		return 2;
	}
	if (strcmp(argv[1], "list") == 0) {
		long wantPid = argc >= 3 ? strtol(argv[2], NULL, 16) : -1;
		return list_devices(wantPid);
	}
	long wantPid = strtol(argv[1], NULL, 16);
	const char *wantSerial = NULL;
	long wantLoc = -1;
	long wantUp = -1;
	int wantIdx = -1;
	int matchIdx = 0;
	int arg = 2;
	while (arg < argc) {
		if (strcmp(argv[arg], "--serial") == 0) {
			if (arg + 1 >= argc) {
				fprintf(stderr, "--serial needs a value\n");
				return 2;
			}
			wantSerial = argv[arg + 1];
			arg += 2;
		} else if (strncmp(argv[arg], "--serial=", 9) == 0) {
			wantSerial = argv[arg] + 9;
			arg++;
		} else if (strcmp(argv[arg], "--loc") == 0) {
			if (arg + 1 >= argc) {
				fprintf(stderr, "--loc needs a value\n");
				return 2;
			}
			wantLoc = strtol(argv[arg + 1], NULL, 0);
			arg += 2;
		} else if (strncmp(argv[arg], "--loc=", 6) == 0) {
			wantLoc = strtol(argv[arg] + 6, NULL, 0);
			arg++;
		} else if (strcmp(argv[arg], "--up") == 0) {
			if (arg + 1 >= argc) {
				fprintf(stderr, "--up needs a value\n");
				return 2;
			}
			wantUp = strtol(argv[arg + 1], NULL, 0);
			arg += 2;
		} else if (strcmp(argv[arg], "--idx") == 0) {
			if (arg + 1 >= argc) {
				fprintf(stderr, "--idx needs a value\n");
				return 2;
			}
			wantIdx = strtol(argv[arg + 1], NULL, 0);
			arg += 2;
		} else {
			break;
		}
	}

	CFMutableDictionaryRef m = IOServiceMatching("IOHIDDevice");
	io_iterator_t it;
	if (IOServiceGetMatchingServices(kIOMainPortDefault, m, &it) !=
	    KERN_SUCCESS) {
		fprintf(stderr, "match fail\n");
		return 1;
	}

	IOHIDDeviceRef dev = NULL;
	io_object_t svc;
	while ((svc = IOIteratorNext(it))) {
		CFTypeRef vid = IORegistryEntryCreateCFProperty(
			svc, CFSTR("VendorID"), kCFAllocatorDefault, 0);
		CFTypeRef pid = IORegistryEntryCreateCFProperty(
			svc, CFSTR("ProductID"), kCFAllocatorDefault, 0);
		long v = -1, p = -1;
		if (vid) {
			CFNumberGetValue((CFNumberRef)vid, kCFNumberLongType,
					 &v);
			CFRelease(vid);
		}
		if (pid) {
			CFNumberGetValue((CFNumberRef)pid, kCFNumberLongType,
					 &p);
			CFRelease(pid);
		}
		if (v == VALVE_VID && p == wantPid) {
			IOHIDDeviceRef d =
				IOHIDDeviceCreate(kCFAllocatorDefault, svc);
			long up = -1, maxF = -1, loc = -1;
			if (d) {
				reg_long(svc, "PrimaryUsagePage", &up);
				if (up < 0) {
					CFTypeRef u = IOHIDDeviceGetProperty(
						d,
						CFSTR(kIOHIDPrimaryUsagePageKey));
					if (u)
						CFNumberGetValue(
							(CFNumberRef)u,
							kCFNumberLongType, &up);
				}
				reg_long(svc, "MaxFeatureReportSize", &maxF);
				reg_long(svc, "LocationID", &loc);
			}
			char serial[128] = "";
			char product[128] = "";
			if (d) {
				reg_string(svc, "SerialNumber", serial,
					   sizeof serial);
				if (!serial[0])
					hid_string(d,
						   CFSTR(kIOHIDSerialNumberKey),
						   serial, sizeof serial);
				reg_string(svc, "Product", product,
					   sizeof product);
				if (!product[0])
					hid_string(d, CFSTR(kIOHIDProductKey),
						   product, sizeof product);
			}
			if (wantSerial && strcmp(wantSerial, serial) != 0) {
				if (d)
					CFRelease(d);
				IOObjectRelease(svc);
				continue;
			}
			if (wantLoc >= 0 && loc != wantLoc) {
				if (d)
					CFRelease(d);
				IOObjectRelease(svc);
				continue;
			}
			if (wantUp >= 0 && up != wantUp) {
				if (d)
					CFRelease(d);
				IOObjectRelease(svc);
				continue;
			}
			if (wantIdx >= 0 && matchIdx++ != wantIdx) {
				if (d)
					CFRelease(d);
				IOObjectRelease(svc);
				continue;
			}
			int command_capable = (up == 0xFF00) || (maxF >= 64);
			IOReturn ro =
				d && command_capable ?
					IOHIDDeviceOpen(d,
							kIOHIDOptionsTypeNone) :
					kIOReturnNotFound;
			if (d && command_capable && ro == kIOReturnSuccess) {
				printf("opened Valve %04lX:%04lX node (usagePage=0x%lX maxFeature=%ld serial=%s product=%s)\n",
				       (long)VALVE_VID, wantPid, up, maxF,
				       serial[0] ? serial : "-",
				       product[0] ? product : "-");
				dev = d;
				IOObjectRelease(svc);
				break;
			}
			if (d && command_capable &&
			    (wantSerial || wantLoc >= 0)) {
				fprintf(stderr,
					"matched Valve %04lX:%04lX usagePage=0x%lX maxFeature=%ld loc=0x%lX serial=%s but open failed: 0x%08X\n",
					(long)VALVE_VID, wantPid, up, maxF, loc,
					serial[0] ? serial : "-", ro);
			}
			if (d)
				CFRelease(d);
		}
		IOObjectRelease(svc);
	}
	IOObjectRelease(it);
	if (!dev) {
		if (wantSerial || wantLoc >= 0)
			fprintf(stderr,
				"no openable Valve %04lX command-capable interface matching serial=%s loc=0x%lX (quit Steam? wrong device?)\n",
				wantPid, wantSerial ? wantSerial : "-",
				wantLoc);
		else
			fprintf(stderr,
				"no openable Valve %04lX command-capable interface (quit Steam? add --serial/--loc?)\n",
				wantPid);
		return 1;
	}

	if (arg < argc && strcmp(argv[arg], "slotwrite") == 0) {
		// Single-session dongle slot write: 0xAD 01 (enter pairing mode) -> 0xA2 <24-byte record>
		// -> 0xAD 00, all on the SAME open handle (Steam keeps one session; pairing mode resets on
		// close). record = [8-byte uuids][16-byte serial]; zeros-serial clears the matching slot.
		// first arg = report id (the ALT channel is report id 3); rest = 24-byte record
		int rid = (arg + 1 < argc) ? atoi(argv[arg + 1]) : 3;
		uint8_t rec[24];
		memset(rec, 0, sizeof rec);
		int n = 0;
		for (int i = arg + 2; i < argc && n < 24; i++)
			rec[n++] = (uint8_t)strtol(argv[i], NULL, 16);
		uint8_t on[2] = { 0x01, 0x00 }, off[1] = { 0x00 };
		printf("# slotwrite rid%d: AD 01 -> A2 <24> -> AD 00 (one session)\n",
		       rid);
		sc_cmd(dev, rid, 0xAD, on, 2);
		sc_cmd(dev, rid, 0xA2, rec, 24);
		sc_cmd(dev, rid, 0xAD, off, 1);
		IOHIDDeviceClose(dev, kIOHIDOptionsTypeNone);
		CFRelease(dev);
		return 0;
	} else if (arg < argc && strcmp(argv[arg], "pair") == 0) {
		// Steamless pairing (firmware-derived): 0xAD [mode=1][channel] enters RF pairing mode;
		// the puck then runs the pairing handshake autonomously. Channel 0 => firmware default
		// (0x3c). Put the controller in pairing via its key-chord, then we poll 0xA3 for the bond.
		uint8_t ch = (arg + 1 < argc) ?
				     (uint8_t)strtol(argv[arg + 1], NULL, 16) :
				     0x00;
		uint8_t start[2] = { 0x01, ch };
		printf("# pair: entering pairing mode (0xAD 01 %02X).\n", ch);
		printf("# >>> Now put the controller into pairing via its key-chord. Polling 0xA3...\n");
		sc_cmd(dev, 2, 0xAD, start, 2);
		int rc = a3_watch(
			dev, 120,
			200); // ~24s: watch status for the controller to bond
		uint8_t stop[1] = { 0x00 };
		printf("# pair: exiting pairing mode (0xAD 00).\n");
		sc_cmd(dev, 2, 0xAD, stop, 1);
		IOHIDDeviceClose(dev, kIOHIDOptionsTypeNone);
		CFRelease(dev);
		return rc;
	} else if (arg < argc && strcmp(argv[arg], "stop") == 0) {
		uint8_t stop[1] = { 0x00 }; // exit pairing mode
		sc_cmd(dev, 2, 0xAD, stop, 1);
		IOHIDDeviceClose(dev, kIOHIDOptionsTypeNone);
		CFRelease(dev);
		return 0;
	} else if (arg < argc && strcmp(argv[arg], "info") == 0) {
		// Read identity + bond status without changing anything.
		printf("\n##### 0x83 attributes #####\n");
		sc_cmd(dev, 2, 0x83, NULL, 0);
		printf("\n##### 0xAE strings (idx 0..7) #####\n");
		for (int a = 0; a < 8; a++) {
			uint8_t aa = (uint8_t)a;
			sc_cmd(dev, 2, 0xAE, &aa, 1);
		}
		printf("\n##### 0xB4 wireless state #####\n");
		sc_cmd(dev, 2, 0xB4, NULL, 0);
		printf("\n##### 0xA3 status / bond #####\n");
		sc_cmd(dev, 2, 0xA3, NULL, 0);
		IOHIDDeviceClose(dev, kIOHIDOptionsTypeNone);
		CFRelease(dev);
		return 0;
	} else if (arg < argc && strcmp(argv[arg], "a3watch") == 0) {
		int count = (arg + 1 < argc) ? atoi(argv[arg + 1]) : 20;
		int delay_ms = (arg + 2 < argc) ? atoi(argv[arg + 2]) : 100;
		int rc = a3_watch(dev, count, delay_ms);
		IOHIDDeviceClose(dev, kIOHIDOptionsTypeNone);
		CFRelease(dev);
		return rc;
	} else if (arg < argc && strcmp(argv[arg], "a3variants") == 0) {
		int rc = a3_variants(dev);
		IOHIDDeviceClose(dev, kIOHIDOptionsTypeNone);
		CFRelease(dev);
		return rc;
	} else if (arg < argc && strcmp(argv[arg], "triton-bond") == 0) {
		if (arg + 3 >= argc) {
			fprintf(stderr,
				"usage: %s <pidHex> [--serial S] triton-bond <0|1> <keyHex16> <controllerSerial>\n",
				argv[0]);
			IOHIDDeviceClose(dev, kIOHIDOptionsTypeNone);
			CFRelease(dev);
			return 2;
		}
		int slot = atoi(argv[arg + 1]);
		int rc = triton_bond_write(dev, slot, argv[arg + 2],
					   argv[arg + 3]);
		IOHIDDeviceClose(dev, kIOHIDOptionsTypeNone);
		CFRelease(dev);
		return rc;
	} else if (arg < argc && strcmp(argv[arg], "triton-transport") == 0) {
		if (arg + 1 >= argc) {
			fprintf(stderr,
				"usage: %s <pidHex> [--serial S] triton-transport <slot0|slot1|slot2|rawByte>\n",
				argv[0]);
			IOHIDDeviceClose(dev, kIOHIDOptionsTypeNone);
			CFRelease(dev);
			return 2;
		}
		int rc = triton_transport_write(dev, argv[arg + 1]);
		IOHIDDeviceClose(dev, kIOHIDOptionsTypeNone);
		CFRelease(dev);
		return rc;
	}
	// probe all non-destructive opcodes for bond/serial data
	else if (arg < argc && strcmp(argv[arg], "find") == 0) {
		// BLOCKLIST = known state-changing/destructive opcodes; never sent here.
		static const int block[] = { 0x80, 0x81, 0x85, 0x86, 0x87, 0x88,
					     0x8D, 0x8E, 0x8F, 0x9F, 0xAD, 0xAF,
					     0xB0, 0xB2, 0xB3, 0xEB, -1 };
		int rid = 2;
		for (int cmd = 0x82; cmd <= 0xFE; cmd++) {
			int blocked = 0;
			for (const int *b = block; *b >= 0; b++)
				if (*b == cmd)
					blocked = 1;
			if (blocked)
				continue;
			// send with empty payload; report only if SET succeeded (supported opcode)
			uint8_t tx[64];
			memset(tx, 0, sizeof tx);
			tx[0] = rid;
			tx[1] = (uint8_t)cmd;
			tx[2] = 0;
			IOReturn rs = IOHIDDeviceSetReport(
				dev, kIOHIDReportTypeFeature, rid, tx, 64);
			if (rs != kIOReturnSuccess)
				continue; // unsupported -> stall, skip silently
			usleep(15000);
			uint8_t rx[64];
			memset(rx, 0, sizeof rx);
			rx[0] = rid;
			CFIndex rl = 64;
			IOReturn rg = IOHIDDeviceGetReport(
				dev, kIOHIDReportTypeFeature, rid, rx, &rl);
			if (rg == kIOReturnSuccess &&
			    rx[1] == cmd) { // echoed -> real reply
				char lbl[24];
				snprintf(lbl, sizeof lbl,
					 "SUPPORTED cmd 0x%02X", cmd);
				dump(lbl, rx, (int)rl);
			}
		}
	} else if (arg < argc &&
		   strcmp(argv[arg], "labels") ==
			   0) { // SAFE read-only label/settings sweep
		int rid = 2;
		printf("\n##### 0x84 GET_ATTRIBUTE_LABEL idx 0..15 #####\n");
		for (int i = 0; i < 16; i++) {
			uint8_t p[2] = { (uint8_t)i, 0 };
			sc_cmd(dev, rid, 0x84, p, 2);
		}
		printf("\n##### 0x8A GET_SETTING_LABEL idx 0..47 #####\n");
		for (int i = 0; i < 48; i++) {
			uint8_t p[2] = { (uint8_t)i, 0 };
			sc_cmd(dev, rid, 0x8A, p, 2);
		}
		printf("\n##### 0x89 GET_SETTINGS_VALUES idx 0..47 #####\n");
		for (int i = 0; i < 48; i++) {
			uint8_t p[2] = { (uint8_t)i, 0 };
			sc_cmd(dev, rid, 0x89, p, 2);
		}
	} else if (arg < argc && strcmp(argv[arg], "seq") == 0) {
		int rc = run_seq(dev, argc, argv, arg + 1, 1);
		IOHIDDeviceClose(dev, kIOHIDOptionsTypeNone);
		CFRelease(dev);
		return rc;
	} else if (arg < argc && strcmp(argv[arg], "seq-noget") == 0) {
		int rc = run_seq(dev, argc, argv, arg + 1, 0);
		IOHIDDeviceClose(dev, kIOHIDOptionsTypeNone);
		CFRelease(dev);
		return rc;
	} else if (arg + 1 < argc &&
		   strncmp(argv[arg], "rid", 3) ==
			   0) { // custom command on one report id
		int rid = atoi(argv[arg] + 3);
		if (rid < 1 || rid > 255) {
			fprintf(stderr, "bad report id: %s\n", argv[arg]);
			return 2;
		}
		uint8_t cmd = (uint8_t)strtol(argv[arg + 1], NULL, 16);
		uint8_t pl[62];
		int pln = 0;
		for (int i = arg + 2; i < argc && pln < 62; i++)
			pl[pln++] = (uint8_t)strtol(argv[i], NULL, 16);
		sc_cmd(dev, rid, cmd, pl, pln);
	} else if (arg <
		   argc) { // custom single command on both command report ids
		uint8_t cmd = (uint8_t)strtol(argv[arg], NULL, 16);
		uint8_t pl[62];
		int pln = 0;
		for (int i = arg + 1; i < argc && pln < 62; i++)
			pl[pln++] = (uint8_t)strtol(argv[i], NULL, 16);
		for (int rid = 1; rid <= 2; rid++)
			sc_cmd(dev, rid, cmd, pl, pln);
	} else { // safe read battery on the working channel (report id 2)
		int rid = 2;
		printf("\n########## string attributes 0xAE (idx 0..15) ##########\n");
		for (int a = 0; a < 16; a++) {
			uint8_t aa = (uint8_t)a;
			sc_cmd(dev, rid, 0xAE, &aa, 1);
		}
		printf("\n########## 0x83 GET_ATTRIBUTES_VALUES ##########\n");
		sc_cmd(dev, rid, 0x83, NULL, 0);
		printf("\n########## 0xB4 DONGLE_GET_WIRELESS_STATE (no idx, then idx 0..3) ##########\n");
		sc_cmd(dev, rid, 0xB4, NULL, 0);
		for (int i = 0; i < 4; i++) {
			uint8_t ii = (uint8_t)i;
			sc_cmd(dev, rid, 0xB4, &ii, 1);
		}
		printf("\n########## 0x82 GET_DIGITAL_MAPPINGS (read) ##########\n");
		sc_cmd(dev, rid, 0x82, NULL, 0);
	}
	IOHIDDeviceClose(dev, kIOHIDOptionsTypeNone);
	CFRelease(dev);
	return 0;
}
