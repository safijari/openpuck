// blink_test.ino -- standalone GPIO bring-up test, isolated from OpenPuck's app logic.
//
// Blinks TEST_PIN once a second. Exists to answer one question in isolation: does digitalWrite() on this
// pin/board actually toggle the physical pin, independent of pwr_switch.cpp's gating logic, USB state, radio,
// or anything else in the real firmware. No project libraries needed -- same FQBN as OpenPuck
// (adafruit:nrf52:feather52840), so no new board package to install.
//
// PHYSICAL PIN != ARDUINO PIN NUMBER on this board core, and that mapping is PER BOARD CORE -- the silkscreen
// pad is a fixed hardware trace, but which Arduino number reaches it depends entirely on which core's
// variant.cpp you build against. TEST_PIN = 29 is physical P0.17 ("017" on this SuperMini clone's silkscreen)
// under the Adafruit Feather core we build against here
// (variants/feather_nrf52840_express/variant.cpp: g_ADigitalPinMap[29] == 17) -- confirmed on real hardware
// and matches pwr_switch.h's PWR_SWITCH_PIN default.
//
// Build + flash (separate sketch dir -- does NOT touch OpenPuck/, list ports first with `arduino-cli board
// list`):
//   arduino-cli compile -b adafruit:nrf52:feather52840 blink_test
//   arduino-cli upload  -b adafruit:nrf52:feather52840 -p <port> blink_test
#define TEST_PIN 29 // physical P0.17 ("017" on the board silkscreen) -- see g_ADigitalPinMap in variant.cpp

void setup()
{
	pinMode(TEST_PIN, OUTPUT);
}

void loop()
{
	digitalWrite(TEST_PIN, HIGH);
	delay(500);
	digitalWrite(TEST_PIN, LOW);
	delay(500);
}
