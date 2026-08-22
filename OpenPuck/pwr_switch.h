// pwr_switch.h -- GPIO trigger emulating a momentary press of the HOST motherboard's power switch.
//
// Optional feature (build with -DOPK_PWR_SWITCH=1, see config.h): fires ONE momentary pulse on
// PWR_SWITCH_PIN when the paired Steam Controller's STEAM button gets a short press (down+up
// within 1s -- the same gesture rf_link.cpp already recognizes for USB remote wakeup) WHILE the
// host is OFF: USB has never enumerated / has stopped enumerating (USBDevice.mounted() == false),
// sustained for HOST_OFF_DEBOUNCE_MS so a brief re-enumeration blip can't false-fire. Assumes the
// USB port keeps standby power while the host is off (so this firmware keeps running and
// listening) and the controller is already bonded -- there is no pairing UI here.
//
// Hardware note: PWR_SWITCH_PIN must drive an ISOLATING stage (relay / optocoupler / transistor)
// wired across the motherboard's front-panel power-switch header -- never short the header
// directly to the nRF's GPIO. This module only knows how to drive one pin; the isolation is on you.
#pragma once

#ifndef PWR_SWITCH_PIN
// Arduino pin number for physical P0.17 ("017" on this SuperMini clone's silkscreen) under the
// Adafruit Feather core this builds against (variants/feather_nrf52840_express/variant.cpp:
// g_ADigitalPinMap[29] == 17) -- confirmed on real hardware with a standalone blink sketch. The
// silkscreen number is NOT the Arduino pin number; don't assume identity on a different board.
#define PWR_SWITCH_PIN 29
#endif
#ifndef PWR_SWITCH_ACTIVE
#define PWR_SWITCH_ACTIVE \
	HIGH // level that closes the switch; set LOW if your driver is active-low
#endif

void pwrSwitchInit(); // call once from setup(): pin to output, released (inactive) level
void pwrSwitchTask(); // call every loop(): edge-detect the Steam short press, gate, pulse, release
