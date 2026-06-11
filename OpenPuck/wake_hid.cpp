#include "wake_hid.h"
#include <Adafruit_TinyUSB.h>

// A bog-standard boot MOUSE descriptor -- the same shape Xbox mode's right-pad mouse uses, which is proven to
// enumerate and wake Windows. We never send a report on it; it exists purely so the host enumerates a
// "HID-compliant mouse" child and grants the device wake-from-sleep privileges. (A boot keyboard was tried
// first and was worse: it didn't enumerate on Windows and suppressed the wake that the mouse class provides.)
static const uint8_t WAKE_HID_DESC[] = { TUD_HID_REPORT_DESC_MOUSE() };
static Adafruit_USBD_HID g_wakeHid;

void wakeHidBegin(){
  g_wakeHid.setBootProtocol(HID_ITF_PROTOCOL_MOUSE);   // boot mouse = a wake device class honored by Windows + Linux
  g_wakeHid.setStringDescriptor("OpenPuck Wake");
  g_wakeHid.setReportDescriptor(WAKE_HID_DESC, sizeof WAKE_HID_DESC);
  g_wakeHid.setPollInterval(10);
  g_wakeHid.begin();
}
