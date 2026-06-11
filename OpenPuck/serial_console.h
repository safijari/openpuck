// serial_console.h -- the CDC debug command line.
//
// Single-letter commands over USB CDC Serial, used during development and protocol RE: toggle RF diag modes,
// poke radio registers live, switch USB mode, edit tunables, inject test haptics, dump capture history. The
// operational firmware never needs it; it's the human/agent interface to everything under the hood. Present
// only in puck modes (clean controller modes drop CDC). See the parser in serial_console.cpp for the full map.
#pragma once

void serialConsolePoll();   // drain + dispatch CDC commands; call every loop()
