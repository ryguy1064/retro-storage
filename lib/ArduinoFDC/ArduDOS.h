#pragma once

// comment this out to remove high-level ArduDOS functions
#define USE_ARDUDOS

// commenting this out will remove the low-level disk monitor
#define USE_MONITOR

// comenting this out will remove support for XModem data transfers
#define USE_XMODEM

void arduDOS();
