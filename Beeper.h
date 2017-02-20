#pragma once

#include "Arduino.h"

typedef enum {
	// IMPORTANT: these are in priority order, 0 = Highest
	BEEPER_SILENCE = 0,             // Silence, see beeperSilence()

	BEEPER_GYRO_CALIBRATED,
	BEEPER_SOS,						// Beeps SOS when lost aircraft
	BEEPER_RX_LOST,                 // Beeps when TX is turned off or signal lost (repeat until TX is okay)
	BEEPER_DISARMING,               // Beep when disarming the board
	BEEPER_ARMING,                  // Beep when arming the board
	BEEPER_BAT_CRIT_LOW,            // Longer warning beeps when battery is critically low (repeats)
	BEEPER_BAT_LOW,                 // Warning beeps when battery is getting low (repeats)
	BEEPER_DISARM_REPEAT,           // Beeps sounded while stick held in disarm position
	BEEPER_ARMED_REMINDER,          // Warning beeps when board is armed (repeats until board is disarmed or throttle is increased)

	BEEPER_SHORT,
	BEEPER_2_SHORT,
	BEEPER_2_LONG
} 
beeperMode;

void silence(beeperMode mode = BEEPER_SILENCE);
void beeper(beeperMode mode);