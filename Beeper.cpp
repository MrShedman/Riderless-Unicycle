#include "Beeper.h"
#include "Pins.h"

#define BEEPER_COMMAND_REPEAT 0xFE
#define BEEPER_COMMAND_STOP   0xFF

/* Beeper Sound Sequences: (Square wave generation)
* Sequence must end with 0xFF or 0xFE. 0xFE repeats the sequence from
* start when 0xFF stops the sound when it's completed.
*
* "Sound" Sequences are made so that 1st, 3rd, 5th.. are the delays how
* long the beeper is on and 2nd, 4th, 6th.. are the delays how long beeper
* is off. Delays are in milliseconds/10 (i.e., 5 => 50ms).
*/
// short fast beep
static const uint8_t beep_shortBeep[] = {
	10, 10, BEEPER_COMMAND_STOP
};
// arming beep
static const uint8_t beep_armingBeep[] = {
	30, 5, 5, 5, BEEPER_COMMAND_STOP
};
// armed beep (first pause, then short beep)
static const uint8_t beep_armedBeep[] = {
	0, 245, 10, 5, BEEPER_COMMAND_STOP
};
// disarming beeps
static const uint8_t beep_disarmBeep[] = {
	15, 5, 15, 5, BEEPER_COMMAND_STOP
};
// beeps while stick held in disarm position (after pause)
static const uint8_t beep_disarmRepeatBeep[] = {
	0, 100, 10, BEEPER_COMMAND_STOP
};
// Long beep and pause after that
static const uint8_t beep_lowBatteryBeep[] = {
	25, 50, BEEPER_COMMAND_STOP
};
// critical battery beep
static const uint8_t beep_critBatteryBeep[] = {
	50, 2, BEEPER_COMMAND_STOP
};
// transmitter-signal-lost tone
static const uint8_t beep_txLostBeep[] = {
	50, 50, BEEPER_COMMAND_STOP
};
// SOS morse code:
static const uint8_t beep_sos[] = {
	10, 10, 10, 10, 10, 40, 40, 10, 40, 10, 40, 40, 10, 10, 10, 10, 10, 70, BEEPER_COMMAND_STOP
};
// 2 fast short beeps
static const uint8_t beep_2shortBeeps[] = {
	5, 5, 5, 5, BEEPER_COMMAND_STOP
};
// 2 longer beeps
static const uint8_t beep_2longerBeeps[] = {
	20, 15, 35, 5, BEEPER_COMMAND_STOP
};
// 3 beeps
static const uint8_t beep_gyroCalibrated[] = {
	20, 10, 20, 10, 20, 10, BEEPER_COMMAND_STOP
};

IntervalTimer beeperTimer;

// Beeper off = 0 Beeper on = 1
static uint8_t beeperIsOn = 0;

// Place in current sequence
volatile uint8_t beeperPos = 0;

typedef struct beeperTableEntry_s
{
	uint8_t mode;
	uint8_t priority; // 0 = Highest
	const uint8_t *sequence;
	const char *name;
} 
beeperTableEntry_t;

volatile beeperTableEntry_t beeperTable[] = {
	{ BEEPER_GYRO_CALIBRATED,       0, beep_gyroCalibrated,   "GYRO_CALIBRATED" },
	{ BEEPER_SOS,			        1, beep_sos,              "SOS" },
	{ BEEPER_RX_LOST,               2, beep_txLostBeep,       "RX_LOST" },
	{ BEEPER_DISARMING,             3, beep_disarmBeep,       "DISARMING" },
	{ BEEPER_ARMING,                4, beep_armingBeep,       "ARMING" },
	{ BEEPER_BAT_CRIT_LOW,          5, beep_critBatteryBeep,  "BAT_CRIT_LOW" },
	{ BEEPER_BAT_LOW,               6, beep_lowBatteryBeep,   "BAT_LOW" },
	{ BEEPER_DISARM_REPEAT,         7, beep_disarmRepeatBeep, "DISARM_REPEAT" },
	{ BEEPER_ARMED_REMINDER,        8, beep_armedBeep,        "ARMED" },
	{ BEEPER_SHORT,					9, beep_shortBeep,		  "SHORT" },
	{ BEEPER_2_SHORT,			   10, beep_2shortBeeps,	  "TWO_SHORT" },
	{ BEEPER_2_LONG,			   11, beep_2longerBeeps,	  "TWO_LONG" },
};

volatile beeperTableEntry_t *currentBeeperEntry = nullptr;

void beeperWorker()
{
	if (currentBeeperEntry->sequence[beeperPos] == BEEPER_COMMAND_REPEAT)
	{
		beeperPos = 0;
	}
	else if (currentBeeperEntry->sequence[beeperPos] == BEEPER_COMMAND_STOP) 
	{
		silence();
	}
	else 
	{
		if (beeperPos & 1)
		{
			// odd
			digitalWriteFast(BEEP_PIN, HIGH);
		}
		else
		{ 
			// even
			digitalWriteFast(BEEP_PIN, LOW);
		}

		beeperTimer.begin(beeperWorker, 10000 * currentBeeperEntry->sequence[beeperPos++]);
	}
}

void beeper(beeperMode mode)
{
	if (mode == BEEPER_SILENCE) 
	{
		silence();
		return;
	}

	volatile beeperTableEntry_t* candidate = &beeperTable[mode - 1];

	if (!currentBeeperEntry || candidate->priority < currentBeeperEntry->priority)
	{
		currentBeeperEntry = candidate;

		beeperPos = 0;

		beeperWorker();
	}
}

void silence(beeperMode mode)
{
	bool should_silence = false;

	if (currentBeeperEntry)
	{
		should_silence = (mode == currentBeeperEntry->mode);
	}

	if (mode == BEEPER_SILENCE || should_silence)
	{
		beeperTimer.end();
		digitalWriteFast(BEEP_PIN, LOW);
		beeperIsOn = 0;
		beeperPos = 0;
		currentBeeperEntry = nullptr;
	}
}