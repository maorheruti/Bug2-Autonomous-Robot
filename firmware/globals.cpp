#include "globals.h"
#include "DbgLog.h"

volatile bool     gStarted      = false;
volatile Pose     gRobotLatest  = {0,0,0,0};
volatile Pose     gTargetLatest = {0,0,0,0};
volatile uint32_t gLastPoseMs   = 0;


// === NEW: latest sensor readings (mm) copied from Sensors.cpp ===
volatile uint32_t gSensCanaryA = 0xA5A55A5A;
volatile uint16_t gSensL_mm = 0;
volatile uint16_t gSensM_mm = 0;
volatile uint16_t gSensR_mm = 0;
volatile uint32_t gSensCanaryB = 0x5A5AA5A5;

volatile SensorMatrixData gSensL_matrix = {};
volatile SensorMatrixData gSensM_matrix = {};
volatile SensorMatrixData gSensR_matrix = {};

volatile SensorZone4Data gSensL_z4 = {{8191,8191,8191,8191}, {0,0,0,0}};
volatile SensorZone4Data gSensM_z4 = {{8191,8191,8191,8191}, {0,0,0,0}};
volatile SensorZone4Data gSensR_z4 = {{8191,8191,8191,8191}, {0,0,0,0}};

volatile uint32_t gSensFrameId = 0;
volatile uint32_t gSensCorruptCount = 0;
volatile uint32_t gSensCorruptLastMs = 0;
volatile uint32_t gSensCanaryTripCount = 0;
volatile uint32_t gSensCanaryLastMs = 0;
volatile uint32_t gSensCrcTripCount = 0;
volatile uint32_t gSensCrcLastMs = 0;

volatile uint32_t gSensCorruptSrc = 0;
volatile uint32_t gSensCorruptLastFrame = 0;
volatile uint16_t gSensCorruptLastRawL = 0;
volatile uint16_t gSensCorruptLastRawM = 0;
volatile uint16_t gSensCorruptLastRawR = 0;

volatile uint32_t gLoopCanaryA = 0xC0FFEEA5;
volatile uint64_t gLoopBeatUs = 0;
volatile uint32_t gLoopStallCount = 0;
volatile uint32_t gLoopStallLastMs = 0;
volatile uint32_t gLoopCanaryB = 0x5AA55AA5;
volatile uint32_t gLoopCanaryTripCount = 0;
volatile uint32_t gLoopCanaryLastMs = 0;

volatile uint32_t gStopCauseTag = 0;
volatile uint32_t gLastStopMs = 0;
volatile uint32_t gLastStopBug2 = 0;
volatile uint32_t gLastStopReasonHash = 0;
char gLastStopReason[24] = {0};
volatile bool gEverStarted = false;

volatile uint32_t gLatchedStopTag = 0;
volatile uint32_t gLatchedStopMs = 0;
volatile uint32_t gLatchedStopBug2 = 0;
volatile uint32_t gLatchedStopReasonHash = 0;
char gLatchedStopReason[24] = {0};

void record_stop_cause(uint32_t tag, uint32_t b2, uint32_t reasonHash, const char* reason){
	noInterrupts();
	gStopCauseTag = tag;
	gLastStopMs = millis();
	gLastStopBug2 = b2;
	gLastStopReasonHash = reasonHash;
	strncpy(gLastStopReason, reason ? reason : "", sizeof(gLastStopReason) - 1);
	gLastStopReason[sizeof(gLastStopReason) - 1] = '\0';
	if (gLatchedStopTag == 0){
		gLatchedStopTag = gStopCauseTag;
		gLatchedStopMs = gLastStopMs;
		gLatchedStopBug2 = gLastStopBug2;
		gLatchedStopReasonHash = gLastStopReasonHash;
		strncpy(gLatchedStopReason, gLastStopReason, sizeof(gLatchedStopReason) - 1);
		gLatchedStopReason[sizeof(gLatchedStopReason) - 1] = '\0';
	}
	interrupts();

	dbg_push(EV_MARK, 510, (int32_t)tag, (int32_t)reasonHash);

}

void reset_stop_cause(){
	noInterrupts();
	gStopCauseTag = 0;
	gLastStopMs = 0;
	gLastStopBug2 = 0;
	gLastStopReasonHash = 0;
	gLastStopReason[0] = '\0';
	gLatchedStopTag = 0;
	gLatchedStopMs = 0;
	gLatchedStopBug2 = 0;
	gLatchedStopReasonHash = 0;
	gLatchedStopReason[0] = '\0';
	interrupts();
}


