#ifndef __Aton
#define __Aton

#include "base.h"



extern float distScale;
extern float steerScale;
extern float suckSpeed;
extern float spitSpeed;

extern float shiftServoBeg;
extern float shiftServoEnd;

extern float stopDropServoBeg;
extern float stopDropServoEnd;
extern float reverseSkid;
extern float disableGyro;

void atonService(void);

extern int atonSelect;
extern float atonNum;
extern int stationNum;
extern int atonMode;
extern int blockAton;


#define kbMoveRapid  (int)cVal[9]*1000
#define kbTurnRapid  (int)cVal[10]*1000
#define c1c2DistL1L1 (int)cVal[11]
#define c1c2DistL1L2 (int)cVal[12]
#define c1c2DistL1L3 (int)cVal[13]
#define c1c2DistL2L1 (int)cVal[14]
#define c1c2DistL2L2 (int)cVal[15]
#define c1c2DistL2L3 (int)cVal[16]
#define c1c2DistL3L1 (int)cVal[17]
#define c1c2DistL3L2 (int)cVal[18]
#define c1c2DistL3L3 (int)cVal[19]

#define storeU        (int)cVal[20]*10
#define storeW        (int)cVal[21]*10

#define pickupU       (int)cVal[22]*10
#define pickupW       (int)cVal[23]*10

#define carryU        (int)cVal[24]*10
#define carryW        (int)cVal[25]*10

#define ramU          (int)cVal[26]*10
#define ramW          (int)cVal[27]*10

#define hurdleU       (int)cVal[28]*10
#define hurdleW       (int)cVal[29]*10

//void atonSpinRobot(float angle,void(*finishAdr)());
//void atonRotateAndShoot(float angle,void(*finishAdr)());

#endif
