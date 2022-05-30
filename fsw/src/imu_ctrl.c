/*
**  Copyright 2022 Open STEMware Foundation
**  All Rights Reserved.
**
**  This program is free software; you can modify and/or redistribute it under
**  the terms of the GNU Affero General Public License as published by the Free
**  Software Foundation; version 3 with attribution addendums as found in the
**  LICENSE.txt
**
**  This program is distributed in the hope that it will be useful, but WITHOUT
**  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
**  FOR A PARTICULAR PURPOSE.  See the GNU Affero General Public License for more
**  details.
**  
**  This program may also be used under the terms of a commercial or enterprise
**  edition license of cFSAT if purchased from the copyright holder.
**
**  Purpose:
**    Implement the Berry IMU control and data processing
**
**  Notes:
**    1. 
**
**  References:
**    1. OpenSatKit Object-based Application Developer's Guide
**    2. cFS Application Developer's Guide
**
*/

/*
** Include Files:
*/

#include <string.h>
#include <math.h>

#include "app_cfg.h"
#include "imu_i2c.h"
#include "imu_ctrl.h"

/***********************/
/** Macro Definitions **/
/***********************/

#define IMU_MOUNT_UP 1  // Up is 'normal' mounting. Set to 0 for upside down mount

#define RAD_TO_DEG 57.29578
#define PI          3.14159265358979323846


/**********************/
/** Global File Data **/
/**********************/

static IMU_CTRL_Class_t*  ImuCtrl = NULL;


/*******************************/
/** Local Function Prototypes **/
/*******************************/


/******************************************************************************
** Function: IMU_CTRL_Constructor
**
** Initialize the IMU Controller object to a known state
**
** Notes:
**   1. This must be called prior to any other function.
**
*/
void IMU_CTRL_Constructor(IMU_CTRL_Class_t *ImuCtrlPtr, INITBL_Class_t* IniTbl)
{
   
   ImuCtrl = ImuCtrlPtr;
   
   memset(ImuCtrl, 0, sizeof(IMU_CTRL_Class_t));

   IMU_I2C_Constructor(&ImuCtrl->ImuI2c, INITBL_GetStrConfig(IniTbl, CFG_IMU_DEVICE_FILE));
   
   ImuCtrl->SensorDeltaTime = INITBL_GetIntConfig(IniTbl, CFG_IMU_SENSOR_DELTA_TIME);
   ImuCtrl->DeltaTime = (float)ImuCtrl->SensorDeltaTime / 1000.0; 

   // TODO - Define in JSON ini file after float support added
   ImuCtrl->AccelerometerScaleFactor    = 0.0573;
   ImuCtrl->GyroScaleFactor             = 0.07;
   ImuCtrl->ComplimentaryFilterConstant = 0.97;

   CFE_MSG_Init(CFE_MSG_PTR(ImuCtrl->RateTlm.TelemetryHeader), CFE_SB_ValueToMsgId(INITBL_GetIntConfig(IniTbl, CFG_IMU_RATE_TLM_MID)), sizeof(BERRY_IMU_RateTlm_t));

} /* End IMU_CTRL_Constructor() */



/******************************************************************************
** Function: IMU_CTRL_ResetStatus
**
** Reset counters and status flags to a known reset state.
**
** Notes:
**   1. Any counter or variable that is reported in HK telemetry that doesn't
**      change the functional behavior should be reset.
**
*/
void IMU_CTRL_ResetStatus(void)
{

   IMU_I2C_ResetStatus();
   
   return;

} /* End IMU_CTRL_ResetStatus() */

 
/******************************************************************************
** Function: IMU_CTRL_SetSensorDeltaTimeCmd
**
** Notes:
**   1. No limits placed on commanded value.
**
*/
bool IMU_CTRL_SetSensorDeltaTimeCmd(void* DataObjPtr, const CFE_MSG_Message_t *MsgPtr)
{
   
   const BERRY_IMU_SetSensorDeltaTime_Payload_t *Cmd = CMDMGR_PAYLOAD_PTR(MsgPtr, BERRY_IMU_SetSensorDeltaTime_t);
  
   CFE_EVS_SendEvent (IMU_CTRL_SET_SENSOR_DELTA_TIME_EID, CFE_EVS_EventType_INFORMATION, 
                      "IMU sensor sampling delta time changed from %u to %u milliseconds", ImuCtrl->SensorDeltaTime, Cmd->SensorDeltaTime);

   ImuCtrl->SensorDeltaTime = Cmd->SensorDeltaTime;
   ImuCtrl->DeltaTime = (float)ImuCtrl->SensorDeltaTime / 1000.0; 
  
   return true;
   
} /* End IMU_CTRL_SetSensorDeltaTimeCmd() */


/******************************************************************************
** Function: IMU_CTRL_SetAccelerometerScaleFactorCmd
**
** Notes:
**   1. No limits placed on commanded value.
**
*/
bool IMU_CTRL_SetAccelerometerScaleFactorCmd(void* DataObjPtr, const CFE_MSG_Message_t *MsgPtr)
{
   
   const BERRY_IMU_SetAccelerometerScaleFactor_Payload_t *Cmd = CMDMGR_PAYLOAD_PTR(MsgPtr, BERRY_IMU_SetAccelerometerScaleFactor_t);
  
   CFE_EVS_SendEvent (IMU_CTRL_SET_ACCELEROMETER_SCALE_FACTOR_EID, CFE_EVS_EventType_INFORMATION, 
                      "IMU accelerometer scale factor changed from %0.6f to %0.6f milliseconds", 
                      ImuCtrl->AccelerometerScaleFactor, Cmd->ScaleFactor);
                      
   ImuCtrl->AccelerometerScaleFactor = Cmd->ScaleFactor;
  
   return true;
   
} /* End IMU_CTRL_SetAccelerometerScaleFactorCmd() */


/******************************************************************************
** Function: IMU_CTRL_SetFilterConstantCmd
**
** Notes:
**   1. No limits placed on commanded value.
**
*/
bool IMU_CTRL_SetFilterConstantCmd(void* DataObjPtr, const CFE_MSG_Message_t *MsgPtr)
{
   
   const BERRY_IMU_SetFilterConstant_Payload_t *Cmd = CMDMGR_PAYLOAD_PTR(MsgPtr, BERRY_IMU_SetFilterConstant_t);
  
   CFE_EVS_SendEvent (IMU_CTRL_SET_FILTER_CONSTANT_EID, CFE_EVS_EventType_INFORMATION,
                     "IMU complimentary filter constant changed from  %0.6f to %0.6f", 
                     ImuCtrl->ComplimentaryFilterConstant, Cmd->ComplimentaryConstant);

   ImuCtrl->ComplimentaryFilterConstant = Cmd->ComplimentaryConstant;  
  
   return true;
   
} /* End IMU_CTRL_SetFilterConstantCmd() */


/******************************************************************************
** Function: IMU_CTRL_SetGyroScaleFactorCmd
**
** Notes:
**   1. No limits placed on commanded value.
**
*/
bool IMU_CTRL_SetGyroScaleFactorCmd(void* DataObjPtr, const CFE_MSG_Message_t *MsgPtr)
{
   
   const BERRY_IMU_SetGyroScaleFactor_Payload_t *Cmd = CMDMGR_PAYLOAD_PTR(MsgPtr, BERRY_IMU_SetGyroScaleFactor_t);
  
   CFE_EVS_SendEvent (IMU_CTRL_SET_GYRO_SCALE_FACTOR_EID, CFE_EVS_EventType_INFORMATION, 
                      "IMU gyro scale factor changed from %0.6f to %0.6f milliseconds", 
                      ImuCtrl->GyroScaleFactor, Cmd->ScaleFactor);
                      
   ImuCtrl->GyroScaleFactor = Cmd->ScaleFactor;
  
   return true;   
   
} /* End IMU_CTRL_SetGyroSCaleFactorCmd() */


/******************************************************************************
** Function: IMU_CTRL_ChildTask
**
** Notes:
**   1. This is a callback from an infinite loop so it needs to have some mechanism
**      to release control so it doesn't hog the CPU
*/
bool IMU_CTRL_ChildTask(CHILDMGR_Class_t* ChildMgr)
{
   
   IMU_I2C_ReadAccelerometer(ImuCtrl->AccelerometerRaw);
   IMU_I2C_ReadGyroscope(ImuCtrl->GyroRaw);

   /* Convert Gyro raw to degrees per second */
   ImuCtrl->GyroRateX = (float) ImuCtrl->GyroRaw[0] * ImuCtrl->GyroScaleFactor;
   ImuCtrl->GyroRateY = (float) ImuCtrl->GyroRaw[1] * ImuCtrl->GyroScaleFactor;
   ImuCtrl->GyroRateZ = (float) ImuCtrl->GyroRaw[2] * ImuCtrl->GyroScaleFactor;

   /* Calculate the angles from the gyro */
   ImuCtrl->GyroAngleX += ImuCtrl->GyroRateX * ImuCtrl->DeltaTime;
   ImuCtrl->GyroAngleY += ImuCtrl->GyroRateY * ImuCtrl->DeltaTime;
   ImuCtrl->GyroAngleZ += ImuCtrl->GyroRateZ * ImuCtrl->DeltaTime;

   /* TODO - Very algorithm. The raw counts have not been scaled prior to use below */
   /* Convert Accelerometer values to degrees */
   ImuCtrl->AccelerometerAngleX = (float) (atan2(ImuCtrl->AccelerometerRaw[1], ImuCtrl->AccelerometerRaw[2]) + PI) * RAD_TO_DEG;
   ImuCtrl->AccelerometerAngleY = (float) (atan2(ImuCtrl->AccelerometerRaw[2], ImuCtrl->AccelerometerRaw[0]) + PI) * RAD_TO_DEG;

   /* Account for potential different mountings */
   #if IMU_MOUNT_UP == 1
   
      /* IMU is mounted up the correct way */
      ImuCtrl->AccelerometerAngleX -= (float)180.0;
      if (ImuCtrl->AccelerometerAngleY > 90.0)
         ImuCtrl->AccelerometerAngleY -= (float)270.0;
      else
         ImuCtrl->AccelerometerAngleY += (float)90.0;
   
   #else
	
      /* IMU is mounted upside down */
      /* Change the rotation value of the accelerometer to -/+ 180 and move the Y axis '0' point to up  */
      
      if (ImuCtrl->AccelerometerAngleX > 180)
         ImuCtrl->AccelerometerAngleX -= (float)360.0;

      ImuCtrl->AccelerometerAngleY -= 90.0;
      if (ImuCtrl->AccelerometerAngleY > 180.0)
         ImuCtrl->AccelerometerAngleY -= (float)360.0;

   #endif

   /* Complementary filter used to combine the accelerometer and gyro values */
   ImuCtrl->FilterAngleX = ImuCtrl->ComplimentaryFilterConstant * (ImuCtrl->FilterAngleX + ImuCtrl->GyroRateX * ImuCtrl->DeltaTime) + 
                           (1.0 - ImuCtrl->ComplimentaryFilterConstant) * ImuCtrl->AccelerometerAngleX;
                           
   ImuCtrl->FilterAngleY = ImuCtrl->ComplimentaryFilterConstant * (ImuCtrl->FilterAngleY + ImuCtrl->GyroRateY * ImuCtrl->DeltaTime) + 
                           (1.0 - ImuCtrl->ComplimentaryFilterConstant) * ImuCtrl->AccelerometerAngleY;

   /* Load and send rate telemetry */
   
   ImuCtrl->RateTlm.Payload.RateX = ImuCtrl->GyroRateX;
   ImuCtrl->RateTlm.Payload.RateY = ImuCtrl->GyroRateY;
   ImuCtrl->RateTlm.Payload.RateZ = ImuCtrl->GyroRateZ;
   
   CFE_SB_TimeStampMsg(CFE_MSG_PTR(ImuCtrl->RateTlm.TelemetryHeader));
   CFE_SB_TransmitMsg(CFE_MSG_PTR(ImuCtrl->RateTlm.TelemetryHeader), true);

   OS_TaskDelay(ImuCtrl->SensorDeltaTime);
   
   return true;

} /* End IMU_CTRL_ChildTask() */

