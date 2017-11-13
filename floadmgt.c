/*****************************************************************************

File Name        :  floadmgt.c
Module Short Name:  LoadMgt
VOBName          :  2012.5_FORD_B299MCA_IC
Author           :  sdinesh
Description      :  This module is responsible for balancing the available power
                    against climatic loads.The cluster will monitor available power
                    and current demands on the engine to determine which loads can
                    be activated/deactivated.
Organization     :  Driver Information Software Section,
                    xxxxx
Compiler Name    :  COSMIC V4.7.7
Target Processor :  MC9S12XEQ384
******************************************************************************/

#define FLOADMGT_C
/*****************************************************************************
*                                 System Includes                            *
******************************************************************************/

#include "system.h"
#include "swtmr.h"
#include "sched.h"

/*****************************************************************************
*                                 Project Includes                           *
******************************************************************************/

#include "floadmgt.h"
#include "nvmdef.h"
#include "piodef.h"
#include "rcan.h"
#include "fstmgr.h"
#include "fstmgr_ps.h"
#include "fcfg.h"
#include "foat.h"
#include "fptc.h"
#include "fpwrmode.h"
#include "linebyte.h"
#include "idd01.h"
#include "lineword.h"
#include "fdiag.h"
#include "ffreeze_mode.h"

/*****************************************************************************
*                                 Type Declarations                          *
******************************************************************************/


/*****************************************************************************
*                                 File Scope Prototypes                      *
******************************************************************************/

static void FLOADMGT_ElectricalLoadControl_signal_priority1(void);
static void FLOADMGT_Load_Management_Strategy_priority3(void);
static void FLOADMGT_LowTemp_Electrical_Load_Strategy_priority2(void);
static void FLOADMGT_priority2_status_function(void);
static void FLOADMGT_calculate_AvailableDeltaPower_value(void);
static void FLOADMGT_Current_calculation_function(void);
static void FLOADMGT_Deactivate_All_Loads_TFastSwitchOff(void);
static void FLOADMGT_Set_HRS_HFS_Final_Op(void);
static UINT16 FLOADMGT_validate_NVM_TSWITCHOFF(void);
/*****************************************************************************
*                                 Constants                              *
*----------------------------------------------------------------------------*
* Declaration shall be followed by a comment that gives the following info.  *
* about the constant.                                                        *
* purpose, unit and resolution                                               *
******************************************************************************/


/*****************************************************************************
*                                 Manifest Constants                         *
*----------------------------------------------------------------------------*
* Definition of Manifest constant shall be followed by a comment that        *
* explains the purpose of the constant.                                      *
******************************************************************************/

/*
         purpose : Time out period for the timer used for HFS enable status
 critical section: None
            Unit : None
       Resoultion: None
*/

#define  FLOADMGT_HFS_4MIN_TIMEOUT  ((UINT16)469)

/*
         purpose : Time out period for the timer used for HRS enable status
 critical section: None
            Unit : None
       Resoultion: None
*/

#define  FLOADMGT_HRS_13MIN_TIMEOUT  ((UINT16)1523)

/*
         purpose : Time out period for the timer used for ErrorHandlingDebounceTimer
 critical section: None
            Unit : None
       Resoultion: None
*/

#define  FLOADMGT_ERRORHANDLINGDEBOUNCETIMER_TIMEOUT  (NVM_ErrorHandlingDebounceTimer_U8 * 2)


/*****************************************************************************
*                                 Macro Definitions                          *
*----------------------------------------------------------------------------*
* Definition of macro shall be followed by a comment that explains the       *
* purpose of the macro.                                                      *
******************************************************************************/
#define NO_LOAD_MANAGEMENT               ((UINT8)0)
#define ACTIVATE                         ((UINT8)1)
#define DEACTIVATE                       ((UINT8)2)
#define ACTIVE                            ((UINT8)0x01)

/* Deg celsius is converted to RAW value as per CAN signal
    ex: 5 deg = (5 + 128) / 0.25 = 532  */
#define LOW_TEMP_ACTIVATE_OAT_VAL        ((UINT16)532)
#define ECM_AMBTEMP_MISS_INVALD_DEFAULT  ((UINT16)592)
/* Offset 40 added to match CAN signal */
#define LOW_TEMP_ACTIVATE_ECT_VAL        ((UINT8)100)
#define OAT_OFFSET_VALUE                 ((UINT16)272)
/* Battery Voltage Invalid */
#define BATTERY_VOLTAGE_INVLD            ((UINT8)255)
/* if BV invalid then BV = 13.5V */
#define BATVOL_MISS_INVLD_DEF_VAL        ((UINT8)135)
#define BATVOLT_DISLIMIT_DEF_VAL         ((UINT8)126)
#define BATVOLT_DISLIMIT_MIN_VAL         ((UINT8)90)
#define BATVOLT_DISLIMIT_MAX_VAL         ((UINT8)160)

/* Load Management configuration value's MACRO's. */

/* 00 Full function Load Management active, on heated screens and PTC */
#define FULL_LOAD_MANAGEMENT             ((UINT8)0)

/* 01 Load Management on, PTC ON only */
#define LOAD_MANAGEMENT_ON_PTC_ONLY      ((UINT8)1)

/* ELECTRICAL LOAD CONTROL SIGNAL MACRO's. */

/* 00 Default State - Normal Load Management behaviour, with
   only customer requested loads active. */
#define ELC_DEFAULT_STATE                ((UINT8)0)

/* 01 Turn OFF HFS,HRS and PTC */
#define ELC_TURN_OFF_HFS_HRS_PTC         ((UINT8)1)

/* 02 Turn ON HFS and HRS */
#define ELC_TURN_ON_HFS_HRS              ((UINT8)2)

/* 03 Turn ON HRS */
#define ELC_TURN_ON_HRS                  ((UINT8)3)

/* Maximum Value of Load Management */
#define LOAD_MANAGEMENT_SIGNAL_MAX_VALUE ((UINT32)255)

/* QF Value checked for ECMEstimatedAmbTempQF */
#define PTC_HYST_DOWN_OFFSET             ((UINT8)1)
#define CONVERT_MIN_TO_SEC               ((UINT16)60)   
#define LM_STRAT_THREE_PERIODIC_TIMER    ((UINT16)250)
#define ADP_REQ_TO_ACTIVATE_HRS          ((UINT16)((NVM_PTC_Hyst1_Dn_U8 + PTC_HYST_DOWN_OFFSET) + NVM_HRS_Power_Rating_U8))
#define ADP_REQ_TO_ACTIVATE_HFS          ((UINT16)((NVM_PTC_Hyst1_Dn_U8 + PTC_HYST_DOWN_OFFSET) +(((UINT16)NVM_HFS_Power_Rating_U8) * ((UINT16)5))))
#define ADP_RESOLUTION_FACTOR            ((SINT32)1000)
/* TswitchOnClimateLoads Time */
#define TSWITCHONCLIMATELOADS_DEF_VAL    ((UINT16)750)
#define NVM_FACTORY_LM_DEACT_DEF_VAL     ((UINT8)20)
#define NVM_FACTORY_LM_DEACT_MAX_VAL     ((UINT8)60)
/* TswitchOnClimateLoads Default Time */
#define TSWITCHOFFCLIMATELOADS_DEF_VAL      ((UINT16)750)
#define TSWITCHOFFCLIMATELOADS_MAX_VAL      ((UINT8)50)
#define TFASTSWITCHOFFCLIMATELOADS_DEF_VAL  ((UINT8)75)
#define TFASTSWITCHOFFCLIMATELOADS_MAX_VAL  ((UINT8)20)

/* ADP value Limit */
#define ADP_MAX_LIMIT                    ((UINT16)2000)
#define ADP_MIN_LIMIT                    ((SINT16)-1000)
#define ADP_DEFAULT_VALUE                ((SINT16)-100)

/* Macros for ADP conversion to 8 bit resolution  */

#define ADP_MINIMUM_VALUE                ((UINT32)1000)
#define ADP_CAN_TRMT_FACTOR              ((UINT32)1176)
#define ADP_FACTOR_CONVERT_BY_100        ((UINT32)100)


/* Init value */
#define INIT_VALUE                       ((UINT16)0)

/* DEFROSSETER Timer default value */
#define DEFROSTER_DEFAULT_VALUE          ((UINT8)6)

/* Tswitchoff resolution = 0.1 - convert into sec, then ms and 
divide by 4 for 4 ms timer base count - (1000*10)/4*/
#define TSWITCHOFF_TIMER_COUNT_MULFACTOR    ((UINT16)25)
/* DEFROSSETER Timer init value */
#define DEFROSTER_INIT_VALUE          ((UINT8)0)
/* DEFROSSETER Timer init value */
#define LOAD_CURRENT_DEFAULT_VALUE    ((UINT8)0)

#define LOAD_CURRENT_MESSAGE_RECEIVED   ((UINT8)0)
#define LOAD_CURRENT_MUL_100            ((UINT8)100)
#define LOAD_CURRENT_MUL_10             ((UINT8)10)
#define LOAD_CURRENT_ROUNDOFF           ((UINT8)50)
#define LOAD_CURRENT_ADD_ONE            ((UINT8)100)


#define MAX_BATT_VOLTAGE                ((UINT8)0xFF)
#define TIMER_RESET_TOZERO              ((UINT16)0)


#define LOAD_DEACTIVATEDBY_LM           ((UINT8)1)
#define NO_LOAD_DEACTIVATEDBY_LM        ((UINT8)0)
#define ELC_CFG_BIT_CHK                 ((UINT8)0x01)
#define TFASTSWITCHOFF_RES   ((UINT8)50)
#define FOUR_MS_TIMER_BASE   ((UINT8)4)
/*****************************************************************************
*                                 Globally  accessed Variable Declarations   *
*----------------------------------------------------------------------------*
* Declaration shall be followed by a comment that gives the following info.  *
* about the variable.                                                        *
* purpose, critical section, unit, resolution, Valid Range and ValidityCheck *
******************************************************************************/

/*****************************************************************************
*                                 Locally used Variable Declarations         *
*----------------------------------------------------------------------------*
* Declaration shall be followed by a comment that gives the following info.  *
* about the variable.                                                        *
* purpose, critical section, unit, resolution, Valid Range and ValidityCheck *
******************************************************************************/
/*
purpose:    Used for holding Available Delta Powers value.
unit:       Watts
resolution: 1
critical section : None
*/
static SINT16 l_AvailableDeltaPower_S16;

/*
Purpose: This Variable have the status of Heated Front Screen loads status
         - Active/Inactive.
unit: None.
resolution:1
critical section: None
*/
static UINT8 l_LM_HFSReq_U8;

/*
Purpose: This Variable have the status of Heated Rear Screen loads status
         - Active/Inactive.
unit: None.
resolution:1
critical section: None
*/
static UINT8 l_LM_HRSReq_U8;


/*
Purpose: This Variable is used in Load Management Startegy to decide
         status of HFS.
unit: None.
resolution:1
critical section: None
*/
static UINT16 l_DischargeTimer_U16;

/*
Purpose: This Variable is used in Load Management Startegy to decide
         status of HFS.
unit: None.
resolution:1
critical section: None
*/
static UINT16 l_DischargeResetTimer_U16;

/*
Purpose: This Variable is used for RCAN Values.
unit: None.
resolution:1
critical section: None
*/
static UINT8 l_ElectricLoadControl_U8;

/*
Purpose: This Variable is used for holding the status of HFS activated status in
         Low temperature Electrical Load strategy.
unit: None.
resolution:1
critical section: None
*/
static UINT8 l_HFS_Activated_status_U8;

/*
Purpose: This Variable is used for holding the status of HFS activated status in
         Low temperature Electrical Load strategy.
unit: None.
resolution:1
critical section: None
*/
static UINT8 l_HRS_Activated_status_U8;


/*
Purpose: This Variable is used for holding the status of HFS Deactivated status in
         Load Management Strategy.
unit: None.
resolution:1
critical section: None
*/

static UINT8 l_HFS_Deactivated_status_U8;

/*
purpose:    Used for holding Error handling Timer status, wheather the timer is already started or not
unit:       Watts
resolution: 1
critical section : None
*/
static BOOLEAN l_err_handling_tmr_status_BOOL;
/*
purpose:    Used for holding TSwitchOnClimateLoads Timer used when 2 or more loads
            are to be activated.
unit:       Watts
resolution: 1
critical section : None
*/
static UINT16 l_TSwitchOnClimateLoads_U16;
/*
purpose:    Used to store the OFF state status for Prioriy to Engine Start condition.
unit:       None
resolution: 1
critical section : None
*/
static UINT8 l_priority2_OFF_status_U8;
/*
purpose:    Used to start 13 min timer once to activate HFS for Priority 2 Engine Start condition.
unit:       None
resolution: 1
critical section : None
*/
static UINT8 l_priority2_HFS_status_U8;

/*
Purpose: This Variable is used for holding the status of HRS Deactivated status in
         Load Management Strategy.
unit: None.
resolution:1
critical section: None
*/

static UINT8 l_HRS_Deactivated_status_U8;

/*
Purpose: This Variable is used defroster delay timer 
unit: None.
resolution:1
critical section: None
*/
static UINT8 l_defroster_timer_U8;
/*
** Purpose: Transmit total current value for Rear Defroster,HFS,Heated Mirror and PTC
** critical section: None
** unit: 8 bit
*/

static GLOBAL_RAM_PREFIX UINT8 l_LoadCtrlCurrAct_U8;

/*
** Purpose: Value transmitted in CAN which tells whether PTC/HFS/HRS loads are
            deactivated by normal load management strategy
** critical section: None
** unit: None
*/

static GLOBAL_RAM_PREFIX UINT8 l_LoadCutOff_U8;

/*
Purpose: This Variable is used to know whether factory mode ignition On timer was started
         or not. True - If timer is started in current ignition On cycle else false.
unit: None.
resolution:None
critical section: None
*/
static BOOLEAN  l_FactoryMode_IgnitionON_timer_sts_Bool;

/*
** Purpose: Look-up table for Rear Defroster Battey Vs Current
** critical section: None
** unit: 8 bit
*/

const static UINT8 l_Rear_Defroster_Battery_current_Table_U8[4][2]=
                                        {
                                          (UINT8) 100, (UINT8)0xA0 ,    /* Min & Max Battery Voltage value */
                                          (UINT8) 100, (UINT8)  90,    /* 10 Volt*/
                                          (UINT8) 135, (UINT8) 100,    /* 13.5 Volt*/
                                          (UINT8) 160, (UINT8) 120,    /* 16.0 Volt*/
                                        };

/*
** Purpose: Look-up table for HFS Battey Vs Current
** critical section: None
** unit: 16 bit
*/
const static  UINT16 l_HFS_Battery_current_Table_U16[4][2]=
                                        {
                                          (UINT16) 900  , (UINT16) 1600,    /* Min & Max Battery Voltage value */
                                          (UINT16) 900  , (UINT16) 2300,    /* 9 Volt*/
                                          (UINT16)1350  , (UINT16) 3400,    /* 13.5 Volt*/
                                          (UINT16)1600  , (UINT16) 4000,    /* 16.0 Volt*/
                                        };

/*
** Purpose: Look-up table for Heated Mirror Battey Vs Current
** critical section: None
** unit: 8 bit
*/
const static  UINT8 l_Heated_Mirror_Battery_current_Table_U8[4][2]=
                                        {
                                          (UINT8) 90  , (UINT8) 160,    /* Min & Max Battery Voltage value */
                                          (UINT8) 90  , (UINT8)   0,    /* 9 Volt*/
                                          (UINT8)135  , (UINT8)  15,    /* 13.5 Volt*/
                                          (UINT8)160  , (UINT8)  40,    /* 16.0 Volt*/
                                        };

/*
** Purpose: Look-up table for PTC Battery Vs Current
** critical section: None
** unit: 8 bit
*/
const static  UINT8 l_PTC_Battery_current_A_Table_U8[4][2]=
                                        {
                                          (UINT8) 90  , (UINT8) 160,    /* Min & Max Battery Voltage value */
                                          (UINT8) 90  , (UINT8) 10,     /* 9 Volt*/
                                          (UINT8)135  , (UINT8) 16,     /* 13.5 Volt*/
                                          (UINT8)160  , (UINT8) 19,     /* 16.0 Volt*/
                                        };

const static  UINT8 l_PTC_Battery_current_B_Table_U8[4][2]=
                                        {
                                          (UINT8) 90  , (UINT8) 160,    /* Min & Max Battery Voltage value */
                                          (UINT8) 90  , (UINT8) 25,     /* 9 Volt*/
                                          (UINT8)135  , (UINT8) 34,     /* 13.5 Volt*/
                                          (UINT8)160  , (UINT8) 40,     /* 16.0 Volt*/
                                        };

/*****************************************************************************
*                                 Functions                                  *
******************************************************************************/

/****************************************************************************
Function Name     : FLOADMGT_KSColdInit

Description       : Called after battery connect(if RAM corrupted)
                    to initialise all variables.

Invocation        : Called from the Cold init list of Scheduler.

Parameters        : None

Return Value      : None

Critical Section  : None

******************************************************************************/
void FLOADMGT_KSColdInit(void)
{
	FLOADMGT_KSWakeup();
}
/****************************************************************************
Function Name     : FLOADMGT_KSWarmInit

Description       : Called after battery connect(if RAM not corrupted)
                    to initialise all variables.

Invocation        : Called from the Warm init list of Scheduler.

Parameters        : None

Return Value      : None

Critical Section  : None

******************************************************************************/
void FLOADMGT_KSWarmInit(void)
{
	FLOADMGT_KSWakeup();
}
/****************************************************************************
Function Name     : FLOADMGT_KSWakeup

Description       : Called after wakeup whenever cluster wakes up from cluster
                    sleep.

Invocation        : Invoked by Scheduler during Wakeup.

Parameters        : None

Return Value      : None

Critical Section  : None

******************************************************************************/
void FLOADMGT_KSWakeup(void)
{
	/* Initialise all variables */
	/*Counters in priority3*/
	l_DischargeTimer_U16 = FALSE;
	l_DischargeResetTimer_U16 = FALSE;
	l_defroster_timer_U8 =  DEFROSTER_INIT_VALUE;
	/*ADP + ADP fault handling variable*/
	l_AvailableDeltaPower_S16 = ADP_DEFAULT_VALUE;
	l_err_handling_tmr_status_BOOL = FALSE;
	/*ELC and Load current and Load cutoff signal*/
	l_ElectricLoadControl_U8 = FALSE;
	l_LoadCtrlCurrAct_U8 =  LOAD_CURRENT_DEFAULT_VALUE;
	l_LoadCutOff_U8 = FALSE;
	/*HRS and HFS outputs+status variables*/
	l_LM_HFSReq_U8 = DEACTIVATE;
	l_LM_HRSReq_U8 = DEACTIVATE;
	l_HFS_Deactivated_status_U8 = TRUE;
	l_HRS_Deactivated_status_U8 = TRUE;
	l_HFS_Activated_status_U8 = FALSE;
	l_HRS_Activated_status_U8 = FALSE;
	/*Priority 2 flags to start the timers once*/
	l_priority2_OFF_status_U8 = TRUE;
	l_priority2_HFS_status_U8 = FALSE;
	/*Factory Mode Ignition on timer start status*/
	l_FactoryMode_IgnitionON_timer_sts_Bool = FALSE;
	/*Clear all timers*/
	KernelClear13BitTimer(FLOADMGT_priority3_KSWtimer);
	KernelClear13BitTimer(FLOADMGT_TSwitchOnClimateLoads_KSWtimer);
	KernelClear13BitTimer(FLOADMGT_TSwitchOffClimateLoads_KSWtimer);
	KernelClear13BitTimer(FLOADMGT_TFastSwitchOffClimateLoads_KSWtimer);
	KernelClear13BitTimer(FLOADMGT_HRS_13minutes_KSWtimer);
	KernelClear13BitTimer(FLOADMGT_HFS_4minutes_KSWtimer);
	KernelClear13BitTimer(FLOADMGT_Error_Debounce_KSWtimer);
}

/****************************************************************************
Function Name     : FLOADMGT_KSRRobin

Description       : Process the load management Strategy's and activate the load 

Invocation        : Invoked by Scheduler during normal operation.

Parameters        : None.

Return Type       : None.

Critical Section  : None.

******************************************************************************/
void FLOADMGT_KSRRobin(void)
{
	UINT8 fl_Power_Mode_U8;
	UINT8 fl_get_carmode_U8;
	UINT8 fl_FactoryMode_IgnitionOn_Timer_sts_U8;
	UINT16 fl_FactoryMode_IgnitionOn_Timer_val_U16;

	/* Get the TswitchonClimate loads in Time, if two or more loads are to be switched ON, there 
	** should be a TswitchonClimate delay between activate two loads, default is 3Sec
	*/
	if(NVM_TSwitchOnClimateLoads_U8 > 50)
	{
		/* Default value 3 sec multiplied by 250 */
		l_TSwitchOnClimateLoads_U16 = TSWITCHONCLIMATELOADS_DEF_VAL;
	}
	else
	{
		l_TSwitchOnClimateLoads_U16 = (UINT16)( ((UINT16)NVM_TSwitchOnClimateLoads_U8) * ((UINT16)25) );
	}

	/* Check the ElectricLoadControl signal is missing or never received */
	if((UINT8)0x00 == ((RCAN_SIG_MISSING | RCAN_SIG_NVR_RCVD) & \
		rcan_get_ElectricalLoadControl_status()))
	{
		l_ElectricLoadControl_U8 = rcan_get_ElectricalLoadControl_value();
	}
	else
	{
		l_ElectricLoadControl_U8 = ((UINT8)0);
	}

	/* This interface gives power mode with extended values for start/stop configuration*/
	fl_Power_Mode_U8 = FPM_get_Start_Stop_volvo_State_U8();
	fl_get_carmode_U8 = fsm_ps_get_CarMode_U8();
	/* To avoid timer over flow , other timers are periodically checked*/
	(void)KernelCheck13BitTimer(FLOADMGT_TFastSwitchOffClimateLoads_KSWtimer);

	if((CM_FACTORY == fl_get_carmode_U8)&&(SS_IGN_ON2 == fl_Power_Mode_U8))
	{
		if(FALSE == l_FactoryMode_IgnitionON_timer_sts_Bool)
		{
			if(NVM_T_FactoryLMDeact_U8 > NVM_FACTORY_LM_DEACT_MAX_VAL)
			{
				/* Default value 10 sec */
				fl_FactoryMode_IgnitionOn_Timer_val_U16 = NVM_FACTORY_LM_DEACT_DEF_VAL;
			}
			else
			{
				fl_FactoryMode_IgnitionOn_Timer_val_U16 = (UINT16)((((UINT16)NVM_T_FactoryLMDeact_U8) * ((UINT16)1000))/(UINT16)512);
			}
			KernelStart13BitTimer(TIMER_13BIT_BASE_512MS,fl_FactoryMode_IgnitionOn_Timer_val_U16,\
				FLOADMGT_FactoryMode_IgnitionOn_KSWTimer);

			l_FactoryMode_IgnitionON_timer_sts_Bool = TRUE;
		}
	}
	else
	{
		KernelClear13BitTimer(FLOADMGT_FactoryMode_IgnitionOn_KSWTimer);
		l_FactoryMode_IgnitionON_timer_sts_Bool = FALSE;
	}

	/*If power mode = Running for both Stop start and Non stop start vehicle*/
	if(SS_RUNNING_RUNNING == fl_Power_Mode_U8)
	{
		/*Calculate load current */
		FLOADMGT_Current_calculation_function();
		/*Calculate ADP*/
		FLOADMGT_calculate_AvailableDeltaPower_value();

		if(TIMER_RUNNING != KernelCheck13BitTimer(FLOADMGT_priority3_KSWtimer))
		{
			FLOADMGT_Load_Management_Strategy_priority3();

			KernelStart13BitTimer(TIMER_13BIT_BASE_4MS,LM_STRAT_THREE_PERIODIC_TIMER,\
				FLOADMGT_priority3_KSWtimer);
		}
		/* The low temperature electrical load management Strategy is processed 
		** only when get the status from OAT module after the first sampling is completed 
		*/
		if( (FALSE != foat_get_Startup_flag_status_BOOL()) && (FALSE != l_priority2_OFF_status_U8) )
		{
			FLOADMGT_LowTemp_Electrical_Load_Strategy_priority2();
			/* Clear the variable once if Priority function is called */
			l_priority2_OFF_status_U8 = FALSE;
		}
		else
		{
			FLOADMGT_priority2_status_function();
		}
		FLOADMGT_ElectricalLoadControl_signal_priority1();
	}
	else
	{
		/* If not equal to Running_2 then set ADP value is -100 W */
		l_AvailableDeltaPower_S16 = ADP_DEFAULT_VALUE;
		l_err_handling_tmr_status_BOOL = FALSE;
		/*SS Defrost delay timer te restart when power mode changes to running*/
		l_defroster_timer_U8 =DEFROSTER_INIT_VALUE;
		l_LoadCutOff_U8 = FALSE;
		l_LoadCtrlCurrAct_U8 =LOAD_CURRENT_DEFAULT_VALUE;
		/*Disable priority 2 functionality*/
		l_priority2_HFS_status_U8 = FALSE;

		KernelClear13BitTimer(FLOADMGT_priority3_KSWtimer);
		KernelClear13BitTimer(FLOADMGT_TSwitchOnClimateLoads_KSWtimer);
		KernelClear13BitTimer(FLOADMGT_TSwitchOffClimateLoads_KSWtimer);
		KernelClear13BitTimer(FLOADMGT_HRS_13minutes_KSWtimer);
		KernelClear13BitTimer(FLOADMGT_HFS_4minutes_KSWtimer);
		KernelClear13BitTimer(FLOADMGT_Error_Debounce_KSWtimer);

		/*If Factory mode timer is running send HRS and HFS = No load management*/
		fl_FactoryMode_IgnitionOn_Timer_sts_U8 = KernelCheck13BitTimer(FLOADMGT_FactoryMode_IgnitionOn_KSWTimer);
		if( TIMER_RUNNING == fl_FactoryMode_IgnitionOn_Timer_sts_U8)
		{
			l_LM_HFSReq_U8 = NO_LOAD_MANAGEMENT;
			l_LM_HRSReq_U8 = NO_LOAD_MANAGEMENT;
			l_HFS_Deactivated_status_U8 = FALSE;
			l_HRS_Deactivated_status_U8 = FALSE;
		}
		/*Deactivate HRS and HFS when power mode != Running*/
		else
		{
			FLOADMGT_Deactivate_All_Loads_TFastSwitchOff();
			if(FALSE != l_HFS_Deactivated_status_U8)
			{
				l_LM_HFSReq_U8 = DEACTIVATE;
			}
			if(FALSE != l_HRS_Deactivated_status_U8)
			{
				l_LM_HRSReq_U8 = DEACTIVATE;
			}
		}
	}
}
/****************************************************************************
Function Name     : FLOADMGT_Transout_NORMAL

Description       : Intialize the variables

Invocation        : Invoked by Scheduler during normal operation.

Parameters        : None.

Return Type       : None.

Critical Section  : None.

******************************************************************************/
void FLOADMGT_Transout_NORMAL(void)
{
	/* Initialise all variables */
	l_DischargeTimer_U16 = FALSE;
	l_DischargeResetTimer_U16 = FALSE;
	l_defroster_timer_U8 =  DEFROSTER_INIT_VALUE;
	/*ADP + ADP fault handling variable*/
	l_AvailableDeltaPower_S16 = ADP_DEFAULT_VALUE;
	l_err_handling_tmr_status_BOOL = FALSE;
	/*ELC and Load current and Load cutoff signal*/
	l_ElectricLoadControl_U8 = FALSE;
	l_LoadCtrlCurrAct_U8 =  LOAD_CURRENT_DEFAULT_VALUE;
	l_LoadCutOff_U8 = FALSE;
	/*HRS and HFS outputs+status variables*/
	l_LM_HFSReq_U8 = DEACTIVATE;
	l_LM_HRSReq_U8 = DEACTIVATE;
	l_HFS_Deactivated_status_U8 = TRUE;
	l_HRS_Deactivated_status_U8 = TRUE;
	l_HFS_Activated_status_U8 = FALSE;
	l_HRS_Activated_status_U8 = FALSE;

	/* To avoid overflow */
	KernelClear13BitTimer(FLOADMGT_priority3_KSWtimer);
	KernelClear13BitTimer(FLOADMGT_TSwitchOnClimateLoads_KSWtimer);
	KernelClear13BitTimer(FLOADMGT_TSwitchOffClimateLoads_KSWtimer);
	KernelClear13BitTimer(FLOADMGT_TFastSwitchOffClimateLoads_KSWtimer);
	KernelClear13BitTimer(FLOADMGT_HRS_13minutes_KSWtimer);
	KernelClear13BitTimer(FLOADMGT_HFS_4minutes_KSWtimer);
	KernelClear13BitTimer(FLOADMGT_Error_Debounce_KSWtimer);


}

/****************************************************************************
Function Name     : FLOADMGT_ElectricalLoadControl_signal_priority1

Description       : This function activate loads based on ElectricLoadControl signal Strategy 

Invocation        : Invoked from fspd_KSRRobin.

Parameters        : None.

Return Type       : None.

Critical Section  : None.

******************************************************************************/
static void FLOADMGT_ElectricalLoadControl_signal_priority1(void)
{
	/* to hold timer status */
	UINT8 fl_switchOn_Load_timer_status_U8;
	UINT8 fl_ELC_cfg_bit_pos_chk_U8;

	fl_ELC_cfg_bit_pos_chk_U8 = ELC_CFG_BIT_CHK;
	fl_switchOn_Load_timer_status_U8 = KernelCheck13BitTimer(FLOADMGT_TSwitchOnClimateLoads_KSWtimer);

	if(l_ElectricLoadControl_U8 == ELC_DEFAULT_STATE)
	{
		FLOADMGT_Set_HRS_HFS_Final_Op();
	}
	else if(TESTBIT(NVM_ELECTRIAL_LOAD_SIG_RESP_CFG_U8 , fl_ELC_cfg_bit_pos_chk_U8<<(l_ElectricLoadControl_U8-ELC_CFG_BIT_CHK)))
	{
		if(l_ElectricLoadControl_U8 == ELC_TURN_OFF_HFS_HRS_PTC)
		{
			KernelClear13BitTimer(FLOADMGT_TSwitchOffClimateLoads_KSWtimer);
			FLOADMGT_Deactivate_All_Loads_TFastSwitchOff();
			if(FALSE != l_HFS_Deactivated_status_U8)
			{
				l_LM_HFSReq_U8 = DEACTIVATE;
			}
			if(FALSE != l_HRS_Deactivated_status_U8)
			{
				l_LM_HRSReq_U8 = DEACTIVATE;
			}

		}
		else if((TIMER_RUNNING != fl_switchOn_Load_timer_status_U8) && \
			(l_ElectricLoadControl_U8 == ELC_TURN_ON_HFS_HRS))
		{
			if(ACTIVE != l_LM_HRSReq_U8)
			{
				if(l_AvailableDeltaPower_S16 > (SINT16)ADP_REQ_TO_ACTIVATE_HRS)
				{
					/* Activate HRS */
					l_LM_HRSReq_U8 = ACTIVATE;

					/* Start this timer to avoid 2 or more loads actiavating at a time */
					KernelStart13BitTimer(TIMER_13BIT_BASE_4MS,l_TSwitchOnClimateLoads_U16,\
						FLOADMGT_TSwitchOnClimateLoads_KSWtimer);

				}
			}
			else if((ACTIVE != l_LM_HFSReq_U8)&&(l_AvailableDeltaPower_S16 > (SINT16)ADP_REQ_TO_ACTIVATE_HFS))
			{
				/* Activate HFS */
				l_LM_HFSReq_U8 = ACTIVATE;
			}
			else
			{
				/* do nothing */
			}

		}
		else if(l_ElectricLoadControl_U8 == ELC_TURN_ON_HRS)
		{
			/*If HFS Activated by priority2*/
			if(FALSE != l_HFS_Activated_status_U8)
			{
				l_LM_HFSReq_U8 = ACTIVATE;
			}
			/*If HFS Deactivated by priority3*/
			else if(FALSE != l_HFS_Deactivated_status_U8)
			{
				l_LM_HFSReq_U8 = DEACTIVATE;
			}
			/*If HFS not activated or deactivated by lower priorities*/
			else
			{
				if(DEACTIVATE == l_LM_HFSReq_U8)
				{
					if(TIMER_RUNNING !=fl_switchOn_Load_timer_status_U8)
					{
						l_LM_HFSReq_U8 = NO_LOAD_MANAGEMENT;
					}
				}
				else
				{
					l_LM_HFSReq_U8 = NO_LOAD_MANAGEMENT;
				}
			}
			/* If HRS is activate based on ELC = 2 then should not restart timer once again */
			if(ACTIVE != l_LM_HRSReq_U8)
			{
				if(l_AvailableDeltaPower_S16 > (SINT16)ADP_REQ_TO_ACTIVATE_HRS)
				{
					/* Activate HRS */
					l_LM_HRSReq_U8 = ACTIVATE;
					/* Start this timer to avoid 2 or more loads actiavating at a time */
					KernelStart13BitTimer(TIMER_13BIT_BASE_4MS,l_TSwitchOnClimateLoads_U16,  \
						FLOADMGT_TSwitchOnClimateLoads_KSWtimer);
				}
			}
		}
		else
		{
			/* Do nothing */
		}
	}
	else
	{
		/*If simplified calibration bits are not set then dont respond to ELC signal values 01,10 and 11, maintain
		previous value*/
		FLOADMGT_Set_HRS_HFS_Final_Op();
	}
}

/****************************************************************************
Function Name     : FLOADMGT_LowTemp_Electrical_Load_Strategy_priority2

Description       : This function activate loads based on Low Temp Electrical Load Strategy

Invocation        : Invoked from fspd_KSRRobin.

Parameters        : None.

Return Type       : None.

Critical Section  : None.

******************************************************************************/
static void FLOADMGT_LowTemp_Electrical_Load_Strategy_priority2(void)
{
	UINT8 fl_low_temp_config_U8;

	UINT8 fl_EngineCoolantTemperature_U8;

	UINT16 fl_Ambient_Temperature_U16;

	UINT8 fl_trip_status_U8;

	UINT8 fl_aircondition_status_U8;

	UINT8 fl_ECMEstimatedAmbTempQF_value_U8;

	/* Check for Low Temp. boundary conditions */
	if(NVM_Low_Temp_Config_U8 > 1)
	{
		fl_low_temp_config_U8 = TRUE;
	}
	else
	{
		/* Get Low Temp Electric Load Strategy NVM Config value */
		fl_low_temp_config_U8 = NVM_Low_Temp_Config_U8;
	}

	/* Get EngineCoolantTemperature signal status */

	if((UINT8)0x00 == ((RCAN_SIG_MISSING | RCAN_SIG_NVR_RCVD) & \
		rcan_get_EngineCoolantTemperature_status()))
	{
		/* Get EngineCoolantTemperature signal value */

		fl_EngineCoolantTemperature_U8 = rcan_get_EngineCoolantTemperature_value();

		fl_trip_status_U8 = fcfg_Trip_Computer_Status();

		fl_aircondition_status_U8 = fcfg_Aircondition_Status();

		/* Check for CCC parameters for Trip Computer and Climate control and OAT h/w
		** if it is not fitted then get ambtemp value from CAN message and if CAN signal 
		** is missing assume Ambtemp value is 20 deg.
		*/
		if( (FALSE != foat_get_status_OAT_U8()) || ( (FALSE == fl_trip_status_U8) &&
			(AIR_CONDITION_AUTOMATIC != fl_aircondition_status_U8)) )
		{
			fl_ECMEstimatedAmbTempQF_value_U8 = rcan_get_ECMEstimatedAmbTempQF_value();
			/* Chk Signal status is never received OR missing */
			if(((QF_VALUE_TWO == fl_ECMEstimatedAmbTempQF_value_U8) || \
				(QF_VALUE_THREE == fl_ECMEstimatedAmbTempQF_value_U8)) || \
				((UINT8)0x00 == ((RCAN_SIG_MISSING | RCAN_SIG_NVR_RCVD) & \
				rcan_get_ECMEstimatedAmbTemp_status())))
			{
				/* Get Ambient Temperature signal value */
				fl_Ambient_Temperature_U16 = rcan_get_ECMEstimatedAmbTemp_value();
			}
			else
			{
				/* if Ambient Temp. signal missing or QF is invalid then this Strategy 
				** assume ambient temp value is 20 Deg 
				*/
				fl_Ambient_Temperature_U16 = ECM_AMBTEMP_MISS_INVALD_DEFAULT;
			}
		}
		else
		{
			/* OAT is fitted. Get OAT value */
			fl_Ambient_Temperature_U16 = foat_get_OAT_Unfillter_Value_025_Reso_U16();

			/* Offset added to match OAT value resolution with CAN signal */
			fl_Ambient_Temperature_U16 = fl_Ambient_Temperature_U16 + OAT_OFFSET_VALUE;
		}
		/* Calculate ADP value */
		FLOADMGT_calculate_AvailableDeltaPower_value();

		if((fl_low_temp_config_U8 == ACTIVE)&&           \
			(fl_Ambient_Temperature_U16 < LOW_TEMP_ACTIVATE_OAT_VAL)&&  \
			(fl_EngineCoolantTemperature_U8 < LOW_TEMP_ACTIVATE_ECT_VAL))
		{
			if((l_AvailableDeltaPower_S16 > (SINT16)ADP_REQ_TO_ACTIVATE_HRS) && \
				(l_ElectricLoadControl_U8 == ELC_DEFAULT_STATE))
			{
				/* Set status variable to Activate HRS in priority 1 function */
				l_HRS_Activated_status_U8 = ACTIVE;

				/* Start 13 mins timer for HRS */
				KernelStart13BitTimer(TIMER_13BIT_BASE_512MS,FLOADMGT_HRS_13MIN_TIMEOUT, \
					FLOADMGT_HRS_13minutes_KSWtimer);

				/* Start this timer to avoid 2 or more loads actiavating at a time */
				KernelStart13BitTimer(TIMER_13BIT_BASE_4MS,l_TSwitchOnClimateLoads_U16,  \
					FLOADMGT_TSwitchOnClimateLoads_KSWtimer);
			}

			if(((l_ElectricLoadControl_U8 == ELC_TURN_ON_HRS) || \
				(l_ElectricLoadControl_U8 == ELC_DEFAULT_STATE)) && \
				(l_AvailableDeltaPower_S16 > (SINT16)ADP_REQ_TO_ACTIVATE_HFS))
			{
				/* Set this status variable */
				l_priority2_HFS_status_U8 = TRUE;

			}
		}
		else
		{
			/* do nothing */
		}
	}
}

/****************************************************************************
Function Name     : FLOADMGT_Load_Management_Strategy_priority3

Description       : This function activate loads based HFS Strategy

Invocation        : Invoked from fspd_KSRRobin.

Parameters        : None.

Return Type       : None.

Critical Section  : None.

******************************************************************************/
static void FLOADMGT_Load_Management_Strategy_priority3(void)
{
	UINT8 fl_batvolt_dislimit_U8;
	UINT16 fl_disreset_timerlimit_U16;
	UINT16 fl_distimer_maxlimit_U16;
	UINT8 fl_SS_defroster_timeout_U8;
	UINT8 fl_BatteryVoltage_Signal_status_U8;
	UINT8 fl_Battery_Voltage_U8;
	UINT16 fl_TSwitchOffClimateLoads_Timer_Val_U16;
	UINT8  fl_TSwitchOffClimateLoads_status_U8;

	/*get the TSwitchOffClimateLoads timer status*/
	fl_TSwitchOffClimateLoads_status_U8 = KernelCheck13BitTimer(FLOADMGT_TSwitchOffClimateLoads_KSWtimer);

	/* Get the TswitchOffClimate loads in Time, if two or more loads are to be switched OFF, there 
	** should be a TswitchOffClimate delay between Deactivate two loads, default is 3Sec
	*/
	fl_TSwitchOffClimateLoads_Timer_Val_U16 = FLOADMGT_validate_NVM_TSWITCHOFF();

	fl_BatteryVoltage_Signal_status_U8 =rcan_get_BatteryVoltage_status();
	if(FALSE ==((RCAN_SIG_MISSING | RCAN_SIG_NVR_RCVD) & fl_BatteryVoltage_Signal_status_U8))
	{
		/* Get Battery Voltage signal value - This value used by priority 3 also */
		fl_Battery_Voltage_U8 = rcan_get_BatteryVoltage_value();

		/* check the battery voltage is 0xFF-invalid */
		if(BATTERY_VOLTAGE_INVLD == fl_Battery_Voltage_U8)
		{
			fl_Battery_Voltage_U8 = BATVOL_MISS_INVLD_DEF_VAL;
		}
	}
	else
	{
		fl_Battery_Voltage_U8 = BATVOL_MISS_INVLD_DEF_VAL;

	}

	/* Check for NVM Value of BatteryVoltageDischargeLimit boundary conditions */
	if((NVM_BatteryVoltageDischargeLimit_U8 < BATVOLT_DISLIMIT_MIN_VAL) || \
		(NVM_BatteryVoltageDischargeLimit_U8 > BATVOLT_DISLIMIT_MAX_VAL))
	{
		/* Default Value */
		fl_batvolt_dislimit_U8 = BATVOLT_DISLIMIT_DEF_VAL;
	}
	else
	{
		fl_batvolt_dislimit_U8 = NVM_BatteryVoltageDischargeLimit_U8;
	}

	/* Check for NVM Value of DischargeResetTimerMaxLimit boundary conditions */
	if(NVM_DischargeResetTimerMaxLimit_U8 > 60)
	{
		/* Default Value */
		fl_disreset_timerlimit_U16 = ( ((UINT16)15) * CONVERT_MIN_TO_SEC );
	}
	else
	{
		fl_disreset_timerlimit_U16 =(UINT16)( ((UINT16)NVM_DischargeResetTimerMaxLimit_U8) * CONVERT_MIN_TO_SEC );
	}

	/* Check for NVM Value of DischargeTimerMaxLimit boundary conditions */
	if(NVM_DischargeTimerMaxLimit_U8 > 60)
	{
		/* Default Value */
		fl_distimer_maxlimit_U16 = ( ((UINT16)15) * CONVERT_MIN_TO_SEC );
	}
	else
	{
		fl_distimer_maxlimit_U16 = (UINT16)( ((UINT16)NVM_DischargeTimerMaxLimit_U8) * CONVERT_MIN_TO_SEC);
	}

	/* Check for NVM Value of SS defrost delay timer boundary conditions */
	if(  NVM_SS_Defroster_time > (UINT8)60 )
	{
		/* Default Value */
		fl_SS_defroster_timeout_U8 = DEFROSTER_DEFAULT_VALUE;

	}
	else
	{
		fl_SS_defroster_timeout_U8 = NVM_SS_Defroster_time;
	}


	if((l_ElectricLoadControl_U8 == ELC_DEFAULT_STATE) ||
		(l_ElectricLoadControl_U8 == ELC_TURN_ON_HRS  ))
	{
		/* For DischargeTimer */
		if((l_AvailableDeltaPower_S16 < 0)&& \
			(fl_Battery_Voltage_U8 <= fl_batvolt_dislimit_U8))
		{
			if(l_DischargeTimer_U16<fl_distimer_maxlimit_U16)
			{
				/* Increament Discharge Timer for Stop Start*/
				l_DischargeTimer_U16++;
			}
		}
		/* For DischargeResetTimer */
		if(l_AvailableDeltaPower_S16 >= 0)
		{
			/* Increament Discharge Reset Timer for Non Stop Start*/
			l_DischargeResetTimer_U16++;
		}
		/* Reset both the timers if Discharge reset timer reached max limit*/
		if(l_DischargeResetTimer_U16 >= fl_disreset_timerlimit_U16)
		{
			/* reset the timers */
			l_DischargeTimer_U16 =TIMER_RESET_TOZERO;
			l_DischargeResetTimer_U16 =TIMER_RESET_TOZERO;
		}
	}
	if(l_defroster_timer_U8 <= fl_SS_defroster_timeout_U8)
	{
		l_defroster_timer_U8 ++;
	}

	if((ELC_TURN_OFF_HFS_HRS_PTC == l_ElectricLoadControl_U8)&&\
		(TESTBIT(NVM_ELECTRIAL_LOAD_SIG_RESP_CFG_U8,ELC_TURN_OFF_HFS_HRS_PTC)))
	{
		/*if ELC = 1 then loads are deactivated by priority 1 function itself.*/
	}
	else
	{
		/*When power mode changes to running , for SS Defrost delay time loads need to be deactivated*/
		if(l_defroster_timer_U8 <= fl_SS_defroster_timeout_U8)
		{
			/*Deactivate HRS/HFS*/
			FLOADMGT_Deactivate_All_Loads_TFastSwitchOff();
		}
		/*Deactivate loads if ADP is negative for 15 minutes( default value)*/
		else if(l_DischargeTimer_U16 >= fl_distimer_maxlimit_U16)
		{
			/*Deactivate HRS/HFS*/
			if(FALSE ==l_HFS_Deactivated_status_U8)
			{
				/* Deactivate HFS */
				l_HFS_Deactivated_status_U8 = TRUE;
				KernelStart13BitTimer(TIMER_13BIT_BASE_4MS,fl_TSwitchOffClimateLoads_Timer_Val_U16, \
					FLOADMGT_TSwitchOffClimateLoads_KSWtimer);
			}
			else if((FALSE == l_HRS_Deactivated_status_U8)&&\
				(TIMER_RUNNING != fl_TSwitchOffClimateLoads_status_U8))
			{
				/* Deactivate HRS */
				l_HRS_Deactivated_status_U8 = TRUE;
			}
			else
			{
				/*Both HRS/HFS are deactivated by Load management due to ADP negative*/
				l_LoadCutOff_U8 = TRUE;
			}
		}
		else
		{
			/* No Load management for priority 3*/
			l_HFS_Deactivated_status_U8 = FALSE;
			l_HRS_Deactivated_status_U8 = FALSE;
			l_LoadCutOff_U8 = FALSE;
		}
	}
}
/****************************************************************************
Function Name     : FLOADMGT_Deactivate_All_Loads_TFastSwitchOff

Description       : Function is called to deactivate HRS/HFS/PTC Heaters when electrical load control 
                    signal = 01 or when power mode = running stand by / start in progress.Loads are 
                    deactivated with a delay of TFastSwitchOffClimateLoads between deactivation of 2 loads.

Invocation        : Called by Load Management Priority 1 function and RR .

Parameters        : None.

Return Type       : None

Global Variables  : None.

External interface: None.

Critical Section  : None.
******************************************************************************/
static void FLOADMGT_Deactivate_All_Loads_TFastSwitchOff(void)
{
	UINT8 fl_TFastswitchOff_Load_timer_status_U8;
	UINT8 fl_TFastSwitchOffClimateLoads_U8;

	if( NVM_TFastSwitchOffClimateLoads_U8  > TFASTSWITCHOFFCLIMATELOADS_MAX_VAL )
	{
		/*Load default value - 300 ms*/
		fl_TFastSwitchOffClimateLoads_U8 = TFASTSWITCHOFFCLIMATELOADS_DEF_VAL;
	}
	else
	{
		fl_TFastSwitchOffClimateLoads_U8 =\
			(UINT8)( ( ((UINT16)NVM_TFastSwitchOffClimateLoads_U8) * ((UINT16)TFASTSWITCHOFF_RES) )/((UINT16)FOUR_MS_TIMER_BASE) );
	}

	fl_TFastswitchOff_Load_timer_status_U8 = KernelCheck13BitTimer(FLOADMGT_TFastSwitchOffClimateLoads_KSWtimer);

	if(TIMER_RUNNING != fl_TFastswitchOff_Load_timer_status_U8)
	{
		if(FALSE ==l_HFS_Deactivated_status_U8)
		{
			/* Deactivate HFS */
			l_HFS_Deactivated_status_U8 = TRUE;
			KernelStart13BitTimer(TIMER_13BIT_BASE_4MS,fl_TFastSwitchOffClimateLoads_U8, \
				FLOADMGT_TFastSwitchOffClimateLoads_KSWtimer);
		}
		else if( FALSE != HW_O_PTC_HTR_B_IS_ACTIVE())
		{
			HW_O_PTC_HTR_B_DEACTIVATE();
			KernelStart13BitTimer(TIMER_13BIT_BASE_4MS,fl_TFastSwitchOffClimateLoads_U8, \
				FLOADMGT_TFastSwitchOffClimateLoads_KSWtimer);
		}
		else if( FALSE != HW_O_PTC_HTR_A_IS_ACTIVE())
		{
			HW_O_PTC_HTR_A_DEACTIVATE();
			KernelStart13BitTimer(TIMER_13BIT_BASE_4MS,fl_TFastSwitchOffClimateLoads_U8, \
				FLOADMGT_TFastSwitchOffClimateLoads_KSWtimer);
		}
		else if (FALSE == l_HRS_Deactivated_status_U8)
		{
			/* Deactivate HRS */
			l_HRS_Deactivated_status_U8 = TRUE;
		}
		else
		{
			/*All loads are deactivated */
		}
	}
}
/****************************************************************************
Function Name     : FLOADMGT_Set_HRS_HFS_Final_Op

Description       : Function is called to deactivate HRS/HFS/PTC Heaters when electrical load control 
                    signal = 01 or when power mode = running stand by / start in progress.Loads are 
                    deactivated with a delay of TFastSwitchOffClimateLoads between deactivation of 2 loads.

Invocation        : Called by Load Management Priority 1 function and RR .

Parameters        : None.

Return Type       : None

Global Variables  : None.

External interface: None.

Critical Section  : None.
******************************************************************************/
static void FLOADMGT_Set_HRS_HFS_Final_Op(void)
{
	UINT8 fl_switchOn_Load_timer_status_U8;
	UINT8 fl_switchOff_Load_timer_status_U8;
	UINT16 fl_TSwitchOffClimateLoads_Timer_Val_U16;

	fl_TSwitchOffClimateLoads_Timer_Val_U16 = FLOADMGT_validate_NVM_TSWITCHOFF();

	/*If HFS Activated by priority2*/
	if(FALSE != l_HFS_Activated_status_U8)
	{
		l_LM_HFSReq_U8 = ACTIVATE;
	}
	/*If HFS Deactivated by priority3*/
	else if(FALSE != l_HFS_Deactivated_status_U8)
	{
		l_LM_HFSReq_U8 = DEACTIVATE;
	}
	/*If HFS not activated or deactivated by lower priorities*/
	else
	{
		if(DEACTIVATE == l_LM_HFSReq_U8)
		{
			/* Start this timer to avoid 2 or more loads actiavating at a time */
			KernelStart13BitTimer(TIMER_13BIT_BASE_4MS,l_TSwitchOnClimateLoads_U16,  \
				FLOADMGT_TSwitchOnClimateLoads_KSWtimer);
		}
		else if(ACTIVATE == l_LM_HFSReq_U8)
		{
			/* Start this timer to avoid 2 or more loads actiavating at a time */
			KernelStart13BitTimer(TIMER_13BIT_BASE_4MS,fl_TSwitchOffClimateLoads_Timer_Val_U16,  \
				FLOADMGT_TSwitchOffClimateLoads_KSWtimer);
		}
		else
		{
			/*NO action*/
		}
		l_LM_HFSReq_U8 = NO_LOAD_MANAGEMENT;
	}

	/*If HRS Activated by priority2*/
	if(FALSE != l_HRS_Activated_status_U8)
	{
		l_LM_HRSReq_U8 = ACTIVATE;
	}
	/*If HRS Deactivated by priority3*/
	else if(FALSE != l_HRS_Deactivated_status_U8)
	{
		l_LM_HRSReq_U8 = DEACTIVATE;
	}
	/*If HRS not activated or deactivated by lower priorities*/
	else
	{
		if(DEACTIVATE == l_LM_HRSReq_U8)
		{
			/* If both HFS and HRS change from deactivate to No load , send HFS = No load and 
			after TSwitchOn send HRS = No load , so that BCM activating both loads at same 
			time is prevented*/
			fl_switchOn_Load_timer_status_U8 = KernelCheck13BitTimer(FLOADMGT_TSwitchOnClimateLoads_KSWtimer);
			if(TIMER_RUNNING !=fl_switchOn_Load_timer_status_U8)
			{
				l_LM_HRSReq_U8 = NO_LOAD_MANAGEMENT;
			}
		}
		else if(ACTIVATE == l_LM_HRSReq_U8)
		{
			/* If both HFS and HRS change from activate to No load , send HFS = No load and 
			after TSwitchOn send HRS = No load , so that BCM activating both loads at same 
			time is prevented*/
			fl_switchOff_Load_timer_status_U8 = KernelCheck13BitTimer(FLOADMGT_TSwitchOffClimateLoads_KSWtimer);
			if(TIMER_RUNNING !=fl_switchOff_Load_timer_status_U8)
			{
				l_LM_HRSReq_U8 = NO_LOAD_MANAGEMENT;
			}
		}
		else
		{
			l_LM_HRSReq_U8 = NO_LOAD_MANAGEMENT;
		}
	}
}
/****************************************************************************
Function Name     : FLOADMGT_calculate_AvailableDeltaPower_value

Description       : This function calculate ADP based on CAN signal

Invocation        : Invoked from fspd_KSRRobin.

Parameters        : None.

Return Type       : None.

Critical Section  : None.
******************************************************************************/
static void FLOADMGT_calculate_AvailableDeltaPower_value(void)
{
	UINT8 fl_ActualCurrent_U8;
	UINT8 fl_AvailableCurrent_U8;
	UINT8 fl_ACMCommunicationError_U8;
	UINT8 fl_ACMTemperatureFault_U8;
	UINT8 fl_ACMMechanicalFault_U8;
	UINT8 fl_ACMElectricalFault_U8;
	UINT8 fl_ADP_P1_U8;
	UINT16 fl_ADP_P2_U16;
	SINT16 fl_ADP_P3_S16;
	UINT8 fl_ADP_P4_U8;
	UINT8 fl_ADP_config_U8;
	UINT8 fl_error_handling_flag_U8;
	UINT8 fl_debounce_timer_status_U8;
	UINT8 fl_ActualCurrent_Signal_status_U8;
	SINT32 fl_ADP_Calculation_S32;
	UINT8 fl_BatteryVoltage_Signal_status_U8;
	UINT8 fl_Battery_Voltage_U8;

	/* Check for ADP P1 boundary conditions */
	if(NVM_ADP_P1_U8 > 50)
	{
		fl_ADP_P1_U8 = 4;
	}
	else
	{
		fl_ADP_P1_U8 = NVM_ADP_P1_U8;
	}

	/* Check for ADP P2 boundary conditions */
	if(NVM_ADP_P2_U16 > 1000)
	{
		fl_ADP_P2_U16 = 100;
	}
	else
	{
		fl_ADP_P2_U16 = NVM_ADP_P2_U16;
	}

	/* Check for ADP P3 boundary conditions */
	if(NVM_ADP_P3_S16 > 1000)
	{
		fl_ADP_P3_S16 = 1000;
	}
	else
	{
		fl_ADP_P3_S16 = NVM_ADP_P3_S16;
	}

	/* Check for ADP P4 boundary conditions */
	if(NVM_ADP_P4_U8 > 100)
	{
		fl_ADP_P4_U8 = 100;/*ncheck*/
	}
	else
	{
		fl_ADP_P4_U8 = NVM_ADP_P4_U8;
	}

	/* Check for ADP Config. boundary conditions */
	if(NVM_ADP_Config_U8 > 1)
	{
		fl_ADP_config_U8 = TRUE;
	}
	else
	{
		fl_ADP_config_U8 = NVM_ADP_Config_U8;
	}

	/* Check for ErrorHandlingFlag boundary conditions */
	if(NVM_ErrorHandlingFlag_U8 > 1)
	{
		fl_error_handling_flag_U8 = TRUE;
	}
	else
	{
		fl_error_handling_flag_U8 = NVM_ErrorHandlingFlag_U8;
	}

	/* Chk Signal status is never received OR missing */
	if((UINT8)0x00 == ((RCAN_SIG_MISSING | RCAN_SIG_NVR_RCVD) & \
		rcan_get_ACMCommunicationError_status()))
	{
		/* Get Battery ACM Communication Error value */
		fl_ACMCommunicationError_U8 = rcan_get_ACMCommunicationError_value();

		/* Get Battery ACM Temperature Fault value */
		fl_ACMTemperatureFault_U8 = rcan_get_ACMTemperatureFault_value();

		/* Get Battery ACM Mechanical Fault value */
		fl_ACMMechanicalFault_U8 = rcan_get_ACMMechanicalFault_value();

		/* Get Battery ACM Electrical Fault value */
		fl_ACMElectricalFault_U8 = rcan_get_ACMElectricalFault_value();
	}
	else
	{
		/* Initialise the variable if it is missed */
		fl_ACMCommunicationError_U8 = 0;
		fl_ACMTemperatureFault_U8 = 0;
		fl_ACMMechanicalFault_U8 = 0;
		fl_ACMElectricalFault_U8 = 0;
	}

	/* get the actual current (0x364)signal status */
	fl_ActualCurrent_Signal_status_U8 = rcan_get_ActualCurrent_status();
	/* get the BatteryVoltage(0x428) signal status */
	fl_BatteryVoltage_Signal_status_U8 =rcan_get_BatteryVoltage_status();

	/* Chk Signal status is never received OR missing for battery voltage and acutual current msg */
	if((FALSE == ((RCAN_SIG_MISSING | RCAN_SIG_NVR_RCVD) & fl_ActualCurrent_Signal_status_U8))&&\
		(FALSE==((RCAN_SIG_MISSING | RCAN_SIG_NVR_RCVD) & fl_BatteryVoltage_Signal_status_U8)))
	{

		/* Get Battery Actual Current value */
		fl_ActualCurrent_U8 = rcan_get_ActualCurrent_value();

		/* Get Battery Available Current value */
		fl_AvailableCurrent_U8 = rcan_get_AvailableCurrent_value();

		/* Get Battery Voltage signal value - This value used by priority 3 also */
		fl_Battery_Voltage_U8 = rcan_get_BatteryVoltage_value();

		/* check the battery voltage is 0xFF-invalid */
		if(BATTERY_VOLTAGE_INVLD == fl_Battery_Voltage_U8)
		{
			fl_Battery_Voltage_U8 = BATVOL_MISS_INVLD_DEF_VAL;
		}
		if(ACTIVE == fl_ADP_config_U8)
		{
			if(fl_ActualCurrent_U8 >= fl_AvailableCurrent_U8)
			{
				l_AvailableDeltaPower_S16 = ((SINT16)(-1)*(SINT16)fl_ADP_P2_U16);
			}
			else
			{
				/* Formula for ADP Calculation */
				fl_ADP_Calculation_S32 =  ((SINT32)fl_Battery_Voltage_U8* \
					((SINT32)((fl_AvailableCurrent_U8 - fl_ActualCurrent_U8) - fl_ADP_P1_U8)* (SINT32)fl_ADP_P4_U8));

				/* Convert ADP value with .1 resolution for Battery Voltage and .01 resolution 
				for P4 NVM value)
				*/
				l_AvailableDeltaPower_S16 = (SINT16)(fl_ADP_Calculation_S32/ADP_RESOLUTION_FACTOR);
			}

			if((fl_error_handling_flag_U8 != FALSE) && \
				((fl_ACMCommunicationError_U8 != FALSE) || \
				(fl_ACMTemperatureFault_U8   != FALSE) || \
				(fl_ACMMechanicalFault_U8    != FALSE) || \
				(fl_ACMElectricalFault_U8    != FALSE)))
			{
				/* Start the 5 Sec timer if any one of the err flag is set  */
				if( l_err_handling_tmr_status_BOOL == FALSE )
				{
					KernelStart13BitTimer(TIMER_13BIT_BASE_512MS,FLOADMGT_ERRORHANDLINGDEBOUNCETIMER_TIMEOUT, \
						FLOADMGT_Error_Debounce_KSWtimer);

					l_err_handling_tmr_status_BOOL = TRUE;
				}
			}
			else
			{
				/* if the err debounce time is not complete then clear timer */
				if(l_err_handling_tmr_status_BOOL != FALSE )
				{
					KernelClear13BitTimer(FLOADMGT_Error_Debounce_KSWtimer);
					l_err_handling_tmr_status_BOOL =  FALSE;
				}

			}
			/* Get the debounce timer status */
			fl_debounce_timer_status_U8 = KernelCheck13BitTimer(FLOADMGT_Error_Debounce_KSWtimer);

			if( (TIMER_RUNNING != fl_debounce_timer_status_U8 ) && (l_err_handling_tmr_status_BOOL != FALSE ) )
			{
				l_AvailableDeltaPower_S16 = (-1)*fl_ADP_P3_S16;
			}
		}
		else
		{
			l_AvailableDeltaPower_S16 = ADP_MAX_LIMIT;
			/* To avoid overflow */
			KernelClear13BitTimer(FLOADMGT_Error_Debounce_KSWtimer);
		}
	}
	/*Actual current signal is Never received- send -100W (SRD10726)*/
	else if(FALSE !=(RCAN_SIG_NVR_RCVD & fl_ActualCurrent_Signal_status_U8))
	{
		/* set ADP value is -1000w */
		l_AvailableDeltaPower_S16 = ADP_DEFAULT_VALUE;

	}
	else if((FALSE !=(RCAN_SIG_MISSING & fl_ActualCurrent_Signal_status_U8))||\
		(FALSE !=(RCAN_SIG_MISSING & fl_BatteryVoltage_Signal_status_U8)))
	{
		/* Set ADP = - P3 Watts if Battery voltage / Actual current signal is lost*/
		l_AvailableDeltaPower_S16 = (-1)* fl_ADP_P3_S16;
	}
	else
	{
		/*QAC*/
	}
	/* Valid range checking */
	if(l_AvailableDeltaPower_S16 < ADP_MIN_LIMIT)
	{
		l_AvailableDeltaPower_S16 = ADP_MIN_LIMIT;
	}
	else if(l_AvailableDeltaPower_S16 > (SINT16)ADP_MAX_LIMIT)
	{
		l_AvailableDeltaPower_S16 = ADP_MAX_LIMIT;
	}
	else
	{
		/* Do Nothing */
	}
}

/****************************************************************************
Function Name     : FLOADMGT_get_AvailableDeltaPower_U32

Description       : Return Available Delta Power value.

Invocation        : Called by Can Module.

Parameters        : None.

Return Type       : UINT8

Global Variables  : None.

External interface: None.

Critical Section  : None.

******************************************************************************/

UINT32 FLOADMGT_get_AvailableDeltaPower_U32(void)
{
	UINT32 fl_ADP_signal_output_U32 = ADP_MINIMUM_VALUE;

	/* 0 - 255 is ADP signal Value. Where 0 = -1000W & 255 = 2000W.
	The below is the formula used to linear interpolate ADP value */
	fl_ADP_signal_output_U32 += ((UINT32)l_AvailableDeltaPower_S16);
	fl_ADP_signal_output_U32 = (fl_ADP_signal_output_U32 * ADP_FACTOR_CONVERT_BY_100);
	fl_ADP_signal_output_U32 = (fl_ADP_signal_output_U32/ADP_CAN_TRMT_FACTOR);

	if(fl_ADP_signal_output_U32 > LOAD_MANAGEMENT_SIGNAL_MAX_VALUE)
	{
		fl_ADP_signal_output_U32 = LOAD_MANAGEMENT_SIGNAL_MAX_VALUE;
	}

	return(fl_ADP_signal_output_U32);
}
/****************************************************************************
Function Name     : FLOADMGT_get_LoadCutOff_U8

Description       : Return LoadCutOff Value of Load Management value.

Invocation        : Called by Can Module.

Parameters        : None.

Return Type       : UINT8

Global Variables  : None.

External interface: None.

Critical Section  : None.
******************************************************************************/

UINT8 FLOADMGT_get_LoadCutOff_U8(void)
{
	UINT8 fl_ptc_status_U8;
	UINT8 fl_LoadCutOff_ret_sts_U8;

	fl_ptc_status_U8 = FPTC_get_PTC_deactivate_status_wrt_ADP();

	/*Either HRS/HFS deactivated by priority 3 ( Normal load management) OR PTC heaters deactivated by PTC algo*/
	if((FALSE != fl_ptc_status_U8)||(l_LoadCutOff_U8 != FALSE))
	{
		fl_LoadCutOff_ret_sts_U8 = LOAD_DEACTIVATEDBY_LM;
	}
	else
	{
		fl_LoadCutOff_ret_sts_U8 = NO_LOAD_DEACTIVATEDBY_LM;
	}
	return(fl_LoadCutOff_ret_sts_U8);

}
/****************************************************************************
Function Name     : FLOADMGT_get_LM_HFSReq_U8

Description       : Return Heated Front Screen Status value.

Invocation        : Called by Can Module.

Parameters        : None.

Return Type       : UINT8

Global Variables  : None.

External interface: None.

Critical Section  : None.
******************************************************************************/

UINT8 FLOADMGT_get_LM_HFSReq_U8(void)
{
	return(l_LM_HFSReq_U8);
}

/****************************************************************************
Function Name     : FLOADMGT_get_LM_HRSReq_U8

Description       : Return Heated Rear Screen Status value.

Invocation        : Called by Can Module.

Parameters        : None.

Return Type       : UINT8

Global Variables  : None.

External interface: None.

Critical Section  : None.
******************************************************************************/

UINT8 FLOADMGT_get_LM_HRSReq_U8(void)
{
	return(l_LM_HRSReq_U8);
}
/****************************************************************************
Function Name     : FLOADMGT_priority2_status_function(void)

Description       : Function is called to deactivate HFS load.

Invocation        : Called by Load Management Priority functions.

Parameters        : None.

Return Type       : None

Global Variables  : None.

External interface: None.

Critical Section  : None.
******************************************************************************/

static void FLOADMGT_priority2_status_function(void)
{
	UINT8 fl_HFS_4_minutes_timer_status_U8;
	UINT8 fl_HRS_13_minutes_timer_status_U8;

	if( FALSE != l_priority2_HFS_status_U8)
	{
		if(TIMER_RUNNING != KernelCheck13BitTimer(FLOADMGT_TSwitchOnClimateLoads_KSWtimer))
		{
			/*Once HRS was activated in Priority 2 function, after TSwitchOn period 
			check ADP and activate HFS*/
			l_priority2_HFS_status_U8 = FALSE;
			if(l_AvailableDeltaPower_S16 > (SINT16)ADP_REQ_TO_ACTIVATE_HFS)
			{
				/* Start 4 mins timer for HFS */
				KernelStart13BitTimer(TIMER_13BIT_BASE_512MS,FLOADMGT_HFS_4MIN_TIMEOUT, \
					FLOADMGT_HFS_4minutes_KSWtimer);
				/*Activate HFS */
				l_HFS_Activated_status_U8 = TRUE;
			}
		}
	}
	/* Get the 4 minutes timer */
	fl_HFS_4_minutes_timer_status_U8 = KernelCheck13BitTimer(FLOADMGT_HFS_4minutes_KSWtimer);
	fl_HRS_13_minutes_timer_status_U8 = KernelCheck13BitTimer(FLOADMGT_HRS_13minutes_KSWtimer);


	if(TIMER_RUNNING == fl_HFS_4_minutes_timer_status_U8)
	{
		/* Activate HFS */
		l_HFS_Activated_status_U8 = TRUE;
	}
	else
	{
		l_HFS_Activated_status_U8 = FALSE;
	}

	if(TIMER_RUNNING == fl_HRS_13_minutes_timer_status_U8 )
	{
		/* Activate HRS */
		l_HRS_Activated_status_U8 = TRUE;
	}
	else
	{
		l_HRS_Activated_status_U8 = FALSE;
	}
}

/****************************************************************************
Function Name     : floadmgt_KSSleep

Description       : This function is used to mark entry into Sleep mode

Invocation        : This function is invoked by Kernel Scheduler

Parameters        : void

Return Type       : void

Critical Section  : None

Created           : 10/06/07 by agunasek

Updated           : 10/06/07 by agunasek

******************************************************************************/
void FLOADMGT_KSSleep(void)
{
	FLOADMGT_Transout_NORMAL();
}
/****************************************************************************
Function Name     : FLOADMGT_get_AvailableDeltaPower_S16

Description       : Return Available Delta Power value.

Invocation        : Called by PTC Module.

Parameters        : None.

Return Type       : SINT16

Global Variables  : None.

External interface: None.

Critical Section  : None.

******************************************************************************/

SINT16 FLOADMGT_get_AvailableDeltaPower_S16(void)
{
	return(l_AvailableDeltaPower_S16);
}

/****************************************************************************
Function Name     : floadmgt_Transinto_OFF

Description       : This function is used to mark entry into Sleep mode

Invocation        : This function is invoked by Kernel Scheduler

Parameters        : void

Return Type       : void

Critical Section  : None

Created           : 10/06/07 by agunasek

Updated           : 10/06/07 by agunasek

******************************************************************************/
void FLOADMGT_Transinto_OFF(void)
{
	/* Set the variable in Transinto OFF */
	l_priority2_OFF_status_U8 = TRUE;
	l_DischargeTimer_U16 = FALSE;
	l_DischargeResetTimer_U16 = FALSE;
}
/****************************************************************************
Function Name     : FLOADMGT_Current_calculation_function(void)

Description       : Function is called to calculate the current for rear defroster,
                    HFS,Heated Mirror and PTC .

Invocation        : Called by Load Management Priority functions.

Parameters        : None.

Return Type       : None

Global Variables  : None.

External interface: None.

Critical Section  : None.
******************************************************************************/

static void FLOADMGT_Current_calculation_function(void)
{
	UINT8  fl_Battery_Voltage_U8;
	UINT8  fl_PTC_current_U8;
	UINT8  fl_PTCA_current_U8;
	UINT8  fl_PTCB_current_U8;
	UINT16 fl_HFS_Current_U16;
	UINT8  fl_Heated_Mirror_U8;
	UINT8  fl_rear_defroster_U8;
	UINT16 fl_Total_current_U16;
	UINT16 fl_Battery_Voltage_with_ten_U16;
	UINT16 fl_PTC_ct_mul_100_U16;
	UINT16 fl_RD_ct_mul_10_U16;
	UINT16 fl_HM_ct_mul_10_U16;
	UINT8 fl_HRS_value;
	UINT8 fl_HFS_value;


	fl_HRS_value = rcan_get_PJB_BackLiteHeaterStatus_value();
	fl_HFS_value = rcan_get_PJB_FrontScreenHtrStatus_value();

	if (LOAD_CURRENT_MESSAGE_RECEIVED == ((RCAN_SIG_MISSING | RCAN_SIG_NVR_RCVD) & rcan_get_BatteryVoltage_status()))
	{
		fl_Battery_Voltage_U8 = rcan_get_BatteryVoltage_value();
		if ( MAX_BATT_VOLTAGE == fl_Battery_Voltage_U8)
		{
			fl_Battery_Voltage_U8 =(UINT8)((((UINT32)(IDD01_get_10bit_ad(HW_I_M_SBATT_MONITOR_ADCHAN))* (UINT32)200)/(UINT32)1024) + (UINT32)NVM_voltage_diode_drop_U8);
		}
	}
	else
	{
		fl_Battery_Voltage_U8 =(UINT8)((((UINT32)(IDD01_get_10bit_ad(HW_I_M_SBATT_MONITOR_ADCHAN))* (UINT32)200)/(UINT32)1024) + (UINT32)NVM_voltage_diode_drop_U8);
	}


	if( (fl_HRS_value!=0)&& (FALSE == ((RCAN_SIG_MISSING | RCAN_SIG_NVR_RCVD) & rcan_get_PJB_BackLiteHeaterStatus_status())))
	{
		fl_rear_defroster_U8 = LinearInterpolateByte((UINT8 *)&l_Rear_Defroster_Battery_current_Table_U8[0][0],fl_Battery_Voltage_U8);
		fl_RD_ct_mul_10_U16  =  fl_rear_defroster_U8 * LOAD_CURRENT_MUL_10;
		fl_Heated_Mirror_U8 = LinearInterpolateByte((UINT8 *)&l_Heated_Mirror_Battery_current_Table_U8[0][0],fl_Battery_Voltage_U8);
		fl_HM_ct_mul_10_U16  =  fl_Heated_Mirror_U8 * LOAD_CURRENT_MUL_10;
	}
	else
	{
		fl_RD_ct_mul_10_U16 = (UINT16)0;
		fl_HM_ct_mul_10_U16 = (UINT16)0;
	}


	if( (fl_HFS_value!=0) && (FALSE == ((RCAN_SIG_MISSING | RCAN_SIG_NVR_RCVD) & rcan_get_PJB_FrontScreenHtrStatus_status())))
	{
		fl_Battery_Voltage_with_ten_U16 = fl_Battery_Voltage_U8 * LOAD_CURRENT_MUL_10;
		fl_HFS_Current_U16 = LinearInterpolateWord((UINT16 *)&l_HFS_Battery_current_Table_U16[0][0],fl_Battery_Voltage_with_ten_U16);
	}
	else
	{
		fl_HFS_Current_U16 = (UINT16)0;
	}


	fl_PTCA_current_U8 = LinearInterpolateByte((UINT8 *)&l_PTC_Battery_current_A_Table_U8[0][0],(fl_Battery_Voltage_U8));
	fl_PTCB_current_U8 = LinearInterpolateByte((UINT8 *)&l_PTC_Battery_current_B_Table_U8[0][0],(fl_Battery_Voltage_U8));

	if(FALSE != HW_O_PTC_HTR_A_IS_ACTIVE())
	{
		if(FALSE != HW_O_PTC_HTR_B_IS_ACTIVE())
		{
			/* both are active */
			fl_PTC_current_U8 = fl_PTCA_current_U8 + fl_PTCB_current_U8;
		}
		else
		{
			/* A alone active */
			fl_PTC_current_U8 = fl_PTCA_current_U8;
		}
	}
	else if(FALSE != HW_O_PTC_HTR_B_IS_ACTIVE())
	{
		/* B alone active */
		fl_PTC_current_U8 = fl_PTCB_current_U8;
	}
	else
	{
		/* Both are inactive: Set 0 */
		fl_PTC_current_U8 = (UINT16)0;
	}

	fl_PTC_ct_mul_100_U16 = fl_PTC_current_U8 * LOAD_CURRENT_MUL_100;

	/*Current calculation - l_LoadCtrlCurrAct_U8 to be transmitted in 0x4E3 message*/

	fl_Total_current_U16 = (UINT16)(fl_PTC_ct_mul_100_U16 + fl_HFS_Current_U16 + (fl_RD_ct_mul_10_U16 +fl_HM_ct_mul_10_U16));

	fl_PTC_ct_mul_100_U16 =     fl_Total_current_U16 % LOAD_CURRENT_MUL_100;

	if (    fl_PTC_ct_mul_100_U16 >= LOAD_CURRENT_ROUNDOFF)
	{
		fl_Total_current_U16 = fl_Total_current_U16+LOAD_CURRENT_ADD_ONE;
	}

	l_LoadCtrlCurrAct_U8 =  (UINT8)(fl_Total_current_U16/LOAD_CURRENT_MUL_100);

}
/****************************************************************************
Function Name     : FLOADMGT_LoadCtrlCurrAct_U8(void)

Description       : Transmitted Calculated current value through LoadCtrlCurrAct.

Invocation        : Called by fdiag to read load current through DID EEBC and thscan to transmit the value.

Parameters        : None.

Return Type       : UINT8

Global Variables  : None.

External interface: None.

Critical Section  : None.
******************************************************************************/
UINT8 FLOADMGT_LoadCtrlCurrAct_U8(void)
{
	return(l_LoadCtrlCurrAct_U8);
}
/****************************************************************************
Function Name     : FLOADMGT_diag_control(void)

Description       : LoadCtrlCurrAct controlled through diag command.

Invocation        : Called by fdiag.

Parameters        : UINT8 fload_diag_current_ctrl_val_U8 value given through command.

Return Type       : None.

Global Variables  : None.

External interface: None.

Critical Section  : None.
******************************************************************************/
void FLOADMGT_diag_control(UINT8 fload_diag_current_ctrl_val_U8)
{
	l_LoadCtrlCurrAct_U8 = fload_diag_current_ctrl_val_U8;
}
/****************************************************************************
Function Name     : floadmgt_diag_Release(void)

Description       :Release control of  LoadCtrlCurrAct .

Invocation        : Called by fdiag.

Parameters        : NIL

Return Type       : None.

Global Variables  : None.

External interface: None.

Critical Section  : None.
******************************************************************************/
void FLOADMGT_diag_Release()
{
	l_LoadCtrlCurrAct_U8 =  LOAD_CURRENT_DEFAULT_VALUE;
}
/****************************************************************************
Function Name     : FLOADMGT_get_PTC_Deactivate_status

Description       : Lets the PTC module know whether PTC heaters need to be deactivated since ELC = 1 or
                    SS defrost delay timer is running.

Invocation        : Invoked by fptc.c

Parameters        : None.

Return Type       : None.

Critical Section  : None.

******************************************************************************/
BOOLEAN FLOADMGT_get_PTC_Deactivate_status(void)
{
	UINT8 fl_SS_defroster_timeout_U8;
	BOOLEAN fl_ptc_deactivte_sts_bool = FALSE;
	/* Check for NVM Value of SS defrost delay timer boundary conditions */
	if(  NVM_SS_Defroster_time > (UINT8)60 )
	{
		/* Default Value */
		fl_SS_defroster_timeout_U8 = DEFROSTER_DEFAULT_VALUE;
	}
	else
	{
		fl_SS_defroster_timeout_U8 = NVM_SS_Defroster_time;
	}

	if((ELC_TURN_OFF_HFS_HRS_PTC == l_ElectricLoadControl_U8)||
		(l_defroster_timer_U8 <= fl_SS_defroster_timeout_U8))
	{
		fl_ptc_deactivte_sts_bool = TRUE;
	}
	return(fl_ptc_deactivte_sts_bool);
}
/****************************************************************************
Function Name     : FLOADMGT_validate_NVM_TSWITCHOFF

Description       : Lets the PTC module know whether PTC heaters need to be deactivated since ELC = 1 or
                    SS defrost delay timer is running.

Invocation        : Invoked by fptc.c

Parameters        : None.

Return Type       : None.

Critical Section  : None.

******************************************************************************/
static UINT16 FLOADMGT_validate_NVM_TSWITCHOFF(void)
{
	UINT16 fl_ret_val_U16;
	if(NVM_TSwitchOffClimateLoads_U8 > TSWITCHOFFCLIMATELOADS_MAX_VAL)
	{
		/* Default value 3 sec multiplied by 250 */
		fl_ret_val_U16 = TSWITCHOFFCLIMATELOADS_DEF_VAL;
	}
	else
	{
		fl_ret_val_U16 = (UINT16)(((UINT16)NVM_TSwitchOffClimateLoads_U8) * TSWITCHOFF_TIMER_COUNT_MULFACTOR);
	}
	return(fl_ret_val_U16);
}

/*End of File*/
/*****************************************************************************
*   for each change to this file, be sure to record:                         *
*      1.  who made the change and when the change was made                  *
*      2.  why the change was made and the intended result                   *
*   Following block needs to be repeated for each change
******************************************************************************
*   Note: In the traceability column we need to trace back to the Design Doc.*
*   For the initial version it is traced to the Design Document section.     *
*   For further changes it shall trace to the source of the change which may *
*   be SPSS/SCR/Defect details(Defect may be Testing/validation defect)/Any  *
*   other reason                                                             *
******************************************************************************/
/*---------------------------------------------------------------------------
Date              : 11/12/06
CDSID             : sdinesh
Traceability      : 08B2YYIC_Load_Management_DD.doc
Change Description: Initial Version.
-----------------------------------------------------------------------------*/

