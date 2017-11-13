/*****************************************************************************

File Name        :  fpdc.c
Module Short Name:  PDC Chime Client
VOBName          :  2012.5_ford_b299_mca_ic
Author           :  sdinesh
Description      :  This module will Display the GearPosition in the cluster
                    based on the signals from TCM.
Organization     :  Driver Information Software Section,
                    xxxxx
 List the Target processor with the variant information
----------------------------------------------------------------------------
Compiler Name    :  IAR 2.44B
Target Processor :  MC9S12HZ256
******************************************************************************/

#define FPDC_C

/*****************************************************************************
*                                 System Includes                            *
******************************************************************************/
#include "system.h"
#include "swtmr.h"
/*****************************************************************************
*                                 Project Includes                           *
******************************************************************************/
#include "il_inc.h"
#include "fpdc.h"
#include "rcan.h"
#include "nvmdef.h"

/*****************************************************************************
*                              File Scope Prototypes                         *
******************************************************************************/

/*****************************************************************************
*                                 Constants                                  *
*----------------------------------------------------------------------------*
* Declaration shall be followed by a comment that gives the following info.  *
* about the constant.                                                        *
* purpose, unit and resolution                                               *
******************************************************************************/
/*****************************************************************************
*                                 Macro Definitions                          *
*----------------------------------------------------------------------------*
* Definition of macro shall be followed by a comment that explains the       *
* purpose of the macro.                                                      *
******************************************************************************/

#define SPEAK_REAR        ((UINT8) 0x01)  /* Bit0 PAStimegap- speaker rear */
#define SPEAK_FRONT       ((UINT8) 0x02)  /* Bit1 PAStimegap- speaker front */
#define SPEAKER_PDC_FRONT         ((UINT8) 0x01)  /* PDC alternating chime */
#define SPEAKER_PDC_REVERSE       ((UINT8) 0x00)  /* PDC alternating chime */

#define FRONT_LEFT_SPEAKER         ((UINT8) 0x01)  /* Bit0 gw_speaker_area_rq- front left */
#define FRONT_RIGHT_SPEAKER        ((UINT8) 0x02)  /* Bit1 gw_speaker_area_rq- front right */
#define REAR_LEFT_SPEAKER          ((UINT8) 0x04)  /* Bit2 gw_speaker_area_rq- rearleft speaker */
#define REAR_RIGHT_SPEAKER         ((UINT8) 0x08)  /* gw_speaker_area_rq- rearright speaker */
#define ALL_SPAEAKER               ((UINT8) 0x10)  /* gw_speaker_area_rq- all speaker */

#define ATTENUATION_TIMER          ((NVM_SS_PDC_Attenuation_Timer_CONFIG_U8 *20)/4)   /* stop_attenuation_timer- 2000msec */
#define ALTERNATING_CHIME_TIMER    ((NVM_SS_PDC_Alternating_Chime_timer_CONFIG_U8*20)/4)   /* PDC alternating chime- 500msec */
#define HEARTBEAT_DISTANCE_CHIME   ((NVM_SS_PDC_Heartbeat_Timer_CONFIG_U8 * 40)/4)  /* Heart beat timer for Pdc distance chime -4000 ms*/

/* The following macros are used for the values of the output signals */
#define SIGNAL_VALUE00                  ((UINT8) 0x0000)
#define SIGNAL_VALUE0                   ((UINT8) 0x00)
#define SIGNAL_VALUE1                   ((UINT8) 0x01)
#define SIGNAL_VALUE2                   ((UINT8) 0x02)
#define SIGNAL_VALUE3                   ((UINT8) 0x03)
#define SIGNAL_VALUE5                   ((UINT8) 0x05)                      
#define SIGNAL_VALUE7                   ((UINT8) 0x07)
#define SIGNAL_VALUE8                   ((UINT8) 0x08)
#define SIGNAL_VALUE9                   ((UINT8) 0x09)
#define SIGNAL_VALUEC                   ((UINT8) 0x0C)
#define SIGNAL_VALUEF                   ((UINT8) 0x0F)

#define INFINITE                        ((UINT16) 0x3FF)
#define TRIPLE                          ((UINT8) 0x04)
#define SINGLE                          ((UINT8) 0x02)
#define OFF                             ((UINT8) 0x01)
#define SAMPLE                          ((UINT8) 0x01)
#define MAIN_VOL_ATT_STOP               ((UINT8) 0x02)

#define VALUE_640         ((UINT16) 0x280)
#define VALUE_20          ((UINT8) 0x14)
#define VALUE_0           ((UINT8) 0x00)
#define VALUE1F           ((UINT8) 0x1F)
#define VALUE_10          ((UINT8) 0x0A)
/*PASActive inactive/failure condition macros*/
#define AllSENSOFFDUEFAILUREMODE        ((UINT8) 0x09)
#define ALLSENSOFF                      ((UINT8) 0x00)
#define ALLSENSOFF_A                    ((UINT8) 0x0A)
#define ALLSENSOFF_B                    ((UINT8) 0x0B)
/*System status*/
#define ACTIVE            ((UINT8) 0x02)
#define FAILURE           ((UINT8) 0x01)
#define INACTIVE          ((UINT8) 0x00)
#define MALFUNCTION       ((UINT8) 0x03)

/*Cime state*/
#define ZERO_DURATION     ((UINT8) 0x01)
#define ALTERNATE         ((UINT8) 0x02)
#define DISTANCE          ((UINT8) 0x03)

/*Sinam status */
#define MSG_MISSING       ((UINT8) 0x04)


/*****************************************************************************
*                                 Manifest Constants                         *
*----------------------------------------------------------------------------*
* Definition of Manifest constant shall be followed by a comment that        *
* explains the purpose of the constant.                                      *
******************************************************************************/


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
* purpose, critica l section, unit, resolution, Valid Range and ValidityCheck *
******************************************************************************/
/*
 Purpose          : To store the value of signal Gw_speaker_area send in 0x1C0.
 Critical section : None
 Unit             : None
 Resolutiion      : None
 Valid Range      : 0x00 - 0x0F
 Validity check   : No
*/
static UINT8 l_Gw_speaker_area_U8;
/*
 Purpose          : To store the value of signal Gw_ChimeType_Rq send in 0x1C0.
 Critical section : None
 Unit             : None
 Resolutiion      : None
 Valid Range      : 0x00-0x01
 Validity check   : No
*/
static UINT8 l_Gw_ChimeType_Rq_U8;
/*
 Purpose          : To store the value of signal Gw_ChimeDuration send in 0x1C0.
 Critical section : None
 Unit             : ms
 Resolutiion      : 10
 Valid Range      : 0 - 10000
 Validity check   : No
*/
static UINT16 l_Gw_ChimeDuration_U16;
/*
 Purpose          : To store the value of signal Gw_PauseDuration send in 0x1C0.
 Critical section : None
 Unit             : None
 Resolutiion      : None
 Valid Range      : 0x00-0x
 Validity check   : No
*/
static UINT8 l_Gw_PauseDuration_U8;
/*
 Purpose          : To store the value of signal Gw_ChimeMode_Rq send in 0x1C0.
 Critical section : None
 Unit             : ms
 Resolutiion      : 10
 Valid Range      : 0 - 2500
 Validity check   : No
*/
static UINT8 l_Gw_ChimeMode_Rq_U8;
/*
 Purpose          : To store the value of signal Gw_MainVolAtt_Rq send in 0x1C0.
 Critical section : None
 Unit             : None
 Resolutiion      : 1
 Valid Range      : 0x00-0x03
 Validity check   : No
*/
static UINT8 l_Gw_MainVolAtt_Rq_U8;
/*
 Purpose          : To store the value of signal Gw_ChimeOccur_Rq send in 0x1C0.
 Critical section : None
 Unit             : None
 Resolutiion      : 1
 Valid Range      : 0x00-0x07
 Validity check   : No
*/
static UINT8 l_Gw_ChimeOccur_Rq_U8;
/*
 Purpose          : To store the value of signal Gw_ChimeTypeDat send in 0x1C0.
 Critical section : None
 Unit             : None
 Resolutiion      : 1
 Valid Range      : 0 - 16383
 Validity check   : No
*/
static UINT16 l_Gw_ChimeTypeDat_U16;
/*
 Purpose          : This variable indicates whether to transmit rear or front speaker parameters
                    while playing alternating chime.
 Critical section : None
 Unit             : None
 Resolutiion      : None
 Valid Range      : 0x00-Reverse ; 0x01-Front
 Validity check   : No
*/
static UINT8 l_speaker_U8;
/*
 Purpose          : This holds the value of chime duration ( distance between vehicle and obstacle)
                    PASTimegap[2]-[6] contains chime duration.
 Critical section : None
 Unit             : None
 Resolutiion      : None
 Valid Range      : 0x00-0x1F
 Validity check   : No
*/
static UINT8 l_chime_speak_duration_U8;
/*
 Purpose          : To store the last recieved value of CAN signal PASTimeGap
 Critical section : None
 Unit             : None
 Resolutiion      : None
 Valid Range      : 0x00 to 0x7F
 Validity check   : No
*/
static UINT8 l_last_pastimegap_U8;
/*
 Purpose          : This variable indicates that attenuation start (0x01) command was send to audio unit,
                    hence we will have to send attenuation stop after attenuation timer expiry
 Critical section : None
 Unit             : None
 Resolutiion      : None
 Valid Range      : 0x00-0x01
 Validity check   : No
*/
static BOOLEAN l_attenuation_start_transmitted_bool;
/*
 Purpose          : This variable indicates the attenuation timer is started ,hence cluster needs to check
                    this timer expiry to send stop command
 Critical section : None
 Unit             : None
 Resolutiion      : None
 Valid Range      : 0x00-0x01
 Validity check   : No
*/
static BOOLEAN l_send_stop_attenuation_bool;
/*
 Purpose          : Variable to hold the status of call back function to send malfucntion chime frame.
 Critical section : None
 Unit             : None
 Resolutiion      : None
 Valid Range      : 0x00-0x03
 Validity check   : No
*/
static UINT8 l_failure_chime_counter_U8;
/*
 Purpose          : Variable to hold previous state of the system ,as per the CAN input PASActive.
 Critical section : None
 Unit             : None
 Resolutiion      : None
 Valid Range      : 0x00-Failure,0x01-Inactive,0x02-Active
 Validity check   : No
*/
static UINT8 l_pdc_chime_active_bool;

/*
 Purpose          : Variable to hold previous state of the system ,as per the CAN input PASActive.
 Critical section : None
 Unit             : None
 Resolutiion      : None
 Valid Range      : 0x00-Failure,0x01-Inactive,0x02-Active
 Validity check   : No
*/
static UINT8 l_pas_system_sts_U8;
/*
 Purpose          : Variable to hold Attenuation timer status.
 Critical section : None
 Unit             : None
 Resolutiion      : None
 Valid Range      : TIMER_RUNNING, TIMER_EXPIRED
 Validity check   : No
*/
static UINT8 l_pdc_attenuation_timer_status;
/*
 Purpose          : Variable to hold chime timer status.
 Critical section : None
 Unit             : None
 Resolutiion      : None
 Valid Range      : TIMER_RUNNING, TIMER_EXPIRED
 Validity check   : No
*/
static UINT8 l_pdc_chime_timer_status;
/*
 Purpose          : Variable to hold the transmission status.
 Critical section : None
 Unit             : None
 Resolutiion      : None
 Valid Range      : 0 - 1
 Validity check   : No
*/
static BOOLEAN l_prv_chime_rq_confirm;

/*****************************************************************************
*                              File Scope Prototypes                         *
******************************************************************************/

/*****************************************************************************
*                   Functions                                                *
******************************************************************************/
static void fpdc_Alternating_Chime(void);
static void fpdc_Distance_Chime(void);
static void fpdc_Send_Parking_Aid_Msg(void);
static void fpdc_Stop_Attenuation_Timer(void);
static UINT8 fpdc_Find_Pas_System_Status(UINT8);
static void fpdc_Send_Chime_Duration_Zero_frame(void);

/****************************************************************************
Function Name     : fpdc_KSColdInit

Description       : This is the Cold Initialisation for the PDC module

Invocation        : This function is invoked by Kernel Scheduler

Parameters        : void

Return Type       : void

Critical Section  : None

******************************************************************************/
void fpdc_KSColdInit(void)
{
    fpdc_KSWakeUp();
}
/****************************************************************************
Function Name     : fpdc_KSWarmInit

Description       : This is the Warm Initialisation for the PDC module

Invocation        : This function is invoked by Kernel Scheduler

Parameters        : void

Return Type       : void

Critical Section  : None
******************************************************************************/
void fpdc_KSWarmInit(void)
{
    fpdc_KSWakeUp();
}
/****************************************************************************
Function Name     : fpdc_KSWakeUp

Description       : This is the Wake up process from sleep for the PDC module

Invocation        : This function is invoked by Kernel Scheduler

Parameters        : void

Return Type       : void

Critical Section  : None

******************************************************************************/
void fpdc_KSWakeUp(void)
{
    l_last_pastimegap_U8 = VALUE_0;
    l_failure_chime_counter_U8 = VALUE_0;
    l_speaker_U8 = SPEAKER_PDC_FRONT;
    /*0x1C0 send variables*/
    l_Gw_speaker_area_U8 = SIGNAL_VALUE0;
    l_Gw_ChimeType_Rq_U8=SIGNAL_VALUE0;
    l_Gw_ChimeDuration_U16=SIGNAL_VALUE0;
    l_Gw_PauseDuration_U8=SIGNAL_VALUE0;
    l_Gw_ChimeMode_Rq_U8=SIGNAL_VALUE0;
    l_Gw_MainVolAtt_Rq_U8=SIGNAL_VALUE0;
    l_Gw_ChimeOccur_Rq_U8=SIGNAL_VALUE0;
    l_Gw_ChimeTypeDat_U16=SIGNAL_VALUE00;

    l_chime_speak_duration_U8 = VALUE_0;
    l_send_stop_attenuation_bool = FALSE;
    l_attenuation_start_transmitted_bool = FALSE;
    l_pdc_chime_active_bool=FALSE;
    l_pas_system_sts_U8 = FALSE;
    l_pdc_attenuation_timer_status=SIGNAL_VALUE0;
    l_prv_chime_rq_confirm = TRUE;

    KernelClear13BitTimer(PDC_PASTIMEGAP_KSWTIMER);
    KernelClear13BitTimer(PDC_ATTENUATION_KSWTIMER);
}
/****************************************************************************
Function Name     : fpdc_transinto_ABNVOLT

Description       : This is the abnormal voltage routine for the PDC module

Invocation        : This function is invoked by Kernel Scheduler

Parameters        : void

Return Type       : void

Critical Section  : None

******************************************************************************/
void fpdc_transinto_ABNVOLT(void)
{
    fpdc_KSWakeUp();
}
/****************************************************************************
Function Name     : fpdc_chime_gateway_KSRRobin

Description       : PDC message 0x1B0 processed in this RR and 0x1C0 transmitted accordingly.

Invocation        : This function is invoked by DI kernel from sched.cfg

Parameters        : void

Return Type       : void

Critical Section  : None
******************************************************************************/
void fpdc_Chime_Parameter_Gateway_KSRRobin(void)
{
    UINT8 fl_pastimegap_U8;
    UINT8 fl_pasactive_U8;
    UINT8 fl_pas_system_sts_U8;
    UINT8 fl_chime_speak_duration_U8;
    UINT8 fl_pasactive_sts_U8;

    fl_pastimegap_U8 = rcan_get_PASTimeGap_value();
    fl_pasactive_U8 = rcan_get_PASActive_value();
    fl_pasactive_sts_U8 = rcan_get_PASActive_status();
    l_pdc_attenuation_timer_status=KernelCheck13BitTimer(PDC_ATTENUATION_KSWTIMER);
    l_pdc_chime_timer_status=KernelCheck13BitTimer(PDC_PASTIMEGAP_KSWTIMER);

    l_last_pastimegap_U8 = fl_pastimegap_U8;
    /* Check whether the previous Transmission is done */
    if(FALSE != l_prv_chime_rq_confirm)
    {
        if((FALSE == TESTBIT(fl_pasactive_sts_U8,RCAN_SIG_NVR_RCVD|RCAN_SIG_MISSING)))
        {
            fl_pas_system_sts_U8 = fpdc_Find_Pas_System_Status(fl_pasactive_U8);

            /* PASActive = 0x01 to 0x08 or 0x0C : System status = ACTIVE*/
            if( ACTIVE == fl_pas_system_sts_U8 )
            {
                l_pas_system_sts_U8= ACTIVE;

                /* Chime duration = pastimegap : Bit2 to Bit6*/
                fl_chime_speak_duration_U8 = ((fl_pastimegap_U8 >> SIGNAL_VALUE2) & (VALUE1F));

                /* When the Pas time gap duration is Zero or when both the inputs are  PASTimeGap[0] = 0(Rear Speaker) and 
                PASTimeGap[1] = 0 (Front Speaker).Send the Zero duration frame and stop the attenuation */
                if((VALUE_0 == fl_chime_speak_duration_U8)||((!(TESTBIT(fl_pastimegap_U8,SPEAK_FRONT))) && (!(TESTBIT(fl_pastimegap_U8,SPEAK_REAR)))))
                {
                    if(l_pdc_chime_active_bool!=ZERO_DURATION)
                    {
                        l_pdc_chime_active_bool=ZERO_DURATION;
                        /* If system status is changed to inactive*/
                        fpdc_Send_Chime_Duration_Zero_frame();
                    }
                    else
                    {
                        fpdc_Stop_Attenuation_Timer();
                    }
                }
                /* If PASTimeGap[0] = 1(Rear Speaker) AND PASTimeGap[1] = 1 (Front Speaker)-Play alternating chime*/
                else if( (TESTBIT(fl_pastimegap_U8,SPEAK_FRONT)) && (TESTBIT(fl_pastimegap_U8,SPEAK_REAR)))
                {
                    /*This variable to indicate that Main Vol Att has been send with value = 0x01 - Start Attenuation*/
                    l_attenuation_start_transmitted_bool = TRUE;

                    if (l_pdc_chime_active_bool !=ALTERNATE)
                    {
                        fpdc_Alternating_Chime();
                        l_pdc_chime_active_bool = ALTERNATE;
                        /*Clear the timer it was previously used for distance chime*/
                        KernelStart13BitTimer(TIMER_13BIT_BASE_4MS,ALTERNATING_CHIME_TIMER,PDC_PASTIMEGAP_KSWTIMER);
                    }
                    else
                    {
                        if(TIMER_RUNNING != l_pdc_chime_timer_status)
                        {
                            /*If duration remains unchanged send 0x1C0 every 4 sec ( Heart beat timer)*/
                            fpdc_Alternating_Chime();
                            KernelStart13BitTimer(TIMER_13BIT_BASE_4MS,ALTERNATING_CHIME_TIMER,PDC_PASTIMEGAP_KSWTIMER);
                        }
                    }
                }
                /* If PASTimeGap[0] = 1(Rear Speaker) OR PASTimeGap[1] = 1 (Front Speaker)-Play distance chime*/
                else if( (TESTBIT(fl_pastimegap_U8,SPEAK_FRONT)) || (TESTBIT(fl_pastimegap_U8,SPEAK_REAR)))
                {
                    /*To start the atlternating chime with front speaker*/
                    l_speaker_U8 = SPEAKER_PDC_FRONT;

                    /*This variable to indicate that Main Vol Att has been send with value = 0x01 - Start Attenuation*/
                    l_attenuation_start_transmitted_bool = TRUE;

                    /* Chime duration = pastimegap : Bit2 to Bit6*/

                    if((fl_chime_speak_duration_U8!= l_chime_speak_duration_U8)||(l_pdc_chime_active_bool!=DISTANCE))
                    {
                        l_pdc_chime_active_bool = DISTANCE;
                        l_chime_speak_duration_U8= fl_chime_speak_duration_U8;
                        /*Since duration changed send the values immediately and restart heart beat timer*/
                        fpdc_Distance_Chime();
                        KernelStart13BitTimer(TIMER_13BIT_BASE_4MS,HEARTBEAT_DISTANCE_CHIME,PDC_PASTIMEGAP_KSWTIMER);
                    }
                    else
                    {
                        if(TIMER_RUNNING != l_pdc_chime_timer_status)
                        {
                            /*If duration remains unchanged send 0x1C0 every 4 sec ( Heart beat timer)*/
                            fpdc_Distance_Chime();
                            KernelStart13BitTimer(TIMER_13BIT_BASE_4MS,HEARTBEAT_DISTANCE_CHIME,PDC_PASTIMEGAP_KSWTIMER);
                        }
                    }

                }
                else
                {
                    fpdc_Stop_Attenuation_Timer();
                }

            }
            /* If system status is changed to inactive*/
            else if (INACTIVE == fl_pas_system_sts_U8)
            {
                /*PDC_Chime Reset*/
                l_pdc_chime_active_bool=FALSE;
                if(l_pas_system_sts_U8!=INACTIVE)
                {
                    l_pas_system_sts_U8= INACTIVE;
                    /* If system status is changed to inactive*/
                    fpdc_Send_Chime_Duration_Zero_frame();

                }
                else
                {
                    fpdc_Stop_Attenuation_Timer();
                }
            }
            /* If system status is changed to failure */
            else if (FAILURE == fl_pas_system_sts_U8)
            {
                if((DISTANCE == l_pdc_chime_active_bool)||(ALTERNATE == l_pdc_chime_active_bool))
                {
                    /*PDC_Chime Reset*/
                    l_pdc_chime_active_bool=FALSE;
                    l_pas_system_sts_U8= FAILURE;
                    /* If system status is changed to inactive*/
                    fpdc_Send_Chime_Duration_Zero_frame();
                }
                else if (l_pas_system_sts_U8!=MALFUNCTION)
                {
                    if(((l_failure_chime_counter_U8 >= SIGNAL_VALUE2)&&(FAILURE == l_pas_system_sts_U8))||
                        ( FAILURE != l_pas_system_sts_U8))
                    {
                        /*Malfunction_Chime*/
                        l_pas_system_sts_U8= MALFUNCTION;
                        l_failure_chime_counter_U8 = SIGNAL_VALUE0;

                        l_Gw_ChimeOccur_Rq_U8 = TRIPLE;
                        l_Gw_ChimeDuration_U16 = SIGNAL_VALUE00;
                        l_Gw_PauseDuration_U8 = SIGNAL_VALUE0;
                        l_Gw_ChimeType_Rq_U8 = SAMPLE;
                        l_Gw_ChimeTypeDat_U16 = SIGNAL_VALUE9;
                        l_Gw_speaker_area_U8 = SIGNAL_VALUEF;
                        l_Gw_ChimeMode_Rq_U8 = SIGNAL_VALUE0;
                        l_Gw_MainVolAtt_Rq_U8 = SIGNAL_VALUE0;
                        fpdc_Send_Parking_Aid_Msg();
                    }
                    else
                    {
                        /* Wait for Call back function from FNOS to make Gw_ChimeOccur_Rq to be send as 0
                        before sending Malfunction frame*/
                    }

                }
                else
                {
                    fpdc_Stop_Attenuation_Timer();
                }

            }
            else
            {
                /*For QAC*/
            }
        }
        else if ((FALSE != TESTBIT(fl_pasactive_sts_U8,RCAN_SIG_MISSING)))
        {
            /*PDC_Chime Reset*/
            l_pdc_chime_active_bool=FALSE;
            if(l_pas_system_sts_U8!=MSG_MISSING)
            {
                l_pas_system_sts_U8=MSG_MISSING;
                /* If system status is changed to missing*/
                fpdc_Send_Chime_Duration_Zero_frame();

            }
            else
            {
                fpdc_Stop_Attenuation_Timer();

            }
        }
        else
        {
            /*Never received Do not send any Frame */
            /*Do NOthing */

        }
    }
    else
    {
        /* Process the next input only after getting 
        the confirmation callback for the previuos transmission */
    }
}
/****************************************************************************
Function Name     : PDC_Alternating_Chime

Description       : This function to send 0x1C0 every 500 ms with alternating chime parameters

Invocation        : This function is invoked by pdc_chime_client

Parameters        : void

Return Type       : void

Critical Section  : None        
******************************************************************************/
static void fpdc_Alternating_Chime(void)
{
    l_Gw_speaker_area_U8 = SIGNAL_VALUE0;
    if (SPEAKER_PDC_FRONT == l_speaker_U8)
    {
        l_speaker_U8 = SPEAKER_PDC_REVERSE;
        SETBIT(l_Gw_speaker_area_U8,(FRONT_LEFT_SPEAKER | FRONT_RIGHT_SPEAKER));
        l_Gw_ChimeTypeDat_U16 = NVM_SS_PDC_Beep_Frequeny_Front_CONFIG_U16;
    }
    else
    {
        l_speaker_U8 = SPEAKER_PDC_FRONT;
        SETBIT(l_Gw_speaker_area_U8,(REAR_LEFT_SPEAKER | REAR_RIGHT_SPEAKER));
        l_Gw_ChimeTypeDat_U16 = NVM_SS_PDC_Beep_Frequeny_Rear_CONFIG_U16;
    }

    l_Gw_ChimeType_Rq_U8 = SIGNAL_VALUE0;
    l_Gw_ChimeDuration_U16 = INFINITE;
    l_Gw_PauseDuration_U8 = SIGNAL_VALUE0;
    l_Gw_ChimeMode_Rq_U8 = SIGNAL_VALUE0;
    l_Gw_MainVolAtt_Rq_U8 = SIGNAL_VALUE1;
    l_Gw_ChimeOccur_Rq_U8 = SIGNAL_VALUE2;
    fpdc_Send_Parking_Aid_Msg();
}
/****************************************************************************
Function Name     : fpdc_Distance_Chime

Description       : This function to send 0x1C0 at every 4 sec with distance chime parameters

Invocation        : This function is invoked by pdc_chime_client

Parameters        : void

Return Type       : void

Critical Section  : None
******************************************************************************/
static void fpdc_Distance_Chime(void)
{
    l_Gw_ChimeType_Rq_U8 = SIGNAL_VALUE0;
    l_Gw_speaker_area_U8 = SIGNAL_VALUE0;
    if ( TESTBIT(l_last_pastimegap_U8,SPEAK_FRONT))
    {
        SETBIT(l_Gw_speaker_area_U8,(FRONT_LEFT_SPEAKER | FRONT_RIGHT_SPEAKER));
        l_Gw_ChimeTypeDat_U16 = NVM_SS_PDC_Beep_Frequeny_Front_CONFIG_U16;
    }
    /* if l_last_pastimegap_U8 - SPEAK_REAR (Bit 0) is active*/
    else
    {
        SETBIT(l_Gw_speaker_area_U8,(REAR_LEFT_SPEAKER | REAR_RIGHT_SPEAKER));
        l_Gw_ChimeTypeDat_U16 = NVM_SS_PDC_Beep_Frequeny_Rear_CONFIG_U16;
    }
    /* If chime Duration = 31*/
    if (VALUE1F == l_chime_speak_duration_U8)
    {
        l_Gw_ChimeDuration_U16 = INFINITE;
        l_Gw_PauseDuration_U8 = SIGNAL_VALUE0;
        l_Gw_ChimeMode_Rq_U8 = SIGNAL_VALUE0;
        l_Gw_MainVolAtt_Rq_U8 = SIGNAL_VALUE1;
        l_Gw_ChimeOccur_Rq_U8 = SIGNAL_VALUE2;

    }
    /* If chime Duration = 1-30*/
    else if((VALUE_0< l_chime_speak_duration_U8)&&(VALUE1F > l_chime_speak_duration_U8))
    {
        /*Pause duration has a resolution of 10 hence the value is divided by 10 and transmitted in CAN */
        l_Gw_PauseDuration_U8 = \
            (UINT8)((VALUE_640-((l_chime_speak_duration_U8 - SIGNAL_VALUE1) * (VALUE_20)))/VALUE_10);

        l_Gw_ChimeDuration_U16 = NVM_SS_PDC_Beep_Tone_Duration_CONFIG_U8;
        l_Gw_ChimeMode_Rq_U8 = SIGNAL_VALUE0;
        l_Gw_MainVolAtt_Rq_U8 = SIGNAL_VALUE1;
        l_Gw_ChimeOccur_Rq_U8 = SIGNAL_VALUE5;

    }
    else if ((VALUE_0 == l_chime_speak_duration_U8))
    {
        /* Common parameters updated here */
        l_Gw_ChimeDuration_U16 = SIGNAL_VALUE00;
        l_Gw_PauseDuration_U8 = SIGNAL_VALUE0;
        l_Gw_ChimeMode_Rq_U8 = SIGNAL_VALUE0;
        l_Gw_MainVolAtt_Rq_U8 = SIGNAL_VALUE0;
        l_Gw_ChimeType_Rq_U8 = SIGNAL_VALUE0;
        l_Gw_ChimeOccur_Rq_U8 = OFF;
        l_Gw_ChimeTypeDat_U16 = SIGNAL_VALUE00;
    }
    else
    {
        /*Do nothing*/
    }
    fpdc_Send_Parking_Aid_Msg();
}
/****************************************************************************
Function Name     : fpdc_Stop_Attenuation_Timer

Description       : This function to send attenuation stop command if attenuation start was send previously.

Invocation        : This function is invoked by pdc_chime_client

Parameters        : void

Return Type       : void

Critical Section  : None
******************************************************************************/
static void fpdc_Stop_Attenuation_Timer(void)
{
    /* Start the attenuation timer if we had previously send MainVolAtt = Start ( Start the attenuation)*/
    if( FALSE != l_attenuation_start_transmitted_bool)
    {
        KernelStart13BitTimer(TIMER_13BIT_BASE_4MS,ATTENUATION_TIMER,PDC_ATTENUATION_KSWTIMER);
        l_attenuation_start_transmitted_bool = FALSE;
        l_send_stop_attenuation_bool = TRUE;
    }
    /* On expiry of attenuation timer, send Gw_MainVolAtt_Rq = Stop in frame of Chime.Duration = 0*/
    else if( FALSE != l_send_stop_attenuation_bool)
    {
        if( TIMER_RUNNING != l_pdc_attenuation_timer_status)
        {
            l_Gw_speaker_area_U8 = SIGNAL_VALUE0;
            if ( TESTBIT(l_last_pastimegap_U8,SPEAK_FRONT))
            {
                SETBIT(l_Gw_speaker_area_U8,(FRONT_LEFT_SPEAKER | FRONT_RIGHT_SPEAKER));
            }
            if ( TESTBIT(l_last_pastimegap_U8,SPEAK_REAR))
            {
                SETBIT(l_Gw_speaker_area_U8,(REAR_LEFT_SPEAKER | REAR_RIGHT_SPEAKER));
            }
            l_Gw_ChimeTypeDat_U16 = SIGNAL_VALUE00;
            l_Gw_ChimeType_Rq_U8 = SIGNAL_VALUE0;
            l_Gw_ChimeOccur_Rq_U8 = OFF;
            l_Gw_MainVolAtt_Rq_U8 = MAIN_VOL_ATT_STOP;
            fpdc_Send_Parking_Aid_Msg();
            l_send_stop_attenuation_bool = FALSE;
        }
    }
    else
    {
        /*Once we send stop command , nothing to do until sensor detects another obstacle*/
    }
}
/****************************************************************************
Function Name     : fpdc_Send_Parking_Aid_Msg

Description       : This function updates the CAN tx buffer of 0x1C0 , if the values  written to 
                    MainVoltAtt or ChimeOccur_Rq are different from previous , message is send.

Invocation        : This function is invoked by pdc_chime_client

Parameters        : void

Return Type       : void

Critical Section  : None
******************************************************************************/
static void fpdc_Send_Parking_Aid_Msg(void)
{
    ILPutTxGw_ChimeType_Rq(l_Gw_ChimeType_Rq_U8);
    ILPutTxGw_SpeakerArea_Rq(l_Gw_speaker_area_U8);
    ILPutTxGw_ChimeDuration_Rq(l_Gw_ChimeDuration_U16);
    ILPutTxGw_PauseDuration_Rq(l_Gw_PauseDuration_U8);
    ILPutTxGw_ChimeMode_Rq(l_Gw_ChimeMode_Rq_U8);
    ILPutTxGw_ChimeTypeDat_Rq(l_Gw_ChimeTypeDat_U16);
    /*On Change signals based on these signals message is send */
    ILPutTxGw_MainVolAtt_Rq(l_Gw_MainVolAtt_Rq_U8);
    ILPutTxGw_ChimeOccur_Rq(l_Gw_ChimeOccur_Rq_U8);
    l_prv_chime_rq_confirm = FALSE;

}
/****************************************************************************
Function Name     : fpdc_Find_Pas_System_Status

Description       : Finds the system status , if PASActive = 0x00,0x0A or 0x0B status=Inactive
                    If 0x01-0x08 or 0x0C - Active, If 0x09 - Failure 

Invocation        : This function is invoked by pdc_chime_client

Parameters        : void

Return Type       : void

Critical Section  : None
******************************************************************************/
static UINT8 fpdc_Find_Pas_System_Status(UINT8 fl_pas_active_U8)
{
    UINT8 fl_pas_system_sts_U8;

    /*If PASActive = 0x00 or 0x0A or 0x0B , System status = INACTIVE */
    if((ALLSENSOFF == fl_pas_active_U8)||( ALLSENSOFF_A == fl_pas_active_U8)
        ||(ALLSENSOFF_B == fl_pas_active_U8))
    {
        fl_pas_system_sts_U8 = INACTIVE;
    }
    else if ((fl_pas_active_U8 <= SIGNAL_VALUE8)||(SIGNAL_VALUEC == fl_pas_active_U8))
    {
        fl_pas_system_sts_U8 = ACTIVE;
    }
    /* IF PASActive = 0x09 , system status = Failure : request a malfunction chime*/
    else if ( AllSENSOFFDUEFAILUREMODE == fl_pas_active_U8)
    {
        fl_pas_system_sts_U8 = FAILURE;
    }
    else
    {
        /*Maintain previous state for unused values*/
    }
    return(fl_pas_system_sts_U8);
}
/****************************************************************************
Function Name     : fpdc_Send_Chime_Duration_Zero_frame

Description       : This function sends message 0x1C0 when chime duration = 0 or 0x1B0 missing
                    or System status changes to inactive with values as per PDCDistance chime table
Invocation        : This function is invoked by pdc_chime_client

Parameters        : void

Return Type       : void

Critical Section  : None
******************************************************************************/
static void fpdc_Send_Chime_Duration_Zero_frame(void)
{
    KernelClear13BitTimer(PDC_PASTIMEGAP_KSWTIMER);
    /*To start the atlternating chime with front speaker*/
    l_speaker_U8 = SPEAKER_PDC_FRONT;

    /* Common parameters updated here */
    l_Gw_ChimeDuration_U16 = SIGNAL_VALUE00;
    l_Gw_PauseDuration_U8 = SIGNAL_VALUE0;
    l_Gw_ChimeMode_Rq_U8 = SIGNAL_VALUE0;
    l_Gw_MainVolAtt_Rq_U8 = SIGNAL_VALUE0;

    /* Gateway the PDC chime parameter corresponsing to past time gap = 0x00*/
    /*When 0x1B0 is missed , send last received speaker area to determine front or rear*/
    l_Gw_speaker_area_U8 = SIGNAL_VALUE0;
    if ( TESTBIT(l_last_pastimegap_U8,SPEAK_FRONT))
    {
        SETBIT(l_Gw_speaker_area_U8,(FRONT_LEFT_SPEAKER | FRONT_RIGHT_SPEAKER));
    }
    if ( TESTBIT(l_last_pastimegap_U8,SPEAK_REAR))
    {
        SETBIT(l_Gw_speaker_area_U8,(REAR_LEFT_SPEAKER | REAR_RIGHT_SPEAKER));
    }
    l_Gw_ChimeType_Rq_U8 = SIGNAL_VALUE0;
    l_Gw_ChimeOccur_Rq_U8 = OFF;
    l_Gw_ChimeTypeDat_U16 = SIGNAL_VALUE00;
    fpdc_Send_Parking_Aid_Msg();
}
/****************************************************************************
Function Name     : fpdc_Send_Chime_Duration_Zero_frame

Description       : Signal confirmation required for the other signals of msg 0x1co to be updated

Invocation        : This function is invoked by ilpar.c - confirmation call back

Parameters        : void

Return Type       : void

Critical Section  : None
******************************************************************************/
void ApplGw_ChimeOccur_RqSigConfirmation(void)
{
    if(SIGNAL_VALUE0 == ILGetTx_Gw_ChimeOccur_Rq())
    {
        l_prv_chime_rq_confirm = TRUE;
    }
    ILPutTxGw_MainVolAtt_Rq(SIGNAL_VALUE0);
    ILPutTxGw_ChimeOccur_Rq(SIGNAL_VALUE0);
    if( FAILURE == l_pas_system_sts_U8 )
    {
        l_failure_chime_counter_U8++;
    }
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
Date              :2-Feb-12
CDSID             :sdinesh
Traceability      :CQUCM00038065 - SCR_B232_V4_3_4.doc ,B299MCA_SRD_ver.3.5_D1_Baseline.pdf
Change Description:Initial version.
-----------------------------------------------------------------------------*/