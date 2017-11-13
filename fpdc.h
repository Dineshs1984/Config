/*****************************************************************************

File Name        :  fpdc.h 
Module Short Name:  FPDC
VOBName          :  2012.5_ford_b299_mca_ic
Author           :  sdinesh
Description      :  Park Distance Control (PDC) chime parameters gatewayed by cluster in MSCAN 0x1C0.
Organization     :  Driver Information Software Section,
                    xxxxxx
******************************************************************************/

#ifndef FPDC_H
#define FPDC_H

#ifndef FPDC_C
#define EXTERN  extern
#else
#define EXTERN
#endif

/*****************************************************************************
*                                 Global Constant Declarations               *
*----------------------------------------------------------------------------*
* Declaration shall be followed by a comment that gives the following info.  *
* about the constant.                                                        *
* Purpose, unit and resolution                                               *
******************************************************************************/

/*****************************************************************************
*                                 Global Macro Definitions                   *
*----------------------------------------------------------------------------*
* Definition of macro shall be followed by a comment that explains the       *
* purpose of the macro.                                                      *
******************************************************************************/

/*****************************************************************************
*                                 Type Declaration                           *
*----------------------------------------------------------------------------*
* Declaration of type shall be accompanied by a comment that explains the    *
* purpose and usage of the type.                                             *
******************************************************************************/

/*****************************************************************************
*                                 Global Variable Declarations               *
*----------------------------------------------------------------------------*
* Declaration shall be followed by a comment that gives the following info.  *
* about the variable.                                                        *
* purpose, critical section, unit, resolution, Valid Range and ValidityCheck *
******************************************************************************/

/*****************************************************************************
*                                 Global Function Prototypes                 *
******************************************************************************/
/*
    Function prototype for handling
    Cold Init
*/
EXTERN void fpdc_KSColdInit(void);

/*
    Function prototype for handling
    Warm Init
*/
EXTERN void fpdc_KSWarmInit(void);

/*
    Function prototype for handling
    Wakeup
*/
EXTERN void fpdc_KSWakeUp(void);
/*
    Function prototype for handling 
    Round Robin*/
EXTERN void fpdc_Chime_Parameter_Gateway_KSRRobin(void);
/*
    Function prototype for handling 
    Abnormal voltage condition*/
EXTERN void fpdc_transinto_ABNVOLT(void);

#undef EXTERN
#endif

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



