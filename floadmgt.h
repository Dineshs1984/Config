/*****************************************************************************

File Name        :  floadmgt.h
Module Short Name:  LoadMgt
VOBName          :  2012.5_FORD_B299MCA_IC
Author           :  sdinesh
Description      :  This module is responsible for balancing the available power
                    against climatic loads.The cluster will monitor available power
                    and current demands on the engine to determine which loads can
                    be activated/deactivated.
Organization     :  Driver Information Software Section,
                    xxxxxxxx
Compiler Name    :  COSMIC V4.7.7
Target Processor :  MC9S12XEQ384
******************************************************************************/

#ifndef FLOADMGT_H
#define FLOADMGT_H

#ifndef FLOADMGT_C
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

/* 02 Load Management function turned off */
#define LOAD_MANAGEMENT_OFF           ((UINT8)2)

/* QF Value checked for ECMEstimatedAmbTempQF */
#define QF_VALUE_THREE                   ((UINT8)3)
#define QF_VALUE_TWO                     ((UINT8)2)

/*****************************************************************************
*                                 Global Function Prototypes                 *
******************************************************************************/

/* Function prototype for handling Cold Init */
void FLOADMGT_KSColdInit(void);

/* Function prototype for handling Warm Init */
void FLOADMGT_KSWarmInit(void);

/* Function prototype for handling Wakeup */
void FLOADMGT_KSWakeup(void);

/* Called in Round Robin */
EXTERN void FLOADMGT_KSRRobin(void);

/* Called in fstmgr.cfg */
EXTERN void FLOADMGT_Transout_NORMAL(void);

/* Function will return ADP Value for CAN */
EXTERN UINT32 FLOADMGT_get_AvailableDeltaPower_U32(void);

/* Function will return output status LoadCutOff Value */
EXTERN UINT8 FLOADMGT_get_LoadCutOff_U8(void);

/* Function will return output status of HFS load */
EXTERN UINT8 FLOADMGT_get_LM_HFSReq_U8(void);

/* Function will return output status of HRS load */
EXTERN UINT8 FLOADMGT_get_LM_HRSReq_U8(void);

/* called in transinto Sleep */
EXTERN void FLOADMGT_KSSleep(void);

/* Function will return ADP Value for PTC */
EXTERN SINT16 FLOADMGT_get_AvailableDeltaPower_S16(void);

/* Called in Trans into OFF */
EXTERN void FLOADMGT_Transinto_OFF(void);

/* Called in thscan_update_tx_msg_0x4E3 */
EXTERN UINT8 FLOADMGT_LoadCtrlCurrAct_U8(void);

/*Called by fdiag */
EXTERN void FLOADMGT_diag_control(UINT8 fload_diag_current_ctrl_val_U8);

/*Called by fdiag */
EXTERN void FLOADMGT_diag_Release(void);

/*Called by fptc to know whether PTC heaters need to be deactivated*/
EXTERN BOOLEAN FLOADMGT_get_PTC_Deactivate_status(void);


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
Date              : 04/12/06
CDSID             : sdinesh
Traceability      : 08B2YYIC_LoadManagement_SRS.doc
Change Description: Initial Version.
-----------------------------------------------------------------------------*/
