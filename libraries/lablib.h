/*
 * lablib.h
 *
 *  Created on: 17 ene. 2025
 *      Author: PMM x RSG
 *
 *  Indice: -> INCLUDES
 *          -> PARï¿½METROS DE CONTROL
 *             | Selecciona esquema de control
 *             | Pi de velocidad
 *             | Periodo de control
 *             | DC Bus
 *          -> MACROS GLOBALES
 *             | Par de polos de la mï¿½quina
 *             | Data loggin
 *             | PWM parameters
 *             | Encoder parameters
 *          -> MACROS DE FUNCIONES INLINE
 *             | Tiempos de control
 *             | Matriz de Clarke (PH->AB)
 *             | Matriz inv. de Clarke (AB->PH)
 *             | Matriz de Park (AB->DQ)
 *             | Matriz inv. de Park (DQ->AB)
 *             | Cost Function
 *             | Cambios de unidades
 *          -> MACROS ESPECï¿½FICAS PARA CADA ESQUEMA DE CONTROL
 *             | MFC
 *             | FOC
 *             | MPC
 *
 *
 *  NOTA:  - Las estructuras de datos y variables globales se definen en el archivo "data_type.h"
 *
 *
 */

#ifndef LABLIB_H
#define LABLIB_H

// ----------------------------------------------------------------------
//  Includes
// ----------------------------------------------------------------------
#include "f28x_project.h"
#include "F2838x_Device.h"
#include "math.h"
#include "IQmathLib.h"
#include <stdio.h>

// ----------------------------------------------------------------------
//  Definiciones de Parï¿½metros de Control
// ----------------------------------------------------------------------

// Selecciona el esquema de control
// [MODEL_FREE, FOC, MPC, MPCta, MV5_FCS]
#define MPCta    0x0

// PI de velocidad
#define Kp_w  ((float) 0.8)
#define Ki_w  ((float) 3)

#define CTRLF ((float) 50e3)    // [Hz] Control frequency
#define Vdc   ((float) 500)     // [V] DC bus

/*----------- Selecciï¿½n perfil velocidad -----------*/
// TRAPEZOIDAL
#define W_TRAP  (float) 0x0      // Trapezoidal selection
#define W_TRG  ((float) 500)     // [rpm] Speed target
#define SLP    ((float) 0.05)    // Slope for speed profile
#define T_CNST ((float) 5)       // Stationary time at target frequency
#define TLOAD  ((float) 80)     // [Nm]  Torque target
#define SLP_T  ((float) 0.01)    // Slope for torque profile

// DATASET
//#define W_DATASET              // Dataset selection


// TRANSIENT
//#define W_JUAN_TRANS            //w and torque come at the same time


/*----------- DATA-LOGGING PARAMETER SELECTION -----------*/
/*--- data-logging type selection ---*/
// [DATLOGDEF, DATLOGMPC, DATLOGMID, JUAN_TRANS}
#define DATLOGDEF
// .... if any other data-logging type is added, mirror it on lablib.c file (dataLogging(void)) ... //

#ifdef DATLOGDEF
    #define VAR   (Uint16) 6
#endif

#ifdef DATLOGMPC /*------ Be careful with memory overflow when setting LEN directive!  ------*/
    #define VAR   (Uint16) 9
#endif

#ifdef DATLOGMID
    #define VAR   (Uint16) 4
#endif

#ifdef JUAN_TRANS
    #define VAR   (Uint16) 5
#endif

/*----------- data-logging parameters -----------*/
#define DEC   (Uint16) 1
#define LEN   (Uint16) 1000

/*--- Mitigation of undesirable switching patterns weight ---*/
//#define MUS2  (float) 0.835

/*--- Radial modulation selection ---*/
//#define MOD_RAD

/*--- Model-Free control setup ---*/
#ifdef MODEL_FREE
    /*--- Cost function selection ---*/
    //#define CF_SCAL_PROD
    #define CF_MSE

    /*--- Rate of PWM period for measuring middle current for the OSDC ---*/
    #define RATE ((float) 0.25f)

    /*--- Normalization of the variables ---*/
    //#define GRAD_NORM

    /*--- Null compensation ---*/
    //#define NULL_COMP
    #ifdef NULL_COMP
        #define LEN_BUFF ((Uint16) 100U)
    #endif

    /*--- Scalar product cost function secondary subspaces weights ---*/
    #ifdef CF_SCAL_PROD
        #define KXY1_SP ((float) 0.1)
        #define KXY2_SP ((float) 0.1)
    #endif
#endif
// ----------------------------------------------------------------------
//  Macros Globales
// ----------------------------------------------------------------------

#define P     (float)  3  // Motor Pole Pairs

/*--------- Control parameters definition ----------*/
#define pi  3.141592653589
#define PI2 6.283185307178
#define hvfactor (float) 10.471975511966
#define Pul2rad  (float) 0.0007853
#define IQ_MAX   (float) 14


/*---------------- PWM parameters ------------------*/
#ifdef MV5_FCS
    #define PWM_PERIOD_CTRL  ((Uint32)(100000000/CTRLF))
#else
    #define PWM_PERIOD_CTRL  ((Uint32)(100000000*0.5/CTRLF))
#endif

#define PWM_PERIOD_Q2A   500
#define EPWM_MAX_DB   10     // 0.10 us de dead-band
#define EPWM_MIN_DB   10     // 0.10 us de dead-band


/*--------------- Encoder parameters ---------------*/
#define EQEP_SAME_DIR 0x0 // Direction selection
#define QEPCNTMAX    8000U
#define QEPUTOPRD   15000U//15000L   // Unit Out Time Period

#define velocfactor_x1_32 (float)        4908.738521 //X=1; SYSCLKOUT/32
#define velocfactor_x1_16 (float)        9817.477042 //X=1; SYSCLKOUT/16
#define velocfactor_x4_1  (float)        628318.5307 //X=4; SYSCLKOUT/1

#define velocfactor_x1_8  (float)        19634.95408
#define velocfactor_x16_1 (float)        2513274.122
#define velocfactor_x1_1  (float)        157079.6326

#define VSTEPMAX (float) 0.5//0.05 //0.05
#define QCPRDMAX 40000L
#define QCPRDMIN 10000L
#define Wm_MAX  (float) 110    // rad/seg
#define VELOCK  94247.77961
#define ang2vel 0.0001


// ----------------------------------------------------------------------
//  Macros funciones inline
// ----------------------------------------------------------------------


/*---------- Tiempos del periodo de control -----------*/
#define GET_LOC_TIME(timer) ((Uint32) (timer.RegsAddr->TIM.all))
#define GET_CPU_PRD(timer)  ((Uint32) (timer.RegsAddr->PRD.all)) // Cuentas del periodo de control


/*------------- Matriz de Clarke (PH->AB) --------------*/
// Componente alpha
#define CLARKE_ALPHA(D) ((float)(0.222 * D.A1 + 0.17 * D.A2 + 0.0386 * D.A3 - 0.111 * D.B1 + 0.0386 * D.B2 + 0.17 * D.B3 - 0.111 * D.C1 - 0.209 * D.C2 - 0.209 * D.C3))
// Componente beta
#define CLARKE_BETA(D) ((float)(0.143 * D.A2 + 0.219 * D.A3 - 0.192 * D.B1 - 0.219 * D.B2 - 0.143 * D.B3 + 0.192 * D.C1 + 0.076 * D.C2 - 0.076 * D.C3))
// Componente x1
#define CLARKE_X1(D) ((float)(0.222 * D.A1 + 0.0386 * D.A2 - 0.209 * D.A3 - 0.111 * D.B1 - 0.209 * D.B2 + 0.0386 * D.B3 - 0.111 * D.C1 + 0.17 * D.C2 + 0.17 * D.C3))
// Componente y1
#define CLARKE_Y1(D) ((float)(0.219 * D.A2 + 0.076 * D.A3 + 0.192 * D.B1 - 0.076 * D.B2 - 0.219 * D.B3 - 0.192 * D.C1 - 0.143 * D.C2 + 0.143 * D.C3))
// Componente x2
#define CLARKE_X2(D) ((float)(0.222 * D.A1 - 0.209 * D.A2 + 0.17 * D.A3 - 0.111 * D.B1 + 0.17 * D.B2 - 0.209 * D.B3 - 0.111 * D.C1 + 0.0386 * D.C2 + 0.0386 * D.C3))
// Componente y2
#define CLARKE_Y2(D) ((float)(0.076 * D.A2 - 0.143 * D.A3 - 0.192 * D.B1 + 0.143 * D.B2 - 0.076 * D.B3 + 0.192 * D.C1 - 0.219 * D.C2 + 0.219 * D.C3))


/*----------- Matriz inv. de Clarke (AB->PH) ------------*/
// Fase a1
#define INV_CLARKE_A1(D) ((float)(D.alpha + D.x1 + D.x2))
// Fase b1
#define INV_CLARKE_B1(D) ((float)(-0.5 * D.alpha - 0.866 * D.beta - 0.5 * D.x1 + 0.866 * D.y1 - 0.5 * D.x2 - 0.866 * D.y2))
// Fase c1
#define INV_CLARKE_C1(D) ((float)(-0.5 * D.alpha + 0.866 * D.beta - 0.5 * D.x1 - 0.866 * D.y1 - 0.5 * D.x2 + 0.866 * D.y2))

// Fase a2
#define INV_CLARKE_A2(D) ((float)(0.766 * D.alpha + 0.643 * D.beta + 0.174 * D.x1 + 0.985 * D.y1 - 0.94 * D.x2 + 0.342 * D.y2))
// Fase b2
#define INV_CLARKE_B2(D) ((float)(0.174 * D.alpha - 0.985 * D.beta - 0.94 * D.x1 - 0.342 * D.y1 + 0.766 * D.x2 + 0.643 * D.y2))
// Fase c2
#define INV_CLARKE_C2(D) ((float)(-0.94 * D.alpha + 0.342 * D.beta + 0.766 * D.x1 - 0.643 * D.y1 + 0.174 * D.x2 - 0.985 * D.y2))

// Fase a3
#define INV_CLARKE_A3(D) ((float)(0.174 * D.alpha + 0.985 * D.beta - 0.94 * D.x1 + 0.342 * D.y1 + 0.766 * D.x2 - 0.643 * D.y2))
// Fase b3
#define INV_CLARKE_B3(D) ((float)(0.766 * D.alpha - 0.643 * D.beta + 0.174 * D.x1 - 0.985 * D.y1 - 0.94 * D.x2 - 0.342 * D.y2))
// Fase c3
#define INV_CLARKE_C3(D) ((float)(-0.94 * D.alpha - 0.342 * D.beta + 0.766 * D.x1 + 0.643 * D.y1 + 0.174 * D.x2 + 0.985 * D.y2))


/*------------- Matriz de Park (AB->DQ) --------------*/
// Componente D
#define PARK_D(P,costh,sinth) ((float)(P.alpha*costh + P.beta*sinth))
// Componente Q
#define PARK_Q(P,costh,sinth) ((float)(-P.alpha*sinth + P.beta*costh))
// Componente x1
#define PARK_X1(P,costh,sinth) ((float)(P.x1*costh + P.y1*sinth))
// Componente y1
#define PARK_Y1(P,costh,sinth) ((float)(-P.x1*sinth + P.y1*costh))
// Componente x2
#define PARK_X2(P,costh,sinth) ((float)(P.x2*costh + P.y2*sinth))
// Componente y2
#define PARK_Y2(P,costh,sinth) ((float)(-P.x2*sinth + P.y2*costh))


/*----------- Matriz inv. de Park (DQ->AB) ------------*/
// Componente D
#define INV_PARK_AlPHA(P) ((float)(P.d*costheta - P.q*sintheta))
// Componente Q
#define INV_PARK_BETA(P) ((float)(P.d*sintheta + P.q*costheta))
// Componente x1
#define INV_PARK_X1(P) ((float)(P.x1*costheta - P.y1*sintheta))
// Componente y1
#define INV_PARK_Y1(P) ((float)(P.x1*sintheta + P.y1*costheta))
// Componente x2
#define INV_PARK_X2(P) ((float)(P.x2*costheta - P.y2*sintheta))
// Componente y2
#define INV_PARK_Y2(P) ((float)(P.x2*sintheta + P.y2*costheta))



/*---------------- Cost function ---------------------*/
// Mean Squared Error
#define CF_EVAL_MSE(D) ((float)(D.alpha*D.alpha + D.beta*D.beta + Kxy1*(D.x1*D.x1 + D.y1*D.y1) + Kxy2*(D.x2*D.x2 + D.y2*D.y2)))

// Scalar product
#ifdef CF_SCAL_PROD
    #define CF_EVAL_SCAL_PROD(v,i) ((float)(v.alpha*i.alpha + v.beta*i.beta + Kxy1_SP*(v.x1*i.x1 + v.y1*i.y1) + Kxy2_SP*(v.x2*i.x2 + v.y2*i.y2)))
#endif
/*------------- Cambios de unidades ------------------*/
#define RPM2RADS(rpm) ((float)(0.104719f*(rpm)))
#define NM2PU(Torque) ((float) 0.006667f*Torque)


/*------------- Modulación radial --------------------*/
#define MODRAD_WQ ((float) 0.5f*(rotor.wm_ref*0.0095 + dqxyCurrRef.q*0.0714))
// ----------------------------------------------------------------------
//  Funciones
// ----------------------------------------------------------------------

/*------------- Funciones de control ------------------*/
void vel_profile(void);
void LowPassFilter(void);
void driveSET(void)
void Control_Action_Settings(void);

/*------------- Adquisición de datos ------------------*/
void dataLogging(void)
void conm_diff(void);

/*---------------- Configuraciones --------------------*/
void ConfigureGPIO(void);
void ConfigureEPWM_Q2ADRIVE(volatile struct EPWM_REGS *ePWM_Regs);
void InitEPWM(void);
void ConfigureEPWM(volatile struct EPWM_REGS *ePWM_Regs);
void eQEP_init(void);
void ADC_init(void);

/*------------- Interrupciones ------------------*/
interrupt void adca1_isr(void);
interrupt void eQEP_isr(void);
interrupt void synchronous_isr(void);

// ----------------------------------------------------------------------
//  Macros especificas para cada esquema de control
// ----------------------------------------------------------------------

#ifdef MODEL_FREE

    #define SCHEME   "FCS_MFDC"

    // Funciones propias del model free
    interrupt void adca1_mid_isr(void);
    #ifdef NULL_COMP
        void update_buffer(void);
    #endif

    #define KXY1   (float) 0.6 // XY1 subspace weight
    #define KXY2   (float) 1 // XY2 subspace weight

    #define OSDC(D0,DM,RATE) ((float)(D0 + RATE*(DM - D0)))
    #define CG(REF,MED) ((float)(REF - MED))
    #define NORM_AB_REF(REF) ((float)(1/sqrtf(REF.alpha*REF.alpha + REF.beta*REF.beta)))
    #define NORM(a,b) ((float)(1/sqrtf(a*a + b*b)))

#endif

#ifdef FOC

    #define SCHEME   "FOC"

    #define VD_MAX  ((float) 150)
    #define VXY_MAX ((float) 10)
    #define VQ_MAX ((float) sqrtf(Vdc*Vdc - VD_MAX*VD_MAX -4.0f*VXY_MAX*VXY_MAX))

    #define Kp_id   (float) 15
    #define Ki_id   (float) 5

    #define Kp_iq   (float) 15
    #define Ki_iq   (float) 5

    #define Kp_ix1   (float) 7
    #define Ki_ix1   (float) 5

    #define Kp_iy1   (float) 7
    #define Ki_iy1   (float) 5

    #define Kp_ix2   (float) 7
    #define Ki_ix2   (float) 5

    #define Kp_iy2   (float) 7
    #define Ki_iy2   (float) 5

#endif

#if defined(MPC) || defined(MPCta)
    #ifdef MPC
    #define SCHEME   "FCS_MPC"
    #endif
    #ifdef MPCta
    #define SCHEME   "FCS_MPC_ta"
    #endif
    // Machine parameters
    #define Rs     ((float) 1.0)
    #define Ld     ((float) 0.0711)
    #define Lq     ((float) 0.0711)
    #define Lxy1   ((float) 0.009)
    #define Lxy2   ((float) 0.003)
    #define LAMBDA ((float) 0.85)

    // Weight
    #define KXY1   (float) 0.05 // XY1 subspace weight
    #define KXY2   (float) 0.05 // XY2 subspace weight

    // MACHINE STEADY-SPACE
    // PHI matrix [FI = I + A*Tm]
    #define FIc11(Tm)  ((float)(1 - (Tm)*((Rs)/(Ld))))
    #define FIc22(Tm)  ((float)(1 - (Tm)*((Rs)/(Lq))))
    #define FIc33(Tm)  ((float)(1 - (Tm)*((Rs)/(Lxy1))))
    #define FIc44(Tm)  ((float)(1 - (Tm)*((Rs)/(Lxy1))))
    #define FIc55(Tm)  ((float)(1 - (Tm)*((Rs)/(Lxy2))))
    #define FIc66(Tm)  ((float)(1 - (Tm)*((Rs)/(Lxy2))))
    #define A12(Tm)    ((float)((Tm)*((Lq) / (Ld))))
    #define A21(Tm)    ((float)(0 - (Tm)*((Lq) / (Ld))))
    // GAMMA Matrix [GAMMA = B * Tm]
    #define GAMMA11(Tm) ((float)((Tm)/(Ld)))
    #define GAMMA22(Tm) ((float)((Tm)/(Lq)))
    #define GAMMA33(Tm) ((float)((Tm)/(Lxy1)))
    #define GAMMA44(Tm) ((float)((Tm)/(Lxy1)))
    #define GAMMA55(Tm) ((float)((Tm)/(Lxy2)))
    #define GAMMA66(Tm) ((float)((Tm)/(Lxy2)))
    // DELTA Matrix [DELTA = E*Tm]
    #define DELTA21(Tm) ((float)(0 - (Tm)*((LAMBDA)/(Lq))))
#endif


#ifdef MV5_FCS                          // 5 long vectors that compensate xy1 and xy2
    #define SCHEME   "MV5"
    // Machine parameters
    #define Rs     ((float) 1.0)
    #define Ld     ((float) 0.0711)
    #define Lq     ((float) 0.0711)
    #define Lxy1   ((float) 0.009)
    #define Lxy2   ((float) 0.003)
    #define LAMBDA ((float) 0.85)

    // Weight
    #define KXY1   (float) 0.05 // XY1 subspace weight
    #define KXY2   (float) 0.05 // XY2 subspace weight

    // MACHINE STEADY-SPACE
    // PHI matrix [FI = I + A*Tm]
    #define FIc11(Tm)  ((float)(1 - (Tm)*((Rs)/(Ld))))
    #define FIc22(Tm)  ((float)(1 - (Tm)*((Rs)/(Lq))))
    #define FIc33(Tm)  ((float)(1 - (Tm)*((Rs)/(Lxy1))))
    #define FIc44(Tm)  ((float)(1 - (Tm)*((Rs)/(Lxy1))))
    #define FIc55(Tm)  ((float)(1 - (Tm)*((Rs)/(Lxy2))))
    #define FIc66(Tm)  ((float)(1 - (Tm)*((Rs)/(Lxy2))))
    #define A12(Tm)    ((float)((Tm)*((Lq) / (Ld))))
    #define A21(Tm)    ((float)(0 - (Tm)*((Lq) / (Ld))))
    // GAMMA Matrix [GAMMA = B * Tm]
    #define GAMMA11(Tm) ((float)((Tm)/(Ld)))
    #define GAMMA22(Tm) ((float)((Tm)/(Lq)))
    #define GAMMA33(Tm) ((float)((Tm)/(Lxy1)))
    #define GAMMA44(Tm) ((float)((Tm)/(Lxy1)))
    #define GAMMA55(Tm) ((float)((Tm)/(Lxy2)))
    #define GAMMA66(Tm) ((float)((Tm)/(Lxy2)))
    // DELTA Matrix [DELTA = E*Tm]
    #define DELTA21(Tm) ((float)(0 - (Tm)*((LAMBDA)/(Lq))))
#endif


#endif /* LABLIB_H_ */
