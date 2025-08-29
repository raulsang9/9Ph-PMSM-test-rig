/*
 * data_type.h
 *
 *  Created on: 17 ene. 2025
 *      Author: Usuario
 */

#ifndef DATA_TYPE_H
#define DATA_TYPE_H

#include "lablib.h"

#ifdef W_DATASET
#include "Vel_profile.h"
#endif

// ----------------------------------------------------------------------
//  Declaración estructuras de datos
// ----------------------------------------------------------------------

typedef struct {
    Uint32 A0;
    Uint32 A1;
    Uint32 A2;
    Uint32 B0;
    Uint32 B1;
    Uint32 B2;
} ADCRESULT;

typedef struct {
    float A1;
    float B1;
    float C1;
    float A2;
    float B2;
    float C2;
    float A3;
    float B3;
    float C3;
} PHASE_DATA;

typedef struct {
    float A0;
    float A1;
    float A2;
    float B0;
    float B1;
    float B2;
} SENSE_DATA;

typedef struct {
    float alpha;
    float beta;
    float x1;
    float y1;
    float x2;
    float y2;
} ABXY_DATA;

typedef struct {
    float d;
    float q;
    float x1;
    float y1;
    float x2;
    float y2;
} DQXY_DATA;

typedef struct {
    float thetae;
    float wm_ref;
    float wm_trg;
    float wm_k;
    float wm_km1;
    float we_k;
    float wm_k_filt;
} ROTOR_DATA;

typedef struct {
    Uint32 cont;
    float eqeptmr;
    Uint32 hdx;
    float veloc_radsec;
    float hvel_radsec;
    Uint16 state;
    Uint16 init;
    float transition;
    float res;
} EQEP_DATA;

typedef struct {
    const float KP;
    const float KI;
    float inte_k;
    float inte_km1;
    float error;
    float error_km1;
} PIDATA;

typedef struct {
    const float c11;
    const float c22;
    const float c33;
    const float c44;
    const float c55;
    const float c66;
} DMATRIX;

typedef struct {
    Uint32 FwrdRun;// DI1: GPIO39
    Uint32 RevrRun;// DI3: GPIO45
    Uint32 Reset;  // DI4: GPIO97
    float TorqRef; // AI1: GPIO22 (EPWM16A, Mux 5)
} Q2ADRIVE;


typedef struct {
    float  strSize;
    float  SwFrequency;
    float  LoadTorque;
    float  ControlFrequency;
    float  TargetSpeed;
    float  dataLEN;
    float  NumVAR;
    float  Decimation;
    float  startTime;
    float  thetaend;
    char   ControlStrat[32];
    float data[LEN][VAR];
} ExperimentData;

// ----------------------------------------------------------------------
//  Declaración variables Globales
// ----------------------------------------------------------------------

extern SENSE_DATA OFFSET;
extern SENSE_DATA GAIN;
extern PHASE_DATA PhaseCurrMeas;
extern ABXY_DATA  abxyCurrMeas;
extern ABXY_DATA  abxyCurrRef;
extern DQXY_DATA  dqxyCurrRef;
extern DQXY_DATA  dqxyCurrSat;
extern DQXY_DATA  dqxyCurrMeas;
extern ROTOR_DATA rotor;
extern Q2ADRIVE   Q2A;
extern PIDATA     omega;
extern ExperimentData g_experiment;
extern ADCRESULT adcResult;
extern EQEP_DATA EQEP1;


extern float slp;
extern float cont;
extern Uint16 synch;
extern const float pwm_period_ctrl;
extern const Uint32 ON;
extern const Uint32 OFF;
extern float sintheta;
extern float costheta;
extern Uint16 end_guardado;
extern Uint16 data_l;
extern Uint16 incr;
extern uint16_t x_opt;
extern float elapsed;

extern float Jopt;
extern float Cf;
extern uint16_t x_opt_k;
extern uint16_t x_opt_km1;
extern const float Kxy1;
extern const float Kxy2;
extern const float Sout[19][9];
extern const float CO_freq;
extern const float Tm;
extern const float KINTE;
extern const float seg;
extern float t1;
extern float t2;
extern float t3;
extern float t4;
extern float elapsed2;

#ifdef MUS2
    extern float mus2_var;
#endif

#ifdef MOD_RAD
    extern float ta;
#endif

extern float slp;
extern const float tcte;
extern const float tup;
extern const float tdown;
extern float tsim;
#ifdef W_TRAP
    extern const float tup_t;
#endif

#ifdef W_DATASET
    extern Uint16 indx;
#endif

#ifdef MODEL_FREE

    extern PHASE_DATA PhaseMidCurrMeas;
    extern ABXY_DATA  abxyDeltaRef;
    extern ABXY_DATA  abxyDeltaError;

    extern float mod_delta_ab;
    extern float mod_ab;
    extern float mod_xy1;
    extern float mod_xy2;
    extern Uint16 synchADC;
    extern float rate;
    extern const float Va[18];
    extern const float Vb[18];
    extern const float Vx1[18];
    extern const float Vy1[18];
    extern const float Vx2[18];
    extern const float Vy2[18];
    extern float V0[6];

    extern ABXY_DATA abxyCurrMeas_km1;
    extern ABXY_DATA abxyCurrMeas_k;
    extern ABXY_DATA abxyCurrMeas_kp1;
    extern ABXY_DATA abxyCurrMeas_kpm;
    extern ABXY_DATA deltaCurr0_k;

    #ifdef NULL_COMP
        extern float inv_len_buff;
        extern DQXY_DATA buffer[LEN_BUFF];
        extern DQXY_DATA CG_0_AVG;
        extern int buffer_write_index;
        extern DQXY_DATA new_data;
        extern Uint16 buff_complete;
    #endif

    #ifdef CF_SCAL_PROD
            extern ABXY_DATA vabxy;
            extern const float Kxy1_SP;
            extern const float Kxy2_SP;
    #endif

#endif


#if defined(MPC) || defined(MPCta)

    extern ABXY_DATA  abxyCurrkp1;
    extern ABXY_DATA GAMMA_V[19];
    extern ABXY_DATA  abxyCurrkp1;
    extern ABXY_DATA  abxyCurrkp2;
    extern ABXY_DATA  abxyCurrkp2cnst;
    extern ABXY_DATA  abxyerror;

    extern DQXY_DATA  dqxyVoltk;
    extern DQXY_DATA  dqxyVoltkp1;

    extern DMATRIX FI;
    extern DMATRIX GAMMA;

    extern const float a12;
    extern const float a21;
    extern const float delta21;

    #ifdef MPCta
    extern float ta;
    extern float ta_opt;
    extern float valpha_ref;
    extern float vbeta_ref;
    extern float v_ref_mod;
    extern float k_filt;
    #endif

#endif


#ifdef MV5_FCS

    extern ABXY_DATA  abxyCurrkp1;
    extern ABXY_DATA GAMMA_V[19];
    extern ABXY_DATA  abxyCurrkp1;
    extern ABXY_DATA  abxyCurrkp2;
    extern ABXY_DATA  abxyCurrkp2cnst;
    extern ABXY_DATA  abxyerror;

    extern DQXY_DATA  dqxyVoltk;
    extern DQXY_DATA  dqxyVoltkp1;

    extern DMATRIX FI;
    extern DMATRIX GAMMA;

    extern const float a12;
    extern const float a21;
    extern const float delta21;

    extern PHASE_DATA CMPA_VV[19];
    extern PHASE_DATA CMPB_VV[19];

#endif


extern _iq mod_delta_ab_IQ;
#endif /* DATA_TYPE_H_ */
