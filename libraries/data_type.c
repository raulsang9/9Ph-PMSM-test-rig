/*
 * data_type.c
 *
 *  Created on: 18 ene. 2025
 *      Author: pdobl
 */

#include "data_type.h"

// Declaración de variables comunes en todos los scripts

SENSE_DATA OFFSET = {24.8618f, 24.9488f, 25.541f, 25.4804f, 25.253f, 24.852f};
SENSE_DATA GAIN = {0.012319f, 0.012376f, 0.012623f, 0.012553f, 0.012611f, 0.012325f};

ADCRESULT  adcResult          = {0,0,0,0,0,0};
PHASE_DATA PhaseCurrMeas      = {0,0,0,0,0,0,0,0,0};
ABXY_DATA  abxyCurrMeas       = {0,0,0,0,0,0};
ABXY_DATA  abxyCurrRef        = {0,0,0,0,0,0};
DQXY_DATA  dqxyCurrRef        = {0,0,0,0,0,0};
DQXY_DATA  dqxyCurrMeas       = {0,0,0,0,0,0};
DQXY_DATA  dqxyCurrSat        = {0,IQ_MAX,0,0,0,0};
ROTOR_DATA rotor              = { 0, 0, RPM2RADS(W_TRG), 0, 0, 0};//{ thetae, wm_ref, wm_trg, wm_k, wm_km1}
PIDATA     omega              = {Kp_w,Ki_w,0,0,0,0};
EQEP_DATA  EQEP1              = {0,1,0,0,0,1,0,0,0};

ExperimentData g_experiment =
{
   0U,            // Structure size (bytes/2)
   0.0f,          // Switching frequency (Hz)
   TLOAD,         // Load torque (Q2A)
   CTRLF,         // Control frequency (Hz)
   W_TRG,         // Target speed (rpm)
   LEN,           // Data length (-)
   VAR,           // Number of variables (-)
   DEC,           // Acquisition decimation (S/S)
   0.0f,          // Start acq. time (s)
   0.0f,          // Electrical angle at the end (rad)
   SCHEME,        // Control strategy
};

float  cont     = 0;
Uint16 synch    = 0;

const float pwm_period_ctrl = PWM_PERIOD_CTRL;

const float Sout[19][9] =  {{1,0,0,1,1,0,1,1,0},
                            {1,0,0,1,0,0,1,1,0},
                            {1,0,1,1,0,0,1,1,0},
                            {1,0,1,1,0,0,1,0,0},
                            {1,0,1,1,0,1,1,0,0},
                            {0,0,1,1,0,1,1,0,0},
                            {0,0,1,1,0,1,1,0,1},
                            {0,0,1,0,0,1,1,0,1},
                            {0,1,1,0,0,1,1,0,1},
                            {0,1,1,0,0,1,0,0,1},
                            {0,1,1,0,1,1,0,0,1},
                            {0,1,0,0,1,1,0,0,1},
                            {0,1,0,0,1,1,0,1,1},
                            {0,1,0,0,1,0,0,1,1},
                            {1,1,0,0,1,0,0,1,1},
                            {1,1,0,0,1,0,0,1,0},
                            {1,1,0,1,1,0,0,1,0},
                            {1,0,0,1,1,0,0,1,0},
                            {1,1,1,1,1,1,1,1,1}};

float sintheta = 0;
float costheta = 0;

/*-------------- Pesos Funcion de coste ---------------------*/

const float Kxy1 = KXY1;
const float Kxy2 = KXY2;

float Cf    = 999999999;
float Jopt  = 999999999;

uint16_t x_opt     = 18;
uint16_t x_opt_k   = 18;
uint16_t x_opt_km1 = 18;

// Creo que esta variable se puede eliminar!!!!!!!!!!!!!!!!!!!
float slp    = SLP;       // Constant acceleration and deceleration

#ifdef W_TRAP
    const float tup   = SLP*RPM2RADS(W_TRG);
    const float tdown = SLP*RPM2RADS(W_TRG);//t_hbsaicdsbihdblihqwl
    const float tup_t = SLP_T*TLOAD;
    const float tcte  = T_CNST;    // Stationary time at target frequency
    float tsim        = 2*SLP*RPM2RADS(W_TRG) + T_CNST ;
#endif

#ifdef W_DATASET
    const float tup   = SLP*SPEEDINIT;
    const float tdown = SLP*SPEEDEND;   //t_jkeqwniscdbilkqcdn
    const float tcte  = TPROF;    // Stationary time at target frequency
    float tsim        = SLP*SPEEDINIT + TPROF + SLP*SPEEDEND;
    Uint16 indx = 1;
#endif

#ifdef MUS2
    float mus2_var = 1;
#endif


#ifdef MOD_RAD
    float ta = 1;
#endif

const float Tm      = 1/CTRLF;
const float KINTE   = 0.5/CTRLF;
const float seg     = CTRLF;
const float CO_freq = 400;

// Data logger variables
Uint16 data_l;
Uint16 incr;
Uint16 end_guardado = 0;


/*-------------- Q2A Drive ---------------------*/
Q2ADRIVE Q2A = {0, 0, 0, 0};// {FwrdRun, RevrRun, Reset, TorqRef}
const Uint32 ON = 1;
const Uint32 OFF = 0;

float t1;
float t2;
float t3;
float t4;
float elapsed;
float elapsed2;


#ifdef MODEL_FREE

    PHASE_DATA PhaseMidCurrMeas   = {0,0,0,0,0,0,0,0,0};
    ABXY_DATA  abxyDeltaRef       = {0,0,0,0,0,0};
    ABXY_DATA  abxyDeltaError     = {0,0,0,0,0,0};

    ABXY_DATA abxyCurrMeas_km1 = {0,0,0,0,0,0};
    ABXY_DATA abxyCurrMeas_k   = {0,0,0,0,0,0};
    ABXY_DATA abxyCurrMeas_kp1 = {0,0,0,0,0,0};
    ABXY_DATA abxyCurrMeas_kpm = {0,0,0,0,0,0};
    ABXY_DATA deltaCurr0_k     = {0,0,0,0,0,0};
    Uint16 synchADC = 0;

    const float  Va[18] = { 1.0000, 0.9397, 0.7660, 0.5000, 0.1736,-0.1736,-0.5000,-0.7660,-0.9397,-1.0000,-0.9397,-0.7660,-0.5000,-0.1736, 0.1736, 0.5000, 0.7660, 0.9397};
    const float  Vb[18] = { 0.0000, 0.3420, 0.6428, 0.8660, 0.9848, 0.9848, 0.8660, 0.6428, 0.3420, 0.0000,-0.3420,-0.6428,-0.8660,-0.9848,-0.9848,-0.8660,-0.6428,-0.3420};
    const float Vx1[18] = {-0.1848, 0.1416,-0.0321,-0.0924, 0.1736,-0.1736, 0.0924, 0.0321,-0.1416, 0.1848,-0.1416, 0.0321, 0.0924,-0.1736, 0.1736,-0.0924,-0.0321, 0.1416};
    const float Vy1[18] = { 0.0000, 0.1188,-0.1820, 0.1600,-0.0632,-0.0632, 0.1600,-0.1820, 0.1188,-0.0000,-0.1188, 0.1820,-0.1600, 0.0632, 0.0632,-0.1600, 0.1820,-0.1188};
    const float Vx2[18] = { 0.2267,-0.0394,-0.2130, 0.1133, 0.1736,-0.1736,-0.1133, 0.2130, 0.0394,-0.2267, 0.0394, 0.2130,-0.1133,-0.1736, 0.1736, 0.1133,-0.2130,-0.0394};
    const float Vy2[18] = { 0.0000,-0.2232, 0.0775, 0.1963,-0.1457,-0.1457, 0.1963, 0.0775,-0.2232, 0.0000, 0.2232,-0.0775,-0.1963, 0.1457, 0.1457,-0.1963,-0.0775, 0.2232};

    // Normalization variables
    float mod_delta_ab  = 0;

    float mod_ab  = 0;
    float mod_xy1 = 0;
    float mod_xy2 = 0;

    float V0[6] = {0,0,0,0,0,0};
    float rate  = 1/RATE;

    #ifdef NULL_COMP
        float inv_len_buff = 1/LEN_BUFF;
        DQXY_DATA buffer[LEN_BUFF] = {0};
        DQXY_DATA CG_0_AVG = {0};
        int buffer_write_index = 0;
        DQXY_DATA new_data = {0};
        Uint16 buff_complete = 0;
    #endif

    #ifdef CF_SCAL_PROD
            ABXY_DATA vabxy = {0,0,0,0,0,0};
            const float Kxy1_SP = KXY1_SP;
            const float Kxy2_SP = KXY2_SP;
    #endif
#endif

#if defined(MPC) || defined(MPCta)

    ABXY_DATA GAMMA_V[19] = {{ 0.6399, -0.0000, -0.1182,  0.0000,  0.1450, -0.0000},
                             { 0.6013,  0.2188,  0.0906,  0.0760, -0.0252, -0.1428},
                             { 0.4902,  0.4113, -0.0205, -0.1164, -0.1363,  0.0496},
                             { 0.3199,  0.5541, -0.0591,  0.1024,  0.0725,  0.1256},
                             { 0.1111,  0.6301,  0.1111, -0.0404,  0.1111, -0.0932},
                             {-0.1111,  0.6301, -0.1111, -0.0404, -0.1111, -0.0932},
                             {-0.3199,  0.5541,  0.0591,  0.1024, -0.0725,  0.1256},
                             {-0.4902,  0.4113,  0.0205, -0.1164,  0.1363,  0.0496},
                             {-0.6013,  0.2188, -0.0906,  0.0760,  0.0252, -0.1428},
                             {-0.6399,  0.0000,  0.1182, -0.0000, -0.1450,  0.0000},
                             {-0.6013, -0.2188, -0.0906, -0.0760,  0.0252,  0.1428},
                             {-0.4902, -0.4113,  0.0205,  0.1164,  0.1363, -0.0496},
                             {-0.3199, -0.5541,  0.0591, -0.1024, -0.0725, -0.1256},
                             {-0.1111, -0.6301, -0.1111,  0.0404, -0.1111,  0.0932},
                             { 0.1111, -0.6301,  0.1111,  0.0404,  0.1111,  0.0932},
                             { 0.3199, -0.5541, -0.0591, -0.1024,  0.0725, -0.1256},
                             { 0.4902, -0.4113, -0.0205,  0.1164, -0.1363, -0.0496},
                             { 0.6013, -0.2188,  0.0906, -0.0760, -0.0252,  0.1428},
                             { 0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000}};

    DMATRIX FI    = {FIc11(1/CTRLF)  , FIc22(1/CTRLF)  , FIc33(1/CTRLF)  , FIc44(1/CTRLF)  , FIc55(1/CTRLF)  , FIc66(1/CTRLF)  };
    DMATRIX GAMMA = {GAMMA11(1/CTRLF), GAMMA22(1/CTRLF), GAMMA33(1/CTRLF), GAMMA44(1/CTRLF), GAMMA55(1/CTRLF), GAMMA66(1/CTRLF)};

    const float a12     = A12(1/CTRLF)    ;
    const float a21     = A21(1/CTRLF)    ;
    const float delta21 = DELTA21(1/CTRLF);

    ABXY_DATA  abxyCurrkp1     = {0,0,0,0,0,0};      //{ alpha, beta, x1, y1, x2, y2}
    ABXY_DATA  abxyCurrkp2     = {0,0,0,0,0,0};      //{ alpha, beta, x1, y1, x2, y2}
    ABXY_DATA  abxyCurrkp2cnst = {0,0,0,0,0,0};      //{ alpha, beta, x1, y1, x2, y2}
    ABXY_DATA  abxyerror       = {0,0,0,0,0,0};      //{ alpha, beta, x1, y1, x2, y2}

    DQXY_DATA  dqxyVoltk   = { 0, 0, 0, 0, 0, 0};//{ d, q, x1, y1, x2, y2}
    DQXY_DATA  dqxyVoltkp1 = { 0, 0, 0, 0, 0, 0};//{ d, q, x1, y1, x2, y2}

    #ifdef MPCta
    float ta         = 1;
    float ta_opt     = 1;
    float valpha_ref = 0;
    float vbeta_ref  = 0;
    float v_ref_mod  = 0;
    float k_filt     = 0.4;
    #endif

#endif


#ifdef FOC
    // Currents PI variables
    PIDATA     ID  = {Kp_id, Ki_id,  0, 0, 0, 0};//{ KP,KI , inte_k, inte_km1, error, error_km1}
    PIDATA     IQ  = {Kp_iq, Ki_iq,  0, 0, 0, 0};//{ KP,KI , inte_k, inte_km1, error, error_km1}
    PIDATA     IX1 = {Kp_ix1,Ki_ix1, 0, 0, 0, 0};//{ KP,KI , inte_k, inte_km1, error, error_km1}
    PIDATA     IY1 = {Kp_iy1,Ki_iy1, 0, 0, 0, 0};//{ KP,KI , inte_k, inte_km1, error, error_km1}
    PIDATA     IX2 = {Kp_ix2,Ki_ix2, 0, 0, 0, 0};//{ KP,KI , inte_k, inte_km1, error, error_km1}
    PIDATA     IY2 = {Kp_iy2,Ki_iy2, 0, 0, 0, 0};//{ KP,KI , inte_k, inte_km1, error, error_km1}



    // Saturations
    float Vn       = Vdc/2; // Carrier based Vmax
    DQXY_DATA dqxyVoltSat = {VD_MAX, 0, VXY_MAX, VXY_MAX, VXY_MAX, VXY_MAX};

    PHASE_DATA PhaseVoltMeas   = {0,0,0,0,0,0,0,0,0};//{ A1, B1, C1, A2, B2, C2, A3, B3, C3}
    PHASE_DATA PhaseVoltRef    = {0,0,0,0,0,0,0,0,0};//{ A1, B1, C1, A2, B2, C2, A3, B3, C3}
    PHASE_DATA PhaseVoltRefpu  = {0,0,0,0,0,0,0,0,0};//{ A1, B1, C1, A2, B2, C2, A3, B3, C3}

    ABXY_DATA  abxyVoltMeas    = {0,0,0,0,0,0};      //{ alpha, beta, x1, y1, x2, y2}
    ABXY_DATA  abxyVoltRef     = {0,0,0,0,0,0};      //{ alpha, beta, x1, y1, x2, y2}

    DQXY_DATA  dqxyVoltRef = { 0, 0, 0, 0, 0, 0};    //{     d,    q, x1, y1, x2, y2}
#endif

#ifdef MV5_FCS

    ABXY_DATA GAMMA_V[19] = {{ 0.4491 , 0.3768},
                             { 0.2931 , 0.5077},
                             { 0.1018 , 0.5774},
                             {-0.1018 , 0.5774},
                             {-0.2931 , 0.5077},
                             {-0.4491 , 0.3768},
                             {-0.5509 , 0.2005},
                             {-0.5863 ,-0.0000},
                             {-0.5509 ,-0.2005},
                             {-0.4491 ,-0.3768},
                             {-0.2931 ,-0.5077},
                             {-0.1018 ,-0.5774},
                             { 0.1018 ,-0.5774},
                             { 0.2931 ,-0.5077},
                             { 0.4491 ,-0.3768},
                             { 0.5509 ,-0.2005},
                             { 0.5863 , 0.0000},
                             { 0.5509 , 0.2005},
                             { 0.0000,  0.0000}};


    DMATRIX FI    = {FIc11(1/CTRLF)  , FIc22(1/CTRLF)  , FIc33(1/CTRLF)  , FIc44(1/CTRLF)  , FIc55(1/CTRLF)  , FIc66(1/CTRLF)  };
    DMATRIX GAMMA = {GAMMA11(1/CTRLF), GAMMA22(1/CTRLF), GAMMA33(1/CTRLF), GAMMA44(1/CTRLF), GAMMA55(1/CTRLF), GAMMA66(1/CTRLF)};

    const float a12     = A12(1/CTRLF)    ;
    const float a21     = A21(1/CTRLF)    ;
    const float delta21 = DELTA21(1/CTRLF);

    ABXY_DATA  abxyCurrkp1     = {0,0,0,0,0,0};      //{ alpha, beta, x1, y1, x2, y2}
    ABXY_DATA  abxyCurrkp2     = {0,0,0,0,0,0};      //{ alpha, beta, x1, y1, x2, y2}
    ABXY_DATA  abxyCurrkp2cnst = {0,0,0,0,0,0};      //{ alpha, beta, x1, y1, x2, y2}
    ABXY_DATA  abxyerror       = {0,0,0,0,0,0};      //{ alpha, beta, x1, y1, x2, y2}

    DQXY_DATA  dqxyVoltk   = { 0, 0, 0, 0, 0, 0};//{ d, q, x1, y1, x2, y2}
    DQXY_DATA  dqxyVoltkp1 = { 0, 0, 0, 0, 0, 0};//{ d, q, x1, y1, x2, y2}

    PHASE_DATA CMPA_VV[19] = {
        {1.000000, 0.000000, 0.000000, 1.000000, 0.120615, 0.000000, 1.000000, 0.652704, 0.000000},
        {0.879385, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 1.000000, 0.347296, 0.000000},
        {0.652704, 0.000000, 1.000000, 1.000000, 0.000000, 0.000000, 1.000000, 0.120615, 0.000000},
        {0.347296, 0.000000, 1.000000, 0.879385, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000},
        {0.120615, 0.000000, 1.000000, 0.652704, 0.000000, 1.000000, 1.000000, 0.000000, 0.000000},
        {0.000000, 0.000000, 1.000000, 0.347296, 0.000000, 1.000000, 0.879385, 0.000000, 0.000000},
        {0.000000, 0.000000, 1.000000, 0.120615, 0.000000, 1.000000, 0.652704, 0.000000, 1.000000},
        {0.000000, 0.000000, 0.879385, 0.000000, 0.000000, 1.000000, 0.347296, 0.000000, 1.000000},
        {0.000000, 1.000000, 0.652704, 0.000000, 0.000000, 1.000000, 0.120615, 0.000000, 1.000000},
        {0.000000, 1.000000, 0.347296, 0.000000, 0.000000, 0.879385, 0.000000, 0.000000, 1.000000},
        {0.000000, 1.000000, 0.120615, 0.000000, 1.000000, 0.652704, 0.000000, 0.000000, 1.000000},
        {0.000000, 1.000000, 0.000000, 0.000000, 1.000000, 0.347296, 0.000000, 0.000000, 0.879385},
        {0.000000, 1.000000, 0.000000, 0.000000, 1.000000, 0.120615, 0.000000, 1.000000, 0.652704},
        {0.000000, 0.879385, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 1.000000, 0.347296},
        {1.000000, 0.652704, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 1.000000, 0.120615},
        {1.000000, 0.347296, 0.000000, 0.000000, 0.879385, 0.000000, 0.000000, 1.000000, 0.000000},
        {1.000000, 0.120615, 0.000000, 1.000000, 0.652704, 0.000000, 0.000000, 1.000000, 0.000000},
        {1.000000, 0.000000, 0.000000, 1.000000, 0.347296, 0.000000, 0.000000, 0.879385, 0.000000},
        {0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000}};

    PHASE_DATA CMPB_VV[19] = {
        {0.000000, 1.000000, 0.347296, 0.000000, 0.000000, 0.879385, 0.000000, 0.000000, 1.000000},
        {0.000000, 1.000000, 0.120615, 0.000000, 1.000000, 0.652704, 0.000000, 0.000000, 1.000000},
        {0.000000, 1.000000, 0.000000, 0.000000, 1.000000, 0.347296, 0.000000, 0.000000, 0.879385},
        {0.000000, 1.000000, 0.000000, 0.000000, 1.000000, 0.120615, 0.000000, 1.000000, 0.652704},
        {0.000000, 0.879385, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 1.000000, 0.347296},
        {1.000000, 0.652704, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 1.000000, 0.120615},
        {1.000000, 0.347296, 0.000000, 0.000000, 0.879385, 0.000000, 0.000000, 1.000000, 0.000000},
        {1.000000, 0.120615, 0.000000, 1.000000, 0.652704, 0.000000, 0.000000, 1.000000, 0.000000},
        {1.000000, 0.000000, 0.000000, 1.000000, 0.347296, 0.000000, 0.000000, 0.879385, 0.000000},
        {1.000000, 0.000000, 0.000000, 1.000000, 0.120615, 0.000000, 1.000000, 0.652704, 0.000000},
        {0.879385, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 1.000000, 0.347296, 0.000000},
        {0.652704, 0.000000, 1.000000, 1.000000, 0.000000, 0.000000, 1.000000, 0.120615, 0.000000},
        {0.347296, 0.000000, 1.000000, 0.879385, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000},
        {0.120615, 0.000000, 1.000000, 0.652704, 0.000000, 1.000000, 1.000000, 0.000000, 0.000000},
        {0.000000, 0.000000, 1.000000, 0.347296, 0.000000, 1.000000, 0.879385, 0.000000, 0.000000},
        {0.000000, 0.000000, 1.000000, 0.120615, 0.000000, 1.000000, 0.652704, 0.000000, 1.000000},
        {0.000000, 0.000000, 0.879385, 0.000000, 0.000000, 1.000000, 0.347296, 0.000000, 1.000000},
        {0.000000, 1.000000, 0.652704, 0.000000, 0.000000, 1.000000, 0.120615, 0.000000, 1.000000},
        {1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000, 1.000000}};
#endif


_iq mod_delta_ab_IQ = _IQ(0.0f);
