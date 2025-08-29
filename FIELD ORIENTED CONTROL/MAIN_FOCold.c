//#############################################################################
//
// FILE:   Main Project
//
// TITLE:  Main project to control 9ph SiC converter
//
// VERSION: V0.05
//
// LAST EDITOR: RSG
//
// DATE: 27/11/2024 
//
// COMMENT: Nueva versiÃ³n cambiando las variables a los tipos de variables creados en PARAMETERS.h
//
// PENDING UPDATES:
// ----------------
// > Move ADC and eQEP initialization to another script outside main code
// > Configure eQEP for attending index interruptions, index initializations, hysteresis bands, etc.
// > Eliminate CPU Timer synchronization. Instead employ PWM clocking.
// > Include IQmath library and use it for reducing computational burden.
// > Check rotor position polarity and velocity measurement.
// > Clean code comments for keeping only essentials.
// > Try using local variables instead of globals one for reducing memory access time.
// > Review Clarke transformation.
// > Calibrate ADC offset and gain for each current probe.
// > Utilize data structure with different fields and its data pointers for addressing data packs (currents, speed, position, ...)
// > And so on ...
//#############################################################################
// Included Files
#include <PARAMETERS.h>


// Exogenous function
extern void ConfigureGPIO(void);
extern void InitEPWM(void);
extern void ConfigureEPWM(volatile struct EPWM_REGS *ePWM_Regs);
void eQEP_init(void);
void ADC_init(void);

interrupt void adca1_isr(void);
interrupt void eQEP_isr(void);
interrupt void synchronous_isr(void);

SENSE_DATA OFFSET = {24.8618f, 24.9488f, 25.541f, 25.4804f, 25.253f, 24.852f};

SENSE_DATA GAIN = {0.012319f, 0.012376f, 0.012623f, 0.012553f, 0.012611f, 0.012325f};

// EQep ISR variable
EQEP_DATA  EQEP1  = {0,1,0,0,0,1,0,0,0};// {cont, eqeptmr, hdx, veloc_radsec, hvel_radsec, state, init, transition, res}

// Synchronization variable
float cont = 0;
Uint16 synch = 0;
const float pwm_period_ctrl = PWM_PERIOD_CTRL;

/*--------- Data logger variables --------------------*/
int data_l;
int incr;
float guardar[LEN][VAR];
int end_guardado = 0;

/*-------------- Q2A Drive ---------------------*/
Q2ADRIVE Q2A = {0, 0, 0, 0};// {FwrdRun, RevrRun, Reset, TorqRef}
const Uint32 ON = 1;
const Uint32 OFF = 0;

// Velocity PI variables
PIDATA     omega  = {Kp_w,Ki_w, 0, 0, 0, 0};//{ KP,KI , inte_k, inte_km1, error, error_km1}
ROTOR_DATA rotor  = { 0, 0, RPM2RADS(W_TRG), 0, 0, 0};//{ thetae, wm_ref, wm_trg, wm_k, wm_km1}

// Currents PI variables
PIDATA     ID  = {Kp_id, Ki_id,  0, 0, 0, 0};//{ KP,KI , inte_k, inte_km1, error, error_km1}
PIDATA     IQ  = {Kp_iq, Ki_iq,  0, 0, 0, 0};//{ KP,KI , inte_k, inte_km1, error, error_km1}
PIDATA     IX1 = {Kp_ix1,Ki_ix1, 0, 0, 0, 0};//{ KP,KI , inte_k, inte_km1, error, error_km1}
PIDATA     IY1 = {Kp_iy1,Ki_iy1, 0, 0, 0, 0};//{ KP,KI , inte_k, inte_km1, error, error_km1}
PIDATA     IX2 = {Kp_ix2,Ki_ix2, 0, 0, 0, 0};//{ KP,KI , inte_k, inte_km1, error, error_km1}
PIDATA     IY2 = {Kp_iy2,Ki_iy2, 0, 0, 0, 0};//{ KP,KI , inte_k, inte_km1, error, error_km1}



// Saturations
float Vn       = Vdc/2;
float sintheta = 0;
float costheta = 0;

DQXY_DATA dqxyCurrSat  = { 0,IQ_MAX, 0, 0, 0, 0};//{ d, q, x1, y1, x2, y2}
DQXY_DATA dqxyVoltSat = {VD_MAX, 0, VXY_MAX, VXY_MAX, VXY_MAX, VXY_MAX};


/* --------------- Current & Voltage variables ------------*/
// ADC count results
ADCRESULT  adcResult       = {0,0,0,0,0,0};      //{ A0, A1, A2, B0, B1, B2}
// Phase voltages & currents
PHASE_DATA PhaseVoltMeas   = {0,0,0,0,0,0,0,0,0};//{ A1, B1, C1, A2, B2, C2, A3, B3, C3}
PHASE_DATA PhaseVoltRef    = {0,0,0,0,0,0,0,0,0};//{ A1, B1, C1, A2, B2, C2, A3, B3, C3}
PHASE_DATA PhaseVoltRefpu  = {0,0,0,0,0,0,0,0,0};//{ A1, B1, C1, A2, B2, C2, A3, B3, C3}
PHASE_DATA PhaseCurrMeas   = {0,0,0,0,0,0,0,0,0};//{ A1, B1, C1, A2, B2, C2, A3, B3, C3}
// ABXY voltages & currents
ABXY_DATA  abxyVoltMeas    = {0,0,0,0,0,0};      //{ alpha, beta, x1, y1, x2, y2}
ABXY_DATA  abxyVoltRef     = {0,0,0,0,0,0};      //{ alpha, beta, x1, y1, x2, y2}
ABXY_DATA  abxyCurrMeas    = {0,0,0,0,0,0};      //{ alpha, beta, x1, y1, x2, y2}
// En el codigo principal hay otra vabxya ref

// DQXY voltages & currents
DQXY_DATA  dqxyVoltRef = { 0, 0, 0, 0, 0, 0};//{ d, q, x1, y1, x2, y2}
DQXY_DATA  dqxyCurrMeas= { 0, 0, 0, 0, 0, 0};//{ d, q, x1, y1, x2, y2}
DQXY_DATA  dqxyCurrRef = { 0, 0, 0, 0, 0, 0};//{ d, q, x1, y1, x2, y2}


// Velocity profile construction
float slp   = 0.15;        // Constant acceleration and deceleration
float tcte  = 60;            // Stationary time at target frequency
const float Tm = TM;
const float KINTE = 0.5*TM;
const float seg = 1/TM;
float tup;

float t1,t2,t3,elapsed1,elapsed2;

///////////////////////////////////////////////////////////////////
//////////////////////        MAIN        /////////////////////////
///////////////////////////////////////////////////////////////////
void main(void){

    dqxyVoltSat.q = VQ_MAX;

    InitSysCtrl();

    InitGpio();

    DINT; //Clean all the interruptions

    InitPieCtrl();

    // Disable all the CPU interruptions
    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();

    // ADC initialization
    ADC_init();

    eQEP_init();
    // Definition of the new defined interruptions
    EALLOW;
    PieVectTable.ADCA1_INT  = &adca1_isr;       //function for ADCA interrupt 1
    PieVectTable.EQEP1_INT  = &eQEP_isr;        //Interrupt service routine for Enhanced Quadrature Encoder Pulse 1. (custom)
    PieVectTable.EPWM1_INT  = &synchronous_isr;
    EDIS;

    InitCpuTimers();

    ConfigCpuTimer(&CpuTimer0, 200, (Uint32)((float)2e6)*TM);
    CpuTimer0Regs.TCR.all = 0x4000;
    // Configure GPIO
    ConfigureGPIO();

    // Configure EPWM
    InitEPWM();

    // Enable global Interrupts and higher priority real-time debug events:
    //
    IER |= M_INT1; //Enable group 1 interrupts
    IER |= M_INT3; //Enable group 3 interrupts
    IER |= M_INT5;

    // Activación de las interrupciones
    //

    PieCtrlRegs.PIEIER1.bit.INTx1  = 1;  // ADC  interrupt
    PieCtrlRegs.PIEIER3.bit.INTx1  = 1;
    PieCtrlRegs.PIEIER5.bit.INTx1  = 1;

    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

        //
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;    //Synchronization of all EPWM clocks
    EDIS;

    CpuTimer0Regs.TCR.bit.TSS = 0;           // start timer0


    while(cont < 3*seg)
    {
        while(synch == 0);
        synch = 0;
        cont++;

        EPwm1Regs.CMPA.bit.CMPA = pwm_period_ctrl*0.5333;
        EPwm2Regs.CMPA.bit.CMPA = pwm_period_ctrl*0.5;
        EPwm3Regs.CMPA.bit.CMPA = pwm_period_ctrl*0.5;

              // Second converter
        EPwm4Regs.CMPA.bit.CMPA = pwm_period_ctrl*0.5;
        EPwm5Regs.CMPA.bit.CMPA = pwm_period_ctrl*0.5;
        EPwm6Regs.CMPA.bit.CMPA = pwm_period_ctrl*0.5;

              // Third converter
        EPwm7Regs.CMPA.bit.CMPA = pwm_period_ctrl*0.5;
        EPwm8Regs.CMPA.bit.CMPA = pwm_period_ctrl*0.5;
        EPwm9Regs.CMPA.bit.CMPA = pwm_period_ctrl*0.5;
    }

    cont = 0;
    EQep1Regs.QPOSCNT = 0;
    rotor.thetae = 0;



    while(cont < tsim*seg)
    //while(1)
    {
        while(synch == 0);
        t3 = GET_LOC_TIME(CpuTimer0);
        elapsed2 = fabs(t3 - t1)*0.005;
        synch = 0;
        cont++;

        t1 = GET_LOC_TIME(CpuTimer0);


        // Control Q2A Drive
        if(Q2A.RevrRun == ON)GpioDataRegs.GPBSET.bit.GPIO39   = 1;//DI1
        else        GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;

        if(Q2A.FwrdRun == ON)GpioDataRegs.GPBSET.bit.GPIO45   = 1;//DI3
        else        GpioDataRegs.GPBCLEAR.bit.GPIO45 = 1;

        if(Q2A.Reset == ON)GpioDataRegs.GPDSET.bit.GPIO97   = 1;//DI4
        else        GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

        EPwm12Regs.CMPA.bit.CMPA = (Uint32)(PWM_PERIOD_Q2A*NM2PU(Q2A.TorqRef));// AI1

        // Curva del ensayo
        if (cont < seg*tup)
        {
           Q2A.RevrRun = OFF;
           rotor.wm_ref += rotor.wm_trg/(tup*seg);
        }

        else if (cont < (tup + tcte)*seg)
        {
           if (cont > (tup + 0.5)*seg)
           {
               Q2A.TorqRef = TLOAD;
           }
           rotor.wm_ref = rotor.wm_trg;
        }
        else
        {
            Q2A.TorqRef = 0;
           rotor.wm_ref -= rotor.wm_trg/(tup*seg);
        }

      omega.error = rotor.wm_ref - rotor.wm_k;
      omega.inte_k = KINTE*(omega.error + omega.error_km1) + omega.inte_km1;
      dqxyCurrRef.q = omega.KP*(omega.error + omega.KI*omega.inte_k);

      //SATURATION

      if (dqxyCurrRef.q >= dqxyCurrSat.q)
      {
          dqxyCurrRef.q = dqxyCurrSat.q;
      }
      else if (dqxyCurrRef.q <= -dqxyCurrSat.q)
      {
          dqxyCurrRef.q = -dqxyCurrSat.q;
      }

      // UPDATE VARIABLES
      omega.error_km1 = omega.error;
      omega.inte_km1  = omega.inte_k;

      sintheta = sin(rotor.thetae);
      costheta = cos(rotor.thetae);


    if(cont>(tup)*seg && end_guardado==0)
    {
    data_l++;
    if (data_l==DEC)
    {
    //DATA LOGGER
      if (incr < LEN)
       {
          guardar[incr][0] = dqxyCurrMeas.d;
          guardar[incr][1] = dqxyCurrRef.d ;
          guardar[incr][2] = dqxyCurrMeas.q;
          guardar[incr][3] = dqxyCurrRef.q ;
          guardar[incr][4] = dqxyVoltRef.d;
          guardar[incr][5] = dqxyVoltRef.q;
          guardar[incr][6] = rotor.wm_k;
          guardar[incr][7] = rotor.thetae;

            incr++;
            data_l = 0;
            if(incr == LEN)
             {
               end_guardado = 1;
             }
           }
         }
        }

        PhaseCurrMeas.C1 = -(PhaseCurrMeas.A1 + PhaseCurrMeas.B1);
        PhaseCurrMeas.C2 = -(PhaseCurrMeas.A2 + PhaseCurrMeas.B2);
        PhaseCurrMeas.C3 = -(PhaseCurrMeas.A3 + PhaseCurrMeas.B3);

        // Transformada de Clarke (abc -> abxy)
        abxyCurrMeas.alpha = CLARKE_ALPHA(PhaseCurrMeas);
        abxyCurrMeas.beta  = CLARKE_BETA(PhaseCurrMeas) ;
        abxyCurrMeas.x1    = CLARKE_X1(PhaseCurrMeas);
        abxyCurrMeas.y1    = CLARKE_Y1(PhaseCurrMeas);
        abxyCurrMeas.x2    = CLARKE_X2(PhaseCurrMeas);
        abxyCurrMeas.y2    = CLARKE_Y2(PhaseCurrMeas);

        // Transformada de Park (abxy -> dqxy)
        dqxyCurrMeas.d    = PARK_D(abxyCurrMeas,costheta,sintheta);
        dqxyCurrMeas.q    = PARK_Q(abxyCurrMeas,costheta,sintheta);
        dqxyCurrMeas.x1   = PARK_X1(abxyCurrMeas,costheta,sintheta);
        dqxyCurrMeas.y1   = PARK_Y1(abxyCurrMeas,costheta,sintheta);
        dqxyCurrMeas.x2   = PARK_X2(abxyCurrMeas,costheta,sintheta);
        dqxyCurrMeas.y2   = PARK_Y2(abxyCurrMeas,costheta,sintheta);


        // PI: D current
        ID.error = dqxyCurrRef.d - dqxyCurrMeas.d;
        ID.inte_k = KINTE*(ID.error + ID.error_km1) + ID.inte_km1;
        dqxyVoltRef.d = ID.KP*(ID.error + ID.KI*ID.inte_k);

        if (dqxyVoltRef.d >= dqxyVoltSat.d)
        {
            dqxyVoltRef.d = dqxyVoltSat.d;
        }
        else if (dqxyVoltRef.d <= -dqxyVoltSat.d)
        {
            dqxyVoltRef.d = -dqxyVoltSat.d;
        }
        ID.error_km1 = ID.error;
        ID.inte_km1 = ID.inte_k;

        // PI: Q current
        IQ.error = dqxyCurrRef.q - dqxyCurrMeas.q;
        IQ.inte_k = KINTE*(IQ.error + IQ.error_km1) + IQ.inte_km1;
        dqxyVoltRef.q = IQ.KP*(IQ.error + IQ.KI*IQ.inte_k);

        if (dqxyVoltRef.q >= dqxyVoltSat.q)
        {
            dqxyVoltRef.q = dqxyVoltSat.q;
        }
        else if (dqxyVoltRef.q <= -dqxyVoltSat.q)
        {
            dqxyVoltRef.q = -dqxyVoltSat.q;
        }
        IQ.error_km1 = IQ.error;
        IQ.inte_km1 = IQ.inte_k;

        // PI: X1 current
        IX1.error = dqxyCurrRef.x1 - dqxyCurrMeas.x1;
        IX1.inte_k = KINTE*(IX1.error + IX1.error_km1) + IX1.inte_km1;
        dqxyVoltRef.x1 = IX1.KP*(IX1.error + IX1.KI*IX1.inte_k);

        if (dqxyVoltRef.x1 >= dqxyVoltSat.x1)
        {
            dqxyVoltRef.x1 = dqxyVoltSat.x1;
        }
        else if (dqxyVoltRef.x1 <= -dqxyVoltSat.x1)
        {
            dqxyVoltRef.x1 = -dqxyVoltSat.x1;
        }
        IX1.error_km1 = IX1.error;
        IX1.inte_km1 = IX1.inte_k;

        // PI: Y1 current
        IY1.error = dqxyCurrRef.y1 - dqxyCurrMeas.y1;
        IY1.inte_k = KINTE*(IY1.error + IY1.error_km1) + IY1.inte_km1;
        dqxyVoltRef.y1 = IY1.KP*(IY1.error + IY1.KI*IY1.inte_k);

        if (dqxyVoltRef.y1 >= dqxyVoltSat.y1)
        {
            dqxyVoltRef.y1 = dqxyVoltSat.y1;
        }
        else if (dqxyVoltRef.y1 <= -dqxyVoltSat.y1)
        {
            dqxyVoltRef.y1 = -dqxyVoltSat.y1;
        }
        IY1.error_km1 = IY1.error;
        IY1.inte_km1 = IY1.inte_k;

        // PI: X2 current
        IX2.error = dqxyCurrRef.x2 - dqxyCurrMeas.x2;
        IX2.inte_k = KINTE*(IX2.error + IX2.error_km1) + IX2.inte_km1;
        dqxyVoltRef.x2 = IX2.KP*(IX2.error + IX2.KI*IX2.inte_k);

        if (dqxyVoltRef.x2 >= dqxyVoltSat.x2)
        {
            dqxyVoltRef.x2 = dqxyVoltSat.x2;
        }
        else if (dqxyVoltRef.x2 <= -dqxyVoltSat.x2)
        {
            dqxyVoltRef.x2 = -dqxyVoltSat.x2;
        }
        IX2.error_km1 = IX2.error;
        IX2.inte_km1 = IX2.inte_k;

        // PI: Y2 current
        IY2.error = dqxyCurrRef.y2 - dqxyCurrMeas.y2;
        IY2.inte_k = KINTE*(IY2.error + IY2.error_km1) + IY2.inte_km1;
        dqxyVoltRef.y2 = IY2.KP*(IY2.error + IY2.KI*IY2.inte_k);

        if (dqxyVoltRef.y2 >= dqxyVoltSat.y2)
        {
            dqxyVoltRef.y2 = dqxyVoltSat.y2;
        }
        else if (dqxyVoltRef.y2 <= -dqxyVoltSat.y2)
        {
            dqxyVoltRef.y2 = -dqxyVoltSat.y2;
        }
        IY2.error_km1 = IY2.error;
        IY2.inte_km1 = IY2.inte_k;


        // Transformada inversa de Park (dqxy -> abxy)
        abxyVoltRef.alpha = INV_PARK_AlPHA(dqxyVoltRef);
        abxyVoltRef.beta  = INV_PARK_BETA(dqxyVoltRef);
        abxyVoltRef.x1    = INV_PARK_X1(dqxyVoltRef);
        abxyVoltRef.y1    = INV_PARK_Y1(dqxyVoltRef);
        abxyVoltRef.x2    = INV_PARK_X2(dqxyVoltRef);
        abxyVoltRef.y2    = INV_PARK_Y2(dqxyVoltRef);

        // Transformada inversa de Clarke (abxy -> abc)
        PhaseVoltRef.A1 = INV_CLARKE_A1(abxyVoltRef);
        PhaseVoltRef.B1 = INV_CLARKE_B1(abxyVoltRef);
        PhaseVoltRef.C1 = INV_CLARKE_C1(abxyVoltRef);
        PhaseVoltRef.A2 = INV_CLARKE_A2(abxyVoltRef);
        PhaseVoltRef.B2 = INV_CLARKE_B2(abxyVoltRef);
        PhaseVoltRef.C2 = INV_CLARKE_C2(abxyVoltRef);
        PhaseVoltRef.A3 = INV_CLARKE_A3(abxyVoltRef);
        PhaseVoltRef.B3 = INV_CLARKE_B3(abxyVoltRef);
        PhaseVoltRef.C3 = INV_CLARKE_C3(abxyVoltRef);

        // Signal adaptation
        PhaseVoltRefpu.A1 = (PhaseVoltRef.A1/Vn + 1)*0.5;
        PhaseVoltRefpu.B1 = (PhaseVoltRef.B1/Vn + 1)*0.5;
        PhaseVoltRefpu.C1 = (PhaseVoltRef.C1/Vn + 1)*0.5;

        PhaseVoltRefpu.A2 = (PhaseVoltRef.A2/Vn + 1)*0.5;
        PhaseVoltRefpu.B2 = (PhaseVoltRef.B2/Vn + 1)*0.5;
        PhaseVoltRefpu.C2 = (PhaseVoltRef.C2/Vn + 1)*0.5;

        PhaseVoltRefpu.A3 = (PhaseVoltRef.A3/Vn + 1)*0.5;
        PhaseVoltRefpu.B3 = (PhaseVoltRef.B3/Vn + 1)*0.5;
        PhaseVoltRefpu.C3 = (PhaseVoltRef.C3/Vn + 1)*0.5;


        // First converter
        EPwm1Regs.CMPA.bit.CMPA = (Uint32)(pwm_period_ctrl*PhaseVoltRefpu.A1);
        EPwm2Regs.CMPA.bit.CMPA = (Uint32)(pwm_period_ctrl*PhaseVoltRefpu.B1);
        EPwm3Regs.CMPA.bit.CMPA = (Uint32)(pwm_period_ctrl*PhaseVoltRefpu.C1);

        // Second converter
        EPwm4Regs.CMPA.bit.CMPA = (Uint32)(pwm_period_ctrl*PhaseVoltRefpu.A2);
        EPwm5Regs.CMPA.bit.CMPA = (Uint32)(pwm_period_ctrl*PhaseVoltRefpu.B2);
        EPwm6Regs.CMPA.bit.CMPA = (Uint32)(pwm_period_ctrl*PhaseVoltRefpu.C2);

        // Third converter
        EPwm7Regs.CMPA.bit.CMPA = (Uint32)(pwm_period_ctrl*PhaseVoltRefpu.A3);
        EPwm8Regs.CMPA.bit.CMPA = (Uint32)(pwm_period_ctrl*PhaseVoltRefpu.B3);
        EPwm9Regs.CMPA.bit.CMPA = (Uint32)(pwm_period_ctrl*PhaseVoltRefpu.C3);

        EPwm10Regs.CMPA.bit.CMPA = PWM_PERIOD_Q2A*0.5;

        t2 = GET_LOC_TIME(CpuTimer0);

        elapsed1 = fabs(t1 - t2)*0.005;
      }
        // Q2A off
        GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;// Forward Run(DI1) -> off
        GpioDataRegs.GPBCLEAR.bit.GPIO45 = 1;// Reverse Run(DI3) -> off
        EPwm10Regs.CMPA.bit.CMPA = PWM_PERIOD_Q2A*0;// Torque ref -> 0

        // 9Ph off
        EPwm1Regs.CMPA.bit.CMPA = pwm_period_ctrl*0;
        EPwm2Regs.CMPA.bit.CMPA = pwm_period_ctrl*0;
        EPwm3Regs.CMPA.bit.CMPA = pwm_period_ctrl*0;

        EPwm4Regs.CMPA.bit.CMPA = pwm_period_ctrl*0;
        EPwm5Regs.CMPA.bit.CMPA = pwm_period_ctrl*0;
        EPwm6Regs.CMPA.bit.CMPA = pwm_period_ctrl*0;

        EPwm7Regs.CMPA.bit.CMPA = pwm_period_ctrl*0;
        EPwm8Regs.CMPA.bit.CMPA = pwm_period_ctrl*0;
        EPwm9Regs.CMPA.bit.CMPA = pwm_period_ctrl*0;


}
///////////////////////////////////////////////////////////////////
//////////////////////      END MAIN      /////////////////////////
///////////////////////////////////////////////////////////////////

interrupt void synchronous_isr(void)
{
    synch    = 1;

    EPwm1Regs.ETCLR.bit.INT = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

interrupt void adca1_isr(void)
{

    adcResult.A0 =  AdcaResultRegs.ADCRESULT0;
    adcResult.B0 =  AdcbResultRegs.ADCRESULT0;
    adcResult.A1 =  AdcaResultRegs.ADCRESULT1;
    adcResult.B1 =  AdcbResultRegs.ADCRESULT1;
    adcResult.A2 =  AdcaResultRegs.ADCRESULT2;
    adcResult.B2 =  AdcbResultRegs.ADCRESULT2;

    PhaseCurrMeas.A1 = adcResult.A0*GAIN.A0 - OFFSET.A0;
    PhaseCurrMeas.A2 = adcResult.A1*GAIN.A1 - OFFSET.A1;
    PhaseCurrMeas.A3 = adcResult.A2*GAIN.A2 - OFFSET.A2;
    PhaseCurrMeas.B1 = adcResult.B0*GAIN.B0 - OFFSET.B0;
    PhaseCurrMeas.B2 = adcResult.B1*GAIN.B1 - OFFSET.B1;
    PhaseCurrMeas.B3 = adcResult.B2*GAIN.B2 - OFFSET.B2;


    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;


    if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
     {
       AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;
       AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
     }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


interrupt void eQEP_isr(){ // eQEP Interrupt Subrutine

    EQEP1.transition=fabs(rotor.wm_k);

    if(EQEP1.state==1 && EQEP1.transition>15)
    {
         EQep1Regs.QCAPCTL.bit.CCPS = 0x4; //SYSCLKOUT/16
         EQep1Regs.QCAPCTL.bit.UPPS = 0x0; //X=1
         EQEP1.state=2;
    }
    if(EQEP1.state==2)
    {
         if(EQEP1.transition>20)
         {
              EQep1Regs.QCAPCTL.bit.CCPS = 0x0; //SYSCLKOUT/1
              EQep1Regs.QCAPCTL.bit.UPPS = 0x2; //X=4
              EQEP1.state=3;
         }
         if(EQEP1.transition<5)
         {
              EQep1Regs.QCAPCTL.bit.CCPS = 0x5; //SYSCLKOUT/32
              EQep1Regs.QCAPCTL.bit.UPPS = 0x0; // X=1
              EQEP1.state=1;
         }
    }
    if(EQEP1.state==3 && EQEP1.transition<15)
    {
         EQep1Regs.QCAPCTL.bit.CCPS = 0x4; //SYSCLKOUT/16
         EQep1Regs.QCAPCTL.bit.UPPS = 0x0; // X = 1
         EQEP1.state=2;
    }



    #ifdef EQEP_SAME_DIR
        if(EQep1Regs.QEPSTS.bit.QDF != 1)
        {
             EQEP1.eqeptmr = 0 - (float)EQep1Regs.QCPRDLAT;
             EQEP1.hdx     = 0 - (float)EQep1Regs.QPOSLAT;
        }
        else
        {
             EQEP1.eqeptmr = (float)EQep1Regs.QCPRDLAT;
             EQEP1.hdx     = (float)QEPCNTMAX - (float)EQep1Regs.QPOSLAT;
        }
    #else
        if(EQep1Regs.QEPSTS.bit.QDF != 1){
             EQEP1.eqeptmr = (float)EQep1Regs.QCPRDLAT;
             EQEP1.hdx     = (float)EQep1Regs.QPOSLAT;
        }
        else
        {
             EQEP1.eqeptmr = 0 - (float)EQep1Regs.QCPRDLAT;
             EQEP1.hdx     =  (float)EQep1Regs.QPOSLAT - (float)QEPCNTMAX;
        }
    #endif

    EQEP1.hvel_radsec = (float)EQEP1.hdx*(float)hvfactor;



    if(EQEP1.state==1 && EQEP1.eqeptmr!=0)
    {
         EQEP1.veloc_radsec = (float)velocfactor_x1_32/(float)EQEP1.eqeptmr;
    }
    else if(EQEP1.state==2 && EQEP1.eqeptmr!=0)
    {
         EQEP1.veloc_radsec = (float)velocfactor_x1_16/(float)EQEP1.eqeptmr;
    }
    else if(EQEP1.state==3 && EQEP1.eqeptmr!=0)
    {
         EQEP1.veloc_radsec = (float)velocfactor_x4_1/(float)EQEP1.eqeptmr;
    }
    if(EQEP1.hdx==0 && EQEP1.eqeptmr<30000)
    {
         EQEP1.veloc_radsec = 0;
    }
    else if (EQEP1.eqeptmr==0)
   {
        EQEP1.veloc_radsec = 0;
   }


    if(EQep1Regs.QEPSTS.bit.COEF)
    {
         EQep1Regs.QEPSTS.bit.COEF = 1;
         EQEP1.veloc_radsec = rotor.wm_km1;
    }
    if(EQep1Regs.QEPSTS.bit.CDEF)
    {
         EQEP1.veloc_radsec = rotor.wm_km1;
         EQep1Regs.QEPSTS.bit.CDEF = 1;
    }


    rotor.wm_k      = (float) EQEP1.veloc_radsec;
    rotor.thetae    = P*Pul2rad*EQep1Regs.QPOSCNT;


    if (rotor.wm_k>rotor.wm_km1 + VSTEPMAX)
    {
         rotor.wm_k = rotor.wm_km1 + VSTEPMAX;
    }
    else if (rotor.wm_k < rotor.wm_km1 - VSTEPMAX)
    {
         rotor.wm_k = rotor.wm_km1 - VSTEPMAX;
    }

    if ((rotor.wm_k > Wm_MAX)||(rotor.wm_k < -Wm_MAX))
    {
         rotor.wm_k = rotor.wm_km1;
    }

    rotor.wm_km1 = rotor.wm_k;

    EQEP1.init = 0;

    EQep1Regs.QCLR.bit.UTO      = 1;      // Clears Unit Time Out Interrupt Flag
    EQep1Regs.QCLR.bit.INT      = 1;      // Clears Global EQEP1 Interrupt Flag
    PieCtrlRegs.PIEACK.bit.ACK5 = 1;      // Clear the PIEACK of Group 5 for enables Interrupt Resquest at CPU Level
}
