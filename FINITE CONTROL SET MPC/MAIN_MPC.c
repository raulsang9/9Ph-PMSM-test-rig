//#############################################################################
//
// FILE:   Main Project
//
// TITLE:  Main project to control 9ph SiC converter
//
// VERSION: V0.1
//
// LAST EDITOR: BRG & RSG & PMM
//
// DATE: 21/01/2025
//
//COMMENT:

//#############################################################################
// Included Files
#include "lablib.h"
#include "data_type.h"

#if defined(MPC) || defined(MPCta)
///////////////////////////////////////////////////////////////////
//////////////////////        MAIN        /////////////////////////
///////////////////////////////////////////////////////////////////
void main(void){

    int k = 0;

    for(k = 0; k<19; k++)
    {
        GAMMA_V[k].alpha *= GAMMA11(Tm) * Vdc;
        GAMMA_V[k].beta  *= GAMMA22(Tm) * Vdc;
        GAMMA_V[k].x1    *= GAMMA33(Tm) * Vdc;
        GAMMA_V[k].y1    *= GAMMA44(Tm) * Vdc;
        GAMMA_V[k].x2    *= GAMMA55(Tm) * Vdc;
        GAMMA_V[k].y2    *= GAMMA66(Tm) * Vdc;
    }

    g_experiment.strSize = sizeof(g_experiment);
    g_experiment.startTime = 0.9f*tup;

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

    ConfigCpuTimer(&CpuTimer0, 200, (Uint32)((float)2e6)*Tm);
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

    // Activacion de las interrupciones
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


    while(cont < 2*seg)
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
    {
         t1 = GET_LOC_TIME(CpuTimer0);
         driveSET();

         vel_profile();

        /*-------------------------- SPEED PI -------------------------------*/
        omega.error = rotor.wm_ref - rotor.wm_k_filt;
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


        sintheta = sinf(rotor.thetae);
        costheta = cosf(rotor.thetae);


        // Dentro del bucle de adquisición
         if (cont > g_experiment.startTime * seg && end_guardado == 0)
         {
            data_l++;
            if (data_l == DEC)
            {
                if (incr < LEN)
                {
                    dataLogging(); //Guardar SD

                    incr++;

                    data_l = 0;
                    if (incr == LEN)
                    {
                        end_guardado = 1;
                        g_experiment.thetaend = rotor.thetae;
                    }
                }
            }
         }

         // Reference current
         abxyCurrRef.alpha = INV_PARK_AlPHA(dqxyCurrRef);
         abxyCurrRef.beta  = INV_PARK_BETA(dqxyCurrRef);


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

        /*--------------------- OPEN-LOOP OBSERVER & COST FUNCTION --------------------*/

        // K+1 CURRENTS
        #ifdef MPCta
        abxyCurrkp1.alpha     = FI.c11        *abxyCurrMeas.alpha     +  a12*rotor.we_k*abxyCurrMeas.beta        +  ta*GAMMA_V[x_opt].alpha     -  delta21*rotor.we_k*sintheta;
        abxyCurrkp1.beta      = a21*rotor.we_k*abxyCurrMeas.alpha     +          FI.c22*abxyCurrMeas.beta        +  ta*GAMMA_V[x_opt].beta      +  delta21*rotor.we_k*costheta;
        abxyCurrkp1.x1        = FI.c33        *abxyCurrMeas.x1                                                   +  ta*GAMMA_V[x_opt].x1                                      ;
        abxyCurrkp1.y1        = FI.c44        *abxyCurrMeas.y1                                                   +  ta*GAMMA_V[x_opt].y1                                      ;
        abxyCurrkp1.x2        = FI.c55        *abxyCurrMeas.x2                                                   +  ta*GAMMA_V[x_opt].x2                                      ;
        abxyCurrkp1.y2        = FI.c66        *abxyCurrMeas.y2                                                   +  ta*GAMMA_V[x_opt].y2                                      ;
        #elif
        abxyCurrkp1.alpha     = FI.c11        *abxyCurrMeas.alpha     +  a12*rotor.we_k*abxyCurrMeas.beta        +  GAMMA_V[x_opt].alpha     -  delta21*rotor.we_k*sintheta;
        abxyCurrkp1.beta      = a21*rotor.we_k*abxyCurrMeas.alpha     +          FI.c22*abxyCurrMeas.beta        +  GAMMA_V[x_opt].beta      +  delta21*rotor.we_k*costheta;
        abxyCurrkp1.x1        = FI.c33        *abxyCurrMeas.x1                                                   +  GAMMA_V[x_opt].x1                                      ;
        abxyCurrkp1.y1        = FI.c44        *abxyCurrMeas.y1                                                   +  GAMMA_V[x_opt].y1                                      ;
        abxyCurrkp1.x2        = FI.c55        *abxyCurrMeas.x2                                                   +  GAMMA_V[x_opt].x2                                      ;
        abxyCurrkp1.y2        = FI.c66        *abxyCurrMeas.y2                                                   +  GAMMA_V[x_opt].y2                                      ;
        #endif
        // K+2 CURRENTS (CONSTANT)
        abxyCurrkp2cnst.alpha = FI.c11        *abxyCurrkp1.alpha      + a12*rotor.we_k*abxyCurrkp1.beta                                      -  delta21*rotor.we_k*sintheta;
        abxyCurrkp2cnst.beta  = a21*rotor.we_k*abxyCurrkp1.alpha      + FI.c22        *abxyCurrkp1.beta                                      +  delta21*rotor.we_k*costheta;
        abxyCurrkp2cnst.x1    = FI.c33        *abxyCurrkp1.x1                                                                                                              ;
        abxyCurrkp2cnst.y1    = FI.c44        *abxyCurrkp1.y1                                                                                                              ;
        abxyCurrkp2cnst.x2    = FI.c55        *abxyCurrkp1.x2                                                                                                              ;
        abxyCurrkp2cnst.y2    = FI.c66        *abxyCurrkp1.y2                                                                                                              ;


        if (cont > g_experiment.startTime * seg && end_guardado == 0)
         {
            conm_diff();
         }


        #ifdef MPCta
        /*---------------------------- ta CALCULATION --------------------------*/
        //Vref = inv(GAMMA) * (dq_ref - FI*xkp1 - DELTA);
        valpha_ref = (abxyCurrRef.alpha - FI.c11*abxyCurrkp1.alpha - a12*rotor.we_k*abxyCurrkp1.beta  + delta21*rotor.we_k*sintheta) / GAMMA11(Tm);
        vbeta_ref  = (abxyCurrRef.beta  - FI.c22*abxyCurrkp1.beta  - a21*rotor.we_k*abxyCurrkp1.alpha - delta21*rotor.we_k*costheta) / GAMMA22(Tm);

        v_ref_mod = sqrt(valpha_ref*valpha_ref + vbeta_ref*vbeta_ref);

        if (v_ref_mod > Vdc*0.64)
        {
            ta_opt = 0.9999;
        }else
        {
            ta_opt = v_ref_mod/(Vdc*0.64);
        }
        ta = k_filt*ta_opt + (1-k_filt)*ta;
        #endif

        /*--------------------- VOLTAGE VECTOR OPTIMIZATION --------------------*/
        Jopt = 9999999999999;x_opt_km1  = x_opt;

        for (k=0; k<19 ; k++)
        {
            // K+2 CURRENTS (VARIABLE)
            #ifdef MPCta
            abxyCurrkp2.alpha = abxyCurrkp2cnst.alpha + ta*GAMMA_V[k].alpha;
            abxyCurrkp2.beta  = abxyCurrkp2cnst.beta  + ta*GAMMA_V[k].beta;
            abxyCurrkp2.x1    = abxyCurrkp2cnst.x1    + ta*GAMMA_V[k].x1;
            abxyCurrkp2.y1    = abxyCurrkp2cnst.y1    + ta*GAMMA_V[k].y1;
            abxyCurrkp2.x2    = abxyCurrkp2cnst.x2    + ta*GAMMA_V[k].x2;
            abxyCurrkp2.y2    = abxyCurrkp2cnst.y2    + ta*GAMMA_V[k].y2;
            #elif
            abxyCurrkp2.alpha = abxyCurrkp2cnst.alpha + GAMMA_V[k].alpha;
            abxyCurrkp2.beta  = abxyCurrkp2cnst.beta  + GAMMA_V[k].beta;
            abxyCurrkp2.x1    = abxyCurrkp2cnst.x1    + GAMMA_V[k].x1;
            abxyCurrkp2.y1    = abxyCurrkp2cnst.y1    + GAMMA_V[k].y1;
            abxyCurrkp2.x2    = abxyCurrkp2cnst.x2    + GAMMA_V[k].x2;
            abxyCurrkp2.y2    = abxyCurrkp2cnst.y2    + GAMMA_V[k].y2;
            #endif
            // PREDICTION ERROR
            abxyerror.alpha = abxyCurrRef.alpha - abxyCurrkp2.alpha;
            abxyerror.beta  = abxyCurrRef.beta  - abxyCurrkp2.beta;
            abxyerror.x1    = abxyCurrRef.x1    - abxyCurrkp2.x1;
            abxyerror.y1    = abxyCurrRef.y1    - abxyCurrkp2.y1;
            abxyerror.x2    = abxyCurrRef.x2    - abxyCurrkp2.x2;
            abxyerror.y2    = abxyCurrRef.y2    - abxyCurrkp2.y2;


            #ifdef MUS2
                if (k==18)
                {
                    mus2_var = MUS2;
                }
                else
                {
                    mus2_var = 1;
                }

                Cf = CF_EVAL_MSE(abxyerror)*mus2_var;
            #else
                Cf = CF_EVAL_MSE(abxyerror);
            #endif

            // COST FUNCTION


            if(Jopt > Cf)
            {
                x_opt = k;
                Jopt  = Cf;
            }
        }

        while(synch == 0);synch = 0;cont++;

        /*------------------- ACTUACIÓN ---------------------*/
        Control_Action_Settings();

        t2 = GET_LOC_TIME(CpuTimer0);elapsed = fabs(t1 - t2);
        }

        g_experiment.SwFrequency *= ((float)(CTRLF/((g_experiment.dataLEN*18.0f*g_experiment.Decimation))));

        GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;

        EPwm1Regs.CMPA.bit.CMPA = pwm_period_ctrl*0;
        EPwm2Regs.CMPA.bit.CMPA = pwm_period_ctrl*0;
        EPwm3Regs.CMPA.bit.CMPA = pwm_period_ctrl*0;

        EPwm4Regs.CMPA.bit.CMPA = pwm_period_ctrl*0;
        EPwm5Regs.CMPA.bit.CMPA = pwm_period_ctrl*0;
        EPwm6Regs.CMPA.bit.CMPA = pwm_period_ctrl*0;

        EPwm7Regs.CMPA.bit.CMPA = pwm_period_ctrl*0;
        EPwm8Regs.CMPA.bit.CMPA = pwm_period_ctrl*0;
        EPwm9Regs.CMPA.bit.CMPA = pwm_period_ctrl*0;

        EPwm10Regs.CMPA.bit.CMPA = PWM_PERIOD_Q2A*0;

    }
///////////////////////////////////////////////////////////////////
//////////////////////      END MAIN      /////////////////////////
///////////////////////////////////////////////////////////////////
#endif
