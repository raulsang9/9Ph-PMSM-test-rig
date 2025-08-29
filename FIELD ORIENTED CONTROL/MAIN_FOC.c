//#############################################################################
//
// FILE:   MAIN_FOC.c
//
// TITLE:  Main project to control 9ph SiC converter
//
// VERSION: V0.1
//
// LAST EDITOR: RSG
//
// DATE: 21/07/2025
//
//COMMENT: Todavía no se ha probado en bancada

//#############################################################################
// Included Files
#include "lablib.h"
#include "data_type.h"

#ifdef FOC
///////////////////////////////////////////////////////////////////
//////////////////////        MAIN        /////////////////////////
///////////////////////////////////////////////////////////////////
void main(void){

    dqxyVoltSat.q = VQ_MAX;

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



        /*--------------------- PI: D current --------------------*/
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

        /*--------------------- PI: D current --------------------*/
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

        /*--------------------- PI: X1 current --------------------*/
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

        /*--------------------- PI: Y1 current --------------------*/
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

        /*--------------------- PI: X2 current --------------------*/
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

        /*--------------------- PI: Y2 current --------------------*/
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
