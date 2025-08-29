/*
 * lablib.c
 *
 *  Created on: 17 ene. 2025
 *      Author: PMM x RSG
 *
 *  Indice: -> FUNCIONES
 *             | ~ ParTrapezoidal()
 *             | ~ ParDataset()
 *             | LowPassFilter(void)
 *          -> CONFIGURACIONES
 *             | ConfigureGPIO()
 *             | ConfigureEPWM_Q2ADRIVE(volatile struct EPWM_REGS *ePWM_Regs)
 *             | InitEPWM(void)
 *             | ConfigureEPWM(volatile struct EPWM_REGS *ePWM_Regs)
 *             | eQEP_init(void)
 *             | ADC_init(void)
 *             | conm_diff(void)
 *          -> INTERRUPCIONES
 *             | adca1_isr(void)
 *             | eQEP_isr()
 *             | synchronous_isr(void)
 *
 */

#include "f28x_project.h"
#include "f2838x_device.h"
#include "f2838x_examples.h"
#include "math.h"
#include "IQmathLib.h"
#include <stdio.h>
#include "lablib.h"
#include "data_type.h"


// ----------------------------------------------------------------------
//  FUNCIONES VARIAS
// ----------------------------------------------------------------------

void vel_profile(void)
{
    //Tload:_____
    //     /     |
    // ___/      |__
    //  tup     tcte    -Cortesia de RdF

    #ifdef W_TRAP
        if (cont < seg * (tup))
        {
            rotor.wm_ref += rotor.wm_trg / ((tup) * seg);
        }
        else if (cont < ((tup) + tcte) * seg)
        {
            rotor.wm_ref = rotor.wm_trg;
            Q2A.RevrRun = ON;
            if (cont < ((tup) + 0.5f) * seg)
            {
                Q2A.TorqRef = 0;
            }
            else if (cont < ((tup) + 0.5f + tup_t) * seg)
            {
                Q2A.TorqRef += TLOAD / ((tup_t) * seg);
            }
            else
            {
                Q2A.TorqRef = TLOAD;
            }
        }
        else
        {
            Q2A.TorqRef = 0;
            rotor.wm_ref -= rotor.wm_trg / ((tup) * seg);
        }
    #endif

    //Tload:.  /\_.
    //     / \/   |
    // ___/       |__
    //  tup     tend    -Cortesia de RdF
    #ifdef W_DATASET

        if (cont < seg * (tup))
        {
            Q2A.RevrRun = ON;
            rotor.wm_ref += SPEEDINIT / ((tup) * seg);
            Q2A.TorqRef  += TLOAD     / ((tup) * seg);
        }
        else if (cont < ((tup) + tcte) * seg)
        {
            if(cont > ((tup) + vel_prof[indx][0]) * seg){indx++;}

            rotor.wm_ref += vel_prof[indx][1] * Tm;

            Q2A.TorqRef = TLOAD;
        }
        else
        {
            rotor.wm_ref -= SPEEDEND / ((tup) * seg);
            Q2A.TorqRef  -= TLOAD    / ((tup) * seg);
        }
    #endif

    //Tload:_____
    //     /     \
    // ___/       \__
    //  tup       tcte    -Cortesia de RdF & Rutmin

    #ifdef W_JUAN_TRANS
        if (cont < seg * (tup))
        {
            rotor.wm_ref += rotor.wm_trg / ((tup) * seg);
            Q2A.RevrRun = ON;
            Q2A.TorqRef += TLOAD / ((tup) * seg);
        }
        else if (cont < ((tup) + tcte) * seg)
        {
            rotor.wm_ref = rotor.wm_trg;
            Q2A.TorqRef = TLOAD;

        }
        else
        {
            Q2A.TorqRef -= TLOAD / ((tup) * seg);
            rotor.wm_ref -= rotor.wm_trg / ((tup) * seg);
        }
    #endif
}


// Función para actualizar el buffer y calcular la media
#ifdef NULL_COMP
    void update_buffer(void) {
        // Procesar componente 'd'
        float oldest_d = buffer[buffer_write_index].d;
        CG_0_AVG.d = (CG_0_AVG.d * LEN_BUFF - oldest_d + new_data.d)* inv_len_buff;
        buffer[buffer_write_index].d = new_data.d;

        // Procesar componente 'q'
        float oldest_q = buffer[buffer_write_index].q;
        CG_0_AVG.q = (CG_0_AVG.q * LEN_BUFF - oldest_q + new_data.q)* inv_len_buff;
        buffer[buffer_write_index].q = new_data.q;

        // Procesar componente 'x1'
        float oldest_x1 = buffer[buffer_write_index].x1;
        CG_0_AVG.x1 = (CG_0_AVG.x1 * LEN_BUFF - oldest_x1 + new_data.x1)* inv_len_buff;
        buffer[buffer_write_index].x1 = new_data.x1;

        // Procesar componente 'y1'
        float oldest_y1 = buffer[buffer_write_index].y1;
        CG_0_AVG.y1 = (CG_0_AVG.y1 * LEN_BUFF - oldest_y1 + new_data.y1)* inv_len_buff;
        buffer[buffer_write_index].y1 = new_data.y1;

        // Procesar componente 'x2'
        float oldest_x2 = buffer[buffer_write_index].x2;
        CG_0_AVG.x2 = (CG_0_AVG.x2 * LEN_BUFF - oldest_x2 + new_data.x2)* inv_len_buff;
        buffer[buffer_write_index].x2 = new_data.x2;

        // Procesar componente 'y2'
        float oldest_y2 = buffer[buffer_write_index].y2;
        CG_0_AVG.y2 = (CG_0_AVG.y2 * LEN_BUFF - oldest_y2 + new_data.y2)* inv_len_buff;
        buffer[buffer_write_index].y2 = new_data.y2;

        // Actualizar índice de escritura (circular)
        buffer_write_index = (buffer_write_index + 1) % LEN_BUFF;

        if ((buffer_write_index > (LEN_BUFF - 2.0f)) && (buff_complete == 0))
        {
            buff_complete = 1;
        }
    }
#endif
void LowPassFilter(void)
{
    // Variable estÃ¡tica para almacenar la salida anterior (y[k-1])
    static float y_prev = 0.0f;

    // Calcular la constante de tiempo: tau = 1 / (2*pi*CO_freq)
    float tau = 1.0f / (2.0f * (float)M_PI * CO_freq);

    // Calcular el coeficiente alpha
    float alpha = Tm / (tau + Tm);

    // EcuaciÃ³n discreta del filtro:
    //   y[k] = y[k-1] + alpha * (x[k] - y[k-1])
    // En este caso, x[k] = rotor_wm_k (velocidad medida)
    float y = y_prev + alpha * (rotor.wm_k - y_prev);

    // Sobrescribimos en rotor_wm_k para que ya quede filtrada
    rotor.wm_k_filt = y;

    // Actualizamos el estado para la siguiente iteraciÃ³n
    y_prev = y;
}

void conm_diff(void)
{
    int i;
    for(i = 0; i < 9; ++i)
    {
        g_experiment.SwFrequency += fabsf(Sout[x_opt][i] - Sout[x_opt_km1][i]);
    }
}

void driveSET(void)
{
    if(Q2A.RevrRun == ON)GpioDataRegs.GPBSET.bit.GPIO39   = 1;//DI1
    else        GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;

    if(Q2A.FwrdRun == ON)GpioDataRegs.GPBSET.bit.GPIO45   = 1;//DI3
    else        GpioDataRegs.GPBCLEAR.bit.GPIO45 = 1;

    if(Q2A.Reset == ON)GpioDataRegs.GPDSET.bit.GPIO97   = 1;//DI4
    else        GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    EPwm12Regs.CMPA.bit.CMPA = (Uint32)(PWM_PERIOD_Q2A*NM2PU(Q2A.TorqRef));// AI1

}

void dataLogging(void)
{
#ifdef DATLOGDEF
    g_experiment.data[incr][0] = dqxyCurrMeas.d;
    g_experiment.data[incr][1] = dqxyCurrRef.q;
    g_experiment.data[incr][2] = dqxyCurrMeas.q;
    g_experiment.data[incr][3] = rotor.wm_k;
    g_experiment.data[incr][4] = rotor.wm_ref;
    g_experiment.data[incr][5] = x_opt;
#endif

#ifdef DATLOGMPC
    g_experiment.data[incr][0] = dqxyCurrMeas.d;
    g_experiment.data[incr][1] = dqxyCurrRef.q;
    g_experiment.data[incr][2] = dqxyCurrMeas.q;
    g_experiment.data[incr][3] = dqxyCurrMeas.x1;
    g_experiment.data[incr][4] = dqxyCurrMeas.y1;
    g_experiment.data[incr][5] = dqxyCurrMeas.x2;
    g_experiment.data[incr][6] = dqxyCurrMeas.y2;
    g_experiment.data[incr][7] = rotor.wm_k;
    g_experiment.data[incr][8] = x_opt;
#endif

#ifdef MODEL_FREE
#ifdef DATLOGMID
    g_experiment.data[incr][0] = PhaseCurrMeas.A1;
    g_experiment.data[incr][1] = PhaseMidCurrMeas.A1;
    g_experiment.data[incr][2] = PhaseCurrMeas.B1;
    g_experiment.data[incr][3] = PhaseMidCurrMeas.B1;
#endif
#endif

#ifdef JUAN_TRANS
    g_experiment.data[incr][0] = dqxyCurrMeas.d;
    g_experiment.data[incr][1] = dqxyCurrRef.q;
    g_experiment.data[incr][2] = dqxyCurrMeas.q;
    g_experiment.data[incr][3] = rotor.wm_k;
    g_experiment.data[incr][4] = rotor.wm_ref;
#endif


}
// ----------------------------------------------------------------------
//  CONFIGURACIONES
// ----------------------------------------------------------------------

void ConfigureGPIO()
{
        EALLOW;                             // Enable writing to EALLOW protected registers

        GpioCtrlRegs.GPAMUX1.all = 0;       // GPIO15 ... GPIO0 = General Purpose I/O
        GpioCtrlRegs.GPAMUX2.all = 0;       // GPIO31 ... GPIO16 = General Purpose I/O
        GpioCtrlRegs.GPBMUX1.all = 0;       // GPIO47 ... GPIO32 = General Purpose I/O
        GpioCtrlRegs.GPBMUX2.all = 0;       // GPIO63 ... GPIO48 = General Purpose I/O
        GpioCtrlRegs.GPCMUX1.all = 0;       // GPIO79 ... GPIO64 = General Purpose I/O
        GpioCtrlRegs.GPCMUX2.all = 0;       // GPIO87 ... GPIO80 = General Purpose I/O
        GpioCtrlRegs.GPADIR.all = 0;
        GpioCtrlRegs.GPBDIR.all = 0;        // GPIO63-32 as inputs
        GpioCtrlRegs.GPCDIR.all = 0;        // GPIO87-64 as inputs

        GpioCtrlRegs.GPAMUX1.bit.GPIO0       = 1; // ePWM1A active
        GpioCtrlRegs.GPADIR.bit.GPIO0        = 1; // Configure GPIO0 as digital Output
        GpioDataRegs.GPACLEAR.bit.GPIO0      = 1; // Clear GPIO0

        GpioCtrlRegs.GPAMUX1.bit.GPIO1       = 1; // ePWM1B active
        GpioCtrlRegs.GPADIR.bit.GPIO1        = 1; // Configure GPIO1 as digital Output
        GpioDataRegs.GPACLEAR.bit.GPIO1      = 1; // Clear GPIO1

        GpioCtrlRegs.GPAMUX1.bit.GPIO2       = 1; // ePWM2A active
        GpioCtrlRegs.GPADIR.bit.GPIO2        = 1; // Configure GPIO2 as digital Output
        GpioDataRegs.GPACLEAR.bit.GPIO2      = 1; // Clear GPIO2

        GpioCtrlRegs.GPAMUX1.bit.GPIO3       = 1; // ePWM2B active
        GpioCtrlRegs.GPADIR.bit.GPIO3        = 1; // Configure GPIO3 as digital Output
        GpioDataRegs.GPACLEAR.bit.GPIO3      = 1; // Clear GPIO3

        GpioCtrlRegs.GPAMUX1.bit.GPIO4       = 1; // ePWM3A active
        GpioCtrlRegs.GPADIR.bit.GPIO4        = 1; // Configure GPIO4 as digital Output
        GpioDataRegs.GPACLEAR.bit.GPIO4      = 1; // Clear GPIO4

        GpioCtrlRegs.GPAMUX1.bit.GPIO5       = 1; // ePWM3B active
        GpioCtrlRegs.GPADIR.bit.GPIO5        = 1; // Configure GPIO5 as digital Output
        GpioDataRegs.GPACLEAR.bit.GPIO5      = 1; // Clear GPIO5

        GpioCtrlRegs.GPAMUX1.bit.GPIO6       = 1; // ePWM4A active
        GpioCtrlRegs.GPADIR.bit.GPIO6        = 1; // Configure GPIO6 as digital Output
        GpioDataRegs.GPACLEAR.bit.GPIO6      = 1; // Clear GPIO6

        GpioCtrlRegs.GPAMUX1.bit.GPIO7       = 1; // ePWM4B active
        GpioCtrlRegs.GPADIR.bit.GPIO7        = 1; // Configure GPIO7 as digital Output
        GpioDataRegs.GPACLEAR.bit.GPIO7      = 1; // Clear GPIO7

        GpioCtrlRegs.GPAMUX1.bit.GPIO8       = 1; // ePWM5A active
        GpioCtrlRegs.GPADIR.bit.GPIO8        = 1; // Configure GPIO8 as digital Output
        GpioDataRegs.GPACLEAR.bit.GPIO8      = 1; // Clear GPIO8

        GpioCtrlRegs.GPAMUX1.bit.GPIO9       = 1; // ePWM5B active
        GpioCtrlRegs.GPADIR.bit.GPIO9        = 1; // Configure GPIO9 as digital Output
        GpioDataRegs.GPACLEAR.bit.GPIO9      = 1; // Clear GPIO9

        GpioCtrlRegs.GPAMUX1.bit.GPIO10      = 1; // ePWM6A active
        GpioCtrlRegs.GPADIR.bit.GPIO10       = 1; // Configure GPIO10 as digital Output
        GpioDataRegs.GPACLEAR.bit.GPIO10      = 1; // Clear GPIO10

        GpioCtrlRegs.GPAMUX1.bit.GPIO11      = 1; // ePWM6B active
        GpioCtrlRegs.GPADIR.bit.GPIO11       = 1; // Configure GPIO11 as digital Output
        GpioDataRegs.GPACLEAR.bit.GPIO11     = 1; // Clear GPIO11

        GpioCtrlRegs.GPAMUX1.bit.GPIO12       = 1; // ePWM7Aactive
        GpioCtrlRegs.GPADIR.bit.GPIO12        = 1; // Configure GPIO12 as digital Output
        GpioDataRegs.GPACLEAR.bit.GPIO12      = 1; // Clear GPIO12

        GpioCtrlRegs.GPAMUX1.bit.GPIO13      = 1; // ePWM7B active
        GpioCtrlRegs.GPADIR.bit.GPIO13       = 1; // Configure GPIO13 as digital Output
        GpioDataRegs.GPACLEAR.bit.GPIO13      = 1; // Clear GPIO13

        GpioCtrlRegs.GPAMUX1.bit.GPIO14      = 1; // ePWM8A active
        GpioCtrlRegs.GPADIR.bit.GPIO14       = 1; // Configure GPIO14 as digital Output
        GpioDataRegs.GPACLEAR.bit.GPIO14     = 1; // Clear GPIO14

        GpioCtrlRegs.GPAMUX1.bit.GPIO15      = 1; // ePWM8B active
        GpioCtrlRegs.GPADIR.bit.GPIO15       = 1; // Configure GPIO15 as digital Output
        GpioDataRegs.GPACLEAR.bit.GPIO15     = 1; // Clear GPIO15

        GpioCtrlRegs.GPAGMUX2.bit.GPIO16      = 01; // ePWM9A active
        GpioCtrlRegs.GPAMUX2.bit.GPIO16       = 01; // ePWM9A active
        GpioCtrlRegs.GPADIR.bit.GPIO16       = 1; // Configure GPIO16 as digital Output
        GpioDataRegs.GPACLEAR.bit.GPIO16     = 1; // Clear GPIO16

        GpioCtrlRegs.GPAGMUX2.bit.GPIO17     = 01; // ePWM9B active
        GpioCtrlRegs.GPAMUX2.bit.GPIO17      = 01; // ePWM9B active
        GpioCtrlRegs.GPADIR.bit.GPIO17       = 1;  // Configure GPIO17 as digital Output
        GpioDataRegs.GPACLEAR.bit.GPIO17     = 1;  // Clear GPIO17

        GpioCtrlRegs.GPAGMUX2.bit.GPIO18     = 01; // ePWM10a active - Fan application
        GpioCtrlRegs.GPAMUX2.bit.GPIO18      = 01; // ePWM10a active - Fan application
        GpioCtrlRegs.GPADIR.bit.GPIO18       = 1;  // Configure GPIO18 as digital Output
        GpioDataRegs.GPACLEAR.bit.GPIO18     = 1;  // Clear GPIO18

        GpioCtrlRegs.GPAMUX2.bit.GPIO20      = 1;  // Encoder signal A active
        GpioCtrlRegs.GPADIR.bit.GPIO20       = 1;  // Configure GPIO20 as eQEP A
        GpioDataRegs.GPACLEAR.bit.GPIO20     = 1;  // Clear GPIO20

        GpioCtrlRegs.GPAMUX2.bit.GPIO21      = 1;  // Encoder signal B active
        GpioCtrlRegs.GPADIR.bit.GPIO21       = 1;  // Configure GPIO21 as eQEP B
        GpioDataRegs.GPACLEAR.bit.GPIO21     = 1;  // Clear GPIO21

        GpioCtrlRegs.GPAMUX2.bit.GPIO23      = 1;  // Encoder signal index
        GpioCtrlRegs.GPADIR.bit.GPIO23       = 1;  // Configure GPIO23 as index
        GpioDataRegs.GPACLEAR.bit.GPIO23     = 1;  // Clear GPIO23


        GpioCtrlRegs.GPAMUX2.bit.GPIO22      = 1;  // EPWM12A
        GpioCtrlRegs.GPAGMUX2.bit.GPIO22     = 1;  // EPWM12A
        GpioCtrlRegs.GPADIR.bit.GPIO22       = 1;  // Configure GPIO22 as OUTPUT
        GpioDataRegs.GPACLEAR.bit.GPIO22     = 1;  // Clear GPIO22

        GpioCtrlRegs.GPAMUX2.bit.GPIO30      = 1;  // EPWM16A
        GpioCtrlRegs.GPAGMUX2.bit.GPIO30     = 3;  // EPWM16A
        GpioCtrlRegs.GPADIR.bit.GPIO30       = 1;  // Configure GPIO30 as OUTPUT
        GpioDataRegs.GPACLEAR.bit.GPIO30     = 1;  // Clear GPIO30

        GpioCtrlRegs.GPAMUX2.bit.GPIO19      = 1;  // EPWM10B
        GpioCtrlRegs.GPAGMUX2.bit.GPIO19     = 1;  // EPWM10B
        GpioCtrlRegs.GPADIR.bit.GPIO19       = 1;  // Configure GPIO19 as OUTPUT
        GpioDataRegs.GPACLEAR.bit.GPIO19     = 1;  // Clear GPIO19

        GpioCtrlRegs.GPBMUX1.bit.GPIO39      = 0;
        GpioCtrlRegs.GPBDIR.bit.GPIO39       = 1;
        GpioDataRegs.GPBCLEAR.bit.GPIO39     = 1;  // Clear GPIO39 DI1


        GpioCtrlRegs.GPDMUX1.bit.GPIO97      = 0;
        GpioCtrlRegs.GPDDIR.bit.GPIO97       = 1;
        GpioDataRegs.GPDCLEAR.bit.GPIO97     = 1;  // Clear GPIO97 DI4


        GpioCtrlRegs.GPBMUX1.bit.GPIO45      = 0;
        GpioCtrlRegs.GPBDIR.bit.GPIO45       = 1;
        GpioDataRegs.GPBCLEAR.bit.GPIO45     = 1;  // Clear GPIO45 DI3

        EDIS;                                     // Disable writing to EALLOW protected registers
}

void ConfigureEPWM_Q2ADRIVE(volatile struct EPWM_REGS *ePWM_Regs)
{
    // Basic ePWM configuration
     EALLOW;
     //EPwm1Regs.CMPA.bit.CMPA = 0x0800;                   // Set compare A value to 2048 counts
     ePWM_Regs->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
     ePWM_Regs->TBPRD = PWM_PERIOD_Q2A;              // Set timer period
     ePWM_Regs->TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
     ePWM_Regs->TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
     ePWM_Regs->TBCTR = 0x0000;                  // Clear counter
     ePWM_Regs->TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Clock ratio to SYSCLKOUT
     ePWM_Regs->TBCTL.bit.CLKDIV = TB_DIV1;

     // Setup shadow register load on ZERO
      //
      ePWM_Regs->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
      ePWM_Regs->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
      ePWM_Regs->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
      ePWM_Regs->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

      // Enable ePWM1 as master
      //
      ePWM_Regs->EPWMSYNCOUTEN.bit.ZEROEN= 1;

        //
        // Set actions
        //
        ePWM_Regs->AQCTLA.all=0x0090;
        ePWM_Regs->AQCTLB.all=0x0900;

        //
        // Interrupt where we will change the Compare Values
        //
        ePWM_Regs->ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
        ePWM_Regs->ETSEL.bit.INTEN = 1;                // Enable INT
        ePWM_Regs->ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 3rd event


     EDIS;
}


void InitEPWM(void)
{
        ConfigureEPWM(&EPwm1Regs);
        ConfigureEPWM(&EPwm2Regs);
        ConfigureEPWM(&EPwm3Regs);
        ConfigureEPWM(&EPwm4Regs);
        ConfigureEPWM(&EPwm5Regs);
        ConfigureEPWM(&EPwm6Regs);
        ConfigureEPWM(&EPwm7Regs);
        ConfigureEPWM(&EPwm8Regs);
        ConfigureEPWM(&EPwm9Regs);
        ConfigureEPWM_Q2ADRIVE(&EPwm10Regs);
        ConfigureEPWM_Q2ADRIVE(&EPwm12Regs);
        ConfigureEPWM_Q2ADRIVE(&EPwm16Regs);

}

void ConfigureEPWM(volatile struct EPWM_REGS *ePWM_Regs)
{
    // Basic ePWM configuration
     EALLOW;
     //EPwm1Regs.CMPA.bit.CMPA = 0x0800;                   // Set compare A value to 2048 counts

        #ifdef MV5_FCS
             ePWM_Regs->TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
        #else
             ePWM_Regs->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
        #endif
     ePWM_Regs->TBPRD = PWM_PERIOD_CTRL;              // Set timer period
     ePWM_Regs->TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
     ePWM_Regs->TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
     ePWM_Regs->TBCTR = 0x0000;                  // Clear counter
     ePWM_Regs->TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Clock ratio to SYSCLKOUT
     ePWM_Regs->TBCTL.bit.CLKDIV = TB_DIV1;

     // Setup shadow register load on ZERO
      //
      ePWM_Regs->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
      ePWM_Regs->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
      ePWM_Regs->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
      ePWM_Regs->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

      // Enable ePWM1 as master
      //
      ePWM_Regs->EPWMSYNCOUTEN.bit.ZEROEN= 1;

        //
        // Set actions
        //

        #ifdef MV5_FCS
              ePWM_Regs->AQCTLA.all=0x0210;
              ePWM_Regs->AQCTLB.all=0x0120;
        #else
              ePWM_Regs->AQCTLA.all=0x0090;
              ePWM_Regs->AQCTLB.all=0x0900;
        #endif

        //
        // Interrupt where we will change the Compare Values
        //
        ePWM_Regs->ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
        ePWM_Regs->ETSEL.bit.INTEN = 1;                // Enable INT
        ePWM_Regs->ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 3rd event

        //
        // Active Low PWMs - Setup Deadband
        //
        ePWM_Regs->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
        ePWM_Regs->DBCTL.bit.POLSEL = DB_ACTV_HIC;
        ePWM_Regs->DBCTL.bit.IN_MODE = DBA_ALL;
        ePWM_Regs->DBRED.bit.DBRED = EPWM_MIN_DB;
        ePWM_Regs->DBFED.bit.DBFED = EPWM_MAX_DB;

     EDIS;
}

void eQEP_init(void)
{
    EALLOW;

        // Define eQEPA, eQEPB and eQEPI sources
         //SysCtrlRegs             // Enable the SYSCLKOUT to the GPIO
         //SysCtrlRegs.PCLKCR1.bit.EQEP1ENCLK    = 1;             // EQEP1 Module is Clocked by the SYSCLKOUT

         CpuSysRegs.PCLKCR4.bit.EQEP1 = 1;
         EQep1Regs.QEPSRCSEL.bit.QEPASEL = 0; //eQEPA Device GPIO20
         EQep1Regs.QEPSRCSEL.bit.QEPBSEL = 0; //eQEPB Device GPIO21
         EQep1Regs.QEPSRCSEL.bit.QEPISEL = 0; //eQEPI Device GPIO23

         EQep1Regs.QDECCTL.bit.QAP = 0;  // Polarity of eQEPA None
         EQep1Regs.QDECCTL.bit.QBP = 0;  // Polarity of eQEPB None
         EQep1Regs.QDECCTL.bit.QIP = 0;  // Polarity of eQEPI None

         EQep1Regs.QDECCTL.bit.QSRC = 0; // Position Counter Source (Quadrature)

         EQep1Regs.QDECCTL.bit.XCR = 0;   //  x2 resolution: Count rising edge only
         EQep1Regs.QDECCTL.bit.SWAP = 0;  // Quadrature-clock are not swap
         EQep1Regs.QDECCTL.bit.IGATE = 0; // Disable gating of Index Pulse

         // Reviewed
         EQep1Regs.QPOSMAX = QEPCNTMAX;      // Maximum position
         EQep1Regs.QPOSCNT = 0;           // Position counter
         EQep1Regs.QPOSINIT = 5709;          // Initial value

         // Reviewed
         //EQep1Regs.QEPCTL.all=0x821E; // Description is on the lines above
         EQep1Regs.QEPCTL.all=0x901E; // Description is on the lines above
         // Reviewed
         EQep1Regs.QPOSCMP =  QEPCNTMAX; // EQEP Position Compare
         EQep1Regs.QCTMR   = 0;          // Capture timer

         EQep1Regs.QPOSCTL.all = 0x0000; //Position comparison
         // Reviewed
         EQep1Regs.QUPRD = QEPUTOPRD; //Unit Time Out Period
         EQep1Regs.QCAPCTL.bit.CEN  = 0x1;
         EQep1Regs.QCAPCTL.bit.UPPS = 0x0;
         EQep1Regs.QCAPCTL.bit.CCPS = 0x4;

         //Reviewed
         EQep1Regs.QEINT.bit.UTO = 1;
         IER |= M_INT5;
         PieCtrlRegs.PIEIER5.bit.INTx1 = 1;
         PieCtrlRegs.PIEACK.bit.ACK5   = 1;

    EDIS;
}

void Control_Action_Settings(void)
{
    #if defined(MV5_FCS)
        // First converter
        EPwm1Regs.CMPA.bit.CMPA = pwm_period_ctrl*CMPA_VV[x_opt].A1;
        EPwm2Regs.CMPA.bit.CMPA = pwm_period_ctrl*CMPA_VV[x_opt].B1;
        EPwm3Regs.CMPA.bit.CMPA = pwm_period_ctrl*CMPA_VV[x_opt].C1;

        EPwm1Regs.CMPB.bit.CMPB = pwm_period_ctrl*CMPB_VV[x_opt].A1;
        EPwm2Regs.CMPB.bit.CMPB = pwm_period_ctrl*CMPB_VV[x_opt].B1;
        EPwm3Regs.CMPB.bit.CMPB = pwm_period_ctrl*CMPB_VV[x_opt].C1;

        // Second converter
        EPwm4Regs.CMPA.bit.CMPA = pwm_period_ctrl*CMPA_VV[x_opt].A2;
        EPwm5Regs.CMPA.bit.CMPA = pwm_period_ctrl*CMPA_VV[x_opt].B2;
        EPwm6Regs.CMPA.bit.CMPA = pwm_period_ctrl*CMPA_VV[x_opt].C2;

        EPwm4Regs.CMPB.bit.CMPB = pwm_period_ctrl*CMPB_VV[x_opt].A2;
        EPwm5Regs.CMPB.bit.CMPB = pwm_period_ctrl*CMPB_VV[x_opt].B2;
        EPwm6Regs.CMPB.bit.CMPB = pwm_period_ctrl*CMPB_VV[x_opt].C2;

        // Third converter
        EPwm7Regs.CMPA.bit.CMPA = pwm_period_ctrl*CMPA_VV[x_opt].A3;
        EPwm8Regs.CMPA.bit.CMPA = pwm_period_ctrl*CMPA_VV[x_opt].B3;
        EPwm9Regs.CMPA.bit.CMPA = pwm_period_ctrl*CMPA_VV[x_opt].C3;

        EPwm7Regs.CMPB.bit.CMPB = pwm_period_ctrl*CMPB_VV[x_opt].A3;
        EPwm8Regs.CMPB.bit.CMPB = pwm_period_ctrl*CMPB_VV[x_opt].B3;
        EPwm9Regs.CMPB.bit.CMPB = pwm_period_ctrl*CMPB_VV[x_opt].C3;
    #elif defined(MODEL_FREE) || defined(MPC)
        #ifdef MOD_RAD
            /*EPwm1Regs.CMPA.bit.CMPA = pwm_period_ctrl*((1.0f - Sout[x_opt][0]) * ta + Sout[x_opt][0]);
            EPwm2Regs.CMPA.bit.CMPA = pwm_period_ctrl*((1.0f - Sout[x_opt][0]) * ta + Sout[x_opt][1]);
            EPwm3Regs.CMPA.bit.CMPA = pwm_period_ctrl*((1.0f - Sout[x_opt][0]) * ta + Sout[x_opt][2]);

            EPwm1Regs.CMPB.bit.CMPB = pwm_period_ctrl*(1.0f - Sout[x_opt][0]);
            EPwm2Regs.CMPB.bit.CMPB = pwm_period_ctrl*(1.0f - Sout[x_opt][1]);
            EPwm3Regs.CMPB.bit.CMPB = pwm_period_ctrl*(1.0f - Sout[x_opt][2]);

            // Second converter
            EPwm4Regs.CMPA.bit.CMPA = pwm_period_ctrl*((1.0f - Sout[x_opt][0]) * ta + Sout[x_opt][3]);
            EPwm5Regs.CMPA.bit.CMPA = pwm_period_ctrl*((1.0f - Sout[x_opt][0]) * ta + Sout[x_opt][4]);
            EPwm6Regs.CMPA.bit.CMPA = pwm_period_ctrl*((1.0f - Sout[x_opt][0]) * ta + Sout[x_opt][5]);

            EPwm4Regs.CMPB.bit.CMPB = pwm_period_ctrl*(1.0f - Sout[x_opt][3]);
            EPwm5Regs.CMPB.bit.CMPB = pwm_period_ctrl*(1.0f - Sout[x_opt][4]);
            EPwm6Regs.CMPB.bit.CMPB = pwm_period_ctrl*(1.0f - Sout[x_opt][5]);

            // Third converter
            EPwm7Regs.CMPA.bit.CMPA = pwm_period_ctrl*((1.0f - Sout[x_opt][0]) * ta + Sout[x_opt][6]);
            EPwm8Regs.CMPA.bit.CMPA = pwm_period_ctrl*((1.0f - Sout[x_opt][0]) * ta + Sout[x_opt][7]);
            EPwm9Regs.CMPA.bit.CMPA = pwm_period_ctrl*((1.0f - Sout[x_opt][0]) * ta + Sout[x_opt][8]);

            EPwm7Regs.CMPB.bit.CMPB = pwm_period_ctrl*(1.0f - Sout[x_opt][6]);
            EPwm8Regs.CMPB.bit.CMPB = pwm_period_ctrl*(1.0f - Sout[x_opt][7]);
            EPwm9Regs.CMPB.bit.CMPB = pwm_period_ctrl*(1.0f - Sout[x_opt][8]);*/

            EPwm1Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][0]* ta;
            EPwm2Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][1]* ta;
            EPwm3Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][2]* ta;

            // Second converter
            EPwm4Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][3]* ta;
            EPwm5Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][4]* ta;
            EPwm6Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][5]* ta;

            // Third converter
            EPwm7Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][6]* ta;
            EPwm8Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][7]* ta;
            EPwm9Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][8]* ta;

        #else
            EPwm1Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][0];
            EPwm2Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][1];
            EPwm3Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][2];

            /*EPwm1Regs.CMPB.bit.CMPB = pwm_period_ctrl*(1.0f - Sout[x_opt][0]);
            EPwm2Regs.CMPB.bit.CMPB = pwm_period_ctrl*(1.0f - Sout[x_opt][1]);
            EPwm3Regs.CMPB.bit.CMPB = pwm_period_ctrl*(1.0f - Sout[x_opt][2]);*/

            EPwm4Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][3];
            EPwm5Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][4];
            EPwm6Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][5];

            /*EPwm4Regs.CMPB.bit.CMPB = pwm_period_ctrl*(1.0f - Sout[x_opt][3]);
            EPwm5Regs.CMPB.bit.CMPB = pwm_period_ctrl*(1.0f - Sout[x_opt][4]);
            EPwm6Regs.CMPB.bit.CMPB = pwm_period_ctrl*(1.0f - Sout[x_opt][5]);*/

            EPwm7Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][6];
            EPwm8Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][7];
            EPwm9Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][8];

            /*EPwm7Regs.CMPB.bit.CMPB = pwm_period_ctrl*(1.0f - Sout[x_opt][6]);
            EPwm8Regs.CMPB.bit.CMPB = pwm_period_ctrl*(1.0f - Sout[x_opt][7]);
            EPwm9Regs.CMPB.bit.CMPB = pwm_period_ctrl*(1.0f - Sout[x_opt][8]);*/

            EPwm10Regs.CMPA.bit.CMPA = PWM_PERIOD_Q2A*0.5;
        #endif
    #elif defined(MPCta)
            EPwm1Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][0]*ta;
            EPwm2Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][1]*ta;
            EPwm3Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][2]*ta;

            EPwm4Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][3]*ta;
            EPwm5Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][4]*ta;
            EPwm6Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][5]*ta;

            EPwm7Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][6]*ta;
            EPwm8Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][7]*ta;
            EPwm9Regs.CMPA.bit.CMPA = pwm_period_ctrl*Sout[x_opt][8]*ta;

            EPwm10Regs.CMPA.bit.CMPA = PWM_PERIOD_Q2A*0.5;

    #elif defined(FOC)
        /*--------------------- ACTUACIÓN CARRIER BASED --------------------*/
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

    #else
    #endif
}

void ADC_init(void)
{
    EALLOW;

    // ConfiguraciÃ³n comÃºn de los mÃ³dulos ADCA y ADCB
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; // Divisor ADCCLK /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6;

    // Configurar resoluciÃ³n y modo de seÃ±al
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    // Configurar posiciÃ³n del pulso de interrupciÃ³n y habilitar ADC
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    // Retardo de inicializaciÃ³n
    DELAY_US(1000);

    AdcaRegs.ADCBURSTCTL.bit.BURSTEN=0;
    AdcaRegs.ADCSOCPRICTL.bit.SOCPRIORITY = 0;
    AdcbRegs.ADCBURSTCTL.bit.BURSTEN=0;
    AdcbRegs.ADCSOCPRICTL.bit.SOCPRIORITY = 0;

    // Configurar SOC para ADCA (Canales A0, A1, A2)
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;     // A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14;    // Tiempo de adquisiciÃ³n
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;   // Disparo por ePWM1 SOCA

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;     // A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 14;    // Tiempo de adquisiciÃ³n
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5;   // Disparo por ePWM1 SOCA

    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 2;     // A2
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 14;    // Tiempo de adquisiciÃ³n
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 5;   // Disparo por ePWM1 SOCA

    #ifdef MODEL_FREE
    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 0;     // A0
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = 14;
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 6;   // Disparo por ePWM1 SOCB

    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 1;     // A1
    AdcaRegs.ADCSOC4CTL.bit.ACQPS = 14;
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = 6;

    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 2;     // A2
    AdcaRegs.ADCSOC5CTL.bit.ACQPS = 14;
    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = 6;
    #endif

    // Configurar SOC para ADCB (Canales B0, B1, B2)
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0;     // B0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14;
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 1;     // B1
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 14;
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 5;

    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 2;     // B2
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = 14;
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 5;

    #ifdef MODEL_FREE
    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 0;     // B0
    AdcbRegs.ADCSOC3CTL.bit.ACQPS = 14;
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 6;

    AdcbRegs.ADCSOC4CTL.bit.CHSEL = 1;     // B1
    AdcbRegs.ADCSOC4CTL.bit.ACQPS = 14;
    AdcbRegs.ADCSOC4CTL.bit.TRIGSEL = 6;

    AdcbRegs.ADCSOC5CTL.bit.CHSEL = 2;     // B2
    AdcbRegs.ADCSOC5CTL.bit.ACQPS = 14;
    AdcbRegs.ADCSOC5CTL.bit.TRIGSEL = 6;
    #endif

    // Configurar interrupciones para ADCA y ADCB al completar el Ãºltimo SOC
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 2; // Disparo en SOC2
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 0;

    #ifdef MODEL_FREE
    AdcaRegs.ADCINTSEL1N2.bit.INT2SEL = 5; // Disparo en SOC5
    AdcaRegs.ADCINTSEL1N2.bit.INT2E = 1;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;
    AdcaRegs.ADCINTSEL1N2.bit.INT2CONT = 0;
    #endif


    // Desactivar disparo adicional de SOCs por interrupciones
    AdcaRegs.ADCINTSOCSEL1.all = 0x0000;
    AdcbRegs.ADCINTSOCSEL1.all = 0x0000;

    EPwm1Regs.ETSEL.bit.SOCAEN = 0;

    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO; // SOCA al inicio del periodo

    EPwm1Regs.ETSEL.bit.SOCAEN = 1;

    EPwm1Regs.ETPS.bit.SOCAPRD = 1;            // Disparo en cada evento

    #ifdef MODEL_FREE
        EPwm1Regs.ETSEL.bit.SOCBEN = 0;

        EPwm1Regs.ETSEL.bit.SOCBSEL    = ET_CTRU_CMPA;
        EPwm1Regs.ETSEL.bit.SOCBSELCMP = 1;

        EPwm1Regs.ETSEL.bit.SOCBEN = 1;

        EPwm1Regs.ETPS.bit.SOCBPRD = 1;

        EPwm1Regs.CMPC = 0.5f*PWM_PERIOD_CTRL*RATE;
    #endif
    EDIS;
}

// ----------------------------------------------------------------------
//  INTERRUPCIONES
// ----------------------------------------------------------------------

interrupt void adca1_isr(void)
{
    t2 = GET_LOC_TIME(CpuTimer0);

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

    PhaseCurrMeas.C1 = -(PhaseCurrMeas.A1 + PhaseCurrMeas.B1);
    PhaseCurrMeas.C2 = -(PhaseCurrMeas.A2 + PhaseCurrMeas.B2);
    PhaseCurrMeas.C3 = -(PhaseCurrMeas.A3 + PhaseCurrMeas.B3);


    #ifdef MODEL_FREE
        abxyCurrMeas_km1 = abxyCurrMeas_k;

        abxyCurrMeas_k.alpha = CLARKE_ALPHA(PhaseCurrMeas);
        abxyCurrMeas_k.beta  = CLARKE_BETA(PhaseCurrMeas) ;
        abxyCurrMeas_k.x1    = CLARKE_X1(PhaseCurrMeas);
        abxyCurrMeas_k.y1    = CLARKE_Y1(PhaseCurrMeas);
        abxyCurrMeas_k.x2    = CLARKE_X2(PhaseCurrMeas);
        abxyCurrMeas_k.y2    = CLARKE_Y2(PhaseCurrMeas);

        #ifdef NULL_COMP
            if (x_opt_km1 == 18)
            {
                deltaCurr0_k.alpha = abxyCurrMeas_k.alpha - abxyCurrMeas_km1.alpha;
                deltaCurr0_k.beta  = abxyCurrMeas_k.beta  - abxyCurrMeas_km1.beta ;
                deltaCurr0_k.x1    = abxyCurrMeas_k.x1    - abxyCurrMeas_km1.x1   ;
                deltaCurr0_k.y1    = abxyCurrMeas_k.y1    - abxyCurrMeas_km1.y1   ;
                deltaCurr0_k.x2    = abxyCurrMeas_k.x2    - abxyCurrMeas_km1.x2   ;
                deltaCurr0_k.y2    = abxyCurrMeas_k.y2    - abxyCurrMeas_km1.y2   ;

                new_data.d  = PARK_D(deltaCurr0_k,costheta,sintheta);
                new_data.q  = PARK_Q(deltaCurr0_k,costheta,sintheta);
                new_data.x1 = PARK_X1(deltaCurr0_k,costheta,sintheta);
                new_data.y1 = PARK_Y1(deltaCurr0_k,costheta,sintheta);
                new_data.x2 = PARK_X2(deltaCurr0_k,costheta,sintheta);
                new_data.y2 = PARK_Y2(deltaCurr0_k,costheta,sintheta);

                update_buffer();
            }
        #endif
    #endif

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;


    if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
     {
       AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;
       AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
     }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

#ifdef MODEL_FREE

    interrupt void adca1_mid_isr(void)
    {
        t1 = GET_LOC_TIME(CpuTimer0);elapsed = fabs(t2 - t1);
        adcResult.A0 =  AdcaResultRegs.ADCRESULT0;
        adcResult.B0 =  AdcbResultRegs.ADCRESULT0;
        adcResult.A1 =  AdcaResultRegs.ADCRESULT1;
        adcResult.B1 =  AdcbResultRegs.ADCRESULT1;
        adcResult.A2 =  AdcaResultRegs.ADCRESULT2;
        adcResult.B2 =  AdcbResultRegs.ADCRESULT2;

        PhaseMidCurrMeas.A1 = adcResult.A0*GAIN.A0 - OFFSET.A0;
        PhaseMidCurrMeas.A2 = adcResult.A1*GAIN.A1 - OFFSET.A1;
        PhaseMidCurrMeas.A3 = adcResult.A2*GAIN.A2 - OFFSET.A2;
        PhaseMidCurrMeas.B1 = adcResult.B0*GAIN.B0 - OFFSET.B0;
        PhaseMidCurrMeas.B2 = adcResult.B1*GAIN.B1 - OFFSET.B1;
        PhaseMidCurrMeas.B3 = adcResult.B2*GAIN.B2 - OFFSET.B2;

        PhaseMidCurrMeas.C1 = -(PhaseMidCurrMeas.A1 + PhaseMidCurrMeas.B1);
        PhaseMidCurrMeas.C2 = -(PhaseMidCurrMeas.A2 + PhaseMidCurrMeas.B2);
        PhaseMidCurrMeas.C3 = -(PhaseMidCurrMeas.A3 + PhaseMidCurrMeas.B3);

        abxyCurrMeas_kpm.alpha = CLARKE_ALPHA(PhaseMidCurrMeas);
        abxyCurrMeas_kpm.beta  = CLARKE_BETA(PhaseMidCurrMeas) ;
        abxyCurrMeas_kpm.x1    = CLARKE_X1(PhaseMidCurrMeas);
        abxyCurrMeas_kpm.y1    = CLARKE_Y1(PhaseMidCurrMeas);
        abxyCurrMeas_kpm.x2    = CLARKE_X2(PhaseMidCurrMeas);
        abxyCurrMeas_kpm.y2    = CLARKE_Y2(PhaseMidCurrMeas);

        AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;
        synchADC = 1;


        if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT2)
         {
           AdcaRegs.ADCINTOVFCLR.bit.ADCINT2 = 1;
           AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1;
         }
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;
    }

#endif

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
        if(EQep1Regs.QEPSTS.bit.QDF != 1){                     // Forward Direction QDF=1, but de position of the encoder is reverse
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

    LowPassFilter();

    rotor.we_k      = P*rotor.wm_k;

    rotor.thetae    = P*Pul2rad*(float)EQep1Regs.QPOSCNT;


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


interrupt void synchronous_isr(void)
{
    synch    = 1;

    EPwm1Regs.ETCLR.bit.INT = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}
