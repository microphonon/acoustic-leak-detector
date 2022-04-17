/* Firmware for acoustic leak detector running on 48-pin MCU
   MSP430FR5994*. Developed with TI MSP-EXP430FR5994 Launchpad
   using Code Composer Studio v9.3 on Ubuntu-Linux.

   MCLK = SMCLK = 8 MHz sourced by DCO
   ACLK = 32768 Hz sourced by Timer_B
   SMCLK sources Timer A1 for ADC12

   Enable DSP by by placing the following line
   ${PROJECT_ROOT}/dsplib/include
   in CCS Build: MSP430 Compiler: Include Options
   Must have the TI dsplib folder is in project directory

     P1.0 NC
     P1.1 NC
     P1.2 Timer A1 output (TA1.1) TP2
     P1.3 Pulsed bias
     P1.4 A4 (ADC input) TP1
     P1.5 NC
     P1.6 UCB0 SDA
     P1.7 UCB0 SCL

     P2.0 Green LED
     P2.1 Red LED
     P2.2--2.7 NC

     P3.0 NC
     P3.1 NC
     P3.2 NC
     P3.3 Option to toggle this pin after each ADC12 conversion (TP3)
     P3.4 Leak alarm
     P3.5--3.7 NC

     P4.0--4.7 NC

     Firmware v 38 for Crowd Supply FFT PCB.
     MPH April 2022  */

#include <msp430.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include "DSPLib.h"

//Un-comment the following line if sensor should go into sleep mode when alarm detected
//#define SLEEP
# define FIRMWARE 38
# define SLAVE_ADDRESS 0x77
# define MICSAMPLES 5  //Number of times microphone is polled for impulse noise rejection
# define SBYTES 12 //Number of status bytes
# define COMMAND_BYTES 8 //Number control commands
# define DATASET 20 //Default value for arraysize
# define TRIGGER 18 //Default value for alarmsize
 //The following delays assume Timer B is 1.024 kHz (DCO/32)
# define IWAIT 5 // Time to wait before enabling ADC
# define NWAIT 45 //Time interval between multiple acquisitions
# define SAMPLING_PERIOD 239 //Counts for 8 MHz SMCLK; 240 gives 33.333 kHz
# define SAMPLE_TIME 200 //Set at 200 for ~5 us
# define ADC_SAMPLES 256
# define ADC_MAX 0x0FFF //Maximum of 12-bit ADC
# define SAT_POINTS 10 // Maximum allowable number of ADC_SAMPLES that reach saturation voltage
//Use frequency step 130.2 Hz for ADC_SAMPLES = 256 at 33.333 kHz
# define FFT_LOW 54 //130*54 = 7000 Hz
# define FFT_HIGH 88 //130*88 = 11.5 kHz
# define MEAN_MULT 2 //Multiplies the standard deviation to check for environmental noise
//Next 2 parameters help set background noise level
# define BACKGROUND_SAMPLES 30 //Default value 30 seconds; can be changed by user in range 10--255
# define NOISE_MULT 2 //Larger NOISE_MULT allows more fluctuation in ambient background
# define BLINK 100 //LED flash time
# define ALARM_ON P3OUT |= BIT4; //Alarm high on P3.4
# define ALARM_CLEAR P3OUT &= ~BIT4;
# define BIAS_ON P1OUT |= BIT3; //Pulsed bias on P1.3
# define BIAS_OFF P1OUT &= ~BIT3;

uint8_t i,sum_ok,sum_trigger,sum_noise,sat_flag;
volatile uint8_t mflag, nflag, eflag, aflag, count_flag;
uint8_t trigger_count[256], ok_count[256], noise_count[256]; //Counting arrays
uint32_t sum4, sum5, var5, sdev, avg_sum4, sig_array[MICSAMPLES], *PSigs, noise_array[256];
uint16_t sens_nv, bavg, bdev; //Global variables for ambient background
int32_t diff;
uint8_t Status_array[SBYTES], RxData[COMMAND_BYTES + COMMAND_BYTES], Size_check[COMMAND_BYTES], RxBuffer;
volatile uint8_t RxCount;
volatile uint8_t *PRxData; //Pointers to received data array
uint16_t ADC_array[ADC_SAMPLES],*ADC;

void SetTimer(void);
void SetClock(void);
void SetPins(void);
void SetI2C(void);
void SetADC(void);
void SetDMA(void);
void ReadCommands(void);
void Sleep(void);
int16_t CompSens(uint8_t ind);
void MeasureNoise(void);
uint32_t CheckMic(void);
uint16_t Rate(uint8_t rindx);
void SetParam(uint8_t p1, uint8_t p2);
void blink(uint8_t led_select);
/* Initial firmware settings. Any adjustment to these 5 settings by master
will persist on power off. These variables must be placed outside main program. */
#pragma PERSISTENT(arraysize)
uint8_t arraysize = DATASET;
#pragma PERSISTENT(alarmsize)
uint8_t alarmsize = TRIGGER;
#pragma PERSISTENT(poll_nv)
uint8_t poll_nv = 0x02; // Polling period: 1 sec (0x01) 30 sec (0x09)
#pragma PERSISTENT(bsamples)
uint8_t bsamples = BACKGROUND_SAMPLES;
#pragma PERSISTENT(led)
uint8_t led = 0x01; //On (0x01), off (0x00)

// Analog input signal and FFT result
DSPLIB_DATA(input,MSP_ALIGN_FFT_Q15(ADC_SAMPLES))
_q15 input[ADC_SAMPLES];

void main(void) {

    WDTCTL = WDTPW | WDTHOLD;   //Stop watchdog timer
    SetPins();
    SetI2C();
    SetClock();
    SetTimer();
    SetADC();
    SetDMA();
    PMMCTL0_H |= PMMPW_H; //Unlock Vref registers for ADC use
     __enable_interrupt(); //Enable global interrupts.
     /* Status_array elements 5--9 are different from these default values
     when read from persistent storage. */
      Status_array[0] = 0x00; //Alarm status; 0x00 is OK
      Status_array[1] = 0x00; //Noise warning
      Status_array[2] = 0x00; //Below threshold events
      Status_array[3] = 0x00; //Above threshold events
      Status_array[4] = 0x00; //Noise events
      Status_array[5] = bsamples;
      Status_array[6] = arraysize;
      Status_array[7] = alarmsize;
      Status_array[8] = poll_nv; //Loop period selection byte
      // Status_array[8] = 0x01; //Always set loop period to 1 sec on startup
      Status_array[9] = led;
      Status_array[10] = 0x01; //Sensor is active
      Status_array[11] = FIRMWARE; //Firmware version

    //Initialize the three event counter arrays; will be working with array subset defined by arraysize
    for (i=0; i < arraysize; i++)
    {
        trigger_count[i]=0;
        ok_count[i]=0;
        noise_count[i]=0;
    }
    for (i=0; i < MICSAMPLES; i++) //For averaging ADC acquisitions
    {
        sig_array[i]=0;
    }
    for (i=0; i < COMMAND_BYTES; i++)
    {
        Size_check[i] = i+2; //Use this array to check for presence of command byte pairs
    }
    RxCount = 0;
    aflag = 1; //aflag = 1 acquires background
    /* The following clear-reset sequences must be used to enable Channels 1 and 2
       of the DMA, which are used for I2C data transfer. */
    DMA1CTL &= ~DMAEN;
    DMA1CTL |= DMAEN;

    DMA2CTL &= ~DMAEN;
    DMA2CTL |= DMAEN;

    while(1)  //Top of main loop
    {
/*  Check if new data received from master. Receive data asynchronous with slave loop,
    but slave parameters only updated at start (here). Must be 2,4,6...or 2*COMMAND_BYTES or else ignored.
    Update Status_array with any new bytes. Slave will send Status_array any time requested. */
        while(1) //Probably don't need to use a while loop here
        {
            if(RxCount==0) break;
            count_flag=0;
            for(i=0; i < COMMAND_BYTES; i++) //Check if any byte pairs arrived on I2C bus from master
            {
                if (RxCount == Size_check[i]) //Master can send commands in any order
                {
                    count_flag=1;
                    break;
                }
            }
            if (count_flag==0) break; //Incorrect input
            ReadCommands(); //Correct byte count found; read master command bytes
        }

        if(aflag==1)
        {
        //The background count was changed or there was a system startup/reset. Get background level
            MeasureNoise();
            sens_nv = bavg + bdev;
            aflag = 0;
        }

    //Signal acquisition code follows
         for (i=0; i < MICSAMPLES; i++) sig_array[i]=0; //Clear just in case
         TB0CCR0 = Rate(poll_nv); //Polling period for main loop
         LPM0;      //Wait in low power mode for timeout or status query from master
         /* Timeout.  Check the microphone signal level and perform impulse noise rejection.
            mflag, nflag, and eflag are used to update the counting arrays */
         sig_array[0] = CheckMic();
         if (sig_array[0] == 0) //Check for saturation of ADC and record as noise
         {
             mflag = 0;
             nflag = 0;
             eflag = 1;
             if (led==1) //Blink red LED twice
             {
                 blink(2);
                 TB0CCR0 = BLINK + BLINK + BLINK;
                 LPM0;
                 blink(2);
             }
         }
         else if (sig_array[0] < sens_nv) //Nothing heard
             {
                 mflag=0;
                 nflag=1;
                 eflag=0;
                 if (led==1) blink(1); //Blink green LED
             }
         else
         /* Heard something exceeding sensitivity threshold but below ADC saturation.
            Do rapid succession of microphone measurements. */
             {
                 sat_flag=0;
                 for (i=1; i < MICSAMPLES; i++)
                 {
                     TB0CCR0=NWAIT; //Pause in LPM0 between acquisitions
                     LPM0;
                     sig_array[i] = CheckMic();
                     if (sig_array[i]==0) //Check for ADC saturation
                     {
                         sat_flag=1;
                         break;
                     }
                 }
                 if (sat_flag==1) //Record as environmental noise
                 {
                     mflag = 0;
                     nflag = 0;
                     eflag = 1;
                     if (led==1) //Blink red LED twice
                     {
                         blink(2);
                         TB0CCR0 = BLINK + BLINK + BLINK;
                         LPM0;
                         blink(2);
                     }
                 }
                 else //No ADC saturation occurred so analyze the microphone data set
                 {
                     mflag = 0; //Re-purpose mflag to count above threshold events
                     sum4 = 0;
                     PSigs = sig_array; //Point to first element in sig_array
                     for (i=0; i < MICSAMPLES; ++i)
                     {
                         if (*PSigs > sens_nv) ++mflag;
                         sum4 += *PSigs++;
                     }
                     if (mflag == MICSAMPLES) //We have sustained signal. Check if this is from environmental noise
                     {
                         avg_sum4 = sum4/MICSAMPLES; //Average the integrated FFT signals
                         //Calculate standard deviation
                         sum5 = 0;
                         PSigs = sig_array;
                         for (i=0; i < MICSAMPLES; ++i)
                         {
                             diff = (*PSigs++) - avg_sum4;
                             sum5 += diff*diff;
                         }
                         var5 = sum5/(MICSAMPLES-1); //variance
                         sdev = sqrt(var5); //standard deviation
                         /* Compare standard deviation to signal average. Large standard deviation is signature of
                        environmental noise. Increasing MEAN_MULT allows for more fluctuation in FFT data. */
                         if(MEAN_MULT*sdev > avg_sum4) //Environmental noise detected
                         {
                             mflag = 0;
                             nflag = 0;
                             eflag = 1;
                             if (led==1) //Blink red LED twice
                             {
                                blink(2);
                                TB0CCR0 = BLINK + BLINK + BLINK;
                                LPM0;
                                blink(2);
                             }
                         }
                         else
                         {
                             mflag=1; //Suspected leak signal detected
                             nflag=0;
                             eflag=0;
                             if (led==1) blink(2); //Blink red LED once
                         }
                     }
                     else //Impulse noise detected; Not a leak signal, but count event as quiet
                     {
                         mflag=0;
                         nflag=1;
                         eflag=0;
                         if (led==1) //Blink green LED twice
                         {
                             blink(1);
                             TB0CCR0 = BLINK + BLINK + BLINK;
                             LPM0;
                             blink(1);
                         }
                     }
                 }
             }
         /* Add new data to event count arrays. Arrays continuously updated; oldest data removed
            and newest data appended. Shift array indexes by 1. Remove oldest data first. */
              for (i=0; i < arraysize-1; i++)
                  {
                      trigger_count[i]=trigger_count[i+1];
                      ok_count[i]=ok_count[i+1];
                      noise_count[i]=noise_count[i+1];
                  }
              //New data appended
              trigger_count[arraysize-1]=mflag;
              ok_count[arraysize-1]=nflag;
              noise_count[arraysize-1]=eflag;
              //Sum the arrays with new data; the three counts will be placed in Status_array
              sum_ok = 0;
              sum_trigger = 0;
              sum_noise = 0;
              for (i=0; i < arraysize; i++)
              {
                  sum_trigger += trigger_count[i];
                  sum_ok += ok_count[i];
                  sum_noise += noise_count[i];
              }
              //Count data placed in status array
              Status_array[2] = sum_ok;
              Status_array[3] = sum_trigger;
              Status_array[4] = sum_noise;
              //Check for the alarm condition
              #ifndef SLEEP
              {
                  if(sum_ok + sum_trigger + sum_noise >= alarmsize) //Don't clear alarm until enough data accumulated
                  {
                          if (sum_trigger >= alarmsize)
                          {
                              Status_array[0]=0x01; //Alarm flag
                              //Alarm identified; clear arrays so can start counting fresh
                              for (i=0; i < arraysize; i++)
                              {
                                  trigger_count[i]=0;
                                  ok_count[i]=0;
                                  noise_count[i]=0;
                              }
                          }
                          else Status_array[0] = 0x00; //No alarm
                  }
              }
              #else
                  {
                      if (sum_trigger < alarmsize) Status_array[0] = 0x00; //No alarm
                      else
                      {
                          Status_array[0]=0x01; //Alarm flag
                          //Alarm identified; clear arrays so can start counting fresh
                          for (i=0; i < arraysize; i++)
                          {
                              trigger_count[i]=0;
                              ok_count[i]=0;
                              noise_count[i]=0;
                          }
                      }
                  }
            #endif
                  //No alarm but check for sustained noise; include suspected leak signals
                  if(Status_array[0] == 0)
                  {
                      if (sum_trigger + sum_noise >= alarmsize) Status_array[1]=0x01; //Set noise warning byte
                      else Status_array[1]=0x00;
                  }

             #ifndef SLEEP
             {
                 if (Status_array[0] == 0x01)  ALARM_ON //Set alarm pin
                 else ALARM_CLEAR //Clear alarm pin
             }
            #else
             {
                 if (Status_array[0] == 0x01) //Set output alert pin and go to sleep
                 {
                     Status_array[11]=0x00; //Sleep status byte set
                     ALARM_ON //Set alarm pin
                     Sleep();
                 }
             }
            #endif
    } //End of main polling loop
}

    #pragma vector=TIMER0_B0_VECTOR
     __interrupt void Timer_B (void)
     {
         LPM0_EXIT;
     }

    #pragma vector = EUSCI_B0_VECTOR
    __interrupt void USCI_B0_ISR(void)
     {
        switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
         {
         case USCI_NONE:         break;           // Vector 0: No interrupts
         case USCI_I2C_UCALIFG:  break;           // Vector 2: ALIFG
         case USCI_I2C_UCNACKIFG:break;         // Vector 4: NACKIFG
         case USCI_I2C_UCSTTIFG: break;         // Vector 6: STTIFG
         case USCI_I2C_UCSTPIFG:                // Vector 8: STPIFG; Also enable UCSTPIE
                             UCB0IFG &= ~UCSTPIFG;
                             DMA1CTL &= ~DMAEN;
                             DMA1CTL |= DMAEN;
                             break;
         case USCI_I2C_UCRXIFG3: break;          // Vector 10: RXIFG3
         case USCI_I2C_UCTXIFG3: break;          // Vector 14: TXIFG3
         case USCI_I2C_UCRXIFG2: break;          // Vector 16: RXIFG2
         case USCI_I2C_UCTXIFG2: break;          // Vector 18: TXIFG2
         case USCI_I2C_UCRXIFG1: break;          // Vector 20: RXIFG1
         case USCI_I2C_UCTXIFG1: break;          // Vector 22: TXIFG1
         case USCI_I2C_UCRXIFG0: break;          // Vector 24: RXIFG0
         case USCI_I2C_UCTXIFG0:  break;         // Vector 26: TXIFG0
         case USCI_I2C_UCBCNTIFG: break;         // Vector 28: BCNTIFG
         case USCI_I2C_UCCLTOIFG: break;         // Vector 30: clock low timeout. Try to reset I2C bus
         case USCI_I2C_UCBIT9IFG: break;         // Vector 32: 9th bit
         default: break;
   }
 }

#pragma vector=DMA_VECTOR
__interrupt void DMA_ISR(void)
{
  switch(__even_in_range(DMAIV,16))
  {
    case 0: break;
    case 2: // DMA0IFG = DMA Channel 0. Transfer ADC data
        __bic_SR_register_on_exit(LPM0_bits);
      break;
    case 4: break;  // DMA1IFG = DMA Channel 1. Transmit I2C data, no interrupt used
    case 6:  // DMA2IFG = DMA Channel 2. Receive single I2C command bytes
        if(RxCount == 0)
        {
            PRxData = RxData; //Place pointer at start of received data array
        }
        *PRxData++ = RxBuffer;
        RxCount++; //Number of (paired) command bytes
        DMA2CTL |= DMAEN;
        break;
    case 8: break;  // DMA3IFG = DMA Channel 3
    case 10: break; // DMA4IFG = DMA Channel 4
    case 12: break; // DMA5IFG = DMA Channel 5
    case 14: break; // DMA6IFG = DMA Channel 6
    case 16: break; // DMA7IFG = DMA Channel 7
    default: break;
  }
}

  void SetPins(void)
    {
          PM5CTL0 &= ~LOCKLPM5; //Unlock GPIO
/*          P1.0 NC
            P1.1 NC
            P1.2 Timer A1 output (TA1.1) TP2
            P1.3 Pulsed bias
            P1.4 A4 (ADC input) TP1
            P1.5 NC
            P1.6 UCB0 SDA
            P1.7 UCB0 SCL */
          P1DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT5;
          P1OUT |= BIT0; //If P1.0 is connected to Vcc (early design)
          //P1.2 Timer A1 output (TA1.1)
          P1SEL0 |= BIT2;
          P1SEL1 &= ~BIT2;
          //Pulsed bias off
          P1OUT &= ~BIT3;
          //P1.4 ADC input
          P1SEL1 |= BIT4;
          P1SEL0 |= BIT4;
          //Configure P1.6 and P1.7 for I2C
          P1SEL0 &= ~(BIT6 + BIT7);
          P1SEL1 |= BIT6 | BIT7;
/*          P2.0 Green LED
            P2.1 Red LED
            P2.2--2.7 NC */
          P2DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;
          P2OUT &= ~(BIT0 + BIT1); //LEDs off at start
/*          P3.0 NC
            P3.1 NC
            P3.2 NC
            P3.3 Option to toggle this pin after each ADC12 conversion (TP3)
            P3.4 Leak alarm
            P3.5--3.7 NC */
          P3DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;
          P3OUT &= ~BIT4;
          /* P4.0--4.7 NC */
          P4DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;
          //Set pins for 32.768 kHz crystal to source LFXT
          PJSEL0 = BIT4 | BIT5;
          PJSEL1 &= ~(BIT4 + BIT5);
    }

 void SetClock(void) //Use 32768 Hz external crystal
        {
            CSCTL0_H = CSKEY_H;                     // Unlock CS registers
            CSCTL1 = DCOFSEL_6;                     // Set DCO to 8 MHz
            // Set SMCLK = MCLK = DCO, ACLK = LFXTCLK
            CSCTL2 = SELA__LFXTCLK | SELS__DCOCLK | SELM__DCOCLK;
            CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1; //Set frequency dividers to 1 (default)
            CSCTL4 &= ~LFXTOFF;
            do
               {
                   CSCTL5 &= ~LFXTOFFG;                // Clear XT1 fault flag
                   SFRIFG1 &= ~OFIFG;
               }while (SFRIFG1 & OFIFG);               // Test oscillator fault flag
               CSCTL0_H = 0;                           // Lock CS registers
        }

 void SetTimer(void) //TB0 module at 1024 kHz
        {
            /* MC_1 to count up to TB0CCR0, link to ACLK at 32.768 kHz
               Divide it by 8 then 4 to give 1024 Hz. */
            TB0CTL |= MC_1 + TBSSEL__ACLK + ID_3 + TBCLR;
            TB0EX0 |= TBIDEX_3; //Divide by 4
            TB0CCTL0 = CCIE; //Enable the Timer B interrupt
        }

 void SetI2C(void) //Setup UCB0 as slave
        {
            UCB0CTLW0 = UCSWRST;                      // Software reset enabled
            UCB0CTLW0 |= UCMODE_3 + UCSYNC;           // I2C mode, sync mode (Do not set clock in slave mode)
            UCB0CTLW1 |= UCCLTO_0;                    //No timeout
            UCB0I2COA0 = SLAVE_ADDRESS | UCOAEN;      // Set slave addr; enable it
            UCB0CTLW0 &= ~UCSWRST;                    // Clear reset register
            UCB0IE = UCSTPIE;                       //Enable stop interrupt
        }

 void SetDMA(void) //Configure DMA Channels 0,1,2
 {
     // Set the DMA triggers. See Table 9-11 on 5994 data sheet
      DMACTL0 = DMA0TSEL_26 + DMA1TSEL_19; //Ch 0 is ADC12 end of conversion; Ch 1 is UCB0TXIFG0
      DMACTL1 = DMA2TSEL_18; //Ch 2 is UCB0RXIFG0
      //DMACTL4 = DMARMWDIS; // Read-modify-write disable. Probably not needed

      /* Ch 0 Single transfer, increment destination address only, enable DMA interrupt to
       signal that all conversions are complete and let DMA ISR escape from LPM0 */
      DMA0CTL = DMADT_0 + DMADSTINCR_3 + DMAIE;
      DMA0SZ = ADC_SAMPLES;
      DMA0SAL = &ADC12MEM0; //Ignore the 515-D warnings
      DMA0DAL = &ADC_array[0]; //Ignore the 515-D warnings

      // Ch 1 Repeated single byte transfers, increment source only
      DMA1CTL = DMADT_4 + DMASRCINCR_3 + DMADSTINCR_0 + DMADSTBYTE + DMASRCBYTE + DMAEN;
      DMA1SZ = SBYTES;
      DMA1SAL = &Status_array[0]; //Ignore the 515-D warnings
      DMA1DAL = &UCB0TXBUF; //Ignore the 515-D warnings

      // Ch 2 Single byte transfers, do not increment addresses, enable the interrupt
      DMA2CTL = DMADT_0 + DMASRCINCR_0 + DMADSTINCR_0 + DMADSTBYTE + DMASRCBYTE + DMAEN + DMAIE;
      DMA2SZ = 1;
      DMA2SAL = &UCB0RXBUF; //Ignore the 515-D warnings
      DMA2DAL = &RxBuffer; //Ignore the 515-D warnings
 }

void SetADC(void) //Configure timer and ADC12
        {   /* Sample audio input using Timer_A1 sourced to SMCLK at 8 MHz
               Timer output can be monitored on P1.3 (TA1.1)  */
            TA1CTL = TASSEL__SMCLK | MC__UP; //SMCLK, up mode
            TA1CCR0 = SAMPLING_PERIOD;  // PWM Period = 30 us or 33.333 kHz sampling frequency
            TA1CCTL1 = OUTMOD_3;  // Set/reset
            TA1CCR1 = SAMPLE_TIME; // PWM Duty Cycle

            ADC12CTL0 |= ADC12ON; //ADC12 on
            /* ADC triggered by Timer_A1 at 33.333 kHz; ADC clock is SMCLK at 8 MHz; see 5994 data sheet
               TA1.1 output selected with ADC12SHS_4; SMCLK; single-channel repeat */
            ADC12CTL1 = ADC12SHS_4 | ADC12SSEL_3 | ADC12CONSEQ_2;
            //ADC12CTL2 = ADC12RES_1;  //10-bit conversion
            ADC12CTL2 = ADC12RES_2;  //12-bit conversion
            ADC12MCTL0 |= ADC12INCH_4;   //Input on A4 (P1.4)
            ADC12MCTL0 |= ADC12VRSEL_1; //ADC range between Vref and ground. Vref is set in CheckMic()
        }

uint32_t CheckMic()
 {
        uint16_t k, sat_count;
        uint32_t fft_sum;
        const uint16_t fft_start = FFT_LOW + FFT_LOW; //2x needed to account for real & complex component at each frequency
        const uint16_t fft_count = fft_start + FFT_HIGH - FFT_LOW + 1; //Frequency elements in FFT sub-array
        int16_t fft_array[ADC_SAMPLES],a,b,*pfft;
        // Initialize the fft parameter structure.
        msp_status status;
        msp_fft_q15_params fftParams;
        fftParams.length = ADC_SAMPLES;
        fftParams.bitReverse = true;
        //fftParams.twiddleTable = MAP_msp_cmplx_twiddle_table_2048_q15;
        fftParams.twiddleTable = 0; //Don't need twiddle coefficients when LEA is used
        for(k=0; k<ADC_SAMPLES; k++) ADC_array[k] = 0; //Clear the ADC sampling array
        fft_sum = 0; //Clear integrated spectrum
        BIAS_ON
        REFCTL0 |= REFVSEL_1 + REFON; // Enable internal 2.0V reference; settling time 40 us
        TB0CCR0=IWAIT;  //Delay by IWAIT in LPM0 to miss 3V turn-on transient
        LPM0;
        // P3OUT &= ~BIT3; //Toggles after each conversion; Un-comment for testing
        //Delay timeout. Get analog signal for specified number of ADC_SAMPLES
        ADC12IFGR0 = 0; //Should be clear but clear anyway
        DMA0CTL |= DMAEN; //Enable DMA for repeated single transfers
        ADC12CTL0 |= ADC12ENC; //Turn on the ADC
        LPM0; //Wait in LPM0
        while (DMA0CTL & DMAEN); //Interrupt occurred. Check DMA transfer is complete before disabling ADC
        ADC12CTL0 &= ~ADC12ENC; //Disable ADC
        REFCTL0 &= ~REFON; // Disable internal 2.0V reference
        BIAS_OFF
        // Perform real FFT with fixed scaling on ADC data
        sat_count=0;
        ADC = ADC_array; //Point to first element of ADC_array
        for (k=0;k<ADC_SAMPLES;k++)
        {
            input[k] = *ADC++; //Copy time data to input array
            //Check for ADC saturation; ADC_MAX is 4095 for 12-bit
            if (input[k] == ADC_MAX)
            {
                sat_count++; //Keep track of how many sample points hit saturation
                if (sat_count == SAT_POINTS) return 0;
            }
        }
        status = MAP_msp_fft_fixed_q15(&fftParams, input);
        msp_checkStatus(status);
        for (k=0;k<ADC_SAMPLES;k++) fft_array[k] = input[k]; //Copy the complex FFT data
        pfft = fft_array + fft_start; //Point pfft to first element of FFT sub-array
        //Calculate complex FFT amplitude over specified frequency range
        for (k=fft_start;k<fft_count;k++)
        {
            a = *pfft++; //real component
            b = *pfft++; //imaginary component
            fft_sum += sqrt(a*a + b*b);
        }
        return fft_sum;
}

void SetParam(uint8_t p1, uint8_t p2)
{
    uint8_t ii;
    //Default status array used only for system reset
    uint8_t Default[] = {0,0,0,0,0,BACKGROUND_SAMPLES,DATASET,TRIGGER,2,1,1,FIRMWARE};
    switch(p1){
    case 0x72:  //Sensor reset
        if(p2 != 0x72) ; //Bad argument, ignore
        else
        {
            for(ii=0;ii<SBYTES;ii++) Status_array[ii] = Default[ii];
            aflag = 1; //Get new background
        }
        break;
        case 0x61:  //Alarm status
            if(p2 == 0x30) Status_array[0] = 0x00; //Clear alarm
            else if (p2 == 0x31) Status_array[0] = 0x01; //Set alarm
            else ; //Bad argument, ignore
            break;
        case 0x70:  //Low power mode
            if(p2 == 0x30) Status_array[10] = 0x00; //Sleep in LPM0
            else if(p2 == 0x31) Status_array[10] = 0x01; //Sensor active
            else break; //Bad argument, ignore
            break;
        case 0x62:   //Background samples (10--255)
            if(p2 < 0x0A) ; //Bad argument, ignore
            else
            {
                Status_array[5] = p2; //Set background acquire samples
                aflag = 1; //Acquire new background
            }
            break;
       case 0x65:   //Set size of counting arrays (10--255)
           if(p2 < 0x0A) ; //Bad argument, ignore
           else Status_array[6] = p2;
           break;
       case 0x74:   //Set trigger threshold count; check for logical error in main program
           Status_array[7] = p2;
           break;
       case 0x6F:   //Set polling period
           if((p2 < 0x01)||(p2 > 0x09)) ; //Bad argument, ignore
           else Status_array[8] = p2;
           break;
       case 0x6C:    //LED state
           if(p2 == 0x30) Status_array[9] = 0x00; //LEDs off
           else if (p2 == 0x31) Status_array[9] = 0x01; //LEDs on
           else break; //Bad argument, ignore
           break;
       default: break;
           }
        }

uint16_t Rate(uint8_t rindx) //Timer is 1.024 kHz
 {
      switch(rindx){
      case 0x01: return 1024; //1 sec
      case 0x02: return 2048; //2 sec
      case 0x03: return 3072; //3 sec
      case 0x04: return 5120; //5 sec
      case 0x05: return 10240; //10 sec
      case 0x06: return 15360; //15 sec
      case 0x07: return 20480; //20 sec
      case 0x08: return 25600; //25 sec
      case 0x09: return 30720; //30 sec
      default:   return 1024;
      }
 }

void MeasureNoise() //Measure the background acoustic level at startup
    {
        uint8_t m;
        const uint8_t bgpoll = 1; //1 second
        uint32_t bsum, dsum, *BSigs;
        uint16_t bvar;
        int32_t sdiff;
        for(m=0; m < bsamples; m++) noise_array[m]=0;
        while(1) //Loop until background is sufficiently quiet
        {
            for(m=0; m < bsamples; m++) noise_array[m]=0; //Clear noise array
            for(m=0; m < bsamples; m++) //Acquisition time = bsamples*Rate(bgpoll)
            {
                TB0CCR0 = Rate(bgpoll); //Polling period
                LPM0;      //Wait in low power mode for timeout or status query from master
                //Timeout interrupt exits from LPM0. Get the microphone signal.
                noise_array[m] = CheckMic();
                //Check for ADC saturation. Discard and repeat if needed
                if (noise_array[m] == 0) m--;
            }
            bsum = 0; //Clear the signal array sum
            BSigs = noise_array; //Point to first array element
            for (m=0; m < bsamples; ++m)
            {
                bsum += *BSigs++;
            }
            bavg = bsum/bsamples; //Average the integrated FFT signals
            //Calculate standard deviation
            dsum = 0;
            BSigs = noise_array; //Point to first element
            for (m=0; m < bsamples; ++m)
            {
                sdiff = (*BSigs++) - bavg;
                dsum += sdiff*sdiff;
            }
            bvar = dsum/(bsamples-1); //variance with Bessel correction
            bdev = sqrt(bvar); //standard deviation
            /* Compare standard deviation to average. Large standard deviation is signature of
               unstable background. Decreasing NOISE_DIVIDE allows for more fluctuation in FFT data. */
            if(NOISE_MULT*bdev > bavg)
            {
                if (led==1) //Background too noisy. Blink red LED twice and re-run while loop
                {
                    blink(2);
                    TB0CCR0 = BLINK + BLINK + BLINK;
                    LPM0;
                    blink(2);
                }
            }
            else break; //Quiet environment. Background baseline parameters found (bavg, bdev)
        }
    }

void ReadCommands() //Valid command string received from master
{
    for(i=0;i<RxCount;i=i+2)
    {
        // SetParam(RxData[i],RxData[i+1]); //Update the status array
        SetParam(*(RxData+i),*(RxData+i+1)); //Update the status array
    }
    RxCount=0; //All data from master read so clear buffer count
    //Clear both counting arrays; any command received from master clears the counts
    for (i=0; i < arraysize; i++)
    {
        trigger_count[i]=0;
        ok_count[i]=0;
        noise_count[i]=0;
    }
    /* Show the three sums clear in status, but do not need to explicitly clear them.
       The cleared event arrays will sum to zero. */
        Status_array[2] = 0; //Clear the below threshold count (sum_ok)
        Status_array[3] = 0; //Clear the above threshold count (sum_trigger)
        Status_array[4] = 0; //Clear the noise count (sum_noise)
        //Check alarm status byte
        if (Status_array[0] == 0x00) //No leak alarm
        {
            ALARM_CLEAR //Clear alarm pin
        }
        else if (Status_array[0] == 0x01) //Alarm condition detected by slave or has been set by master
        {
            ALARM_ON //Set alarm pin high
          #ifndef SLEEP
            Status_array[10]=0x01;
          #else
            Status_array[10]=0x00; //Put sensor into LPM0
         #endif
        }
        else ; //Garbage input, ignore
        //Put the following 5 parameters into persistent storage.
        bsamples = Status_array[5];
        arraysize = Status_array[6];
        alarmsize = Status_array[7];
        if(alarmsize > arraysize) //Coerce bad data from master. Can't have trigger count > event count
        {
            alarmsize = arraysize; //Make them the same
            Status_array[7] = arraysize;
        }
        poll_nv = Status_array[8];
        led = Status_array[9];
        if (Status_array[10] == 0x00) //ASCII command P0 puts slave into LPM0
        {
            Sleep();
        }
}

void blink(uint8_t led_select) //Green (P2.0): 1; Red (P2.1): 2
    {
        if (led_select==2) P2OUT |= BIT1;
        else if (led_select==1) P2OUT |= BIT0;
        else ;
        TB0CCR0=BLINK;
        LPM0;
        P2OUT &= ~BIT0;
        P2OUT &= ~BIT1;
    }

void Sleep()
{
    /* Idle here until the ASCII sequence P1 is received from master on I2C.
       This is a polled wakeup from LPM0 because CPU is needed to check for data transfer. */
    RxCount=0; //Should already be clear
    while(1) //This loop lets master poll status bytes in LPM0
    {
        TB0CCR0=1024; //Wakeup every 1 second to check for sleep escape command (P1) from master
        LPM0;
        if(RxCount==2)
        {
            if(RxData[0]==0x70 && RxData[1]==0x31) //This is the 2-byte wakeup sequence
            {
                Status_array[10]=0x01;
                RxCount=0;
                break;
            }
            else RxCount=0;
        }
        else RxCount=0;
    }
}

