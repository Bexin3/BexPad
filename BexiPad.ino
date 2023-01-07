#include "Keyboard.h"



const int cDiv = 2;        //dividor of clock, higher the value better the ADC accuracy, may require recalibration
int key0 = 32;             //code of key 1
int key1 = 108;            //code of key 2
float act0 = 0.5f;         //activation fraction of key 1
float act1 = 0.5f;         //activation fraction of key 2
bool RapidTrigger = false;  //Rapid Trigger
float RTrange = 0.1f;      //Rapid trigger fraction
int res = 8;               //define resolution - 8/10/12 bits, will require recalibration

// Calibration Values
const int min0 = 20;
const int min1 = 20;
const int max0 = 200;
const int max1 = 200;

// Dont change

volatile bool key0on = false;
volatile bool key1on = false;
volatile int RTlimit = 0;
const int gClk = 3;  //used to define which generic clock we will use for ADC
volatile int Analog0 = 0;
volatile int Analog1 = 0;
volatile float a0 = 0.0f;
volatile float a1 = 0.0f;
volatile int PA0 = 0;
volatile int PA1 = 0;
volatile int resolution;
volatile int RTR0;
volatile int RTR1;
volatile int RTL0 = 0;
volatile int RTL1 = 0;

void setup() {

  //genericClockSetup(gClk,cDiv); //setup generic clock and routed it to ADC
  resolution = pow(2, res) - 1;
  genericClockSetup(gClk, cDiv);
  ADCSetup();
  a0 = ((max0 - min0) / float(resolution));
  a1 = ((max1 - min1) / float(resolution));
  act0 = int(act0 * a0 * resolution);
  act1 = int(act1 * a1 * resolution);
  RTR0 = int(RTrange * a0 * resolution);
  RTR1 = int(RTrange * a1 * resolution);
  RTL0 = RTL0;
  RTL1 = RTL1;
  Keyboard.begin();
}




void loop() {

  readKey0();
  readKey1();

}

void keyb0() {
  if (key0on == false && Analog0 > act0) {
    Keyboard.press(key0);
    key0on = true;
  } else if (key0on == true && Analog0 < act0) {
    Keyboard.release(key0);
    key0on = false;
  }
}

void keyb1() {
  if (key1on == false && Analog1 > act1) {
    Keyboard.press(key1);
    key1on = true;
  } else if (key1on == true && Analog1 < act1) {
    Keyboard.release(key1);
    key1on = false;
  }
}



void keyrt0() {
  if (!key0on) {
    if (Analog0 > act0 && Analog0 - RTL0 > RTR0) {
      Keyboard.press(key0);
      key0on = true;
    };
    if (RTL0 > Analog0) {
      RTL0 = Analog0;
    };
  } else {
    if (RTL0 - Analog0 > RTR0) {
      Keyboard.release(key0);
      key0on = false;
    };                                                                                                                                                                               
    if (RTL0 < Analog0) {                                                                                                                                                               
      RTL0 = Analog0;
    };
  };
}

void keyrt1() {
  if (!key1on) {
    if (Analog1 > act1 && Analog1 - RTL1 > RTR1) {
      Keyboard.press(key1);
      key1on = true;
    };
    if (RTL1 > Analog1) {
      RTL1 = Analog1;
    };
  } else {
    if (RTL1 - Analog1 > RTR1) {
      Keyboard.release(key1);
      key1on = false;
    };
    if (RTL1 < Analog1) {
      RTL1 = Analog1;
    };
  };
}

//function for configuring ports or pins, note that this will not use the same pin numbering scheme as Arduino

void readKey0() {

  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_DIV2 | ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_PIN0;

  /* Start the ADC using a software trigger. */
  ADC->SWTRIG.bit.START = true;

  /* Wait for the result ready flag to be set. */
  while (ADC->INTFLAG.bit.RESRDY == 0)
    ;

  /* Clear the flag. */
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  /* Read the value. */
  if (PA0 != ADC->RESULT.reg) {
    Analog0 = int(a0 * (ADC->RESULT.reg - min0));
    if (RapidTrigger) {
      keyrt0();
    } else {
      keyb0();
    };
    ;
  };
  PA0 = ADC->RESULT.reg;
}



void readKey1() {

  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_DIV2 | ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_PIN2;

  /* Start the ADC using a software trigger. */
  ADC->SWTRIG.bit.START = true;

  /* Wait for the result ready flag to be set. */
  while (ADC->INTFLAG.bit.RESRDY == 0)
    ;

  /* Clear the flag. */
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  /* Read the value. */
  if (PA1 != ADC->RESULT.reg) {
    Analog1 = int(a1 * (ADC->RESULT.reg - min1));
    if (RapidTrigger) {
      keyrt1();
    } else {
      keyb1();
    };
    ;
  };
  PA1 = ADC->RESULT.reg;
}




void genericClockSetup(int clk, int dFactor) {
  // Enable the APBC clock for the ADC
  REG_PM_APBCMASK |= PM_APBCMASK_ADC;

  //This allows you to setup a div factor for the selected clock certain clocks allow certain division factors: Generic clock generators 3 - 8 8 division factor bits - DIV[7:0]
  GCLK->GENDIV.reg |= GCLK_GENDIV_ID(clk) | GCLK_GENDIV_DIV(dFactor);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;

  //configure the generator of the generic clock with 48MHz clock
  GCLK->GENCTRL.reg |= GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(clk);  // GCLK_GENCTRL_DIVSEL don't need this, it makes divide based on power of two
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;

  //enable clock, set gen clock number, and ID to where the clock goes (30 is ADC)
  GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(clk) | GCLK_CLKCTRL_ID(30);
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;
}


void PortSetup() {
  /* Set PB09 as an input pin. */
  PORT->Group[1].DIRCLR.reg = PORT_PA02;

  /* Enable the peripheral multiplexer for PB09. */
  PORT->Group[1].PINCFG[9].reg |= PORT_PINCFG_PMUXEN;

  /* Set PB09 to function B which is analog input. */
  PORT->Group[1].PMUX[4].reg = PORT_PMUX_PMUXO_B;


  /* Set PB09 as an input pin. */
  PORT->Group[1].DIRCLR.reg = PORT_PB08;

  /* Enable the peripheral multiplexer for PB09. */
  PORT->Group[1].PINCFG[9].reg |= PORT_PINCFG_PMUXEN;

  /* Set PB09 to function B which is analog input. */
  PORT->Group[1].PMUX[4].reg = PORT_PMUX_PMUXO_B;
}



void ADCSetup() {




  uint32_t bias = (*((uint32_t *)ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
  uint32_t linearity = (*((uint32_t *)ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
  linearity |= ((*((uint32_t *)ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;

  /* Wait for bus synchronization. */
  while (ADC->STATUS.bit.SYNCBUSY) {};

  /* Write the calibration data. */
  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);



  while (ADC->STATUS.bit.SYNCBUSY) {};

  /* Use the internal VCC reference. This is 1/2 of what's on VCCA.
   since VCCA is typically 3.3v, this is 1.65v.
*/
  ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;

  /* Only capture one sample. The ADC can actually capture and average multiple
   samples for better accuracy, but there's no need to do that for this
   example.
*/
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1;

  /* Set the clock prescaler to 512, which will run the ADC at
   8 Mhz / 512 = 31.25 kHz.
   Set the resolution to 12bit.
*/
  if (res == 8) {
    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_RESSEL_8BIT;
  } else if (res == 10) {
    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_RESSEL_10BIT;
  } else if (res == 12) {
    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_RESSEL_12BIT;
  } else {
    Serial.println("Unsupported resolution, change the value res to 8 10 or 12");
  };

  /* Configure the input parameters.

   - GAIN_DIV2 means that the input voltage is halved. This is important
     because the voltage reference is 1/2 of VCCA. So if you want to
     measure 0-3.3v, you need to halve the input as well.

   - MUXNEG_GND means that the ADC should compare the input value to GND.

   - MUXPOST_PIN3 means that the ADC should read from AIN3, or PB09.
     This is A2 on the Feather M0 board.
*/



  while (ADC->STATUS.bit.SYNCBUSY) {};

  /* Enable the ADC. */
  ADC->CTRLA.bit.ENABLE = true;
}


