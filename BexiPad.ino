#include "Keyboard.h"



const int cDiv = 2;        //dividor of clock, higher the value better the ADC accuracy, may require recalibration
const int key0 = 32;             //code of key 1
const int key1 = 108;            //code of key 2
float act0 = 0.5f;         //activation fraction of key 1
float act1 = 0.5f;         //activation fraction of key 2
bool RapidTrigger = true;  //Rapid Trigger
const float RTrange = 0.1f;      //Rapid trigger fraction
const int res = 8;               //define resolution - 8/10/12 bits, will require recalibration
const int Aprox = 3;             //define by how much an ADC read value has to change for calculation code to get executed

// Calibration Values
const int min0 = 20;  //Minimum meassured value for key 1
const int min1 = 20;  //Minimum meassured value for key 2
const int max0 = 200; //Maximum meassured value for key 1
const int max1 = 200; //Minimum meassured value for key 2

// Dont change - variables that just get used during the code runtime

volatile bool key0on = false;
volatile bool key1on = false;
volatile int RTlimit = 0;
const int gClk = 3;  
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


  calc();                             //Calculates values based on calibration and Resolution
  genericClockSetup(gClk, cDiv);      //Sets up clock speeds
  ADCSetup();                         //Configures ADC
  Keyboard.begin();                   //Start of USBHID communication
  activateKey1();                     //First cycle for Key1
  readKey1();

}




void loop() {



  activateKey0();
  calcKey1();
  readKey0();
  activateKey1();
  calcKey0();
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



void activateKey0() {

  /* Setup from which pin to read from, with refference to what, and what GAIN to use */

  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_DIV2 | ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_PIN0;

  /* Start the ADC using a software trigger. */
  ADC->SWTRIG.bit.START = true;
}


void calcKey0() {

  /* Execute calculations if the values have changed, and based on if Rapid Trigger is enabled */

  if (abs(PA0-Analog0)>Aprox) {
    PA0 = Analog0;
    Analog0 = int(a0 * (PA0 - min0));
    if (RapidTrigger) {
      keyrt0();
    } else {
      keyb0();
    };
  };
}

void readKey0() {

  /* Wait for ADC values to be ready */

  while (ADC->INTFLAG.bit.RESRDY == 0)
    ;

  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  /* Write down ADC values */

  Analog0 = ADC->RESULT.reg;
}

void activateKey1() {

  /* Setup from which pin to read from, with refference to what, and what GAIN to use */

  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_GAIN_DIV2 | ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_PIN2;

  ADC->SWTRIG.bit.START = true;
}

void calcKey1() {

  /* Execute calculations if the values have changed, and based on if Rapid Trigger is enabled */

  if (abs(PA1-Analog1)>Aprox) {
    PA1 = Analog1;
    Analog1 = int(a1 * (PA1 - min1));
    if (RapidTrigger) {
      keyrt1();
    } else {
      keyb1();
    };
  };
}

void readKey1() {

  /* Wait for ADC values to be ready */

  while (ADC->INTFLAG.bit.RESRDY == 0)
    ;

  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  /* Write down ADC values */

  Analog1 = ADC->RESULT.reg;
}

void calc() {
  resolution = pow(2, res) - 1; 
  a0 = ((max0 - min0) / float(resolution));
  a1 = ((max1 - min1) / float(resolution));
  act0 = int(act0 * a0 * resolution);
  act1 = int(act1 * a1 * resolution);
  RTR0 = int(RTrange * a0 * resolution);
  RTR1 = int(RTrange * a1 * resolution);
  RTL0 = RTL0;
  RTL1 = RTL1;
}



void genericClockSetup(int clk, int dFactor) {
  // Enable the APBC clock for the ADC
  REG_PM_APBCMASK |= PM_APBCMASK_ADC;

  //This allows you to setup a div factor for the selected clock certain clocks allow certain division factors: Generic clock generators 3 - 8 8 division factor bits - DIV[7:0]
  GCLK->GENDIV.reg |= GCLK_GENDIV_ID(clk) | GCLK_GENDIV_DIV(dFactor);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;

  //configure the generator of the generic clock with 48MHz clock
  GCLK->GENCTRL.reg |= GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(clk);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
    ;

  //enable clock, set gen clock number, and ID to where the clock goes (30 is ADC)
  GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(clk) | GCLK_CLKCTRL_ID(30);
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;
}


void ADCSetup() {



  /* Calibrate values. */
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

  /* Number of ADC samples to capture */
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1;

  /* Sets resolution and uses smallest possible divider so cDIV has the most control */
  if (res == 8) {
    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_RESSEL_8BIT;
  } else if (res == 10) {
    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_RESSEL_10BIT;
  } else if (res == 12) {
    ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV4 | ADC_CTRLB_RESSEL_12BIT;
  } else {
    Serial.println("Unsupported resolution, change the value res to 8 10 or 12");
  };



  ADC->SAMPCTRL.reg = 0x00; //Ensures speed isnt limites

  while (ADC->STATUS.bit.SYNCBUSY) {};

  /* Enable the ADC. */
  ADC->CTRLA.bit.ENABLE = true;
}
