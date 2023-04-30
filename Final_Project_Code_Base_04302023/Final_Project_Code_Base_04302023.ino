// OPEN A NEW SKETCH WINDOW IN ARDUINO
// CLICK IN THIS BOX, CTL-A, CTL-C (Copy code from text box.)
// CLICK IN SKETCH, CTL-A, CTL-V (Paste code into sketch.)

// Breathing Rate Detection System -- Final Integration
//
// Pieced together from code created by: Clark Hochgraf and David Orlicki Oct 18, 2018
// Modified by: Mark Thompson April 2020 to integrate MATLAB read and Write 
//              and integrate the system

#include <MsTimer2.h>
#include <SPI.h>
#include <Tone2.h>


const int TSAMP_MSEC = 100;
const int NUM_SAMPLES = 2000;  //512 or 3600;
const int NUM_SUBSAMPLES = 160;
const int DAC0 = 3, DAC1 = 4, DAC2 = 5, LM61 = A0, VDITH = A1;
const int V_REF = 5.0;
const int SPKR = 12; // d12  PB4
unsigned long startMillis = 0;
const unsigned long interval = 1000; // 1 second
unsigned long currentMillis = 0;

volatile boolean sampleFlag = false;

const long DATA_FXPT = 1000; // Scale value to convert from float to fixed
const float INV_FXPT = 1.0 / DATA_FXPT; // division slow: precalculate


int nSmpl = 1, sample;

float xv, yv, yLF, yMF, yHF, stdLF, stdMF, stdHF, yLPF_FIR;
float printArray[9];
int numValues = 0;

Tone toneT2;
Tone toneT1;


int loopTick = 0;
bool statsReset;
bool isToneEn = false;

unsigned long startUsec, endUsec, execUsec;

int alarmCode = 0;
float threshold = 0.1;

//  Define a structure to hold statistics values for each filter band
struct stats_t
{
  int tick = 1;
  float mean, var, stdev;
} statsLF, statsMF, statsHF;

//**********************************************************************
void setup()
{

  configureArduino();
  Serial.begin(115200);delay(5);

  //Handshake with MATLAB 
  Serial.println(F("%Arduino Ready"));
  while (Serial.read() != 'g'); // spin

  toneT2.begin(13);
  toneT1.begin(SPKR);

  startMillis = millis();

  MsTimer2::set(TSAMP_MSEC, ISR_Sample); // Set sample msec, ISR name
  MsTimer2::start(); // start running the Timer  

}
////**********************************************************************
void loop()
{
  syncSample();  // Wait for the interupt when actually reading ADC data

  // Breathing Rate Detection

  // Declare variables

  float readValue, floatOutput;  //  Input data from ADC after dither averaging or from MATLAB
  long fxdInputValue, lpfInput, lpfOutput;  
  long eqOutput;  //  Equalizer output
  long WorkingSignal;  //  Equalizer output
  int alarmCode;  //  Alarm code
 

  // ******************************************************************
  //  When finding the impulse responses of the filters use this as an input
  //  Create a Delta function in time with the first sample a 1 and all others 0
  //  xv = (loopTick == 0) ? 1.0 : 0.0; // impulse test input

  // ******************************************************************
  //  Use this when the test vector generator is used as an input
  //xv = testVector();

  //long fxdInputVal = long(DATA_FXPT * xv + 0.5);
  // ******************************************************************
  //  Read input value in ADC counts  -- Get simulated data from MATLAB
  //readValue = ReadFromMATLAB();

  // ******************************************************************
  //  Read input value from ADC using Dithering, and averaging
  readValue = analogReadDitherAve();


  //  Convert the floating point number to a fixed point value.  First
  //  scale the floating point value by a number to increase its resolution
  //  (use DATA_FXPT).  Then round the value and truncate to a fixed point
  //  INT datatype

  fxdInputValue = long(DATA_FXPT * readValue + 0.5);

  //  Execute the equalizer
    eqOutput = Equalizer_FIR(fxdInputValue, loopTick);
  
  //  Execute the noise filter.  
    WorkingSignal = FIR_LPF_Noise(eqOutput, loopTick);

  //  Convert the output of the equalizer by scaling floating point
    yv = float(WorkingSignal) * INV_FXPT;

  //*******************************************************************
  // Uncomment this when measuring execution times
  //startUsec = micros();

  // ******************************************************************
  //  Compute the output of the filter using the cascaded SOS sections
  yLF = IIR_LOW(yv);  // second order systems cascade (LPF) 
  yMF = IIR_MID(yv);  // second order systems cascade (BPF)
  yHF = IIR_HIGH(yv); // second order systems cascade (HPF)


  /*Statistics*/
  
  statsReset = (statsLF.tick%100 == 0);

  getStats( yLF, statsLF, statsReset);
  stdLF = statsLF.stdev;

  getStats( yMF, statsMF, statsReset);
  stdMF = statsMF.stdev;

  getStats( yHF, statsHF, statsReset);
  stdHF = statsHF.stdev;
  
  //*******************************************************************
  // Uncomment this when measuring execution times
   //endUsec = micros();
   //execUsec = execUsec + (endUsec-startUsec);

  //  Call the alarm check function to determine what breathing range 
    alarmCode = AlarmCheck( stdLF, stdMF, stdHF );

  //  Call the alarm function to turn on or off the tone
  setAlarm(alarmCode, isToneEn );

  
 // To print data to the serial port, use the WriteToSerial function.  
 //
 //  This is a generic way to print out variable number of values
 //
 //  There are two input arguments to the function:
 //  printArray -- An array of values that are to be printed starting with the first column
 //  numValues -- An integer indicating the number of values in the array.  
 
   printArray[0] = loopTick;  //  The sample number -- always print this
   printArray[1] = readValue;        //  Column 2, INPUT SEQUENCE
   printArray[2] = yLF;       //  Column 3, LPF OUTPUT SEQUENCE
   printArray[3] = yMF;       //  Column 4, BPF OUTPUT SEQUENCE
   printArray[4] = yHF;       //  Column 5, HPF OUTPUT SEQUENCE
   printArray[5] = stdLF;
   printArray[6] = stdMF;
   printArray[7] = stdHF;
   printArray[8] = float(alarmCode);
 

   numValues = 9;  // The number of columns to be sent to the serial monitor (or MATLAB)

 WriteToSerial( numValues, printArray);  //  Write to the serial monitor (or MATLAB)

  if (++loopTick >= NUM_SAMPLES){
    Serial.print("Average execution time (uSec) = ");
    Serial.println( float(execUsec)/NUM_SAMPLES );
    while(true); // spin forever
  }

} // loop()

//******************************************************************
int AlarmCheck(float stdLF, float stdMF, float stdHF) {
  if (stdLF > threshold || stdMF > threshold || stdHF > threshold) 
  {
    if (stdLF > stdMF && stdLF > stdHF) 
    {
      alarmCode = 1;  // Low Breathing
    }

    else if (stdMF > stdLF && stdMF > stdHF) 
    {
      alarmCode = 0;  // Normal Breathing
    }

    else if (stdHF > stdLF && stdHF > stdMF) 
    {
      alarmCode = 2;  // High section
      currentMillis = millis();
      
    }

    else 
    {
      alarmCode = 3;  // Indeterminant state
    }
  }

  else 
  {
    alarmCode = 4;  // Non-operational
  }
  return alarmCode;
}  // end AlarmCheck

//*******************************************************************
int FIR_Generic(long inputX, int sampleNumber)
{   
  // Starting with a generic FIR filter impelementation customize only by
  // changing the length of the filter using MFILT and the values of the
  // impulse response in h

  // Filter type: FIR
  //  Set the constant HFXPT to the sum of the values of the impulse response
  //  This is to keep the gain of the impulse response at 1.
  //
  const int HFXPT = 1, MFILT = 4;
  
  int h[] = {};

  int i;
  const float INV_HFXPT = 1.0/HFXPT;
  static long xN[MFILT] = {inputX}; 
  long yOutput = 0;

  //
  // Right shift old xN values. Assign new inputX to xN[0];
  //
  for ( i = (MFILT-1); i > 0; i-- )
  {
    xN[i] = xN[i-1];
  }
   xN[0] = inputX;
  
  //
  // Convolve the input sequence with the impulse response
  //
  
  for ( i = 0; i < MFILT; i++)
  {
    
    // Explicitly cast the impulse value and the input value to LONGs then multiply
    // by the input value.  Sum up the output values
    
    yOutput = yOutput + long(h[i]) * long( xN[i] );
  }

  //  Return the output, but scale by 1/HFXPT to keep the gain to 1
  //  Then cast back to an integer
  //

  // Skip the first MFILT  samples to avoid the transient at the beginning due to end effects
  if (sampleNumber < MFILT ){
    return long(0);
  }else{
    return long(float(yOutput) * INV_HFXPT);
  }
}

int FIR_LPF_Noise(long inputX, int sampleNumber) {
  // Starting with a generic FIR filter impelementation customize only by
  // changing the length of the filter using MFILT and the values of the
  // impulse response in h

  // Filter type: FIR
  //  Set the constant HFXPT to the sum of the values of the impulse response
  //  This is to keep the gain of the impulse response at 1.

  // LPF FIR Filter Coefficients MFILT = 51, Fc = 75
  // LPF FIR Filter Coefficients MFILT = 71, Fc = 75
  const int HFXPT = 2048, MFILT = 71;
  int h[] = {1, 2, 1, 0, -2, -3, -2, 0, 3, 6, 5, 0, -6, -11,
  -9, 0, 12, 19, 15, 0, -20, -32, -26, 0, 33, 54, 44, 0,
  -60, -101, -88, 0, 151, 323, 459, 511, 459, 323, 151, 0, -88, -101,
  -60, 0, 44, 54, 33, 0, -26, -32, -20, 0, 15, 19, 12, 0,
  -9, -11, -6, 0, 5, 6, 3, 0, -2, -3, -2, 0, 1, 2,
  1};


  int i;
  const float INV_HFXPT = 1.0 / HFXPT;
  static long xN[MFILT] = { inputX };
  long yOutput = 0;

  //
  // Right shift old xN values. Assign new inputX to xN[0];
  //
  for (i = (MFILT - 1); i > 0; i--) {
    xN[i] = xN[i - 1];
  }
  xN[0] = inputX;

  //
  // Convolve the input sequence with the impulse response
  //

  for (i = 0; i < MFILT; i++) {

    // Explicitly cast the impulse value and the input value to LONGs then multiply
    // by the input value.  Sum up the output values

    yOutput = yOutput + long(h[i]) * long(xN[i]);
  }

  //  Return the output, but scale by 1/HFXPT to keep the gain to 1
  //  Then cast back to an integer
  //

  // Skip the first MFILT  samples to avoid the transient at the beginning due to end effects
  if (sampleNumber < MFILT) {
    return long(0);
  } else {
    return long(float(yOutput) * INV_HFXPT);
  }
}

int Equalizer_FIR(long inputX, int sampleNumber)
{   
  // Starting with a generic FIR filter impelementation customize only by
  // changing the length of the filter using MFILT and the values of the
  // impulse response in h

  // Filter type: FIR
  //  Set the constant HFXPT to the sum of the values of the impulse response
  //  This is to keep the gain of the impulse response at 1.
  //
  const int HFXPT = 1, MFILT = 4;
  
  int h[] = {1, 1, -1, -1};

  int i;
  const float INV_HFXPT = 1.0/HFXPT;
  static long xN[MFILT] = {inputX}; 
  long yOutput = 0;

  //
  // Right shift old xN values. Assign new inputX to xN[0];
  //
  for ( i = (MFILT-1); i > 0; i-- )
  {
    xN[i] = xN[i-1];
  }
   xN[0] = inputX;
  
  //
  // Convolve the input sequence with the impulse response
  //
  
  for ( i = 0; i < MFILT; i++)
  {
    
    // Explicitly cast the impulse value and the input value to LONGs then multiply
    // by the input value.  Sum up the output values
    
    yOutput = yOutput + long(h[i]) * long( xN[i] );
  }

  //  Return the output, but scale by 1/HFXPT to keep the gain to 1
  //  Then cast back to an integer
  //

  // Skip the first MFILT  samples to avoid the transient at the beginning due to end effects
  if (sampleNumber < MFILT ){
    return long(0);
  }else{
    return long(float(yOutput) * INV_HFXPT);
  }
}
//*******************************************************************************
float IIR_LOW(float xv)
{  
  //  ***  Copy variable declarations from MATLAB generator to here  ****

//Filter specific variable declarations
const int numStages = 3;
static float G[numStages];
static float b[numStages][3];
static float a[numStages][3];

//  *** Stop copying MATLAB variable declarations here
  
  int stage;
  int i;
  static float xM0[numStages] = {0.0}, xM1[numStages] = {0.0}, xM2[numStages] = {0.0};
  static float yM0[numStages] = {0.0}, yM1[numStages] = {0.0}, yM2[numStages] = {0.0};
  
  float yv = 0.0;
  unsigned long startTime;

//  ***  Copy variable initialization code from MATLAB generator to here  ****
//CHEBY low, order 5, R = 0.5, 12 BPM
G[0] = 0.0054630;
b[0][0] = 1.0000000; b[0][1] = 0.9990372; b[0][2]= 0.0000000;
a[0][0] = 1.0000000; a[0][1] =  -0.9554256; a[0][2] =  0.0000000;
G[1] = 0.0054630;
b[1][0] = 1.0000000; b[1][1] = 2.0015569; b[1][2]= 1.0015579;
a[1][0] = 1.0000000; a[1][1] =  -1.9217194; a[1][2] =  0.9289864;
G[2] = 0.0054630;
b[2][0] = 1.0000000; b[2][1] = 1.9994059; b[2][2]= 0.9994068;
a[2][0] = 1.0000000; a[2][1] =  -1.9562202; a[2][2] =  0.9723269;
//  **** Stop copying MATLAB code here  ****

  //  Iterate over each second order stage.  For each stage shift the input data
  //  buffer ( x[kk] ) by one and the output data buffer by one ( y[k] ).  Then bring in 
  //  a new sample xv into the buffer;
  //
  //  Then execute the recusive filter on the buffer
  //
  //  y[k] = -a[2]*y[k-2] + -a[1]*y[k-1] + g*b[0]*x[k] + b[1]*x[k-1] + b[2]*x[k-2] 
  //
  //  Pass the output from this stage to the next stage by setting the input
  //  variable to the next stage x to the output of the current stage y
  //  
  //  Repeat this for each second order stage of the filter

  
  for (i =0; i<numStages; i++)
    {
      yM2[i] = yM1[i]; yM1[i] = yM0[i];  xM2[i] = xM1[i]; xM1[i] = xM0[i], xM0[i] = G[i]*xv;
      yv = -a[i][2]*yM2[i] - a[i][1]*yM1[i] + b[i][2]*xM2[i] + b[i][1]*xM1[i] + b[i][0]*xM0[i];
      yM0[i] = yv;
      xv = yv;
    }
//
//  execUsec += micros()-startTime;
  
  return yv;
}

float IIR_MID(float xv)
{  
  //  ***  Copy variable declarations from MATLAB generator to here  ****

//Filter specific variable declarations
const int numStages = 6;
static float G[numStages];
static float b[numStages][3];
static float a[numStages][3];

//  *** Stop copying MATLAB variable declarations here
  
  int stage;
  int i;
  static float xM0[numStages] = {0.0}, xM1[numStages] = {0.0}, xM2[numStages] = {0.0};
  static float yM0[numStages] = {0.0}, yM1[numStages] = {0.0}, yM2[numStages] = {0.0};
  
  float yv = 0.0;
  unsigned long startTime;

//  ***  Copy variable initialization code from MATLAB generator to here  ****
//CHEBY bandpass, order 6, R= 1.0, [12 40] BPM

G[0] = 0.0901845;
b[0][0] = 1.0000000; b[0][1] = 2.0000078; b[0][2]= 1.0000019;
a[0][0] = 1.0000000; a[0][1] =  -1.8518182; a[0][2] =  0.9232096;
G[1] = 0.0901845;
b[1][0] = 1.0000000; b[1][1] = -2.0000071; b[1][2]= 1.0000013;
a[1][0] = 1.0000000; a[1][1] =  -1.8074575; a[1][2] =  0.9321473;
G[2] = 0.0901845;
b[2][0] = 1.0000000; b[2][1] = 2.0024227; b[2][2]= 1.0024286;
a[2][0] = 1.0000000; a[2][1] =  -1.9081449; a[2][2] =  0.9445785;
G[3] = 0.0901845;
b[3][0] = 1.0000000; b[3][1] = 1.9975695; b[3][2]= 0.9975754;
a[3][0] = 1.0000000; a[3][1] =  -1.9504289; a[3][2] =  0.9714626;
G[4] = 0.0901845;
b[4][0] = 1.0000000; b[4][1] = -2.0024054; b[4][2]= 1.0024112;
a[4][0] = 1.0000000; a[4][1] =  -1.8034239; a[4][2] =  0.9729274;
G[5] = 0.0901845;
b[5][0] = 1.0000000; b[5][1] = -1.9975875; b[5][2]= 0.9975933;
a[5][0] = 1.0000000; a[5][1] =  -1.9757181; a[5][2] =  0.9915253;
//  **** Stop copying MATLAB code here  ****

  //  Iterate over each second order stage.  For each stage shift the input data
  //  buffer ( x[kk] ) by one and the output data buffer by one ( y[k] ).  Then bring in 
  //  a new sample xv into the buffer;
  //
  //  Then execute the recusive filter on the buffer
  //
  //  y[k] = -a[2]*y[k-2] + -a[1]*y[k-1] + g*b[0]*x[k] + b[1]*x[k-1] + b[2]*x[k-2] 
  //
  //  Pass the output from this stage to the next stage by setting the input
  //  variable to the next stage x to the output of the current stage y
  //  
  //  Repeat this for each second order stage of the filter

  
  for (i =0; i<numStages; i++)
    {
      yM2[i] = yM1[i]; yM1[i] = yM0[i];  xM2[i] = xM1[i]; xM1[i] = xM0[i], xM0[i] = G[i]*xv;
      yv = -a[i][2]*yM2[i] - a[i][1]*yM1[i] + b[i][2]*xM2[i] + b[i][1]*xM1[i] + b[i][0]*xM0[i];
      yM0[i] = yv;
      xv = yv;
    }
//
//  execUsec += micros()-startTime;
  
  return yv;
}

float IIR_HIGH(float xv)
{  
  //  ***  Copy variable declarations from MATLAB generator to here  ****
//Filter specific variable declarations
const int numStages = 4;
static float G[numStages];
static float b[numStages][3];
static float a[numStages][3];

//  *** Stop copying MATLAB variable declarations here
  
  int stage;
  int i;
  static float xM0[numStages] = {0.0}, xM1[numStages] = {0.0}, xM2[numStages] = {0.0};
  static float yM0[numStages] = {0.0}, yM1[numStages] = {0.0}, yM2[numStages] = {0.0};
  
  float yv = 0.0;
  unsigned long startTime;

//  ***  Copy variable initialization code from MATLAB generator to here  ****
//CHEBY high, order 8, R = 0.5, 40 BPM
G[0] = 0.6899088;
b[0][0] = 1.0000000; b[0][1] = -2.0004653; b[0][2]= 1.0001304;
a[0][0] = 1.0000000; a[0][1] =  -0.3786331; a[0][2] =  0.1766708;
G[1] = 0.6899088;
b[1][0] = 1.0000000; b[1][1] = -2.0258801; b[1][2]= 1.0262211;
a[1][0] = 1.0000000; a[1][1] =  -1.2983826; a[1][2] =  0.6726531;
G[2] = 0.6899088;
b[2][0] = 1.0000000; b[2][1] = -1.9741196; b[2][2]= 0.9744486;
a[2][0] = 1.0000000; a[2][1] =  -1.6588484; a[2][2] =  0.8741666;
G[3] = 0.6899088;
b[3][0] = 1.0000000; b[3][1] = -1.9995350; b[3][2]= 0.9998699;
a[3][0] = 1.0000000; a[3][1] =  -1.7975113; a[3][2] =  0.9655216;

//  **** Stop copying MATLAB code here  ****

  //  Iterate over each second order stage.  For each stage shift the input data
  //  buffer ( x[kk] ) by one and the output data buffer by one ( y[k] ).  Then bring in 
  //  a new sample xv into the buffer;
  //
  //  Then execute the recusive filter on the buffer
  //
  //  y[k] = -a[2]*y[k-2] + -a[1]*y[k-1] + g*b[0]*x[k] + b[1]*x[k-1] + b[2]*x[k-2] 
  //
  //  Pass the output from this stage to the next stage by setting the input
  //  variable to the next stage x to the output of the current stage y
  //  
  //  Repeat this for each second order stage of the filter

  
  for (i =0; i<numStages; i++)
    {
      yM2[i] = yM1[i]; yM1[i] = yM0[i];  xM2[i] = xM1[i]; xM1[i] = xM0[i], xM0[i] = G[i]*xv;
      yv = -a[i][2]*yM2[i] - a[i][1]*yM1[i] + b[i][2]*xM2[i] + b[i][1]*xM1[i] + b[i][0]*xM0[i];
      yM0[i] = yv;
      xv = yv;
    }
//
//  execUsec += micros()-startTime;
  
  return yv;
}
//*******************************************************************
void getStats(float xv, stats_t &s, bool reset)
{
  float oldMean, oldVar;
  
  if (reset == true)
  {
    s.stdev = sqrt(s.var/s.tick);
    s.tick = 1;
    s.mean = xv;
    s.var = 0.0;  
  }
  else
  {
    oldMean = s.mean;
    s.mean = oldMean + (xv - oldMean)/(s.tick+1);
    oldVar = s.var; 
    s.var = oldVar + (xv - oldMean)*(xv - s.mean);      
  }
  s.tick++;  
}

//*******************************************************************
float analogReadDitherAve(void)
{ 
 
float sum = 0.0;
int index;
  for (int i = 0; i < NUM_SUBSAMPLES; i++)
  {
    index = i;
    digitalWrite(DAC0, (index & B00000001)); // LSB bit mask
    digitalWrite(DAC1, (index & B00000010));
    digitalWrite(DAC2, (index & B00000100)); // MSB bit mask
    sum += analogRead(LM61);
  }
  return sum/NUM_SUBSAMPLES; // averaged subsamples 

}
//*********************************************************************
void case2() {
  static unsigned long previousMillis = 0;
  const unsigned long interval = 1000; // 1 second
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    toneT1.play(1000);
    previousMillis = currentMillis;
  } else {
    toneT1.stop();
  }
}
//*********************************************************************
void setAlarm(int aCode, boolean isToneEn)
{  

  
  if (aCode == 0)
  {
      toneT1.stop();
  }
  else if (aCode == 1)
  {
    toneT1.play(400);
  }
  else if (aCode == 2)
  {
   if (currentMillis - startMillis >= interval) 
    {
      toneT1.play(1000);
      startMillis = currentMillis;
    } 
    else 
    {
      toneT1.stop();
    }
  }
  else if (aCode == 4)
  {
   toneT1.play(200);
  }
    else if (aCode == 3)
  {
      toneT1.stop();
  }
} // setBreathRateAlarm()

//*************************************************************
float testVector(void)
{
  // Variable rate sinusoidal input
  // Specify segment frequencies in bpm.
  // Test each frequency for nominally 60 seconds.
  // Adjust segment intervals for nearest integer cycle count.
    
  const int NUM_BAND = 9; // 6
  const float CAL_FBPM = 10.0, CAL_AMP = 2.0; 
  
  const float FBPM[NUM_BAND] = {12.0, 30.0, 40.0, 50.0, 60.0, 70.0, 75.0, 80.0, 100.0}; 
  // LPF test - 5.0, 10.0, 15.0, 20.0, 30.0, 70.0
  // For the general filter test use those frequencies: 5, 10, 20, 30, 50, 60
  // FIR LPF test - 12, 30, 40, 50.0, 60.0, 70.0, 75.0, 80.0, 100.0

  static float bandAmp[NUM_BAND] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  // 1.0, 1.0, 1.0, 1.0, 1.0, 1.0

  //  Determine the number of samples (around 600) that will give you an even number
  //  of full cycles of the sinewave.  This is done to avoid a large discontinuity 
  //  between bands.  This forces the sinewave in each band to end near a value of zer
  
  static int bandTick = int(int(FBPM[0]+0.5)*(600/FBPM[0]));
  static int simTick = 0, band = 0;
  static float Fc = FBPM[0]/600, cycleAmp = bandAmp[0];

  //for (int i = 0; i < NUM_BAND; i++) bandAmp[i] = CAL_AMP*(CAL_FBPM/FBPM[i]);  

  //  Check to see if the simulation tick has exceeded the number of tick in each band.
  //  If it has then switch to the next frequency (band) again computing how many
  //  ticks to go through to end up at the end of a cycle.
  
  if ((simTick >= bandTick) && (FBPM[band] > 0.0))
  {
    //  The simTick got to the end of the band cycle.  Go to the next frequency
    simTick = 0;
    band++;
    Fc = FBPM[band]/600.0;
    cycleAmp = bandAmp[band];
    bandTick = int(int(FBPM[band]+0.5)*(600/FBPM[band]));
  }
 
  float degC = 0.0; // DC offset
  degC += cycleAmp*sin(TWO_PI*Fc*simTick++);  
  //degC += 1.0*(tick/100.0); // drift: degC / 10sec
  //degC += 0.1*((random(0,101)-50.0)/29.0); // stdev scaled from 1.0
  return degC;
}

//*******************************************************************
void configureArduino(void)
{
  pinMode(DAC0,OUTPUT); digitalWrite(DAC0,LOW);
  pinMode(DAC1,OUTPUT); digitalWrite(DAC1,LOW);
  pinMode(DAC2,OUTPUT); digitalWrite(DAC2,LOW);

  pinMode(SPKR, OUTPUT); digitalWrite(SPKR,LOW);


  analogReference(DEFAULT); // DEFAULT, INTERNAL
  analogRead(LM61); // read and discard to prime ADC registers
  Serial.begin(115200); // 11 char/msec 
}


//**********************************************************************
void WriteToSerial( int numValues, float dataArray[] )
{

  int index=0; 
  for (index = 0; index < numValues; index++)
  {
    if (index >0)
    {
      Serial.print('\t');
    }
      Serial.print(dataArray[index], DEC);
  }

  Serial.print('\n');
  delay(20);

}  // end WriteToMATLAB

////**********************************************************************
float ReadFromMATLAB()
{
  int charCount;
  bool readComplete = false;
  char inputString[80], inChar;


  // Wait for the serial port

  readComplete = false;
  charCount = 0;
  while ( !readComplete )
  {
    while ( Serial.available() <= 0);
    inChar = Serial.read();

    if ( inChar == '\n' )
    {
      readComplete = true;
    }
    else
    {
      inputString[charCount++] = inChar;
    }
  }
  inputString[charCount] = 0;
  return atof(inputString);

} // end ReadFromMATLAB

//*******************************************************************
void syncSample(void)
{
  while (sampleFlag == false); // spin until ISR trigger
  sampleFlag = false;          // disarm flag: enforce dwell  
}

//**********************************************************************
void ISR_Sample()
{
  sampleFlag = true;
}
