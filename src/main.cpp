#include <Arduino.h>
#include <HX711.h>

#define CC1BOARD // CC1BOARD or devboard

#ifdef devboard
//#define HSE_VALUE 24000000 // set crystal oscilaltor to 24MHz
#define Clk3 PB7
#define Clk2 PB9
#define Clk1 PB13
#define Clk0 PB6
#define Dout3 PB5
#define Dout2 PB8
#define Dout1 PB14
#define Dout0 PB15
#define triggerOutputPin PA2
#define enableInputPin PA3
#endif

#ifdef CC1BOARD
#define HSE_VALUE 24000000 // set crystal oscilaltor to 24MHz for elegoo board
#define Clk0 PB13
#define Clk1 PC7
#define Clk2 PC6
#define Clk3 PC9
#define Dout0 PB14
#define Dout1 PC8
#define Dout2 PB15
#define Dout3 PA8
#endif

/*
______________
|      |      |01
| 4(3) | 3(2) |
|______|______|   Loadcell NR (index NR)
|      |      |
| 1(0) | 2(1) |23
|______|______|
     Front
*/

HX711 loadcell[4];
// HardwareSerial Serial2(PA3, PA2);    //RS232, used as IO
HardwareSerial Serial1(PA10, PA9); // debug uart
int32_t zeroValue[4];                 // value of unloaded platform (untared)
int32_t zeroOffset[4];                // max change observed while sitting idle
int32_t lastSample[4];                // previous ADC reading
int32_t currentSample[4];             // latest reading from ADC
int32_t ADCValueChange[4];            // change since last ADC sample
int32_t totalChange;                  // difference between current and last ADC rteading of ALL cells added together
int32_t totalSample;                  // ADC values of cell 0-3 added together
int32_t totalSampleZero;              // ADC values of untared cells 0-3 added
int32_t totalOffset;
int32_t TAPthreshold = 2500;          // minimum "slope" amount
int32_t maxTAPthreshold = 100000;     // maximum "slope" amount
int32_t TAPWeightTrigger = 40000;     // total weight on the plate (above tare) to trigger a TAP (to trigger if probing really slowly)
int32_t TAPReleaseThreshold = 20000;  // ADC reading must fall below this value to clear the TAP
int32_t TAPduration = 350;            // Duration of output when a TAP is detected (us)
uint32_t waitTime = 3125;             // time to wait before next ADC sample (us)
uint32_t TAPstart;                    // time at start of trigger output (ms)
uint32_t lastSampleTime;              // Time when last cell was sampled (us)
int cell = 0;
bool triggered = 0;
bool firstround = 1;
bool CAL = false;

void logo()
{
  // Serial2.println(" @@@@    @@@@@   @@@@@@  @@@  @@    @@@@     @@@@     @@@@      @@@      @@@  ");
  // Serial2.println("@@  @@@  @@  @@@ @@@     @@@  @@     @@@@     @@@@     @@@@      @@@      @@@ ");
  // Serial2.println("@@   @@  @@   @@ @@@     @@@@ @@       @@@      @@@      @@@@     @@@@     @@@");
  // Serial2.println("@@   @@  @@  @@@ @@@@@@  @@ @ @@         @@@      @@@      @@@      @@@      @");
  // Serial2.println("@@   @@  @@@@@   @@@@@@  @@ @@@@   @@     @@@@     @@@@      @@@      @@@     ");
  // Serial2.println("@@   @@  @@      @@@     @@  @@@   @@@      @@@      @@@@     @@@@     #@@@   ");
  // Serial2.println("@@@ @@@  @@      @@@     @@  @@@    @@@@      @@@      @@@      @@@@     @@@@ ");
  // Serial2.println(" @@@@@   @@      @@@@@@@ @@   @@      @@@@     @@@@      @@@      @@@      @@@");
  // Serial2.println("                                                                              ");
  // Serial2.println("                                                                              ");
  Serial2.println(" @@@@   @@@@@@  @@%  @@ @@@@@@@@  @@    @@   @@  @@@@@    @@  @@@      @@@    ");
  Serial2.println("@@  @@@ @@      @@@  @@    @@    @@@@   @@   @@  @@  @@@  @@   @@@@     @@@@  ");
  Serial2.println("@@      @@      @@@  @@    @@    @@@@   @@   @@  @@   @@  @@     @@@      @@@@");
  Serial2.println("@@      @@@@@@  @@@@ @@    @@   @@ @@   @@   @@  @@  @@@  @@       @@@      @@");
  Serial2.println("@@      @@@@@@  @@ @ @@    @@   @@  @@  @@   @@  @@@@@    @@  @     @@@@     @");
  Serial2.println("@@      @@      @@ @@@@    @@   @@@@@@  @@   @@  @@  @@   @@  @@      @@@     ");
  Serial2.println("@@  @@@ @@      @@  @@@    @@  @@   @@@ @@@ @@@  @@  @@@  @@   @@@      @@@   ");
  Serial2.println(" @@@@   @@@@@@  @@  @@@    @@  @@    @@  @@@@@   @@   @@  @@    @@@      @@@  ");
  Serial2.println("                                                                              ");
  Serial2.println("                                                                              ");
  Serial2.println("@@@@@@@@  @@    @@@@@     @@@@     @@@@     @@@@     @@@     @@@@      @@@    ");
  Serial2.println("   @@    @@@@   @@  @@@    @@@@     @@@@     @@@@     @@@      @@@@     @@@@  ");
  Serial2.println("   @@    @@@@   @@   @@      @@@      @@@      @@@@     @@@      @@@      @@@@");
  Serial2.println("   @@   @@ @@   @@  @@@  @     @@@      @@@      @@@      @@@       @@@     @@");
  Serial2.println("   @@   @@  @@  @@@@@    @@     @@@@     @@@@      @@@      @@@      @@@@    @");
  Serial2.println("   @@   @@@@@@  @@       @@@      @@@      @@@@     @@@@     @@@@      @@@    ");
  Serial2.println("   @@  @@   @@@ @@        @@@@      @@@      @@@      @@@@     @@@@      @@@  ");
  Serial2.println("   @@  @@    @@ @@          @@@@     @@@@      @@@      @@@      @@@      @@@ ");
}

void tapDetected(int32_t loadcellNR)
{
  TAPstart = millis();
  triggered = true;
  Serial2.print("TAP! ");   //output on RS232
  Serial2.print(loadcellNR);
  //  Serial2.print("CELL: ");
  //  Serial2.print(" change: ");
  //  Serial2.print(ADCValueChange[cell]);
  //  Serial2.print("  total sample: ");
  //  Serial2.print(totalSample);
  //  Serial2.print("  total change: ");
  //  Serial2.print(totalChange); // output to RS232
  //  Serial2.print(" detection time ");
  //  Serial2.println(millis());

  Serial1.print("TAP! ");
  Serial1.print("CELL: ");
  Serial1.print(loadcellNR);
  Serial1.print(" single change: ");
  Serial1.print(ADCValueChange[cell]);
  Serial1.print("  total sample: ");
  Serial1.print(totalSample);
  Serial1.print("  total change: ");
  Serial1.print(totalChange); // output to 2nd serial header
  Serial1.print(" detection time ");
  Serial1.println(millis());
}

void check(int32_t loadcellNR)
{
  // set ADC value change to zero if its more than X% of the unloaded raw value(attempt to reject bad reading)
  if ((abs(currentSample[loadcellNR]) > abs(zeroValue[loadcellNR] * 0.6))) 
  {
    currentSample[loadcellNR] = lastSample[loadcellNR];
    //return;     // reject reading by reusing the old reading
  }

  ADCValueChange[loadcellNR] = abs(currentSample[loadcellNR] - lastSample[loadcellNR]); // calculate absolute change since previous sample for current loadcell
  totalSample = 0;
  totalChange = 0;
  totalOffset = 0;

  if (triggered == 1) // skip all checks if centauriTAP is triggered
  {
    return;
  }

  for (size_t i = 0; i < 4; i++)
  {
    totalSample += abs(currentSample[i]);
    totalChange += abs(ADCValueChange[i]);
    totalOffset += abs(zeroOffset[i]);
  }

  if ((ADCValueChange[loadcellNR]) >= (TAPthreshold)) 
  {
    tapDetected(loadcellNR);    //cell 0-3 = single cell change trigger
    return;
  }

  if ((abs(totalSample) > TAPWeightTrigger) && (abs(totalChange) < 80000)) 
  {                                                                                                    
    tapDetected(6);   //cell 6 = weight trigger
    return;
  }

  if (((totalChange - totalOffset) >= TAPthreshold * 1) && (abs(totalChange) < abs(zeroValue[loadcellNR]) * 0.5)) 
  {
    tapDetected(5);   //cell 5 = multi cell change trigger
    return;
  }
}

void readCell(int32_t loadcellNR)
{
  lastSample[loadcellNR] = currentSample[loadcellNR]; 
  currentSample[loadcellNR] = (loadcell[loadcellNR].get_value());
}

void calibration()
{
  CAL = true;
  for (size_t i = 0; i < 4; i++)
  {
    zeroValue[i] = (loadcell[i].get_value(20));         // get zero value(to set false reading rejection)
    totalSampleZero += abs(loadcell[i].get_value(20));
    Serial1.println("beep");
    loadcell[i].tare(20); // Tare loadcell with the avg of 20 samples
    Serial1.println("boop");
    delay(100); // wait for settings to take

    for (size_t j = 0; j < 81; j++) // do 80 readings
    {
      readCell(i);
      // Serial1.println("skadoosh");
      if ((abs(currentSample[i]) > abs(zeroOffset[i])) && (abs(currentSample[i]) < 7500))
      // if the ADC value is bigger than the current zeroOffset
      {
        zeroOffset[i] = currentSample[i];
        
      }
    }
    Serial1.print("Cell NR ");
    Serial1.print(zeroValue[i]);
    Serial1.print(" zero offset:");
    Serial1.println(zeroOffset[i]);
  }
  // Serial1.println(totalSampleZero);
  CAL = false;
}

void setup()
{
  Serial2.begin(9600);  // RS232, T-PA2/R-PA3
  Serial1.begin(9600);  // extra UART
  Serial.begin(115200); // USB
  //delay(2500);
  Serial1.println("OpenCentauri TAP v0.1.0");
  Serial1.println("Setting up loadcells");
  loadcell[0].begin(Dout0, Clk0);
  loadcell[1].begin(Dout1, Clk1); // init 4 loadcells
  loadcell[2].begin(Dout2, Clk2);
  loadcell[3].begin(Dout3, Clk3);
  Serial1.println("Setting gain to 64");
  loadcell[0].set_gain(64);
  loadcell[1].set_gain(64); // setting gain 64 or 128 (channel A)
  loadcell[2].set_gain(64); // or 128 for higher sensitivity
  loadcell[3].set_gain(64);
  calibration();
}

void loop()
{
  while (1)
  {
    // updateIO();   //doing serial only
    if ((micros() > (lastSampleTime + waitTime)))
    {
      readCell(cell);
      lastSampleTime = micros();
      check(cell);
      cell++;
    }
    if (cell > 3)
    {
      cell = 0;
      firstround = 0; // reset when all cells are read for the first time
    }
    if (millis() >= (TAPstart + TAPduration))
    {
      triggered = false;    
    }

    if ((micros() < 1000000) && (lastSampleTime > 4000000000))
    {
      lastSampleTime = micros(); // attempt to catch the moment when micros rolls over to zero.
    }
  }
}
