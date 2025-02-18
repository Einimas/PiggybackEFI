// EFI controller based on MAP and Points ONLY, V2
// This version includes a complete AFR mapping table.
// Thresholded rate-of-change-of-MAP is also used to emulate delta-TPS.
// (C) 2021 John Patterson Consulting, LLC
// You are free to modify and redistribute this code without restriction.


// ----------------- Program constants -----------------:

// Pin numbers:
const int pointsPin = 12;
const int mapPin = 6;
//const int injectorIn = 7;
const int lambdaPin = 9;
const int iatPin = 5;
const int tpsPin = 4;
const int cltPin = 3;
const int injectorPin = 21;
const int injectorPin1 = 20;
const int injectorPin2 = 13;
const int injectorPin3 = 14;
const int ignitionPin = 19;
const int LEDPin = 8;



// Timing constants:
const unsigned long injectorUpdateInterval = 30000;
const unsigned long interruptDebounce = 3000;
const unsigned long primeTime = 300;
const unsigned long pumpTime = 0;
const int serialReportModulo = 10;
const int RPMrunningAverageDepth = 5;
const unsigned long blinkTime = 250;

// Engine-related constants:
const int pulsesPerRevolution = 2;
const double displacement_Liters = 1.6;
const double injector_MaxGramsPerMinute = 260.0;  // Will depend on your injector and fuel pressure
const double stallRPM = 300.0;                      /// maybe use with idle air valve????
//const double Pascals_Per_ADC_Unit = 122.1;
//const int vacuum_ADC_value = 102;
const double min_DC = 0.07;                      // Minimum duty cycle for idling
const double vacuum_ROC_Multiplier = 0.0005;     // Add or remove AFR proportional to the rate of change of MAP
const double vacuum_ROC_Cut_In = 51710;          // MAP pressure over which vacuum ROC will be applied

//Fuel AFR selector (gas or E51):
//const double fuel_comp = 1.0;    // Gas
const double fuel_comp = 0.70;     // E51

//// All 14.7:
//// AFR mapping:
// BAR: 0.2-1.2    
//double AFR_500RPM[10] =  {14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700};
//double AFR_1000RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700};
//double AFR_1500RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700};
//double AFR_2000RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700};
//double AFR_2500RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700};
//double AFR_3000RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700};
//double AFR_3500RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700};
//double AFR_4000RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700};
//double AFR_4500RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700};
//double AFR_5000RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 14.700};

//// Richer at high load only:
//// AFR mapping:
// BAR: 0.2-1.2    
//double AFR_500RPM[10] =  {14.500, 14.500, 14.000, 13.700, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_1000RPM[10] = {14.500, 14.500, 14.000, 13.700, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_1500RPM[10] = {14.500, 14.500, 14.000, 13.700, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_2000RPM[10] = {14.500, 14.500, 14.000, 13.700, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_2500RPM[10] = {14.500, 14.500, 14.000, 13.700, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_3000RPM[10] = {14.500, 14.500, 14.000, 13.700, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_3500RPM[10] = {14.500, 14.500, 14.000, 13.700, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_4000RPM[10] = {14.500, 14.500, 14.000, 13.700, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_4500RPM[10] = {14.500, 14.500, 14.000, 13.700, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_5000RPM[10] = {14.500, 14.500, 14.000, 13.700, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};

//// Richer at high load only with rich idle:
//// AFR mapping:
// BAR: 0.2-1.2    
//double AFR_500RPM[10] =  {13.500, 13.500, 13.000, 13.500, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_1000RPM[10] = {13.500, 13.500, 14.000, 13.700, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_1500RPM[10] = {14.500, 14.500, 14.000, 13.700, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_2000RPM[10] = {14.500, 14.500, 14.000, 13.700, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_2500RPM[10] = {14.500, 14.500, 14.000, 13.700, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_3000RPM[10] = {14.500, 14.500, 14.000, 13.700, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_3500RPM[10] = {14.500, 14.500, 14.000, 13.700, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_4000RPM[10] = {14.500, 14.500, 14.000, 13.700, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_4500RPM[10] = {14.500, 14.500, 14.000, 13.700, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_5000RPM[10] = {14.500, 14.500, 14.000, 13.700, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};

// Richer at idle only:
// AFR mapping:
// BAR: 0.2-1.2             
double AFR_500RPM[10] =  {12.000, 12.000, 12.000, 12.000, 12.000, 12.000, 12.000, 12.000, 12.000, 12.000};
double AFR_1000RPM[10] = {13.500, 13.500, 13.500, 13.500, 13.500, 13.200, 13.000, 12.500, 12.500, 12.000};
double AFR_1500RPM[10] = {14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 13.500, 13.500, 13.000};
double AFR_2000RPM[10] = {14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 13.500, 13.500};
double AFR_2500RPM[10] = {14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000};
double AFR_3000RPM[10] = {14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000};
double AFR_3500RPM[10] = {14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000};
double AFR_4000RPM[10] = {14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000};
double AFR_4500RPM[10] = {14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000};
double AFR_5000RPM[10] = {14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000, 14.000};

// Lean coasting:
//// AFR mapping:
// BAR: 0.2-1.2   
//double AFR_500RPM[10] =  {13.500, 13.500, 13.500, 13.500, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_1000RPM[10] = {13.500, 13.500, 13.500, 13.500, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_1500RPM[10] = {14.000, 14.000, 14.000, 14.000, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_2000RPM[10] = {14.000, 14.000, 14.000, 14.000, 13.500, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_2500RPM[10] = {15.000, 14.000, 14.000, 14.000, 13.700, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_3000RPM[10] = {16.500, 15.500, 14.000, 14.000, 14.000, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_3500RPM[10] = {17.000, 16.500, 15.000, 14.000, 14.000, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_4000RPM[10] = {18.000, 17.500, 16.000, 15.000, 14.000, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_4500RPM[10] = {19.000, 18.500, 17.000, 16.000, 15.000, 13.200, 13.000, 12.700, 12.500, 12.000};
//double AFR_5000RPM[10] = {19.500, 18.500, 17.000, 16.000, 15.000, 13.200, 13.000, 12.700, 12.500, 12.000};


// Physical parameters:
//const double airTempKelvin = 298;   // -----------------------------------------------------------------------------------------------------//
const double airMolarMass = 28.9;
const double R_Liter_Pascal_Kelvin = 8314.0;


// ----------------- Global Variables -----------------:
unsigned long updateLastMicros = 0;
unsigned long pointsLastMicros = 0;
unsigned long pointsDiff = 100000000;
bool stalled = 0;
int serialReportCount = 0;
double RPMrunningAverage[RPMrunningAverageDepth];
int RPMrunningAverageIndex = 0;
unsigned long blinkLastMillis = 0;
bool blinkState = 0;
unsigned long lastMAP = 0;

                                                                                         ////////////////////////////
unsigned long lastIAT = 0;
unsigned long lastLAMBDA = 0;
unsigned long airTempKelvin = 0;
unsigned long airTempmargin = 0;
// ----------------- Functions -----------------:

// ISR triggered when the points open:
void points()
{
  //Determine if interrupt is valid based on time since last interrupt:
  if((micros() - pointsLastMicros) > interruptDebounce)
  {
    // Compute difference in time from last points opening:
    pointsDiff = micros() - pointsLastMicros;
    
    // Reset points timer:
    pointsLastMicros = micros();
  }
  
}

// Function to obtain AFR target:
double AFR(double speedRPM, double pressurePa)
{
  // Determine index of MAP pressure in AFR table to use:
  int MAP_index = 0;
  if(pressurePa < 1)
  {
    MAP_index = 0;
  }
  else if(pressurePa < 2)
  {
    MAP_index = 1;
  }
  else if(pressurePa < 3)
  {
    MAP_index = 2;
  }
  else if(pressurePa < 4)
  {
    MAP_index = 3;
  }
  else if(pressurePa < 5)
  {
    MAP_index = 4;
  }
  else if(pressurePa < 6)
  {
    MAP_index = 5;
  }
  else if(pressurePa < 7)
  {
    MAP_index = 6;
  }
  else if(pressurePa < 8)
  {
    MAP_index = 7;
  }
  else if(pressurePa < 9)
  {
    MAP_index = 8;
  }
  else
  {
    MAP_index = 9;
  }

  // Determine RPM range to use and return relevant AFR:
  if(speedRPM < 500)
  {
    return AFR_500RPM[MAP_index];
  }
  else if(speedRPM < 1000)
  {
    return AFR_1000RPM[MAP_index];
  }
  else if(speedRPM < 1500)
  {
    return AFR_1500RPM[MAP_index];
  }
  else if(speedRPM < 2000)
  {
    return AFR_2000RPM[MAP_index];
  }
  else if(speedRPM < 2500)
  {
    return AFR_2500RPM[MAP_index];
  }
  else if(speedRPM < 3000)
  {
    return AFR_3000RPM[MAP_index];
  }
  else if(speedRPM < 3500)
  {
    return AFR_3500RPM[MAP_index];
  }
  else if(speedRPM < 4000)
  {
    return AFR_4000RPM[MAP_index];
  }
  else if(speedRPM < 4500)
  {
    return AFR_4500RPM[MAP_index];
  }
  else
  {
    return AFR_5000RPM[MAP_index];
  }
}


// Function to compute RPM based on time since last points opening:
double RPM()
{
  // If pointsDiff is longer than the time since points() was last called,
  // just use pointsDiff to compute the RPM:
  if(pointsDiff > (micros() - pointsLastMicros))
  {
    // RPM = (Pulses/sec)/(Pulses/Revolution)*(60 sec/min)
    RPMrunningAverage[RPMrunningAverageIndex] = (60000000.0/pointsDiff)/pulsesPerRevolution;
  }
  // If it has been longer than pointsDiff since points() was last called,
  // use the elapsed time to compute the RPM:
  else
  {
    // RPM = (Pulses/sec)/(Pulses/Revolution)*(60 sec/min)
    RPMrunningAverage[RPMrunningAverageIndex] = (60000000.0/(micros() - pointsLastMicros))/pulsesPerRevolution;
  }

  //Increment running average index, reset if needed:
  RPMrunningAverageIndex++;
  if(RPMrunningAverageIndex >= RPMrunningAverageDepth)
  {
    RPMrunningAverageIndex = 0;
  }

  // Compute running average sum:
  double sum = 0;
  for(int i = 0; i < RPMrunningAverageDepth; i++)
  {
    sum = sum + RPMrunningAverage[i];
  }

  // Return running average of RPM:
  return sum/RPMrunningAverageDepth;
}

// Function to measure the Manifold Absolute Pressure (MAP):           /////////////// Adapted to 0281006059 bosch 4bar map 0.2bar-0,215v(44duty); 4bar-4,461v(914duty) (1bar vac, 3bar boost) /////////////////////
double MAP()
{
  double MAPsensor = analogRead(mapPin);
  
  float estimatedMAP = map(MAPsensor,43,913,0,37);  //////divided into 38 cells in 0.2bar or 2.9 psi pressure increments (made sense for the sensor i was using. Might increase resolution to 76 if theres enough space in rom

  ///might use mathematical alternative to finding mapcell to map using corrected map function y = (x - in_min) * (out_max - out_min +1) / (in_max - in_min+1) + out_min
  
  
//  float estimatedMAP = (MAPsensor-43)*38/871; wtf with the math???? "hope" the real map equasion doesnt do this shit couse if MAPsensor is 44 acording to formulla outcome value would be 38/871 whitch is nowhere near the needed output,. Bentch test, output should be 0.8 to 1 !!!!
  

  // Ensure only positive MAP values are returned:
  if(estimatedMAP < 0)
  {
    estimatedMAP = 0;
  }
  
  return estimatedMAP;
}

// Function to measure the Intake Air Temperature (IAT):
double IAT()                                                                             /////////////////////////////////////////////////////////////////////////////
{
  double airTempduty = (analogRead(iatPin));        /////////////// 0281006059 bosch -40 to 150 celsius needs lots of "curve magic formullas" figuring out
  
 // airTempCelsius = airTempread value to Celsius conversion magic;    ////
 // airTempKelvin = (airTempCelsius +  273.15 );            

////// fragmented map of the sensor , must be a better way using a formulla.

  /// all temp is offset + 40
if (130<airTempduty<306)
{
  float airTempmargin = map(airTempduty,130,306,0,20);
}
else if (306<airTempduty<541)
{
  float airTempmargin = map(airTempduty,306,541,20,40);
}
else if (541<airTempduty<743)
{
  float airTempmargin = map(airTempduty,541,743,40,60);
}
else if (743<airTempduty<869)
{
  float airTempmargin = map(airTempduty,743,869,60,80);
}
else if (869<airTempduty<940)
{
  float airTempmargin = map(airTempduty,869,940,80,100);
}
else if (940<airTempduty<976)
{
  float airTempmargin = map(airTempduty,940,976,100,120);
}
else if (976<airTempduty<996)
{
  float airTempmargin = map(airTempduty,976,996,120,140);
}
  // eliminate offset to Celsius and then convert to Kelvin
float airTempKelvin = airTempmargin+40+273.15;
  // Ensure only positive MAP values are returned:



  
  if(airTempKelvin < 0)
  {
    airTempKelvin = 0;
  }
  
  return airTempKelvin;
}

// Function to report human-readable stats to the serial port:
void reportStats(double RPM_report, double MAP_report, double DC_report)
{
  // Report engine RPM:
  Serial.print("Engine RPM: ");
  Serial.println(RPM_report);
  Serial.print("Manifold Absolute Pressure: ");
  Serial.print("kPa: ");
  Serial.print(MAP_report/1000.0);
  Serial.print(" PSI: ");
  Serial.println(MAP_report/6894.76);
  Serial.print("Injector Duty Cycle: ");
  Serial.print(DC_report*100.0);
  Serial.println("%");
  Serial.println(" ");
}

// Function to compute the required fuel injector duty cycle:
double getInjectorDC()
{
  //Get measurements:
  double RPM_value = RPM();
  double MAP_value = MAP();
  double airTempKelvin = IAT();
//  double LAMBDA_value = LAMBDA();
  
  // Compute the volume flow rate of air (L/min) into the engine
  // (Otto cycle with 2 revolutions per intake stroke assumed):
  double volFlowRate = displacement_Liters*RPM_value/2.0;

  // Compute the density of air (g/L) using the Ideal Gas Law (PV=nRT):////////////////////////////////////////////////////////////////////////////////
  // Density = mass/volume = n(airMolarMass)/V = P(airMolarMass)/(RT)
  double airDensity = MAP_value*airMolarMass/(R_Liter_Pascal_Kelvin*airTempKelvin);

  // Compute the mass air flow rate into the engine:
  double massAirFlow = volFlowRate*airDensity;

  // Get AFR:
  
  // Adjust the AFR based on the rate of change of the MAP level:
  double vacROC = (double)MAP_value - (double)lastMAP;
  lastMAP = MAP_value;

  // Look up AFR in table:
  double AFR_target = AFR(RPM_value,MAP_value);
  
  // If MAP is above threshold, apply rate-of-change factor:
  if(MAP_value > vacuum_ROC_Cut_In)
  {
    AFR_target = AFR_target - vacuum_ROC_Multiplier*vacROC;
  }
  // Otherwise, just use AFR from the table directly.

  // Apply fuel compensator (for E51, E85, etc.)
  AFR_target = fuel_comp*AFR_target;

  // Compute the required mass fuel flow rate into the engine:
  double massFuelFlow = massAirFlow/AFR_target;

  // Compute the duty cycle for the fuel injector 
  // (based on the maximum fuel delivery rate):
  double estimated_DC = massFuelFlow/injector_MaxGramsPerMinute;

  // Ensure that the duty cycle does not exceed 1 or fall below the minimum duty cycle value:
  if(estimated_DC > 1.0)
  {
    estimated_DC = 1.0;
  }
  if(estimated_DC < min_DC)
  {
    estimated_DC = min_DC;
  }

  // Report human-readable stats every serialReportModulo iterations:
  if(serialReportCount < serialReportModulo)
  {
    serialReportCount++;
  }
  else
  {
    serialReportCount = 0;
    reportStats(RPM_value, MAP_value, estimated_DC);
  }
  
  
  return estimated_DC;
}


// ----------------- Main Program -----------------:

void setup() {
  // Set pin modes:
  pinMode(injectorPin, OUTPUT);
  pinMode(pointsPin, INPUT);
  pinMode(LEDPin, OUTPUT);

  // Attach interrupt for detecting points pulses:
  attachInterrupt(digitalPinToInterrupt(pointsPin), points, RISING);

  //Start Serial port:
  Serial.begin(115200);
  Serial.println("EFI controller based on MAP and Points ONLY");
  Serial.println("(C) 2020 John Patterson Consulting, LLC");
  Serial.println(" ");

  //Wait for fuel pump to build pressure:
  Serial.println("Waiting for fuel pump...");
  delay(pumpTime);
  Serial.println(" ");

  // Prime engine with some fuel before starting:
  Serial.println("Priming intake...");
  digitalWrite(injectorPin, HIGH);
  delay(primeTime);
  digitalWrite(injectorPin, LOW);
  Serial.println("Priming complete. Ready to crank!");
  Serial.println(" ");
}

void loop() {
  //Determine if the engine is currently stalled:
  stalled = (RPM() < stallRPM);

  // If engine is running, keep LED on and steady:
  if(!stalled)
  {
    digitalWrite(LEDPin, HIGH);
  }
  // If engine is stalled, blink LED:
  else if(millis() - blinkLastMillis > blinkTime)
  {
    blinkLastMillis = millis();
    blinkState = !blinkState;
//    digitalWrite(LEDPin, blinkState);
  }
  
 
  // Pulse the injector if it is time to do so:
  if((micros() - updateLastMicros) > injectorUpdateInterval)
  {
    // Reset injector update timer:
    updateLastMicros = micros();

    // Compute fuel injector duty cycle:
    double computed_DC = getInjectorDC();

    // Only dispense fuel if engine is running:
    if(!stalled)
    {
      // Open injector for amount of time specified by duty cycle:
      unsigned long injectorMicros = micros();
      while((micros() - injectorMicros) < (unsigned long)(injectorUpdateInterval*computed_DC))
      {
        digitalWrite(injectorPin, HIGH);
      }

      //Close injector:
      digitalWrite(injectorPin, LOW);
    }
  }

}
