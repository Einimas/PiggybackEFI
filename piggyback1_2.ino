// EFI controller based on MAP and Points ONLY, V2
// This version includes a complete AFR mapping table.
// Thresholded rate-of-change-of-MAP is also used to emulate delta-TPS.
// (C) 2021 John Patterson Consulting, LLC
// You are free to modify and redistribute this code without restriction.


// ----------------- Program constants -----------------:

// Pin numbers:
const int pointsPin = 12;
const int mapPin = 7;
//const int injectorIn = 11;
//const int lambdaPin = ;
const int iatPin = 6;
//const int tpsPin = ;
//const int cltPin = ;
const int injectorPin = 17;
const int injectorPin1 = 18;
const int injectorPin2 = 19;
const int injectorPin3 = 20;
const int ignitionPin = 21;
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
const int pulsesPerRevolution = 2;      //  If picking up signal from ignition coil, if picking it up from tach might be 1
const double displacement_Liters = 1.5;
const double injector_MaxGramsPerMinute = 260.0;  // Will depend on your injector and fuel pressure, *4 if using 4 injectors on single trigger
const double stallRPM = 300.0;                      /// maybe use with idle air valve????
//const double Pascals_Per_ADC_Unit = 122.1;
//const int vacuum_ADC_value = 102;
const double min_DC = 0.07;                      // Minimum duty cycle for idling
const double vacuum_ROC_Multiplier = 0.0005;     // Add or remove AFR proportional to the rate of change of MAP
const double vacuum_ROC_Cut_In = 51710;          // MAP pressure over which vacuum ROC will be applied

//Fuel AFR selector (gas or E51):
//const double fuel_comp = 1.0;    // Gas
const double fuel_comp = 1.06;     // lets call it ron95 e15
//const double fuel_comp = 1.3;    // E51 // corrected engines use more etchanol, not less

//IAT sensor calculating constants //
// stole some code from: https://forum.arduino.cc/t/bosch-automotive-pressure-temperature-sensor-0281006059-0281006060/980118/4
//calculated steinheart here: https://forum.arduino.cc/t/bosch-0281006059-temp-sensor/1062239/9
//google sheets: https://docs.google.com/spreadsheets/d/1g7HUZ1HyK0B082gxlFn5IR-zISHINthUldbfXhKSpno/edit?usp=sharing
// 
int tempRead = 0;
//float Vin = 5.00; //
float Vout = 0.00;
float R1 = 6600; // Temperature resistor value
float tempBuff = 0;
float logR2, R2, temperature;

float A= 1.305213816e-03, B = 2.582304849e-04, C = 1.769345582e-07;  // Steinhart-Hart and Hart Coefficients mano skaiciuoti


// Richer at idle only:
// AFR mapping:
// BAR: 0.5-1.5 ATMOSPHERIC            
double AFR_500RPM[10] =  {14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 13.500, 13.000, 12.500, 12.000};
double AFR_1000RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 13.500, 13.500, 13.000, 12.000};
double AFR_1500RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.700, 13.500, 13.500, 13.000, 12.000};
double AFR_2000RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.000, 13.500, 13.500, 13.000, 12.000};
double AFR_2500RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.000, 14.000, 13.000, 13.500, 12.000};
double AFR_3000RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.000, 13.500, 13.500, 13.000, 12.000};
double AFR_3500RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.000, 13.500, 13.500, 13.000, 12.000};
double AFR_4000RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.000, 13.500, 13.500, 13.000, 12.000};
double AFR_4500RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.000, 13.500, 13.500, 13.000, 12.000};
double AFR_5000RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.000, 13.500, 13.500, 13.000, 12.000};
double AFR_5500RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.000, 13.500, 13.500, 13.000, 12.000};
double AFR_6000RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.000, 13.500, 13.500, 13.000, 12.000};
double AFR_6500RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.000, 13.500, 13.500, 13.000, 12.000};
double AFR_7000RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.000, 13.500, 13.500, 13.000, 12.000};
double AFR_7500RPM[10] = {14.700, 14.700, 14.700, 14.700, 14.700, 14.000, 13.500, 13.500, 13.000, 12.000};

// Physical parameters:
//const double airTempKelvin = 298;   // ---------------------------------------------using IAT--------------------------------------------------------//
//const double airMolecularMass = 28.9;
const double airMoldivBoltz =3527.62; //constant mol/boltz
//const double R_Liter_Pascal_Kelvin = 8314.0;


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
//Function to obtain MAP column
 
  double pressMAP() //think of better way of selecting a cell
  {
      int MAP_index= 0;
      float MAP_cell;
      int MAP_index_max= 9;  //0-9 - 0,5-1,5 bar (0.1) increments add collumns or change proportion to raise pressure resolution
      MAP_index= MAP_cell;    
       
    if(MAP_index < 0) 
  {
    MAP_index = 0; //forbids negative values of MAP_index
  }
    if(MAP_index > MAP_index_max)
  {
    MAP_index = MAP_index_max;  //limits MAP_index read off pressMAP by MAP_index_max
  }
    
  return MAP_index;
    }

// Function to obtain AFR target index:
double AFR(double speedRPM, double MAPpress)
{
  // Determine RPM range to use and return relevant AFR:

 // double speedRPM()
 // {
 //   int speedRPM =0;
    
 // }
  int MAP_index= 0; ///convert to array output of (rpm row, map index)
  MAP_index= pressMAP();
  
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


// Function to compute RPM based on time since last points opening:   ////might neet to ajust to work on tach
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

// Function to measure the Manifold Absolute Pressure (MAP):           /////////////// Adapted to 0281006059 bosch 4bar map 0.2bar-0,215v(44duty); 4bar-4,5v(914duty) (1bar vac, 3bar boost) /////////////////////
double MAP()
{
  double MAPsensor = analogRead(mapPin);                    //////duty=1024*(voltage div resistor)/((R sensor)+(voltage div resistor))  (
  
  float estimatedMAP = map(MAPsensor,102,921,50,400);  //////4bar in kpa

  ///might use mathematical alternative to finding mapcell to map using corrected map function y = (x - in_min) * (out_max - out_min +1) / (in_max - in_min+1) + out_min
  

  // Ensure only positive MAP values are returned:
  if(estimatedMAP > 400)
  {
    estimatedMAP = 400;
  }
  if(estimatedMAP < 50)
  {
    estimatedMAP = 50;
  }
  
  return estimatedMAP;
}

// Function to measure the Intake Air Temperature (IAT):
double IAT()                                                                             /////////////////////////////////////////////////////////////////////////////
{
  double airTempduty = (analogRead(iatPin));        /////////////// 0281006059 bosch -40 to 150 celsius might use calculated lookup table if the formulla is too slow on nano328
          
   tempRead = airTempduty;
  if(tempRead)
  {
    tempBuff = tempRead * 5.00;
    Vout = (tempBuff)/1024.0;
    tempBuff = (5.00/Vout) -1; //sensor voltage compensation 
    R2 = R1 / tempBuff;
  }
  
    logR2 = log(R2);
    airTempKelvin = ( 1 / (A + B*logR2 + C*logR2*logR2*logR2));  // Steinhart and Hart Equation. T  = 1 / {A + B[ln(R)] + C[ln(R)]^3}
 //   IATcelsius =  temperature - 273.15;
 //   Serial.print("IAT C: ");
 //   Serial.println(IATcelsius);

  if(airTempKelvin < 0)
  {
    airTempKelvin = 0;
  }
  
  return airTempKelvin;
}

/*// Function to report human-readable stats to the serial port:
void reportStats(double RPM_report, double MAP_report, double DC_report)
{
  // Report engine RPM:
  Serial.print("Engine RPM: ");
  Serial.println(RPM_report);
  Serial.print("Manifold Absolute Pressure: ");
  Serial.print("kPa: ");
  Serial.print(MAP_report/1000.0);
  Serial.print(" BAR: ");
  Serial.println(MAP_report/6894.76);
  Serial.print("Injector Duty Cycle: ");
  Serial.print(DC_report*100.0);
  Serial.println("%");
  Serial.println(" ");
}
*/

// Function to compute the required fuel injector duty cycle:
double getInjectorDC()
{
  //Get measurements:
  double RPM_value = RPM();
  double MAP_value = MAP();
  double IAT_value = IAT();
//  double LAMBDA_value = LAMBDA();
  
  // Compute the volume flow rate of air (L/min) into the engine
  // (Otto cycle with 2 revolutions per intake stroke, 100% efficiency assumed):
  double volFlowRate = displacement_Liters*RPM_value/2.0;
  double airPress = MAP_value /100.00;
// https://en.wikipedia.org/wiki/Density_of_air 
// ro(g/l)= p(in pascal)/T(absolute temperature (K))*m(molecular mass of dry air)/kb(Boltzmann constant constant)
  double airDensity= MAP_value / 10 / IAT_value * airMoldivBoltz;

  
  // Compute the mass air flow rate into the engine: //might add a crude ve coef before reworking it fully
  double massAirFlow = volFlowRate*airDensity;

  // Get AFR:

  double MAP_cell = map(MAP_value,50,400,0,34);
  
  // Adjust the AFR based on the rate of change of the MAP level:
  double vacROC = (double)MAP_cell - (double)lastMAP;
  lastMAP = MAP_cell;

  // Look up AFR in table:
  double AFR_target = AFR(RPM_value,MAP_cell);
  
  // If MAP is above threshold, apply rate-of-change factor:
  if(MAP_cell > vacuum_ROC_Cut_In)
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

 /* // Report human-readable stats every serialReportModulo iterations:
  if(serialReportCount < serialReportModulo)
  {
    serialReportCount++;
  }
  else
  {
    serialReportCount = 0;
    reportStats(RPM_value, MAP_value, estimated_DC);
  }
  */
  
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
 /* Serial.println("EFI controller based on MAP and Points ONLY");
  Serial.println("(C) 2020 John Patterson Consulting, LLC");
  Serial.println(" ");

  //Wait for fuel pump to build pressure:
  Serial.println("Waiting for fuel pump...");
 */
  delay(pumpTime);
  Serial.println(" ");

  // Prime engine with some fuel before starting:
 // Serial.println("Priming intake...");
  digitalWrite(injectorPin, HIGH);
  delay(primeTime);
  digitalWrite(injectorPin, LOW);
 // Serial.println("Priming complete. Ready to crank!");
 // Serial.println(" ");
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
