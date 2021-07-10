#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "thermistor.h"
#include "HardwareSerial.h"
#include  <TimerOne.h>          // Avaiable from http://www.arduino.cc/playground/Code/Timer1
#include  <MsTimer2.h>

#include "EmonLib.h"                   // Include Emon Library
EnergyMonitor emon1;                   // Create an instance


volatile static boolean acinterrupt = 0;

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define Main 10
#define AC 9
#define DC 6
#define CH 8
#define FULLCH 7
#define Temp 5
#define RELAY 4
#define BAZZER 11

//Dimmer
static int clock_tick; // variable for Timer1
static int CH1;
//option 2
volatile int i = 0;             // Variable to use as a counter volatile as it is in an interrupt
volatile boolean zero_cross = 0; // Boolean to store a "switch" to tell us if we have crossed zero
int AC_pin = 3;                // Output to Opto Triac
static int dim = 95;                    // Dimming level (0-128)  0 = on, 128 = 0ff




#define FASTADC 1

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif



//Create Objects:

// Thermistor object
THERMISTOR thermistor(A3,        // Analog pin
                      10000,          // Nominal resistance at 25 ºC
                      3950,           // thermistor's beta coefficient
                      10000);         // Value of the series resistor


//Function Prototype
float acVoltage(int adcpin);
float average(int pin);
int ampare_sense(int pin);
void zero_crosss_int();

//Global Variable
unsigned long currentMillis = 0, previousMillis1 = 0, previousMillis2 = 0, previousMillis3 = 0, previousMillis4 = 0, previousMillis5 = 0, previousMillis6 = 0, previousMillis7 = 0, previousMillis8 = 0, previousMillis9 = 0, previousMillis10 = 0, previousMillis11 = 0, previousMillis12 = 0, tolarange = 0, lastTick = 0, calculationhz = 0;
static boolean lcdprint, lcdclear = 1, acled, dcled, chargingled, temparature, buzzer;
volatile static int ACdelay = 0;
volatile static int OutputCharge = 0;


int dimmerout = 95;
//Flag Variable
static int Batteryflag = 0, Mainsflag = 0, TemparatureFlag = 0, ChargingDimmar = 0, fullcharge = 0;
volatile static boolean charging;
int freqCounter = 0;
void flash() {
  lcdprint = !lcdprint;

}


void setup() {

  //emon1.current(2, 0.45);             // Current: input pin, calibration.

  pinMode(AC_pin, INPUT);
  digitalWrite(AC_pin, LOW);

  pinMode(AC_pin, OUTPUT);// Set AC Load pin as output
  digitalWrite(AC_pin, LOW);

  lcd.begin();
  lcd.backlight();



#if FASTADC
  // set prescale to 8
  sbi(ADCSRA, ADPS1) ;
  sbi(ADCSRA, ADPS0) ;
  cbi(ADCSRA, ADPS2) ;
  //cbi(ADCSRA,ADPS0) ;
#endif

  pinMode(Main, OUTPUT);// Set AC Load pin as output
  pinMode(AC, OUTPUT);// Set AC Load pin as output
  pinMode(DC, OUTPUT);// Set AC Load pin as output
  pinMode(CH, OUTPUT);// Set AC Load pin as output
  pinMode(FULLCH, OUTPUT);// Set AC Load pin as output
  pinMode(Temp, OUTPUT);// Set AC Load pin as output
  pinMode(RELAY, OUTPUT);// Set AC Load pin as output
  pinMode(BAZZER, OUTPUT);// Set AC Load pin as output



  //attachInterrupt(0, zero_crosss_int, RISING);
  //Timer1.initialize(100); // set a timer of length 100 microseconds for 50Hz or 83 microseconds for 60Hz;
  //Timer1.attachInterrupt( timerIsr ); // attach the service routine here
  dim = 95;

  MsTimer2::set(3000, flash); // 500ms period
  MsTimer2::start();

  //All Interrupt off
  //detachInterrupt(digitalPinToInterrupt(2));
  digitalWrite(AC_pin, LOW);


  //Serial.begin(9600);

  lcd.clear();

  Serial.begin(9600);

  //digitalWrite(RELAY, HIGH);
  delay(1000);

}

void timerIsr()
{

  freqCounter++;
  if (zero_cross == true) {

    if (i >= dim) {
      digitalWrite(AC_pin, HIGH); // turn on light
      i = 0; // reset time step counter
      zero_cross = false; //reset zero cross detection
    }
    else {
      i++; // increment time step counter
    }
  }


}



void zero_crosss_int() // function to be fired at the zero crossing to dim the light
{
  // Every zerocrossing interrupt: For 50Hz (1/2 Cycle) => 10ms ; For 60Hz (1/2 Cycle) => 8.33ms
  // 10ms=10000us , 8.33ms=8330us

  //clock_tick = 0;

  zero_cross = true;               // set the boolean to true to tell our dimming function that a zero cross has occured
  i = 0;
  digitalWrite(AC_pin, LOW);       // turn off TRIAC (and AC)


}




float MainsVoltage;
float BatteryVoltage; //Convert to 25.0635V
static int ChargingAmpare;
uint16_t Tempature;


void loop()
{

  //delay(300);
  currentMillis = millis();


  //Frequency count

  if (currentMillis - previousMillis12 > (1000 - tolarange)) {
    previousMillis12 = currentMillis;
    freqCounter = 0;

  }

  if (currentMillis - previousMillis10 > 13) {
    previousMillis10 = currentMillis;

    MainsVoltage = acVoltage(A0);
    ChargingAmpare = ampare_sense(A2);




    //double Irms = emon1.calcIrms(100);  // Calculate Irms only
    //ChargingAmpare = Irms * 100;
    //ChargingAmpare = ChargingAmpare;
    /*
        Serial.print("Tempature :");
        Serial.println(Tempature);
        Serial.print("MainsVoltage:");
        Serial.println(MainsVoltage);
        Serial.print("Battery :");
        Serial.println(BatteryVoltage);
        Serial.print("DIM :");
        Serial.println(dim);		       // Irms

      Serial.print("Ampare 2 :");
      Serial.println(ChargingAmpare);          // Irms*/
    Serial.println(ChargingAmpare);
    Serial.print("Frequency :");
    Serial.println(((freqCounter/10)/10));
  }

  if (currentMillis - previousMillis9 > 233) {
    previousMillis9 = currentMillis;
    BatteryVoltage = (average(A1) * 0.03); //Convert to 25.0635V
  }
  if (currentMillis - previousMillis11 > 333) {
    previousMillis11 = currentMillis;
    Tempature = thermistor.read();
  }




  if (BatteryVoltage > 8 && BatteryVoltage <= 16) {
    Batteryflag = 1;
    digitalWrite(DC, HIGH);
  }
  else
    Batteryflag = 0;

  if (MainsVoltage >= 120 && MainsVoltage <= 300) {
    digitalWrite(Main, HIGH);
  }
  else {
    Mainsflag = 0;
    ChargingDimmar = 0;
    dimmerout = 0;
    ACdelay = 0;
    digitalWrite(Main, LOW);
    dim = 95;
    //All Interrupt off
    detachInterrupt(digitalPinToInterrupt(2));
    Timer1.stop();
    digitalWrite(AC_pin, LOW);

  }
  if (Tempature < 45) {
    TemparatureFlag = 1;
    digitalWrite(Temp, LOW);
  }
  else {
    TemparatureFlag = 0;
    digitalWrite(Temp, HIGH);
    ACdelay = 0;
  }
  if (ACdelay == 10) {
    Mainsflag = 1;
    attachInterrupt(0, zero_crosss_int, RISING);
    Timer1.initialize(100);
    Timer1.attachInterrupt( timerIsr );

  }
  if (MainsVoltage > 300) {
    digitalWrite(AC, HIGH);
  }
  else if (MainsVoltage >= 120 && MainsVoltage <= 300) {
    digitalWrite(AC, LOW);
  }

  charging = ((Batteryflag == 1) && (Mainsflag == 1) && (TemparatureFlag == 1) && (fullcharge == 0));

  if (charging == 1 && ACdelay == 14) {
    OutputCharge = 1;
  }

  if (charging == 0) {
    ChargingDimmar = 0;
    dimmerout = 0;
    dim = 95;


  }
  if (charging == 1) {
    digitalWrite(RELAY, HIGH);
  }
  else {

    digitalWrite(RELAY, LOW);
  }
  if (BatteryVoltage > 14.50) {
    dim = 95;
    digitalWrite(FULLCH, HIGH);
    fullcharge = 1;

  }

  //Ac Mains DELAY
  if ((currentMillis - previousMillis2 > (981 - tolarange)) && (MainsVoltage >= 120 && MainsVoltage <= 300)) {
    // save the last time you blinked the LED
    previousMillis2 = currentMillis;
    ACdelay++;

  }
  //DC FAIL LED
  if (currentMillis - previousMillis3 > (520 - tolarange) && Batteryflag == 0) {
    // save the last time you blinked the LED
    previousMillis3 = currentMillis;
    dcled = !dcled;
    digitalWrite(DC, dcled);
  }
  //AC LOW VOLTAGE
  if (currentMillis - previousMillis5 > (520 - tolarange) && MainsVoltage < 120) {
    // save the last time you blinked the LED
    previousMillis5 = currentMillis;
    acled = !acled;
    digitalWrite(AC, acled);
  }
  //Charging LED
  if ((currentMillis - previousMillis6 > 333) && (charging == 1) && OutputCharge == 1) {
    // save the last time you blinked the LED
    previousMillis6 = currentMillis;
    chargingled = !chargingled;
    digitalWrite(CH, chargingled);
    //Serial.println("Charging ");
  }
  //digitalWrite(CH, HIGH);
  //Temparature failt led
  if (currentMillis - previousMillis7 > (680 - tolarange) && (TemparatureFlag == 0)) {
    // save the last time you blinked the LED
    previousMillis7 = currentMillis;
    temparature = !temparature;
    digitalWrite(Temp, temparature);
  }

  if (currentMillis - previousMillis8 > (360 - tolarange) && ((Batteryflag == 0) || (MainsVoltage < 120 || MainsVoltage > 300) || (TemparatureFlag == 0))) {
    // save the last time you blinked the LED
    previousMillis8 = currentMillis;
    buzzer = !buzzer;
    digitalWrite(BAZZER, buzzer);
  } else {

    digitalWrite(BAZZER, LOW);
  }

  //Trac Section
  if (ChargingAmpare > dim && fullcharge == 0 && OutputCharge == 1)
    ChargingDimmar = 1;

  if ((currentMillis - previousMillis1 > 100) && ((charging == 1) && (ChargingDimmar == 0) && (ChargingAmpare < dim) &&  (OutputCharge == 1))) {
    previousMillis1 = currentMillis;
    //dimmerout--;
    //dim = map(dimmerout, 100,0, 95, 0);
    dim--;

  }
  if ((ChargingDimmar == 1) && (charging == 1) && OutputCharge == 1) {

    //int controlamp = map(dimandch, 0, 100, 95, 0);

    //dim = map(dim + ChargingAmpare, 100, 0, 95, 0);

    if (dim < ChargingAmpare && (dim < 95)) {
      dim++;
    }
    if (dim > ChargingAmpare && (dim > 0)) {
      dim--;
    }
    //Serial.println(" This Stage ");
    //Serial.print(dim);
    //Serial.println();
    //delay(100);

  }








  if (currentMillis - previousMillis4 > 0) {

    tolarange = currentMillis - previousMillis4;
    previousMillis4 = currentMillis;

  }

  if (lcdclear != lcdprint) {
    lcd.clear();
    lcdclear = lcdprint;
  }
  if (lcdprint == 1) {

    lcd.setCursor(0, 0); // top left
    lcd.print("Batt:");
    lcd.setCursor(5, 0); // top left
    lcd.print(BatteryVoltage);
    lcd.setCursor(10, 0); // top left
    lcd.print("V");
    lcd.setCursor(0, 1); // top left
    lcd.print("Mains:");
    lcd.setCursor(6, 1); // top left
    lcd.print(MainsVoltage);
    lcd.setCursor(12, 1); // top left
    lcd.print("V");

  }
  if (lcdprint == 0) {


    lcd.setCursor(0, 0); // top left
    lcd.print("Char:");
    lcd.setCursor(5, 0); // top left
    lcd.print(ChargingAmpare);
    lcd.setCursor(10, 0); // top left
    lcd.print("%");
    lcd.setCursor(0, 1); // top left
    lcd.print("Temp:");
    lcd.setCursor(5, 1); // top left
    lcd.print(Tempature);
    lcd.setCursor(7, 1); // top left
    lcd.print((char)223);
    lcd.setCursor(9, 1); // top left
    lcd.print("Dim:");
    lcd.setCursor(13, 1); // top left
    lcd.print(dim);

  }
}

float acVoltage(int adcpin) {
  float avg = 0, mesure = 0, finaloutput = 0;
  unsigned long sum = 0, squre = 0, adcvalue = 0;
  int returnresult = 0;

  for (int j = 0; j < 20; j++) {
    adcvalue = average(adcpin);
    squre = (long)adcvalue * adcvalue;
    //delayMicroseconds(130);
    sum += squre;
  }

  avg = sum / 20;
  //Serial.print("after average of squre");
  //Serial.println(avg);
  mesure = sqrt(avg);

  // four step voltage calculation
  if ((0 <= mesure) && (mesure <= 253)) {//110v
    finaloutput = (mesure * 0.43208);
  }
  else if ((254 <= mesure) && (mesure <= 553)) {//220
    finaloutput = (mesure * 0.403834);
  }
  else if ((554 <= mesure) && (mesure <= 630)) { //252
    finaloutput = (mesure * 0.40);
  }
  else if ((631 <= mesure) && (mesure <= 930)) {//260---<<
    finaloutput = (mesure * 0.4090);
  }
  returnresult = round(finaloutput);
  returnresult = (int)finaloutput;
  return returnresult;
}


float average(int pin) {
  float sum = 0, avg = 0, val = 0;
  float sendvalue;
  for (int i = 0; i < 5; i++)
  {
    val = analogRead(pin);
    //delayMicroseconds(20);
    sum += val;

  }

  avg = (float)sum / 5;
  sendvalue = round(avg);
  return sendvalue;

}
int ampare_sense(int pin) {
  float avgamp = 0, mesureamp = 0, finaloutputamp = 0;
  unsigned long sumamp = 0, adcvalueamp = 0;
  int returnresultamp = 0;

  for (int j = 0; j < 10; j++) {
    adcvalueamp = average(pin);
    //delayMicroseconds(20);
    sumamp += adcvalueamp;
  }

  avgamp = sumamp / 10;
  returnresultamp = map(avgamp, 0, 1023, 0, 100);
  return (int)returnresultamp;

}
