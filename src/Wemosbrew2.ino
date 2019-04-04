
//-------------------------------------------------------------------
//
// Brew Controller
//
//
// Based on the Arduino PID and PID AutoTune Libraries
// inspired by https://www.instructables.com/id/Arduino-Brew-Mashing-PID-Controller/
// Denis Antoine 2019
//------------------------------------------------------------------

#include <Arduino.h>

// wifi et MQTT
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// PID Library
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Libraries for the Adafruit OLED Shield
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Libraries for the DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// So we can save and retrieve settings
#include <ESP_EEPROM.h>

// Ticker to manage interrupt
#include <Ticker.h>

// config file for Wifi and MQTT
#include "config.h"


// ************************************************
// Pin definitions
// ************************************************

// OLED
#define OLED_RESET 0  // GPIO0

// Status LED
#define STATUSLED_PIN D3 // optional feature

// Output Relay heat
#define RELAY_PIN D0

// Output PWM Pump
#define PWM_OUT D8

// One-Wire Temperature Sensor
#define ONE_WIRE_BUS D4

// Buttons
#define BUTTON_DOWN_PIN D5
#define BUTTON_UP_PIN D6
#define BUTTON_SELECT_PIN D7

//------- Button values Settings -------
#define BUTTON_DOWN 0x04
#define BUTTON_UP 0x02
#define BUTTON_SELECT 0x01


WiFiClient espClient;
PubSubClient client(espClient);
boolean offlineMode = false; // defines if we operate online or offline
boolean statusWifi = false;
boolean statusMQTT = false;

long lastSent = 0;   // timestamp last MQTT message published
long lastReceived = 0;
char message_buff[100]; //Buffer for incomming MQTT messages


// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;
float pctchauf;

// Initialize default StepSetPoint
double stepset[] = {52, 63, 72, 78}; //temperature of steps
long stepd[] = {12, 30, 40, 10}; //duration in minutes of steps

volatile long onTime = 0;

// pid tuning parameters
double Kp;
double Ki;
double Kd;

// EEPROM addresses for persisted data
struct MyEEPROMStruct {
  double  eeSetpoint;
  double  eeKp;
  double  eeKi;
  double  eeKd;
  byte  eePwm;
  } eepromVar1;


//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_M, DIRECT); //P_ON_M Mode is Proportional on measurement to avoid overshoot

// 15 second Time Proportional Output window
int WindowSize = 15000; //to be adjusted
unsigned long windowStartTime;

// Ticker to manage time interrupt
Ticker timer;



// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember=2;
double aTuneStep=1000; // 1 sec step for 15000 window
double aTuneNoise=0.25; // according to temp precision
unsigned int aTuneLookBack=300; // very slow process
boolean tuning = false;
PID_ATune aTune(&Input, &Output);

// ************************************************
// DiSplay Variables and constants
// ************************************************

Adafruit_SSD1306 display(OLED_RESET);
unsigned long lastInput = 0; // last button press

// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, RUN, AUTO, SETP, TUNE_P, TUNE_I, TUNE_D, PWM};
operatingState opState = OFF;

// ************************************************
// Sensor Variables and constants
// *************************************************
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor;

//  PWM variables to control the pump
byte pwm_value =0;
boolean pwm_on =false; // if true then PWM can be set. false means it will be a simple on/off

// ************************************************
// Setup and diSplay initial screen
// ************************************************
void setup()
{
Serial.begin(115200);

// Button init
Serial.println("Initialize Buttons");
pinMode(BUTTON_DOWN_PIN, INPUT);
pinMode(BUTTON_UP_PIN, INPUT);
pinMode(BUTTON_SELECT_PIN, INPUT);

// LED status
pinMode(STATUSLED_PIN, OUTPUT);
digitalWrite(STATUSLED_PIN, HIGH); //High during init

// Initialize Relay Control
Serial.println("Initialize Relay Control");
pinMode(RELAY_PIN, OUTPUT);    // Output mode to drive relay
digitalWrite(RELAY_PIN, LOW);  // make sure it is off to start

// Initialize PWM output
Serial.println("Initialize PWM output");
pinMode(PWM_OUT, OUTPUT);
if (pwm_on == true){
  analogWrite(PWM_OUT, pwm_value);
  }
else {
  digitalWrite(PWM_OUT, LOW);  // simple onoff logic
  }

// Start up the DS18B20 One Wire Temperature Sensor
Serial.println("Start up the DS18B20");
sensors.begin();
if (!sensors.getAddress(tempSensor, 0)) {
  display.println("Sensor Error");
  }
sensors.setResolution(tempSensor, 12);
sensors.setWaitForConversion(false);
display.display();

// Interrupt Ticker each sec
timer.attach_ms(1000, heat);

// Set up the initial (default) values for what is to be stored in EEPROM
eepromVar1.eeSetpoint = 60;
eepromVar1.eeKp = 850;
eepromVar1.eeKi = 0.5;
eepromVar1.eeKd = 0.1;
eepromVar1.eePwm = 150;

Serial.println("Read eeprom");
EEPROM.begin(sizeof(MyEEPROMStruct));
if(EEPROM.percentUsed()!=0) {
  EEPROM.get(0, eepromVar1);
  Serial.println("EEPROM has data from a previous run.");
  Serial.print(EEPROM.percentUsed());
  Serial.println("% of ESP flash space currently used");
  }
else {
  Serial.println("EEPROM size changed - EEPROM data zeroed - commit() to make permanent");
  }

// Assign current variables from stored values
Setpoint = eepromVar1.eeSetpoint;
Kp = eepromVar1.eeKp;
Ki = eepromVar1.eeKi;
Kd = eepromVar1.eeKd;
pwm_value = eepromVar1.eePwm;

// Initialize PID setup
myPID.SetTunings(Kp,Ki,Kd);
myPID.SetSampleTime(5000); // Origine 1000 change 5000
myPID.SetOutputLimits(0, WindowSize);

// Initialize LCD DiSplay
Serial.println("Initialize LCD DiSplay");
display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 64x48)
display.clearDisplay();
display.setRotation(2);
display.setTextSize(1);
display.setTextColor(WHITE);
display.setCursor(0,0);
display.println("init...");
display.display();

// Initialize Connexion
statusWifi = setup_wifi();
client.setServer(mqtt_server, 1883);
client.setCallback(callback);

// End setup
Serial.println("End Set up");
display.clearDisplay();
display.setCursor(0,0);
display.println("OFF");
display.display();

}

// ************************************************
// Timer Interrupt Handler
// ************************************************
void heat()
{
UpdateLedStatus();
// report to  MQTT
if (statusMQTT == true) {
    dtostrf(Input, 4, 2, message_buff);
    client.publish(temperature_topic, message_buff, true);
    dtostrf(Setpoint, 4, 2, message_buff);
    client.publish(setpoint_topic, message_buff, true);
    sprintf(message_buff, "%d", pwm_value);
    dtostrf(pctchauf, 6, 2, message_buff);
    client.publish(pctchauf_topic, message_buff, true);
    client.loop();
    }

if (opState == OFF){
  digitalWrite(RELAY_PIN, LOW);  // make sure relay is off
  if (pwm_on == true){
    analogWrite(PWM_OUT, 0); // switch off the pump
    }
  else {
    digitalWrite(PWM_OUT, LOW);  // switch off the pump
    }
  }

else{
  DriveOutput(); // Pid regulation
  }
}

// ************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************
void loop()
{
statusMQTT = reconnect(); // try to reconnect MQTT
client.loop();

// wait for button release before changing state

while(readButtons() != 0) {} // wait for button press

switch (opState){
  case OFF:
  Off();
  break;
  case RUN:
  Run();
  break;
  case AUTO:
  AutoControl();
  break;
  case SETP:
  Tune_Sp();
  break;
  case TUNE_P:
  TuneP();
  break;
  case TUNE_I:
  TuneI();
  break;
  case TUNE_D:
  TuneD();
  break;
  case PWM:
  setPWM();
  break;
  }
}

// ************************************************
// Initial State - OFF
// DOWN to reconnect
// UP to RUN
// SELECT to start Auto
// ************************************************
void Off()
{
Serial.println("Mode Off");
myPID.SetMode(MANUAL);
digitalWrite(RELAY_PIN, LOW);  // make sure it is off

uint8_t buttons = 0;
while(true){
  buttons = readButtons();
  if (buttons & BUTTON_DOWN) { // DOWN to reconnect
    offlineMode = false;
    opState = OFF; // remains OFF
    return;
    }

  if (buttons & BUTTON_UP){  // UP to RUN
    sensors.requestTemperatures(); // Start an asynchronous temperature reading
    myPID.SetMode(AUTOMATIC);
    windowStartTime = millis();
    opState = RUN; // start control in RUN mode
    return;
    }
  if (buttons & BUTTON_SELECT){ // SELECT to auto steps
    sensors.requestTemperatures(); // Start an asynchronous temperature reading
    myPID.SetMode(AUTOMATIC);
    windowStartTime = millis();
    opState = AUTO; // start control in AUTO mode
    return;
    }

  statusMQTT = reconnect(); // test and reconnect MQTT if down
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  if (statusMQTT == true) {
    client.publish(opState_topic, "OFF", true);
    client.loop();
    display.println("OFF Onl");
    }
  else  {
    display.println("OFF Offl");
    offlineMode = true; // Switch to offline operation
    display.print("Dwn > reconnect");
    }
  display.display();
  }
}

// ************************************************
// PID COntrol State
// DOWN to Auto tune parameters
// UP to OFF
// SELECT to tune SP
// ************************************************
void Run()
{
Serial.println("Mode RUN");
delay(100);
SaveParameters();
myPID.SetTunings(Kp,Ki,Kd);

uint8_t buttons = 0;
while(true){
  buttons = readButtons();
  if ((buttons & BUTTON_DOWN)
  && (abs(Input - Setpoint) < 0.5)){  // Should be at steady-state to start autotune
    StartAutoTune();
    }

  if (buttons & BUTTON_UP){
    opState = OFF;
    return;
    }

  if (buttons & BUTTON_SELECT){
    opState = SETP;
    return;
    }

  statusMQTT = reconnect(); // test and reconnect MQTT if down
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  if (statusMQTT == true) {
    client.publish(opState_topic, "RUN", true);
    dtostrf(Kp, 8, 2, message_buff);
    client.publish(Kp_topic, message_buff, true);
    dtostrf(Ki, 8, 2, message_buff);
    client.publish(Ki_topic, message_buff, true);
    dtostrf(Kd, 8, 2, message_buff);
    client.publish(Kd_topic, message_buff, true);
    client.loop();
    display.println("RUN  Onl");
    }
  else  {
    display.println("RUN Offl");
    }

  display.print("Sp: ");
  display.println(Setpoint);
  display.print("In: ");
  display.println(Input);
  display.print("Out: ");
  display.print((int)pctchauf);
  display.println("%");
  display.print("PWM: ");
  display.println(pwm_value);
  if (tuning){
    display.print("Tuning");

    }
  else{
    display.print("      ");
    }
  display.display();
  delay(100);
  DoControl();

  // Safe mode if no temperature reading
    if (Input <= 0){ //check if disconnected -127
    digitalWrite(RELAY_PIN, LOW); // switch off heat
    opState = OFF;
    return;
    }
  }
}


// ************************************************
// Auto Control
// UP to OFF
// SELECT to start and go to next step
// ************************************************
void AutoControl()
{
int step;
Serial.println("Mode Autocontrole");

uint8_t buttons = 0;
while(true){
  buttons = readButtons();
  if (buttons & BUTTON_UP) { // UP to OFF
    opState = OFF;
    return;
    }

  if (buttons & BUTTON_SELECT){ // Select to execute steps
    for (step = 0; step < 4; step++) { //loop sur 4 steps
      RunStep(step, stepset[step], stepd[step]);
      }
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("Fin de cycle");
    display.display();
    delay(5000);
    opState = RUN; // maintain current state in RUN
    return;
    }

  statusMQTT = reconnect(); // test and reconnect MQTT if down
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  if (statusMQTT == true) {
    client.publish(opState_topic, "AUTO", true);
    sprintf(message_buff, "%d", pwm_value);
    client.publish(pwm_topic, message_buff, true);
    dtostrf(stepset[0], 4, 2, message_buff);
    client.publish(sT1_topic, message_buff, true);
    dtostrf(stepset[1], 4, 2, message_buff);
    client.publish(sT2_topic, message_buff, true);
    dtostrf(stepset[2], 4, 2, message_buff);
    client.publish(sT3_topic, message_buff, true);
    dtostrf(stepset[3], 4, 2, message_buff);
    client.publish(sT4_topic, message_buff, true);
    sprintf(message_buff, "%d", stepd[0]);
    client.publish(sd1_topic, message_buff, true);
    sprintf(message_buff, "%d", stepd[1]);
    client.publish(sd2_topic, message_buff, true);
    sprintf(message_buff, "%d", stepd[2]);
    client.publish(sd3_topic, message_buff, true);
    sprintf(message_buff, "%d", stepd[3]);
    client.publish(sd4_topic, message_buff, true);
    client.loop();
    display.println("AUTO Onl");
    }
  else  {
    display.println("AUTO Offl");
    }

  display.println("S | T | D");

  for (step = 0; step < 4; step++) { // display steps values
    display.print(step+1);
    display.print(" | ");
    display.print(int(stepset[step]));
    display.print("| ");
    display.println(stepd[step]);
    }
  display.display();

  }
}

// ************************************************
// RUN Step
//
// SELECT - Stop
//
// ************************************************
void RunStep(int Step, double StepSetPoint, long StepTimeMin)
{
Serial.println("Step exec =");
Serial.println(Step);
delay(1000);

Setpoint = StepSetPoint; // Update set point
myPID.SetTunings(Kp,Ki,Kd);

long startTime = millis(); // Start timer
long endTime = startTime + StepTimeMin * 60000;
int minutes;

uint8_t buttons = 0;

while(millis() <= endTime) {
  buttons = readButtons();

  if (buttons & BUTTON_SELECT){ // Skip step and go to next
    delay(200);
    minutes = 0;
    return;
    }
  if (buttons & BUTTON_UP){ // extend 1 min
    delay(200);
    endTime += 60000;
    }

  if (buttons & BUTTON_DOWN){ // decrease 1 min
    delay(200);
    endTime -= 60000;
    }

  DoControl();

  // Safe mode if temperature not read
  if (Input <= 0){ //check if disconnected -127
    digitalWrite(RELAY_PIN, LOW); // switch off
    opState = OFF;
    return;
    }

  minutes = (endTime - millis())/60000+1;

  statusMQTT = reconnect();
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  if (statusMQTT == true) {
    sprintf(message_buff, "%d", minutes);
    client.publish(timerStep_topic, message_buff, true);
    sprintf(message_buff, "%d", Step+1);
    client.publish(autoStep_topic, message_buff, true);
    client.loop();
    display.println("Auto Onl");
    }
  else {
    display.println("Auto Offl");
    }

  display.print("STEP: ");
  display.println(Step+1);
  display.print("min: ");
  display.println(minutes);
  display.print("Sp: ");
  display.println(Setpoint);
  display.print("In: ");
  display.println(Input);
  display.display();
  delay(100);
  }
}

// ************************************************
// Setpoint Entry State
// UP/DOWN to change setpoint
// SELECT for tuning parameters
// ************************************************
void Tune_Sp()
{
Serial.println("Mode Reglage Set Point - Select pour RUN");
delay(100);

uint8_t buttons = 0;
while(true){
  buttons = readButtons();
  float increment = 0.1;
  if (buttons & BUTTON_SELECT){
    SaveParameters();
    opState = TUNE_P;
    return;
    }
  if (buttons & BUTTON_UP){
    Setpoint += increment;
    delay(200);
    }
  if (buttons & BUTTON_DOWN){
    Setpoint -= increment;
    delay(200);
    }
  if ((millis() - lastInput) > 10000) {  // return to RUN after 10 seconds idle
    opState = RUN;
    return;
    }
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("SetPoint");
  display.print("Set T:");
  display.setCursor(0,20);
  display.setTextSize(2);
  display.print(Setpoint);
  display.display();
  DoControl();
  }
}

// ************************************************
// Proportional Tuning State
// UP/DOWN to change Kp
// SELECT for Ki
// ************************************************
void TuneP()
{
Serial.println("Mode Reglage Kp - Select pour Ki");
delay(100);

uint8_t buttons = 0;
while(true){
  buttons = readButtons();
  float increment = 1.0;

  if (buttons & BUTTON_SELECT){
    SaveParameters();
    opState = TUNE_I;
    return;
    }
  if (buttons & BUTTON_UP){
    Kp += increment;
    delay(200);
    }
  if (buttons & BUTTON_DOWN){
    Kp -= increment;
    delay(200);
    }
  if ((millis() - lastInput) > 10000){  // return to RUN after 10 seconds idle
    opState = RUN;
    return;
    }
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Reglage Kp");
  display.print("Set Kp:");
  display.setTextSize(2);
  display.setCursor(0,20);
  display.print(Kp);
  display.display();
  DoControl();
  }
}

// ************************************************
// Integral Tuning State
// UP/DOWN to change Ki
// SELECT for Kd
// ************************************************
void TuneI()
{
Serial.println("Mode Reglage Ki - Select pour Kd");
delay(100);

uint8_t buttons = 0;
while(true){
  buttons = readButtons();
  float increment = 0.01;

  if (buttons & BUTTON_SELECT){
    SaveParameters();
    opState = TUNE_D;
    return;
    }
  if (buttons & BUTTON_UP){
    Ki += increment;
    delay(200);
    }
  if (buttons & BUTTON_DOWN){
    Ki -= increment;
    delay(200);
    }
  if ((millis() - lastInput) > 10000){  // return to RUN after 10 seconds idle
    opState = RUN;
    return;
    }
  delay(100);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Reglage Ki");
  display.print("Set Ki:");
  display.setTextSize(2);
  display.setCursor(0,20);
  display.print(Ki);
  display.display();
  DoControl();
  }
}

// ************************************************
// Derivative Tuning State
// UP/DOWN to change Kd
// SELECT for PWM
// ************************************************
void TuneD()
{
Serial.println("Mode Reglage Kd - Select pour PWM");
delay(100);
uint8_t buttons = 0;
while(true){
  buttons = readButtons();
  float increment = 0.01;

  if (buttons & BUTTON_SELECT){
    SaveParameters();
    opState = PWM;
    return;
    }
  if (buttons & BUTTON_UP){
    Kd += increment;
    delay(200);
    }
  if (buttons & BUTTON_DOWN){
    Kd -= increment;
    delay(200);
    }
  if ((millis() - lastInput) > 10000){  // return to RUN after 10 seconds idle
    opState = RUN;
    return;
    }
  delay(100);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Reglage Kd");
  display.print("Set Kd:");
  display.setTextSize(2);
  display.setCursor(0,20);
  display.print(Kd);
  display.display();
  DoControl();
  }
}
// ************************************************
// PWM set
// UP/DOWN to change PWM value
// SELECT for RUN
// ************************************************
void setPWM()
{
Serial.println("Mode Reglage PWM - Select pour RUN");
delay(100);

uint8_t buttons = 0;
while(true){
  buttons = readButtons();
  float increment=4;

  if (buttons & BUTTON_SELECT){
    SaveParameters();
    opState = RUN;
    return;
    }
  if (buttons & BUTTON_UP){
    if (pwm_on == true){
      pwm_value += increment;
      }
    else {
      pwm_value = HIGH;
      }
    delay(200);
    }
  if (buttons & BUTTON_DOWN){
    if (pwm_on == true){
      pwm_value -= increment;
      }
    else{
      pwm_value = LOW;
      }

  delay(200);
  }
  if ((millis() - lastInput) > 10000){  // return to RUN after 10 seconds idle
    opState = RUN;
    return;
    }

  if (pwm_on == true){
    analogWrite(PWM_OUT, pwm_value); //  mode PWM
    }
  else {
    digitalWrite(PWM_OUT, pwm_value);  // mode relay simple
    }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Reglage PWM");
  display.print("Set PWM:");
  display.setTextSize(2);
  display.setCursor(0,30);
  display.print(pwm_value);
  display.display();
  DoControl();
  }
}

// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
{
// update pump setting
if (pwm_on == true){
  analogWrite(PWM_OUT, pwm_value); // mode PWM
  }
else {
  digitalWrite(PWM_OUT, pwm_value);  // mode relay simple
  }

// update temperature control
Input = sensors.getTempC(tempSensor);
sensors.requestTemperatures();
if (Input <= 0){ //check if disconnected -127
  digitalWrite(RELAY_PIN, LOW); // safe
  opState = OFF;
  return;
  }

if (tuning){ // run the auto-tuner
  if (aTune.Runtime()){ // returns 'true' when done
    FinishAutoTune();
    }
  }
else { // Execute control algorithm
  myPID.Compute();
  }

// Time Proportional relay state is updated regularly via timer interrupt.
onTime = Output;
pctchauf = map(Output, 0, WindowSize, 0, 100); // convert  time in percent
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{
long now = millis();
// Set the output
// "on time" is proportional to the PID output
if(now - windowStartTime>WindowSize){ //time to shift the Relay Window
  windowStartTime += WindowSize;
  }
if((onTime > 100) && (onTime > (now - windowStartTime))){
  digitalWrite(RELAY_PIN,HIGH);
  }
else {
  digitalWrite(RELAY_PIN,LOW);
  }
}


// ************************************************
// Start the Auto-Tuning cycle
// ************************************************

void StartAutoTune()
{
// REmember the mode we were in
ATuneModeRemember = myPID.GetMode();

// set up the auto-tune parameters
aTune.SetNoiseBand(aTuneNoise);
aTune.SetOutputStep(aTuneStep);
aTune.SetLookbackSec((int)aTuneLookBack);
tuning = true;
}

// ************************************************
// Update status LED
// ************************************************
void UpdateLedStatus()
{
if (abs(Input - Setpoint) < 1) {// Led on if at temp +- 1 deg
  digitalWrite(STATUSLED_PIN, HIGH);
  }
else {
  digitalWrite(STATUSLED_PIN, LOW);
  }
}


// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{
tuning = false;

// Extract the auto-tune calculated parameters
Kp = aTune.GetKp();
Ki = aTune.GetKi();
Kd = aTune.GetKd();

// Re-tune the PID and revert to normal control mode
myPID.SetTunings(Kp,Ki,Kd);
myPID.SetMode(ATuneModeRemember);

// Persist any changed parameters to EEPROM
SaveParameters();
}

// ************************************************
// Check buttons and time-stamp the last press
// ************************************************
uint8_t readButtons()
{
uint8_t but = 0;
//Serial.println("Attente bouton : ");
delay(100);
if (digitalRead(BUTTON_DOWN_PIN) != 1) {
  but += BUTTON_DOWN;
  }
if (digitalRead(BUTTON_UP_PIN) != 1) {
  but += BUTTON_UP;
  }
if (digitalRead(BUTTON_SELECT_PIN) != 1) {
  but += BUTTON_SELECT;
  }

if (but != 0){
  lastInput = millis();
  }

return but;
}

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
Serial.println("EEPROM");
EEPROM.get(0, eepromVar1);
if ((eepromVar1.eeSetpoint != Setpoint) | (eepromVar1.eeKp != Kp) | (eepromVar1.eeKi != Ki) | (eepromVar1.eeKd != Kd) | (eepromVar1.eePwm != pwm_value)) {
  Serial.println("Changement de parametre");
  eepromVar1.eeSetpoint = Setpoint;
  eepromVar1.eeKp = Kp;
  eepromVar1.eeKi = Ki;
  eepromVar1.eeKd = Kd;
  eepromVar1.eePwm = pwm_value;
  // write the data to EEPROM - ignoring anything that might be there already (re-flash is guaranteed)
  boolean ok1 = EEPROM.commitReset();
  Serial.println((ok1) ? "Commit (Reset) OK" : "Commit failed");

  // set the EEPROM data ready for writing
  EEPROM.put(0, eepromVar1);

  // write the data to EEPROM
  boolean ok = EEPROM.commit();
  Serial.println((ok) ? "Commit OK" : "Commit failed");
  }
}


// ************************************************
// Connexion WiFi
// ************************************************
boolean setup_wifi()
{
delay(10);
Serial.println();
Serial.print("Connexion a ");
Serial.println(wifi_ssid);
WiFi.begin(wifi_ssid, wifi_password);
int i=0;

while ((WiFi.status() != WL_CONNECTED) && (i <= 5)) { //5 attempt
  delay(500);
  Serial.print(".");
  i++;
  }

if (WiFi.status() == WL_CONNECTED)  {
  Serial.println("");
  Serial.println("Connexion WiFi etablie ");
  Serial.print("=> Addresse IP : ");
  Serial.print(WiFi.localIP());

  //display IP
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Connecte IP: ");
  display.println(WiFi.localIP());
  display.display();
  delay(2000);
  return true;
  }
else {
  Serial.println("");
  Serial.println("Echec Connexion WiFi");
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Echec Connexion WiFi");
  display.display();
  delay(2000);
  return false;
  }
}

// ************************************************
// reconnect MQTT
// ************************************************

boolean reconnect()
{
int i=0;
boolean status;
status = client.connected();

if (offlineMode == true) {
  return false;
  } // on sort si on opere offline

if (WiFi.status() == WL_CONNECTED) {
  while ((!client.connected()) && (i <= 5)){
    Serial.print("Connexion au serveur MQTT...");
    if (client.connect("ESP8266Client")){
      Serial.println("OK");
      client.subscribe(cde_setpoint_topic);
      client.subscribe(cde_pwm_topic);
      client.subscribe(cde_Kp_topic);
      client.subscribe(cde_Ki_topic);
      client.subscribe(cde_Kd_topic);
      client.subscribe(cde_sT1_topic);
      client.subscribe(cde_sT2_topic);
      client.subscribe(cde_sT3_topic);
      client.subscribe(cde_sT4_topic);
      client.subscribe(cde_sd1_topic);
      client.subscribe(cde_sd2_topic);
      client.subscribe(cde_sd3_topic);
      client.subscribe(cde_sd4_topic);
      status = true;
      }
    else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println("Perte Com MQTT - reconnecte");
    display.display();
    Serial.print("KO, erreur : ");
    Serial.print(client.state());
    Serial.println(" On attend 5 secondes avant de recommencer");
    delay(5000);
    status = false;
    }
  i++;
    }
  }
else {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Perte Com WiFi - reconnecte");
  display.display();
  statusWifi = setup_wifi();
  status = false;
  }
return status;
}

// ************************************************
// Callback MQTT
// ************************************************
void callback(char* topic, byte* payload, unsigned int length)
{
int i;
char *s;
Serial.print("Message arrived [");
Serial.print(topic);
Serial.print("] ");

for (i = 0; i < length; i++) {
  message_buff[i] = payload[i];
  Serial.print((char)payload[i]);
  }
  Serial.println();
  message_buff[i] = '\0';
  const char *p_payload = message_buff;

s = strstr(topic, cde_setpoint_topic);
if (s != NULL){
  Setpoint = strtod(p_payload, NULL);
  Serial.print("Setpoint :");
  Serial.println(Setpoint);
  }

s = strstr(topic, cde_Kd_topic);
if (s != NULL){
  Kd = strtod(p_payload, NULL);
  Serial.print("Kd :");
  Serial.println(Kd);
  }

s = strstr(topic, cde_Ki_topic);
if (s != NULL){
  Ki = strtod(p_payload, NULL);
  Serial.print("Ki :");
  Serial.println(Ki);
  }

s = strstr(topic, cde_Kp_topic);
if (s != NULL){
  Kp = strtod(p_payload, NULL);
  Serial.print("Kp :");
  Serial.println(Kp);
  }

s = strstr(topic, cde_pwm_topic);
if (s != NULL){
  pwm_value = (int)atof(p_payload);
  Serial.print("PWM :");
  Serial.println(pwm_value);
  }

s = strstr(topic, cde_sT1_topic);
    if (s != NULL){
    stepset[0] = strtod(p_payload, NULL);
    Serial.print("StepT1 :");
    Serial.println(stepset[0]);
    }
s = strstr(topic, cde_sd1_topic);
    if (s != NULL){
    stepd[0] = strtod(p_payload, NULL);
    Serial.print("Stepd1 :");
    Serial.println(stepd[0]);
    }

s = strstr(topic, cde_sT2_topic);
      if (s != NULL){
      stepset[1] = strtod(p_payload, NULL);
      Serial.print("StepT2 :");
      Serial.println(stepset[1]);
      }
s = strstr(topic, cde_sd2_topic);
    if (s != NULL){
      stepd[1] = strtod(p_payload, NULL);
      Serial.print("Stepd2 :");
      Serial.println(stepd[1]);
      }

s = strstr(topic, cde_sT3_topic);
    if (s != NULL){
      stepset[2] = strtod(p_payload, NULL);
      Serial.print("StepT3 :");
      Serial.println(stepset[2]);
      }
s = strstr(topic, cde_sd3_topic);
    if (s != NULL){
      stepd[2] = strtod(p_payload, NULL);
      Serial.print("Stepd3 :");
      Serial.println(stepd[2]);
      }

s = strstr(topic, cde_sT4_topic);
    if (s != NULL){
      stepset[3] = strtod(p_payload, NULL);
      Serial.print("StepT4 :");
      Serial.println(stepset[3]);
      }
s = strstr(topic, cde_sd4_topic);
    if (s != NULL){
      stepd[3] = strtod(p_payload, NULL);
      Serial.print("Stepd4 :");
      Serial.println(stepd[3]);
      }
}
