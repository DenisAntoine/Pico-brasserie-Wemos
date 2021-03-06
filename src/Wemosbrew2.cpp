
//-------------------------------------------------------------------
//
// Brew Controller
//
//
// Based on the Arduino PID and PID AutoTune Libraries
// inspired by https://www.instructables.com/id/Arduino-Brew-Mashing-PID-Controller/
// Denis Antoine 2019
//------------------------------------------------------------------

#include "Arduino.h"

// wifi et MQTT
#include "ESP8266WiFi.h"
//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager

#include "PubSubClient.h"
//#include <ArduinoOTA.h>

// PID Library
#include "PID_v1.h"
#include "PID_AutoTune_v0.h"
// Libraries for the Adafruit OLED Shield
#include "Wire.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
// Libraries for the DS18B20 Temperature Sensor
#include "OneWire.h"
#include "DallasTemperature.h"
// So we can save and retrieve settings
#include "ESP_EEPROM.h"
// Ticker to manage interrupt
#include "Ticker.h"
// config file for Wifi and MQTT
#include "config.h"

#include <ArduinoJson.h>


// ************************************************
//------- Button values Settings -------
// ************************************************
#define BUTTON_DOWN 0x04
#define BUTTON_UP 0x02
#define BUTTON_SELECT 0x01
uint8_t readButtons();

// ************************************************
//------- WIFI MQTT -------------------------------
// ************************************************
WiFiClient espClient;
PubSubClient client(espClient);
boolean offlineMode = false; // defines if we operate online or offline
boolean statusWifi = false;
boolean statusMQTT = false;
//boolean setup_wifi();
void reconnect();
void callback(char* topic, byte* payload, unsigned int length);



unsigned long lastSent = millis();   // timestamp last MQTT message published
const unsigned long FREQ = 15000;
unsigned long lastReceived = 0;
char message_buff[256]; //Buffer for incomming MQTT messages


// ************************************************
// PID Variables and constants
// ************************************************

double Setpoint;
double Input;
double Output;
double pctchauf;
int minutesStep =0;
int cStep =0;

// Initialize default StepSetPoint
double stepset[] = {52, 63, 72, 78}; //temperature of steps
long stepd[] = {12, 30, 40, 10}; //duration in minutes of steps
volatile unsigned long onTime = 0;



// pid tuning parameters
double Kp=2037;
double Ki=5.4;
double Kd=0;

// ************************************************
// EEPROM addresses for persisted data
// ************************************************
struct MyEEPROMStruct {
  double  eeSetpoint;
  double  eeKp;
  double  eeKi;
  double  eeKd;
  byte  eePwm;
  } eepromVar1;
void SaveParameters();



//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_M, DIRECT); //P_ON_M Mode is Proportional on measurement to avoid overshoot

// 15 second Time Proportional Output window
unsigned long WindowSize = 15000; //to be adjusted
unsigned long windowStartTime;

// Ticker to manage time interrupt
Ticker timer;
void DoControl();
void DriveOutput();
void heat();

unsigned long publishOpstate(unsigned long timestamp, unsigned long freq);


// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember=2;
double aTuneStep=1000; // 1 sec step for 15000 window
double aTuneNoise=0.15; // according to temp precision
unsigned int aTuneLookBack=300; // very slow process
boolean tuning = false;
PID_ATune aTune(&Input, &Output);
void StartAutoTune();
void FinishAutoTune();




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

void Off();
void Run();
void AutoControl();
void Tune_Sp();
void TuneP();
void TuneI();
void TuneD();
void setPWM();


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


// Initialize Relay Control
Serial.println("Initialize Relay Control");
pinMode(RELAY_PIN, OUTPUT);    // Output mode to drive relay
digitalWrite(RELAY_PIN, LOW);  // make sure it is off to start
digitalWrite(STATUSLED_PIN, LOW); //Low during init

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

// Set up the initial (default) values for what is to be stored in EEPROM
eepromVar1.eeSetpoint = 60;
eepromVar1.eeKp = 850;
eepromVar1.eeKi = 0.5;
eepromVar1.eeKd = 0.1;
eepromVar1.eePwm = 150;




Serial.println("Read eeprom");
delay(5000);
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
//statusWifi = setup_wifi();
WiFiManager wifiManager;
wifiManager.setTimeout(180);
if(!wifiManager.autoConnect("AutoConnectAP")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    ESP.reset();
    delay(5000);
  } 

Serial.println("connected...yeey :)");
client.setServer(mqtt_server, 1883);
client.setCallback(callback);

// Interrupt Ticker each sec
Serial.println("attache timer");
timer.attach_ms(1000, heat);

// End setup
Serial.println("End Set up");
display.clearDisplay();
display.setCursor(0,0);
display.println("OFF");
display.display();
delay(100);
}



// ************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************
void loop()
{
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

//********************************************************************************************************************************

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
  digitalWrite(STATUSLED_PIN, LOW);
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
  digitalWrite(STATUSLED_PIN, HIGH);
  }
else {
  digitalWrite(RELAY_PIN,LOW);
  digitalWrite(STATUSLED_PIN, LOW); //High during init
  }
}




// ************************************************
// Check buttons and time-stamp the last press
// ************************************************
uint8_t readButtons()
{
uint8_t but = 0;
//Serial.println("Attente bouton : ");
delay(50);
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
// Initial State - OFF
// DOWN to reconnect
// UP to RUN
// SELECT to start Auto
// ************************************************
void Off()
{
Serial.println("Mode Off");
myPID.SetMode(MANUAL);
// update temperature control
Input = sensors.getTempC(tempSensor);
sensors.requestTemperatures();


digitalWrite(RELAY_PIN, LOW);  // make sure it is off
digitalWrite(STATUSLED_PIN, LOW); //

uint8_t buttons = 0;
lastInput = millis();
unsigned long recon = millis();

while(true){
  if (millis()>recon + 60000){
    timer.detach(); //On met en pause le temps de se recon
    reconnect();
    timer.attach_ms(1000, heat);
    recon = millis();
  }
  
  if (millis()>lastInput + 100){
    buttons = readButtons();
  }
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
    delay(50);
    return;
    }
  if (buttons & BUTTON_SELECT){ // SELECT to auto steps
    sensors.requestTemperatures(); // Start an asynchronous temperature reading
    myPID.SetMode(AUTOMATIC);
    windowStartTime = millis();
    opState = AUTO; // start control in AUTO mode
    delay(50);
    return;
    }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  
  display.println("OFF");
  display.display();
  delay(100);
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
uint8_t buttons = 0;
unsigned long recon = millis();
unsigned long buttonT = millis();


timer.detach();
SaveParameters();
myPID.SetTunings(Kp,Ki,Kd);
reconnect();
timer.attach_ms(1000, heat);


while(true){
  if (millis()>recon + 60000){
    timer.detach(); //On met en pause le temps de se recon
    reconnect();
    timer.attach_ms(1000, heat);
    recon = millis();
  }
  if (millis()>buttonT + 200){
    buttons = readButtons();
    buttonT = millis();
    if ((buttons & BUTTON_DOWN)
      && (abs(Input - Setpoint) < 0.5)){  // Should be at steady-state to start autotune
      StartAutoTune();
    }
    if (buttons & BUTTON_UP){
      opState = OFF;
      delay(50);
      return;
      }
    if (buttons & BUTTON_SELECT){
      opState = SETP;
      delay(50);
      return;
    }
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("RUN");
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
    digitalWrite(STATUSLED_PIN, LOW); //High during init
    opState = OFF;
    return;
    }
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

Setpoint = StepSetPoint; // Update set point
myPID.SetTunings(Kp,Ki,Kd);

unsigned long startTime = millis(); // Start timer
unsigned long endTime = startTime + StepTimeMin * 60000;
unsigned long recon = millis();

uint8_t buttons = 0;
cStep = Step;

while(millis() <= endTime) {
  if (millis()>recon + 60000){
    timer.detach(); //On met en pause le temps de se recon
    reconnect();
    timer.attach_ms(1000, heat);
    recon = millis();
  }
  
  buttons = readButtons();

  if (buttons & BUTTON_SELECT){ // Skip step and go to next
    delay(200);
    minutesStep = 0;
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
    digitalWrite(STATUSLED_PIN, LOW); //High during init
    opState = OFF;
    return;
    }

  minutesStep = (endTime - millis())/60000+1;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print("STEP: ");
  display.println(Step+1);
  display.print("min: ");
  display.println(minutesStep);
  display.print("Sp: ");
  display.println(Setpoint);
  display.print("In: ");
  display.println(Input);
  display.display();
  delay(100);
  }
}



// ************************************************
// Auto Control
// UP to OFF
// SELECT to start and go to next step
// ************************************************
void AutoControl()
{
Serial.println("Mode Autocontrole");
int step;
uint8_t buttons = 0;
while(true){
  buttons = readButtons();
  if (buttons & BUTTON_UP) { // UP to OFF
    opState = OFF;
    delay(50);
    return;
    }

  if (buttons & BUTTON_SELECT){ // Select to execute steps
    delay(200);
    for (step = 0; step < 4; step++) { //loop sur 4 steps
      RunStep(step, stepset[step], stepd[step]);
      }
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("Fin de cycle");
    display.display();
    delay(1000);
    opState = RUN; // maintain current state in RUN
    return;
    }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("AUTO");
  display.println("S | T | D");

  for (step = 0; step < 4; step++) { // display steps values
    display.print(step+1);
    display.print(" | ");
    display.print(int(stepset[step]));
    display.print("| ");
    display.println(stepd[step]);
    }
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
    delay(50);
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
    delay(50);
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
  delay(100);
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
Serial.print("Mode Reglage Kp =");
delay(100);
Serial.println(Kp);
uint8_t buttons = 0;
while(true){
  buttons = readButtons();
  float increment = 1.0;

  if (buttons & BUTTON_SELECT){
    SaveParameters();
    opState = TUNE_I;
    delay(50);
    return;
    }
  if (buttons & BUTTON_UP){
    Kp += increment;
    Serial.print("Reglage Kp =");
    Serial.println(Kp);
    delay(200);
    }
  if (buttons & BUTTON_DOWN){
    Kp -= increment;
    Serial.print("Reglage Kp =");
    Serial.println(Kp);
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
  delay(100);
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
    delay(50);
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
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Reglage Ki");
  display.print("Set Ki:");
  display.setTextSize(2);
  display.setCursor(0,20);
  display.print(Ki);
  display.display();
  delay(100);
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
    delay(50);
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
    delay(50);
    return;
    }
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Reglage Kd");
  display.print("Set Kd:");
  display.setTextSize(2);
  display.setCursor(0,20);
  display.print(Kd);
  display.display();
  delay(100);
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

uint8_t buttons = 0;
while(true){
  buttons = readButtons();
  float increment=4;

  if (buttons & BUTTON_SELECT){
    SaveParameters();
    opState = RUN;
    delay(50);
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
  delay(100);
  DoControl();
  }
}

// ************************************************
// reconnect MQTT
// ************************************************

void reconnect()
{
Serial.println("Reconnecte");
while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    String clientId = "picoclient-";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      
      client.subscribe(subscribe_topic);
      delay(500);
      client.loop();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(0,0);
      display.println("Perte Com MQTT - reconnecte");
      display.display();
      
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// ************************************************
// Callback MQTT
// ************************************************
void callback(char* topic, byte* payload, unsigned int length)
{
  StaticJsonDocument<256> doc;
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.println("] ");

  deserializeJson(doc, payload, length);

  if(doc.containsKey("ST")) {
    Setpoint = doc["ST"];
    Serial.print("Setpoint :");
    Serial.println(Setpoint);
  }

  if(doc.containsKey("Kp")) {// Yes!}
    Kp = doc["Kp"];
    Serial.print("Kp :");
    Serial.println(Kp);
  }

  if(doc.containsKey("Ki")) {// Yes!}
    Ki = doc["Ki"];
    Serial.print("Ki :");
    Serial.println(Ki);
  }

  if(doc.containsKey("Kd")) {// Yes!}
    Kd = doc["Kd"];
    Serial.print("Kd :");
    Serial.println(Kd);
  }

  if(doc.containsKey("ST1")) {// Yes!}
    stepset[0] = doc["ST1"];
    Serial.print("Step T1 :");
    Serial.println(stepset[0]);
  }
  if(doc.containsKey("ST2")) {// Yes!}
    stepset[1] = doc["ST2"];
    Serial.print("Step T2 :");
    Serial.println(stepset[1]);
  }
  if(doc.containsKey("ST3")) {// Yes!}
    stepset[2] = doc["ST3"];
    Serial.print("Step T3 :");
    Serial.println(stepset[2]);
  }
  if(doc.containsKey("ST4")) {// Yes!}
    stepset[3] = doc["ST4"];
    Serial.print("Step T4 :");
    Serial.println(stepset[3]);
  }

  if(doc.containsKey("Sd1")) {// Yes!}
    stepd[0] = doc["Sd1"];
    Serial.print("Step d1 :");
    Serial.println(stepd[0]);
  }
  if(doc.containsKey("Sd2")) {// Yes!}
    stepd[1] = doc["Sd2"];
    Serial.print("Step d2 :");
    Serial.println(stepd[1]);
  }
  if(doc.containsKey("Sd3")) {// Yes!}
    stepd[2] = doc["Sd3"];
    Serial.print("Step d3 :");
    Serial.println(stepd[2]);
  }
  if(doc.containsKey("Sd4")) {// Yes!}
    stepd[3] = doc["Sd4"];
    Serial.print("Step d4 :");
    Serial.println(stepd[3]);
  }
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
// Publish status
// ************************************************
unsigned long publishOpstate(unsigned long timestamp, unsigned long freq)
{
  unsigned long returntime = timestamp;
  char numstr[10];
  StaticJsonDocument<256> doc; //document µJson pour MQTT

  if (millis() > timestamp + freq)
  {
    switch (opState){
      case OFF:
      doc["opState"] = "OFF";
      break;
      case RUN:
      doc["opState"] = "RUN";
      break;
      case AUTO:
      snprintf(numstr, 10, "S%d %dm", cStep+1, minutesStep);
      Serial.println(numstr);
      doc["opState"] = numstr;
      
      break;
      case SETP:
      doc["opState"] = "SETP";
      break;
      case TUNE_P:
      doc["opState"] = "TuneP";
      break;
      case TUNE_I:
      doc["opState"] = "TuneI";
      break;
      case TUNE_D:
      doc["opState"] = "TuneD";
      break;
      case PWM:
      doc["opState"] = "PWM";
      break;
      }
      
     
    doc["temperature"] = Input;
    doc["setpoint"] = Setpoint;
    doc["pctchauf"] = pctchauf;
    doc["pwm"] = pwm_value;
    doc["Kp"] = Kp;
    doc["Ki"] = Ki;
    doc["Kd"] = Kd;
    //doc["cStep"] = cStep;
    //doc["cMin"] = minutesStep;
    
    
    size_t n = serializeJson(doc, message_buff);
    Serial.println(n);
    client.publish(publish_topic, message_buff, n);
    client.loop();
    Serial.println("Client Loop");
    returntime = millis();
  }
  return returntime;
}

// ************************************************
// Timer Interrupt Handler
// ************************************************
void heat()
{
lastSent = publishOpstate(lastSent, FREQ);

if (opState == OFF){
  digitalWrite(RELAY_PIN, LOW);  // make sure relay is off
  digitalWrite(STATUSLED_PIN, LOW); //High during init
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
