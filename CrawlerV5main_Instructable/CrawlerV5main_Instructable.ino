//Required libraries
#include <Wire.h> //library for I2C devices
#include <Adafruit_MCP23X17.h> //library for MCP23017 GPIO expander
#include <Adafruit_ADS1X15.h> //library for ADS1115 ADC
#include <Adafruit_NeoPixel.h> //library for WS2812b RGB LED strips
#include <Adafruit_PWMServoDriver.h> //library for PCA9685 servo driver
#include <Preferences.h> //library for storing variables in flash memory
#include <WiFi.h> //library for wireless communication
#include <esp_now.h> //library for bidirectional wireless communication between two ESP32
#include <esp_wifi.h> //needed to adjust the MAC address

//Pin definitions (ESP32)
#define dir 27 //Main engine direction
#define pwm 26 //Main engine speed
#define hdl 25 //Headlights
#define rgb 23 //RGB LED strip
#define shiftpwm 19 //Shifter speed
#define clutchapwm 18 //Clutch A speed
#define clutchbpwm 17 //Clutch B speed
#define diffpwm 16 //Difflock speed

//Pin definitions (MCP23017)
#define diffdir 4 //Difflock direction (pin A4)
#define clutchbdir 5 //Clutch B direction (pin A5)
#define clutchadir 6 //Clutch A direction (pin A6)
#define shiftdir 8 //Shifter direction (pin B0)
#define brakein1 12 //Brake input 1 (pin B4)
#define brakein2 13 //Brake input 2 (pin B5)
#define brakeout 14 //Brake output (pin B6)

//Variables for PWM channels
const int freq       = 20000; //PWM frequency: 20 kHz
const int res        = 8; //PWM with 8 bit
const int engine     = 0; //Channel 0: Main engine
const int shifter    = 1; //Channel 1: Gearshifter
const int clutcha    = 2; //Channel 2: Clutch A
const int clutchb    = 3; //Channel 3: Clutch B
const int difflock   = 4; //Channel 4: Differential lock
const int headlights = 5; //Channel 5: Headlight LEDs

//Add peripherals
Adafruit_MCP23X17 mcp; //define GPIO expander
Adafruit_ADS1115 adc1; //define first ADC
Adafruit_ADS1115 adc2; //define second ADC
int RGBcount = 30; //number of RGB LEDs of the strip
Adafruit_NeoPixel strip(RGBcount, rgb, NEO_GRB + NEO_KHZ800); //define RGB led strip
Adafruit_PWMServoDriver servo = Adafruit_PWMServoDriver(); //define servo driver
//the following lines are copy-pasted from the Adafruit PWMServoDriver library:
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600

//Variables for sensors
#define clutcha_sense  0 //channel 0 of ADS1115 #1 
#define clutchb_sense  1 //channel 1 of ADS1115 #1
#define shifter_sense  2 //channel 2 of ADS1115 #1
#define difflock_sense 3 //channel 3 of ADS1115 #1
#define i_sense        4 //channel 0 of ADS1115 #2
#define u_sense        5 //channel 1 of ADS1115 #2

//Variables that store the values of the sensors
int clutcha_val  = 0; 
int clutchb_val  = 0;
int shifter_val  = 0;
int difflock_val = 0;
int i_val        = 0;
int u_val        = 0;

//Global variables needed for the code
int LEDSpeed        = 10; //this value adjusts the speed of the LED running light by adjusting the fading (fading increments): higher value -> more increments, slower speed
int counter         = 1;
int IntCount        = 0;
int currentIntG     = 0;
int currentIntR     = 0;
int currentIntB     = 0;
int IntMaxG         = 0;
int IntMaxR         = 0;
int IntMaxB         = 0;
int wave            = 0;
int mainspeed       = 0;
int speed           = 0;
int thedir          = 0;
int steering        = 0;
int blink           = 0;
int rctimer         = 0;
int analogcounter   = 0;
int motortimer      = 0;
int diff_open       = 0;
int diff_locked     = 0;
int dirofdiff       = 0;
int gear_fast       = 0;
int gear_slow       = 0;
int dirofshifter    = 0;
int dirofclutcha    = 0;
int dirofclutchb    = 0;
int clutcha_open    = 0;
int clutcha_closed  = 0;
int clutchb_open    = 0;
int clutchb_closed  = 0;
int engineinversion = 0;

//ESPNOW setup is based on the project by Random Nerd Tutorials (with some modifications as needed for this project).
/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.  
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
//MAC addresses: The communication is established between two ESP32 based on their MAC addresses
uint8_t CarMAC[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; //we will set this MAC address as the address of the car
uint8_t broadcastAddress[] = {0xD0, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; //this is the MAC address of the remote control
int signalquality = 10;
// Define a data structure for sending data to the remote control
typedef struct send_message {
  int voltage;
  int current;
  int shifterpos;
  int clutchpos;
  int diffpos;
  int brake;
} send_message;
// Create a structured object
send_message outgoing;
// Define a data structure for receiving information from the remote control
typedef struct receive_message {
  int mainspeed;
  int steering;
  int headlights;
  int gear;
  int clutch;
  int brake;
  int diff;
  int R;
  int G;
  int B;  
} receive_message;
// Create a structured object
receive_message incoming;
// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS&&signalquality<19){ //signal quality is measured by changing the value of "signalquality" according to how many data packages were not received
    signalquality = signalquality +1;
  }else if(signalquality>0){
    signalquality = signalquality -1;
  }  
}
// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
}
esp_now_peer_info_t peerInfo; //make sure that the ESP-Now lines are in the exact same order as in this sketch. Otherwise, there may be malfunctions.

//The preferences library allows you to store variables within the flash so that their value can be retrieved on startup. Used to store user-adjustable settings.
Preferences preferences;

void setup() {
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, CarMAC); //overwrite the MAC address with a custom one
  //Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  //initialize the I2C peripherals 
  mcp.begin_I2C(0x20); //default I2C address of MPC23017 (all ADDR pins at GND)
  adc1.begin(0x48); //default I2C address of ADS1115 (ADDR pin unconnected/at GND)
  adc2.begin(0x49); //adjusted I2C address of ADS1115 (ADDR pin to VDD)
  servo.begin(); //default I2C address of PCA9685 is 0x40
  servo.setOscillatorFrequency(27000000);
  servo.setPWMFreq(50);  // Analog servos run at ~50 Hz updates

  //specify input/output pins
  mcp.pinMode(diffdir, OUTPUT);
  mcp.pinMode(clutchbdir, OUTPUT);
  mcp.pinMode(clutchadir, OUTPUT);
  mcp.pinMode(shiftdir, OUTPUT);
  mcp.pinMode(brakein1, INPUT_PULLUP);
  mcp.pinMode(brakein2, INPUT_PULLUP);
  mcp.pinMode(brakeout, OUTPUT);
  pinMode(dir, OUTPUT);
  mcp.digitalWrite(diffdir, LOW);
  mcp.digitalWrite(clutchbdir, LOW);
  mcp.digitalWrite(clutchadir, LOW);
  mcp.digitalWrite(shiftdir, LOW);
  mcp.digitalWrite(brakeout, LOW);
  digitalWrite(dir, LOW);

  //PWM channel setups
  ledcSetup(engine, freq, res); //PWM channel, frequency, resolution -> main engine
  ledcAttachPin(pwm, engine); //Pin, PWM channel
  ledcWrite(engine, 0); //PWM channel, default value

  ledcSetup(shifter, freq, res); //gear shifter actuator
  ledcAttachPin(shiftpwm, shifter); 
  ledcWrite(shifter, 0); 
  
  ledcSetup(clutcha, freq, res); //clutch a actuator
  ledcAttachPin(clutchapwm, clutcha); 
  ledcWrite(clutcha, 0); 

  ledcSetup(clutchb, freq, res); //clutch b actuator
  ledcAttachPin(clutchbpwm, clutchb); 
  ledcWrite(clutchb, 0); 

  ledcSetup(difflock, freq, res); //diff lock actuator
  ledcAttachPin(diffpwm, difflock); 
  ledcWrite(difflock, 0); 

  ledcSetup(headlights, freq, res); //headlight LEDs
  ledcAttachPin(hdl, headlights); 
  ledcWrite(headlights, 0);    

  Serial.begin(115200); //Not necessary, but kept in the code in case you want to use the serial monitor for debugging

  //RGB LED strip start
  strip.begin();
  strip.clear();
  strip.show(); //initialize all pixels to 'off'
  for (int i=0; i<RGBcount/2; i++){ //testing the strip: Run through all pixels with R, then G, then B
    strip.clear();
    strip.setPixelColor(i,127,0,0);
    strip.setPixelColor(i+15,127,0,0);
    strip.show();
    delay(50);
  }
  for (int i=0; i<RGBcount/2; i++){
    strip.clear();
    strip.setPixelColor(i,0,127,0);
    strip.setPixelColor(i+15,0,127,0);
    strip.show();
    delay(50);
  }
  for (int i=0; i<RGBcount/2; i++){
    strip.clear();
    strip.setPixelColor(i,0,0,127);
    strip.setPixelColor(i+15,0,0,127);
    strip.show();
    delay(50);
  }
  strip.clear();
  strip.show();
  delay(100);

  //retrieve values from the flash memory
  preferences.begin("Defaults", false);
  incoming.headlights = preferences.getInt("headlights", 0);
  incoming.gear = preferences.getInt("gear", 0);
  incoming.clutch = preferences.getInt("clutch", 0);
  incoming.brake = preferences.getInt("brake", 0);
  incoming.diff = preferences.getInt("diff", 0);
  diff_open = preferences.getInt("diffOpen",127);
  diff_locked = preferences.getInt("diffLocked",127);
  dirofdiff = preferences.getInt("diffDir",0);
  gear_fast = preferences.getInt("gearFast",127);
  gear_slow = preferences.getInt("gearSlow",127);
  dirofshifter = preferences.getInt("shifterDir",0);
  dirofclutcha = preferences.getInt("clutchaDir",0),
  dirofclutchb = preferences.getInt("clutchbDir",0);
  clutcha_open = preferences.getInt("clutchaOpen",127);
  clutcha_closed = preferences.getInt("clutchaClosed",127);
  clutchb_open = preferences.getInt("clutchbOpen",127);
  clutchb_closed = preferences.getInt("clutchbClosed",127);
  engineinversion = preferences.getInt("EngInv",0);
}

void loop() {
  switch (analogcounter){ //only one analog channel is read per loop iteration to reduce the time required per loop run (reading the ADS1115 is comparably slow)
    case 0:
      clutcha_val  = readAnalog(clutcha_sense);
      clutcha_val  = map(clutcha_val,0,17670,0,255); //actuator sensor readings are reduced to 8 bit because that resolution is sufficient
      analogcounter = 1;
      break;
    case 1:
      clutchb_val  = readAnalog(clutchb_sense);
      clutchb_val  = map(clutchb_val,0,17670,0,255); 
      analogcounter = 2;
      break;
    case 2:  
      shifter_val  = readAnalog(shifter_sense);
      shifter_val  = map(shifter_val,0,17670,0,255);
      analogcounter = 3;
      break;
    case 3:  
      difflock_val = readAnalog(difflock_sense);
      difflock_val = map(difflock_val,0,17670,0,255);
      analogcounter = 4;
      break;
    case 4:
      i_val        = readAnalog(i_sense);
      analogcounter = 5;
      break;
    case 5:
      u_val        = readAnalog(u_sense);
      analogcounter = 0;
      break;
  }

  diffMotor(); //the actuators are controlled using dedicated functions which are called in the main loop
  shifterMotor();
  clutchMotor();

  //read and write the parallel interface to the auxiliary ESP32; forward the input from the interface to the remote control
  if (mcp.digitalRead(brakein1)==1&&mcp.digitalRead(brakein2)==1){  //11->brake is closed
    outgoing.brake = 1;
  }else if (mcp.digitalRead(brakein1)==0&&mcp.digitalRead(brakein2)==0){  //00->brake is open
    outgoing.brake = 0;
  }else if (mcp.digitalRead(brakein1)==1&&mcp.digitalRead(brakein2)==0){  //10->brake is moving
    outgoing.brake = 2;
  }else{  //01->brake is not calibrated
    outgoing.brake = 3;
  }
  if (incoming.brake>0){
    mcp.digitalWrite(brakeout, HIGH); //1->close brake
  }else{
    mcp.digitalWrite(brakeout, LOW); //0->open brake
  }
  outgoing.voltage = u_val; //voltage is transmitted as the analog readout of the voltage divider (conversion into V is done in the RC)
  outgoing.current = i_val; //current is transmitted as the analog readout of the ACS712 sensor (conversion into A is done in the RC)
  
  //change the direction of the main engine according to the input from the RC and the current speed (direction is only changed when the current speed is close to 0)
  if (incoming.mainspeed<0&&speed<10){
    digitalWrite(dir, HIGH); //switch the values of this line and line 338 if the car drives reverse when trying to drive forward
    thedir = 0;
  }else if(incoming.mainspeed>0&&speed<10){
    digitalWrite(dir, LOW);
    thedir = 1;
  }
 
  if (incoming.steering>149&&incoming.steering<601){//the input for the steering from the RC should always stay in this interval; thus, the servo is only addressed if the value is in this interval
    servo.setPWM(0,0,incoming.steering);
  }

  mainspeed = incoming.mainspeed; //speed input from the RC is stored in this variable to enable interpolated changes

  if (mainspeed<-64&&engineinversion==0){ //to adjust the control of front/rear LED lights, the system needs to detect if the transmitted speed value has been inverted
    engineinversion=1; //that is done by measuring in which direction a speed value of >25 % duty cycle occurs. That has to be the forward direction.
    preferences.putInt("EngInv",engineinversion);
  }else if (mainspeed>64&&engineinversion==1){
    engineinversion=0;
    preferences.putInt("EngInv",engineinversion);
  }

  if (signalquality<6){ //set the speed to 0 if the connection is lost (less than 30% success rate for sending packages to the RC is considered as "connection is lost")
    mainspeed = 0;
    incoming.mainspeed = 0;
  }

  motortimer = motortimer+1; //this variable is used to adjust the PWM interpolation for the main engine 
  if (motortimer>1){ //increasing this value makes the accelerations slower (=smoother) but the car less responsive; reducing this value makes the care more responsive but results in tire skipping when accelerating fast
    updateMotor(); //the speed interpolation for the motor is computed in a separate function
    motortimer=0;
    preferences.putInt("brake",incoming.brake); //the inputs from the RC are stored in the flash memory to be available if the car suffers from a power outage
    preferences.putInt("diff",incoming.diff);
    preferences.putInt("clutch",incoming.clutch);
    preferences.putInt("gear",incoming.gear);
    preferences.putInt("headlights",incoming.headlights);
  }

    if (signalquality>5){ //the RGB LED program is updated if the signal quality is good
      updateRGB();
    }else{ //blink the RGB LEDs in red if the signal is lost
      //mcp.digitalWrite(brakeout, HIGH); //uncomment this line if the brake shall auto-close upon signal loss
      strip.clear();
      if (millis()>blink+1000){
        int onoff = 0;
        for (int i=0; i<RGBcount/2; i++){
          if (onoff==0){
            strip.setPixelColor(i,127,0,0);
            strip.setPixelColor(i+15,127,0,0);
            onoff = 1;  
          }else{       
            onoff = 0;
          }     
        }
        strip.show();
        blink = millis();
      }else if(millis()>blink+500){
        strip.clear();
        strip.show();
      }
    }

    if (incoming.headlights>0){ //the headlight LEDs are PWM-controlled based on the input from the RC
      ledcWrite(headlights,incoming.headlights);
    }else{
      ledcWrite(headlights,0);
    }

  if (millis()>rctimer+25){ //send updates from the car to the RC every 25 ms
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
    rctimer = millis();
  }
}

void clutchMotor(){ //function to control the clutch. Includes the code required to calibrate the clutch

/*
Output code summary for the clutch:
outgoing.clutchpos=0 -> clutch is closed
outgoing.clutchpos=1 -> clutch is open
outgoing.clutchpos=2 -> clutch is moving (regular operation and during calibration)
outgoing.clutchpos=3 -> clutch is not calibrated
outgoing.clutchpos=4 -> storing current position as "clutch open" confirmed (during calibration)
outgoing.clutchpos=5 -> storing current position as "clutch closed" confirmed (during calibration)
outgoing.clutchpos>5 -> analog sensor value of clutch a actuator (6 - 261))
*/

  if (incoming.clutch==3){ //input 3 -> move clutch a in dir 1 for 150 ms, then report the execution to the RC
    mcp.digitalWrite(clutchadir, HIGH);
    ledcWrite(clutcha,30); //motor speed for calibration is low (30/255)
    delay(150);
    ledcWrite(clutcha,0);
    dirofclutcha = dirofclutcha+1; //count the number of calibration steps per direction
    outgoing.clutchpos=2; //this feedback tells the RC that the movement is completed
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
  }
  if (incoming.clutch==4){ //input 4 -> move clutch a in dir 0 for 150 ms, then report the execution to the RC
    mcp.digitalWrite(clutchadir, LOW);
    ledcWrite(clutcha,30);
    delay(150);
    ledcWrite(clutcha,0);
    dirofclutcha = dirofclutcha-1;
    outgoing.clutchpos=2;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));    
  }

  if (incoming.clutch==5){ //input 5 -> move clutch b in dir 1 for 150 ms, then report the execution to the RC
    mcp.digitalWrite(clutchbdir, HIGH);
    ledcWrite(clutchb,30);
    delay(150);
    ledcWrite(clutchb,0);
    dirofclutchb = dirofclutchb+1;
    outgoing.clutchpos=2;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
  }
  if (incoming.clutch==6){ //input 6 -> move clutch b in dir 0 for 150 ms, then report the execution to the RC
    mcp.digitalWrite(clutchbdir, LOW);
    ledcWrite(clutchb,30);
    delay(150);
    ledcWrite(clutchb,0);
    dirofclutchb = dirofclutchb-1;
    outgoing.clutchpos=2;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));    
  }

  if (incoming.clutch==2){ //input 2 -> send the analog value of the actuator (current position) to the RC
    outgoing.clutchpos=clutcha_val+6; //the value is shifted by 6 to make sure the RC knows when an analog value instead of a status update of the clutch is transmitted
  }

  if (incoming.clutch==7){ //input 7 -> store current position as "clutch is open"
    outgoing.clutchpos=4; //tell the RC that the value was successfully stored
    clutcha_open = clutcha_val; //needs to be done for both actuators!
    clutchb_open = clutchb_val;
    preferences.putInt("clutchaOpen",clutcha_open);
    preferences.putInt("clutchbOpen",clutchb_open);
    dirofclutcha = 0; //the direction values are reset to track the direction when moving from one end point ("open") to the other end point ("closed")
    dirofclutchb = 0;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));     
  }
  if (incoming.clutch==8){ //input 8 -> store current position as "clutch is closed"
    outgoing.clutchpos=5; //tell the RC that the value was successfully stored
    clutcha_closed = clutcha_val;
    clutchb_closed = clutchb_val;
    preferences.putInt("clutchaClosed",clutcha_closed);  
    preferences.putInt("clutchaDir",dirofclutcha); //this variable stores the direction that was used for the actuator to close the clutch
    preferences.putInt("clutchbClosed",clutchb_closed);
    preferences.putInt("clutchbDir", dirofclutchb); 
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));     
  }

  if (dirofclutcha==0||dirofclutchb==0){ //if the direction value is (still) zero, the clutch is not calibrated -> the actuators will not try to (dis)engage the clutch
    if (incoming.clutch<2){
      outgoing.clutchpos=3;
    }
    //the control pattern of the clutch needs to account for the fact that the motor can be wired in two directions and the poti of the actuator in two ways (inverting high/low)
  }else if (dirofclutcha<0){ //check the calibration to decide which motor movement direction is needed
    if (clutcha_open>clutcha_closed){ //check the calibration to decide which poti value (higher or lower) is needed
      if (incoming.clutch==1&&clutcha_val<clutcha_open-5){ //the actuator moves to open the clutch if the sensor value is << than the stored value for "clutch open"
        mcp.digitalWrite(clutchadir, HIGH);
        ledcWrite(clutcha, 75); //the actuator is moving faster for regular operation compared with the calibration
        outgoing.clutchpos = 2; //code for "clutch is moving"
      }else if (incoming.clutch==1&&clutcha_val>clutcha_open-3){ //the actuator stops opening the clutch if the sensor value is only slightly lower than the stored value for "clutch open" (beacause of the lag in the system, it will still reach the desired end point)
        ledcWrite(clutcha,0),
        outgoing.clutchpos = 1; //code for "clutch is open"
      }
      if (incoming.clutch==0&&clutcha_val>clutcha_closed+5){ //the actuator moves to close the clutch if the sensor value is >> than the stored value for "clutch closed"
        mcp.digitalWrite(clutchadir, LOW);
        ledcWrite(clutcha, 75);
        outgoing.clutchpos = 2;
      }else if (incoming.clutch==0&&clutcha_val<clutcha_closed+3){
        ledcWrite(clutcha, 0);
        outgoing.clutchpos = 0;
      }
    }else{ //second case for the actuator poti being wired the other way
      if (incoming.clutch==1&&clutcha_val>clutcha_open+5){
        mcp.digitalWrite(clutchadir, HIGH);
        ledcWrite(clutcha, 75);
        outgoing.clutchpos = 2;
      }else if (incoming.clutch==1&&clutcha_val<clutcha_open+3){
        ledcWrite(clutcha,0),
        outgoing.clutchpos = 1;
      }
      if (incoming.clutch==0&&clutcha_val<clutcha_closed-5){
        mcp.digitalWrite(clutchadir, LOW);
        ledcWrite(clutcha, 75);
        outgoing.clutchpos = 2;
      }else if (incoming.clutch==0&&clutcha_val>clutcha_closed-3){
        ledcWrite(clutcha, 0);
        outgoing.clutchpos = 0;
      }      
    }
  }else{ //second case for the actuator motor being wired the other way
    if (clutcha_open>clutcha_closed){
      if (incoming.clutch==1&&clutcha_val<clutcha_open-5){
        mcp.digitalWrite(clutchadir, LOW);
        ledcWrite(clutcha, 75);
        outgoing.clutchpos = 2;
      }else if (incoming.clutch==1&&clutcha_val>clutcha_open-3){
        ledcWrite(clutcha,0),
        outgoing.clutchpos = 1;
      }
      if (incoming.clutch==0&&clutcha_val>clutcha_closed+5){
        mcp.digitalWrite(clutchadir, HIGH);
        ledcWrite(clutcha, 75);
        outgoing.clutchpos = 2;
      }else if (incoming.clutch==0&&clutcha_val<clutcha_closed+3){
        ledcWrite(clutcha, 0);
        outgoing.clutchpos = 0;
      }
    }else{
      if (incoming.clutch==1&&clutcha_val>clutcha_open+5){
        mcp.digitalWrite(clutchadir, LOW);
        ledcWrite(clutcha, 75);
        outgoing.clutchpos = 2;
      }else if (incoming.clutch==1&&clutcha_val<clutcha_open+3){
        ledcWrite(clutcha,0),
        outgoing.clutchpos = 1;
      }
      if (incoming.clutch==0&&clutcha_val<clutcha_closed-5){
        mcp.digitalWrite(clutchadir, HIGH);
        ledcWrite(clutcha, 75);
        outgoing.clutchpos = 2;
      }else if (incoming.clutch==0&&clutcha_val>clutcha_closed-3){
        ledcWrite(clutcha, 0);
        outgoing.clutchpos = 0;
      }      
    }  
  }

  if (dirofclutchb<0){ //equal code as for clutcha, just for the second clutch actuator. No outputs to the RC are generated here (clutch b is basically the secondary actuator, only following the primary one)
    if (clutchb_open>clutchb_closed){
      if (incoming.clutch==1&&clutchb_val<clutchb_open-5){
        mcp.digitalWrite(clutchbdir, HIGH);
        ledcWrite(clutchb, 75);
      }else if (incoming.clutch==1&&clutchb_val>clutchb_open-3){
        ledcWrite(clutchb,0);
      }
      if (incoming.clutch==0&&clutchb_val>clutchb_closed+5){
        mcp.digitalWrite(clutchbdir, LOW);
        ledcWrite(clutchb, 75);
      }else if (incoming.clutch==0&&clutchb_val<clutchb_closed+3){
        ledcWrite(clutchb, 0);
      }
    }else{
      if (incoming.clutch==1&&clutchb_val>clutchb_open+5){
        mcp.digitalWrite(clutchbdir, HIGH);
        ledcWrite(clutchb, 75);
      }else if (incoming.clutch==1&&clutchb_val<clutchb_open+3){
        ledcWrite(clutchb,0);
      }
      if (incoming.clutch==0&&clutchb_val<clutchb_closed-5){
        mcp.digitalWrite(clutchbdir, LOW);
        ledcWrite(clutchb, 75);
      }else if (incoming.clutch==0&&clutchb_val>clutchb_closed-3){
        ledcWrite(clutchb, 0);
      }      
    }
  }else if (dirofclutchb>0){
    if (clutchb_open>clutchb_closed){
      if (incoming.clutch==1&&clutchb_val<clutchb_open-5){
        mcp.digitalWrite(clutchbdir, LOW);
        ledcWrite(clutchb, 75);
      }else if (incoming.clutch==1&&clutchb_val>clutchb_open-3){
        ledcWrite(clutchb,0);
      }
      if (incoming.clutch==0&&clutchb_val>clutchb_closed+5){
        mcp.digitalWrite(clutchbdir, HIGH);
        ledcWrite(clutchb, 75);
      }else if (incoming.clutch==0&&clutchb_val<clutchb_closed+3){
        ledcWrite(clutchb, 0);
      }
    }else{
      if (incoming.clutch==1&&clutchb_val>clutchb_open+5){
        mcp.digitalWrite(clutchbdir, LOW);
        ledcWrite(clutchb, 75);
      }else if (incoming.clutch==1&&clutchb_val<clutchb_open+3){
        ledcWrite(clutchb,0);
      }
      if (incoming.clutch==0&&clutchb_val<clutchb_closed-5){
        mcp.digitalWrite(clutchbdir, HIGH);
        ledcWrite(clutchb, 75);
      }else if (incoming.clutch==0&&clutchb_val>clutchb_closed-3){
        ledcWrite(clutchb, 0);
      }      
    }
  }  
}

void shifterMotor(){ //the code for the shifter works equally as the code for the clutch. It only omits code for a 2nd actuator (the shifter uses only one actuator)

/*
Output code summary for the gear shifter:
outgoing.shifterpos=0 -> slow gear is engaged
outgoing.shifterpos=1 -> fast gear is engaged
outgoing.shifterpos=2 -> gear shifter is moving (regular operation and during calibration)
outgoing.shifterpos=3 -> gear shifter is not calibrated
outgoing.shifterpos=4 -> storing current position as "fast gear engaged" confirmed (during calibration)
outgoing.shifterpos=5 -> storing current position as "slow gear engaged" confirmed (during calibration)
outgoing.shifterpos>5 -> analog sensor value of gear shifter actuator (6 - 261))
*/

  if (incoming.gear==3){
    mcp.digitalWrite(shiftdir, HIGH);
    ledcWrite(shifter,30);
    delay(150);
    ledcWrite(shifter,0);
    dirofshifter = dirofshifter+1;
    outgoing.shifterpos=2;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
  }
  if (incoming.gear==4){
    mcp.digitalWrite(shiftdir, LOW);
    ledcWrite(shifter,30);
    delay(150);
    ledcWrite(shifter,0);
    dirofshifter = dirofshifter-1;
    outgoing.shifterpos=2;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));    
  }

  if (incoming.gear==2){
    outgoing.shifterpos=shifter_val+6;
  }
  if (incoming.gear==5){
    outgoing.shifterpos=4;
    gear_fast = shifter_val;
    preferences.putInt("gearFast",gear_fast);
    dirofshifter = 0;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));     
  }
  if (incoming.gear==6){
    outgoing.shifterpos=5;
    gear_slow = shifter_val;
    preferences.putInt("gearSlow",gear_slow);  
    preferences.putInt("shifterDir",dirofshifter);  
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));     
  }

  if (dirofshifter==0){
    if (incoming.gear<2){
      outgoing.shifterpos=3;
    }
  }else if (dirofshifter<0){
    if (gear_fast>gear_slow){
      if (incoming.gear==1&&shifter_val<gear_fast-5){
        mcp.digitalWrite(shiftdir, HIGH);
        ledcWrite(shifter, 75);
        outgoing.shifterpos = 2;
      }else if (incoming.gear==1&&shifter_val>gear_fast-3){
        ledcWrite(shifter,0),
        outgoing.shifterpos = 1;
      }
      if (incoming.gear==0&&shifter_val>gear_slow+5){
        mcp.digitalWrite(shiftdir, LOW);
        ledcWrite(shifter, 75);
        outgoing.shifterpos = 2;
      }else if (incoming.gear==0&&shifter_val<gear_slow+3){
        ledcWrite(shifter, 0);
        outgoing.shifterpos = 0;
      }
    }else{
      if (incoming.gear==1&&shifter_val>gear_fast+5){
        mcp.digitalWrite(shiftdir, HIGH);
        ledcWrite(shifter, 75);
        outgoing.shifterpos = 2;
      }else if (incoming.gear==1&&shifter_val<gear_fast+3){
        ledcWrite(shifter,0),
        outgoing.shifterpos = 1;
      }
      if (incoming.gear==0&&shifter_val<gear_slow-5){
        mcp.digitalWrite(shiftdir, LOW);
        ledcWrite(shifter, 75);
        outgoing.shifterpos = 2;
      }else if (incoming.gear==0&&shifter_val>gear_slow-3){
        ledcWrite(shifter, 0);
        outgoing.shifterpos = 0;
      }      
    }
  }else{
    if (gear_fast>gear_slow){
      if (incoming.gear==1&&shifter_val<gear_fast-5){
        mcp.digitalWrite(shiftdir, LOW);
        ledcWrite(shifter, 75);
        outgoing.shifterpos = 2;
      }else if (incoming.gear==1&&shifter_val>gear_fast-3){
        ledcWrite(shifter,0),
        outgoing.shifterpos = 1;
      }
      if (incoming.gear==0&&shifter_val>gear_slow+5){
        mcp.digitalWrite(shiftdir, HIGH);
        ledcWrite(shifter, 75);
        outgoing.shifterpos = 2;
      }else if (incoming.gear==0&&shifter_val<gear_slow+3){
        ledcWrite(shifter, 0);
        outgoing.shifterpos = 0;
      }
    }else{
      if (incoming.gear==1&&shifter_val>gear_fast+5){
        mcp.digitalWrite(shiftdir, LOW);
        ledcWrite(shifter, 75);
        outgoing.shifterpos = 2;
      }else if (incoming.gear==1&&shifter_val<gear_fast+3){
        ledcWrite(shifter,0),
        outgoing.shifterpos = 1;
      }
      if (incoming.gear==0&&shifter_val<gear_slow-5){
        mcp.digitalWrite(shiftdir, HIGH);
        ledcWrite(shifter, 75);
        outgoing.shifterpos = 2;
      }else if (incoming.gear==0&&shifter_val>gear_slow-3){
        ledcWrite(shifter, 0);
        outgoing.shifterpos = 0;
      }      
    }   
  }
}

void diffMotor(){ //the code for the diff lock works equally as the code for the clutch. It only omits code for a 2nd actuator (the shifter uses only one actuator)

/*
Output code summary for the diff lock:
outgoing.diffpos=0 -> diff is open
outgoing.diffpos=1 -> diff is locked
outgoing.diffpos=2 -> diff lock actuator is moving (regular operation and during calibration)
outgoing.diffpos=3 -> diff lock is not calibrated
outgoing.diffpos=4 -> storing current position as "diff is open" confirmed (during calibration)
outgoing.diffpos=5 -> storing current position as "diff is locked" confirmed (during calibration)
outgoing.diffpos>5 -> analog sensor value of diff lock actuator (6 - 261))
*/

  if (incoming.diff==3){
    mcp.digitalWrite(difflock, HIGH);
    ledcWrite(difflock,30);
    delay(150);
    ledcWrite(difflock,0);
    dirofdiff = dirofdiff+1;
    outgoing.diffpos=2;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
  }
  if (incoming.diff==4){
    mcp.digitalWrite(difflock, LOW);
    ledcWrite(difflock,30);
    delay(150);
    ledcWrite(difflock,0);
    dirofdiff = dirofdiff-1;
    outgoing.diffpos=2;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));    
  }

  if (incoming.diff==2){
    outgoing.diffpos=difflock_val+6;
  }
  if (incoming.diff==5){
    outgoing.diffpos=4;
    diff_open = difflock_val;
    preferences.putInt("diffOpen",diff_open);
    dirofdiff = 0;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));     
  }
  if (incoming.diff==6){
    outgoing.diffpos=5;
    diff_locked = difflock_val;
    preferences.putInt("diffLocked",diff_locked);  
    preferences.putInt("diffDir",dirofdiff);  
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));     
  }

  if (dirofdiff==0){
    if (incoming.diff<2){
      outgoing.diffpos=3;
    }
  }else if (dirofdiff>0){
    if (diff_locked>diff_open){
      if (incoming.diff==1&&difflock_val<diff_locked-5){
        mcp.digitalWrite(difflock, HIGH);
        ledcWrite(difflock, 75);
        outgoing.diffpos = 2;
      }else if (incoming.diff==1&&difflock_val>diff_locked-3){
        ledcWrite(difflock,0),
        outgoing.diffpos = 1;
      }
      if (incoming.diff==0&&difflock_val>diff_open+5){
        mcp.digitalWrite(difflock, LOW);
        ledcWrite(difflock, 75);
        outgoing.diffpos = 2;
      }else if (incoming.diff==0&&difflock_val<diff_open+3){
        ledcWrite(difflock, 0);
        outgoing.diffpos = 0;
      }
    }else{
      if (incoming.diff==1&&difflock_val>diff_locked+5){
        mcp.digitalWrite(difflock, HIGH);
        ledcWrite(difflock, 75);
        outgoing.diffpos = 2;
      }else if (incoming.diff==1&&difflock_val<diff_locked+3){
        ledcWrite(difflock,0),
        outgoing.diffpos = 1;
      }
      if (incoming.diff==0&&difflock_val<diff_open-5){
        mcp.digitalWrite(difflock, LOW);
        ledcWrite(difflock, 75);
        outgoing.diffpos = 2;
      }else if (incoming.diff==0&&difflock_val>diff_open-3){
        ledcWrite(difflock, 0);
        outgoing.diffpos = 0;
      }      
    }
  }else{
    if (diff_locked>diff_open){
      if (incoming.diff==1&&difflock_val<diff_locked-5){
        mcp.digitalWrite(difflock, LOW);
        ledcWrite(difflock, 75);
        outgoing.diffpos = 2;
      }else if (incoming.diff==1&&difflock_val>diff_locked-3){
        ledcWrite(difflock,0),
        outgoing.diffpos = 1;
      }
      if (incoming.diff==0&&difflock_val>diff_open+5){
        mcp.digitalWrite(difflock, HIGH);
        ledcWrite(difflock, 75);
        outgoing.diffpos = 2;
      }else if (incoming.diff==0&&difflock_val<diff_open+3){
        ledcWrite(difflock, 0);
        outgoing.diffpos = 0;
      }
    }else{
      if (incoming.diff==1&&difflock_val>diff_locked+5){
        mcp.digitalWrite(difflock, LOW);
        ledcWrite(difflock, 75);
        outgoing.diffpos = 2;
      }else if (incoming.diff==1&&difflock_val<diff_locked+3){
        ledcWrite(difflock,0),
        outgoing.diffpos = 1;
      }
      if (incoming.diff==0&&difflock_val<diff_open-5){
        mcp.digitalWrite(difflock, HIGH);
        ledcWrite(difflock, 75);
        outgoing.diffpos = 2;
      }else if (incoming.diff==0&&difflock_val>diff_open-3){
        ledcWrite(difflock, 0);
        outgoing.diffpos = 0;
      }      
    }    
  }
}

void updateMotor(){ //function that controls the main engine; runs an interpolation of the desired speed vs. current speed
  if (thedir==0&&mainspeed<=0||thedir==1&&mainspeed>=0){ //safety precaution: speed changes relative to input speed are only done when they have the same direction
    if (speed<abs(mainspeed)-20){ //simple interpolation logic: The size of the PWM duty cycle change increment depends on the gap between input speed and current speed
      speed = speed+6;  
    }else if (speed<abs(mainspeed)-10){
      speed = speed+4;
    }else if (speed<abs(mainspeed)){
      speed = speed+1;
    }else if(speed>abs(mainspeed)+20){
      speed = speed-6;
    }else if(speed>abs(mainspeed)+10){
      speed = speed-4;
    }else if(speed>abs(mainspeed)&&speed>0){
      speed = speed-1;
    }
  }else{
    speed = speed-6; //if speed input direction and system direction are not identical (=pushing the joystick in reverse while driving forward), the system reduces the speed quickly   
  }

  ledcWrite(engine,speed);
}

int readAnalog(int InputChannel){ //this code unifies the readout of the analog channels of the two ADS1115
  int val = 0;
  if (InputChannel<4){
    val = adc1.readADC_SingleEnded(InputChannel);
  }else{
    val = adc2.readADC_SingleEnded(InputChannel-4); //InputChannel4&InputChannel5 belong to inputs of the 2nd ADS1115
  }
  if (val>17670){ //the ADS1115 in default mode measures 0.0001875 V per count -> 17670 equals 3.31 V; there shouldn't be any value higher than that -> this threshold is used to cap the input
    val=17670;
  }else if(val<0){ //cap at 0 to prevent issues caused by negative reads (noise)
    val=0;
  }
  return val;
}

 void updateRGB(){ //this function updates the RGB LED strip
   if (IntCount>=LEDSpeed){ //you can change the variable LEDspeed to adjust the speed of the running light (has to be changed in the code, not as a parameter in the remote control)
    IntCount = 0;
    if (wave==0){ //the combination of wave & counter determines the position and direction of the running light
      counter = counter+1; //the counter jumps to the next LED position after completing the fading (fading steps are encoded in the IntCount variable, which depends on LEDspeed)
    }else if(wave==1){
      counter = counter-1;
    }
  }

  IntMaxG     = incoming.G; //RGB codes are transmitted directly from the remote control
  IntMaxR     = incoming.R;
  IntMaxB     = incoming.B;
  currentIntG = map(IntCount,0,LEDSpeed-1,0,IntMaxG); //intensity fading is calculated based on the intensity counter value and the LEDspeed, which determines the number of fading steps
  currentIntR = map(IntCount,0,LEDSpeed-1,0,IntMaxR);
  currentIntB = map(IntCount,0,LEDSpeed-1,0,IntMaxB);
  IntCount    = IntCount+1;
  strip.clear();
  strip.setPixelColor(counter,currentIntR,currentIntG,currentIntB); //the current light has the increasing intensity values relative to IntMax
  strip.setPixelColor(counter+15,currentIntR,currentIntG,currentIntB); //11 lights in the front and 11 lights in the rear => code is running for both sides
  
  if (wave==0){ //wave is responsible for the direction of the running light
    strip.setPixelColor(counter-1,IntMaxR,IntMaxG,IntMaxB); //the last light has the maximum intensity
    strip.setPixelColor(counter+14,IntMaxR,IntMaxG,IntMaxB); 
    strip.setPixelColor(counter-2,IntMaxR-currentIntR,IntMaxG-currentIntG,IntMaxB-currentIntB); //and the second last light has the decreasing intensity 
    strip.setPixelColor(counter+13,IntMaxR-currentIntR,IntMaxG-currentIntG,IntMaxB-currentIntB);    
  }else if (wave==1){
    strip.setPixelColor(counter+1,IntMaxR,IntMaxG,IntMaxB);  
    strip.setPixelColor(counter+16,IntMaxR,IntMaxG,IntMaxB);  
    strip.setPixelColor(counter+2,IntMaxR-currentIntR,IntMaxG-currentIntG,IntMaxB-currentIntB);  
    strip.setPixelColor(counter+17,IntMaxR-currentIntR,IntMaxG-currentIntG,IntMaxB-currentIntB);      
  }
  
  if (abs(incoming.mainspeed)>0||outgoing.brake>0){ //these if-conditions limit the running light boundaries to let the outer LEDs be used as head & rear lights when driving
    if (counter>=13&&IntCount>=LEDSpeed){
      wave     = 1;
      counter  = 11;
      IntCount = 0;
    }else if (counter<=1&&IntCount>=LEDSpeed){
      wave     = 0;
      counter  = 3;
      IntCount = 0;
    }
    if (incoming.mainspeed<0&&incoming.brake==0){ //the orientation of the LEDs depends on RGB strip wiring: needs to be adjusted manually accordingly
      if (engineinversion==0){ //account front/rear lights according to engine wiring
        strip.setPixelColor(14,127,0,0); //pixel on the front left: red light when driving backward
        strip.setPixelColor(15,abs(incoming.mainspeed),abs(incoming.mainspeed),abs(incoming.mainspeed)); //pixel on the rear left: white light when driving backward
        strip.setPixelColor(0,127,0,0); //pixel on the front right: red light when driving backward
        strip.setPixelColor(29,abs(incoming.mainspeed),abs(incoming.mainspeed),abs(incoming.mainspeed)); //pixel on the rear left: white light when driving backward        
      }else{
        strip.setPixelColor(15,127,0,0); //other direction
        strip.setPixelColor(14,incoming.mainspeed,incoming.mainspeed,incoming.mainspeed); //the headlight intensity scales with speed
        strip.setPixelColor(29,127,0,0);
        strip.setPixelColor(0,incoming.mainspeed,incoming.mainspeed,incoming.mainspeed);
      }
    }else if(incoming.mainspeed>0&&incoming.brake==0){
      if (engineinversion==0){
        strip.setPixelColor(15,127,0,0); //other direction
        strip.setPixelColor(14,incoming.mainspeed,incoming.mainspeed,incoming.mainspeed); //the headlight intensity scales with speed
        strip.setPixelColor(29,127,0,0);
        strip.setPixelColor(0,incoming.mainspeed,incoming.mainspeed,incoming.mainspeed);
      }else{
        strip.setPixelColor(14,127,0,0); //pixel on the front left: red light when driving backward
        strip.setPixelColor(15,abs(incoming.mainspeed),abs(incoming.mainspeed),abs(incoming.mainspeed)); //pixel on the rear left: white light when driving backward
        strip.setPixelColor(0,127,0,0); //pixel on the front right: red light when driving backward
        strip.setPixelColor(29,abs(incoming.mainspeed),abs(incoming.mainspeed),abs(incoming.mainspeed)); //pixel on the rear left: white light when driving backward  
      }
    }else if(outgoing.brake>0){
      strip.setPixelColor(0,127,0,0); //all outer corner LEDs emit red light when the parking brake is moving/engaged/not calibrated
      strip.setPixelColor(14,127,0,0);
      strip.setPixelColor(15,127,0,0);
      strip.setPixelColor(29,127,0,0);
    }

  }else{ //set counter & wave back when reaching the upper or lower limit
    if (counter>=14&&IntCount>=LEDSpeed){
      wave     = 1;
      counter  = 12;
      IntCount = 0;
    }else if (counter<=0&&IntCount>=LEDSpeed){
      wave     = 0;
      counter  = 2;
      IntCount = 0;
    }
  }
  strip.show();
}