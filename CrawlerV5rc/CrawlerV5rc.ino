//Required libraries
#include <WiFi.h> //library wireless communication
#include <esp_now.h> //library for bidirectional communication between two ESP32
#include <Adafruit_ADS1X15.h> //library for ADS1115 ADC
#include <LiquidCrystal_I2C.h> //library for LCD display
#include <Preferences.h> //library for storing variables in flash memory
#include <esp_wifi.h> //needed to adjust the MAC address

//Pin definitions
#define btn1 25 //button1 (lower left)
#define btn2 26 //button2 (lower right)
#define btn3 27 //button3 (upper right)
#define btn4 33 //button4 (upper left)
#define led1 17 //led1 (lower left button)
#define led2 18 //led2 (lower right button)
#define led3 19 //led3 (upper right button)
#define led4 16 //led4 (upper left button)
#define lsw  23 //left switch
#define rsw  32 //right switch

//Global variables
double batvolt = 0;
double voltage = 0;
double current = 0;
int potival = 0;
int mainspeed = 0;
int mainspeed1 = 0;
int mainspeed2 = 0;
int steering = 0;
int btn1debounce0 = 0;
int btn1debounce1 = 0;
int btn1v = 1;
int btn2debounce0 = 0;
int btn2debounce1 = 0;
int btn2v = 1;
int btn3debounce0 = 0;
int btn3debounce1 = 0;
int btn3v = 1;
int btn4debounce0 = 0;
int btn4debounce1 = 0;
int btn4v = 1;
int lighton = 0;
int brakeon = 0;
int lcdtimer = 0;
int rctimer = 0;
int analogtimer = 0;
int difflockon     = 0;
int adj = 0;
int counter = 0;
int menu = 0;
int menuval = 0;
int calval = 0;
int joylcen = 0;
int joyll = 0;
int joylr = 0;
int joyltol = 0;
int joyrcen = 0;
int joyru = 0;
int joyrd = 0;
int joyrtol = 0;
int joyrinv = 0;
int rbr = 0;
int rbg = 0;
int rbb = 130;
int program = 0;
int lightint = 0;
int leftlimit = 0;
int rightlimit = 0;
int batlow = 0;
int lcdcounter = 0;
int currentflow = 0;

//definition of a non-standard character for the LCD display
byte chr1[8] =  { 
                  B00000,
                  B00100,
                  B01110,
                  B11111,
                  B11111,
                  B01110,
                  B00100,
                  B00000
              };

//Add I2Cperipherals
Adafruit_ADS1115 adc1;
int joyr = 0; //these variables indicate the input channel population of the ADC (right and left joystick, poti)
int joyl = 1;
int poti = 2;
int batt = 3;

LiquidCrystal_I2C lcd(0x27, 20, 4); //the display is a 4-line 20-character LCD display

//ESPNOW setup is based on the project by Random Nerd Tutorials (with some modifications as needed for this project).
/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/get-change-esp32-esp8266-mac-address-arduino/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.  
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
//MAC addresses: The communication is established between two ESP32 based on their MAC addresses
uint8_t RCMAC[] = {0xD0, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; //we will set this MAC address as the address of the remote control
uint8_t broadcastAddress[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; //this is the MAC address of the car
int signalquality = 0;
// Define a data structure for sending data to the remote control
typedef struct send_message {
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
} send_message;
// Create a structured object
send_message outgoing;
// Define a data structure for receiving information from the remote control
typedef struct receive_message {
  int voltage;
  int current;
  int shifterpos;
  int clutchpos;
  int diffpos;
  int brake;
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

//preferences library allows you to store variable values within the flash so that their value can be retrieved on startup. Used to store user-adjustable settings.
Preferences preferences;

//Interrupt functions for reading buttons
void IRAM_ATTR btn1int(){
  btn1debounce1 = millis();
  if (btn1debounce1>btn1debounce0+500){ //debouncing: the button only reacts to new inputs if the last input was at least 0.5 s ago
    btn1v = 0;    
  }
  btn1debounce0 = btn1debounce1;
}

void IRAM_ATTR btn2int(){
  btn2debounce1 = millis();
  if (btn2debounce1>btn2debounce0+500){
    btn2v = 0;    
  }
  btn2debounce0 = btn2debounce1;
}

void IRAM_ATTR btn3int(){
  btn3debounce1 = millis();
  if (btn3debounce1>btn3debounce0+500){
    btn3v = 0;    
  }
  btn3debounce0 = btn3debounce1;
}

void IRAM_ATTR btn4int(){
  btn4debounce1 = millis();
  if (btn4debounce1>btn4debounce0+500){
    btn4v = 0;    
  }
  btn4debounce0 = btn4debounce1;
}

void setup() {
  Serial.begin(115200); //Not necessary, but kept in the code in case you want to use the serial monitor for debugging
  //preferences library allows you to store variable values within the flash so that their value can be retrieved on startup. Used to store user-adjustable settings.
  preferences.begin("Defaults", false);
  joyltol = preferences.getInt("joyLtol",20);
  joyll = preferences.getInt("joyLleft",0);
  joylr = preferences.getInt("joyLright",0);
  joyrtol = preferences.getInt("joyRtol",20);
  joyru = preferences.getInt("joyRup",0);
  joyrd = preferences.getInt("joyRdown",0);
  joyrinv = preferences.getInt("joyRInv",0);
  program = preferences.getInt("Program",3);
  lightint = preferences.getInt("headlights",150);
  brakeon = preferences.getInt("brake",0);
  leftlimit = preferences.getInt("LeftLimit",400);
  rightlimit = preferences.getInt("RightLimit",350);

  //tell the car to move the actuators to the last stored setting upon startup
  if (brakeon>0){ 
    outgoing.brake=1;
  }

  difflockon = preferences.getInt("diff",0);
  if (preferences.getInt("DiffCal",0)>0){
    if (difflockon>0){
      outgoing.diff = 1;
    }else{
      outgoing.diff = 0;
    }
  }else{
    outgoing.diff = 2;
  }

  if (preferences.getInt("ClutchCal",0)>0){
    outgoing.clutch = preferences.getInt("clutch",0);
  }else{
    outgoing.clutch = 2;    
  }

  if (preferences.getInt("ShifterCal",0)>0){
    outgoing.gear = preferences.getInt("gear",0);
  }else{
    outgoing.gear = 2;
  }

  lighton = preferences.getInt("lighton",0);
  if (lighton>0){
    outgoing.headlights=lightint;
  }

  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, RCMAC); //overwrite the MAC address with a custom one
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

  //Specify input/output pins
  pinMode(lsw, INPUT_PULLUP);
  pinMode(rsw, INPUT_PULLUP);
  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2, INPUT_PULLUP);
  pinMode(btn3, INPUT_PULLUP);
  pinMode(btn4, INPUT_PULLUP);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);
  digitalWrite(led4, LOW);
  attachInterrupt(btn1,btn1int,FALLING); //interrupts need to be engaged; the trigger signal is falling (high->low)
  attachInterrupt(btn2,btn2int,FALLING);
  attachInterrupt(btn3,btn3int,FALLING);
  attachInterrupt(btn4,btn4int,FALLING);

  //Start the LCD display
  lcd.init();
  lcd.createChar(1, chr1); //the custom character for the LCD is provided to the library
  lcd.backlight();
  lcd.setCursor(5,0);
  lcd.print("Crawler V5");
  lcd.setCursor(0,2);
  lcd.print("Waiting for Cxn:");
  lcd.setCursor(0,3);
  int runvar  = 0; //these variables are needed to let the LCD display run the "waiting for connection" running symbol line
  int runvar1 = 0;
  int runvar2 = -5;
  while (signalquality<15&&(btn4v+btn3v>0)){ //at least 10 out of 20 data packages need to be received so that the connection is considered as established; pressing UL and UR button allow to skip this step
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing)); 
    delay(50);
    runvar=runvar+1;
    if(runvar>3){ //the next lines show the running symbol line as an indicator for "I'm not stuck forever, I'm doing something!"
      lcd.setCursor(runvar1,3);
      lcd.write(1);
      if(runvar2>=0){
        lcd.setCursor(runvar2,3);
        lcd.print(" ");
      }
      runvar1=runvar1+1;
      if(runvar1==20){
        runvar1=0;
      }
      runvar2=runvar2+1; 
      if(runvar2==20){
        runvar2=0;
      }  
      runvar=0;
    }
  }
  lcd.setCursor(0,3);
  lcd.print("                    ");
  lcd.setCursor(0,2);
  if (signalquality>5){ //connection to the car established
    lcd.print("Pairing:          "); 
    delay(50);
    lcd.setCursor(0,3);
    for (int i=0; i<=19; i++){ //measure current flow direction of ACS712 during startup (idle)
      if (signalquality>10){
        for (int j=0; j<=7; j++){ //keep sending data packages to ensure the car is still online
          esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing)); 
          delay(25);
        }                        
        current = double(incoming.current)*0.0001875*(22+33)/33; //current draw of the car: Analog value of the ACS712 sensor, corrected for its voltage divider with 22 and 33 kOhm
        current = (current-2.5)/.066; //conversion of analog input voltage into current: ACS712 outputs 2.5 V @ 0 A and the signal scales with 66 mV/A (30 A version) [switch to (2.5-current)/.066 if you see a negative current draw]
        if (current>0&&current<1){
          currentflow=currentflow+1;
        }else if (current<0&&current>-1){
          currentflow=currentflow-1;
        }        
        lcd.write(1);
      }else{ //indicate if the connection to the car was lost during this step - in that case, the current won't be displayed in the RC later on
        lcd.setCursor(0,3);
        lcd.print("Connection lost!    ");
        currentflow=0;
      }
    }
    if(currentflow>0){ //check current flow direction to adjust the analog value to current conversion in the main loop accordingly
      currentflow=1;
    }else if (currentflow<0){
      currentflow=-1;
    }
    if (signalquality>10){
      delay(500);
      lcd.setCursor(0,3);
      lcd.print("Connection ready!   "); //connection to the car established
    }
  }else{
    btn4v=1;
    btn3v=1; 
    lcd.print("Search aborted!     "); //search for connection was aborted by pressing UL and UR buttons
  }
  delay(1000);
  lcd.clear();
  adc1.begin();
  delay(250);
  lcdtimer = millis();
  rctimer  = lcdtimer;
}

void loop() {
  if (menu==0){ //the remote control is using multiple menu pages for its function. 0 -> normal operation
    analogInputs(); //read all analog inputs
    digitalInputs(); //read all digital inputs
    if (preferences.getInt("JoyCal",0)>0&&preferences.getInt("SteerCal",0)>0){ //only if the joysticks and the steering servo are calibrated, the RC transmits their values to the car
    outgoing.mainspeed = mainspeed;
    outgoing.steering  = steering;
    }else{
      outgoing.mainspeed = 0;
      outgoing.steering = 0;     
    }
  }else if (menu==1){ //menu 1 -> selection menu to enter the different submenus
    menuInputs();
  }else if (menu==2){ //menu 2 -> calibration of the joysticks
    joyCal();
  }else if (menu==3){ //menu 3 -> calibration of the steering servo
    steerCal();
    outgoing.mainspeed = 0;
    outgoing.steering = steering;  
  }else if (menu==4){ //menu 4 -> calibration of the gear shifter
    shifterCal();
  }else if (menu==5){ //menu 5 -> calibration of the clutch
    clutchCal();
  }else if (menu==6){ //menu 6 -> calibration of the diff lock
    diffCal();
  }else if (menu==7){ //menu 7 -> adjustment of headlight LED brightness
    setLight();   
  }else if (menu==8){ //menu 8 -> adjustment of RGB LED running light color
    RGBInput();    
  }

  if (menu>0&&menu!=3){ //make sure that speed and steering are zero when being in one of the submenus
    outgoing.mainspeed = 0;
    outgoing.steering = 0;
  }

  if (millis()>rctimer+25){ //the RC sends its inputs to the car every 25 ms (20x per second - sufficient for a responsive control)
    if (batlow<11){ //if the low battery voltage threshold of the car is reached, the running light is not executed any longer
      RGBUpdate();
    }else{
      outgoing.R = 0;
      outgoing.G = 0;
      outgoing.B = 0;
    }
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
    rctimer = millis();   
  }
  
  if (millis()>analogtimer+67){ //not all inputs need to be measured every time. This snippet is executed at a lower frequency
    counter = counter + 1;
    if (counter>2){
      potival = readAnalog(poti); //the potentiometer that defines the maximum speed of the car
      if (potival<20){
        potival=20;
      }else if(potival>17580){ //17580 counts *0.0001875 V/count = 3.296 V -> low enough to ensure max speed can be reached by the poti
        potival=17580;
      }
      potival = map(potival,20,17580,51,255); //speed selection: between 20% and 100% of full PWM range
      counter = 0;
    }else if (counter>1){
      batvolt = readAnalog(batt); //battery voltage of the remote control
      batvolt = double(batvolt)*0.0001875*(47+22)/22; //voltage calculation: 1 count equals 0.1875 mV with default settings of ADS1115; voltage divider with 47 and 22 kOhm
    }else{
      preferences.putInt("brake",brakeon);
      preferences.putInt("diff",difflockon);
      preferences.putInt("clutch",outgoing.clutch);
      preferences.putInt("gear",outgoing.gear);
      preferences.putInt("lighton",lighton);
    }
    voltage = double(incoming.voltage)*0.0001875*(100+33)/33; //voltage value of the car's battery: analog value is computed based on the 100 and 33 kOhm voltage divider
    current = double(incoming.current)*0.0001875*(22+33)/33; //current draw of the car: Analog value of the ACS712 sensor, corrected for its voltage divider with 22 and 33 kOhm
    if (currentflow==1){
      current = (current-2.5)/.066; //conversion of analog input voltage into current: ACS712 outputs 2.5 V @ 0 A and the signal scales with 66 mV/A (30 A version) [(current-2.5)/.066 for negative flow]
    }else if (currentflow==-1){
      current = (2.5-current)/.066; //conversion of analog input voltage into current: ACS712 outputs 2.5 V @ 0 A and the signal scales with 66 mV/A (30 A version) [(2.5-current)/.066 for positive flow]
    }
    if (signalquality>10){
      if (voltage<11.1&&current<2||voltage<10.8){ //thresholds for low battery: less than 11.1V@idle (below 2A; 3.7V/cell) or less than 10.8V (3.6V/cell) under load
        if (batlow<20){ //these limits include a safety margin that helps to extend the life of the battery while not making using of its full capacity
          batlow=batlow+1; //reducing these values is done at your own risk! Never discharge the battery below 9.9 V (3.3 V/cell, also not under load!) or charge it to more than 12.6 V (4.2 V/cell)!
        }
      }else if (voltage>=11.25&&batlow>0){ //account for lower readings during startup/shutdown: >=11.25V (3.75V/cell) resets the variable
        batlow=batlow-1;
      }
    }
    analogtimer = millis();
  }

  if (millis()>lcdtimer+250){ //the lcd screen is updated 4 times per second (this low refresh rate looks better than refreshing the display faster because of the response time of the pixels)
    if (menu==0){ //the menu variable determines the "program" for the LCD display.
      updateLCD(); //standard mode: Show status of the car
    }else if(menu==1){
      menuLCD(); //selection menu: Show the different menu entries and the cursor
    }else if(menu==2){
      joyCalLCD(); //LCD screen for joystick calibration
    }else if (menu==3){
      steerLCD(); //LCD screen for steering calibration
    }else if (menu==4){
      shifterLCD(); //LCD screen for gear shifter calibration
    }else if (menu==5){
      clutchLCD(); //LCD screen for clutch calibration
    }else if (menu==6){
      diffLCD(); //LCD screen for diff lock calibration
    }else if(menu==7){
      lightLCD(); //LCD screen for adjusting headlight LED brightness
    }else if(menu==8){
      RGBLCD(); //LCD screen for adjusting RGB LED running light color
    }
    lcdtimer = millis();
    //serialOutput(); //this function can display information in the serial monitor for debugging. Shall be disabled for normal operation as it slows down the code to do so.
  }
}

void serialOutput(){ //left here from debugging. Is only called if serialOutput(); in line 427 is activated.
  Serial.print("Voltage: ");
  Serial.println(incoming.voltage);
  Serial.print("Current: ");
  Serial.println(incoming.current); 
  Serial.print("Shifterpos: ");
  Serial.println(incoming.shifterpos);
  Serial.print("ShifterCal: ");
  Serial.println(preferences.getInt("ShifterCal")); 
  Serial.print("Clutchpos: ");
  Serial.println(incoming.clutchpos);
  Serial.print("ClutchCal: ");
  Serial.println(preferences.getInt("ClutchCal")); 
  Serial.print("Diffpos: ");
  Serial.println(incoming.diffpos);
  Serial.print("Brake: ");
  Serial.println(incoming.brake); 
  Serial.println("---------");
  Serial.print("Sending Shifter: ");
  Serial.println(outgoing.gear);
  Serial.print("Sending Clutch: ");
  Serial.println(outgoing.clutch);
}

int readAnalog(int InputChannel){
  int val = 0;
  val = adc1.readADC_SingleEnded(InputChannel);
  if (val>17670){ //the ADS1115 in default mode measures 0.0001875 V per count -> 17670 equals 3.31 V; there shouldn't be any value higher than that -> this threshold is used to cap the input
    val=17670;
  }else if(val<0){ //cap at 0 to prevent issues caused by negative reads (noise)
    val=0;
  }
  return val;
}

void clutchCal(){ //function for calibrating the clutch
/*
Output code summary for the clutch:
outgoing.clutch=0 -> close the clutch
outgoing.clutch=1 -> open the clutch
outgoing.clutch=2 -> enter clutch calibration
outgoing.clutch=3 -> move clutch a in dir 1
outgoing.clutch=4 -> move clutch a in dir 0
outgoing.clutch=5 -> move clutch b in dir 1
outgoing.clutch=6 -> move clutch b in dir 0
outgoing.clutch=7 -> store current position as "clutch is open"
outgoing.clutch=8 -> store current position as "clutch is closed"
*/
  outgoing.clutch=2;
  if (btn4v==0){
    if (calval==1||calval==2){
      digitalWrite(led4, HIGH);
      if (digitalRead(lsw)>0){ //the left switch decides if clutch a or clutch b is operated upon pressing the upper/lower left button
        outgoing.clutch=3;
      }else{
        outgoing.clutch=5;
      }

      while(incoming.clutchpos!=2){ //the trigger to move the actuator is repeatedly sent until the car replies with "movement completed"
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
      }      
    }
    btn4v=1;
  }
  if (btn1v==0){
    if (calval==1||calval==2){
      digitalWrite(led1, HIGH);
      if (digitalRead(lsw)>0){
        outgoing.clutch=4;
      }else{
        outgoing.clutch=6;
      }
      while(incoming.clutchpos!=2){
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
      }        
    }
    btn1v=1;
  }  
  if (btn3v==0){ //pressing button 3 (upper right button) stores the current position of the clutch
    digitalWrite(led3, HIGH);
    btn3v=1;
    calval=calval+1;
    preferences.putInt("ClutchCal",0); //the calibration of the clutch is erased once a new calibration is done
    if (calval==2){ //first position to be stored: clutch is open
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Input confirmed: ");
      lcd.setCursor(0,2);
      lcd.print("Clutch open");
      outgoing.clutch=7; //the trigger to store this position is repeatedly sent until the car replies with "value is saved"
      while(incoming.clutchpos!=4){
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
      }
      delay(1000);
    }else if (calval==3){ //second position to be stored: clutch is closed
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Input confirmed: ");
      lcd.setCursor(0,2);
      lcd.print("Clutch closed");
      outgoing.clutch=8;
      while(incoming.clutchpos!=5){
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
      }      
      delay(1000);
    }else if (calval==4){ //final step of the calibration: confirm the calibration
      menu=1;
      calval=0;
      preferences.putInt("ClutchCal",1);
      digitalWrite(led2, HIGH);
    }
    lcd.clear();
  }
  if (btn2v==0){ //one step back in the calibration upon pressing the lower right button
    digitalWrite(led2, HIGH);
    btn2v=1;
    calval=calval-1;
    if (calval<0){
      menu=1;
      calval=0;
    }
    lcd.clear();
  }  
}

void clutchLCD(){ //LCD screen for clutch calibration
  digitalWrite(led1, LOW); //reset LED rings of the buttons
  if(menu!=1){
    digitalWrite(led2, LOW);
  }else{
    digitalWrite(led2, HIGH);
  }
  digitalWrite(led3, LOW);
  digitalWrite(led4, LOW);  
  if (calval==0){ //the different steps of the calibration menu each result in a different screen of the LCD display
    if (preferences.getInt("ClutchCal",0)>0){
      lcd.setCursor(0,0);
      lcd.print("Clutch calibration");
      lcd.setCursor(0,1);
      lcd.print("Calibration found   ");
      lcd.setCursor(0,2);
      lcd.print("Override: UR button");  
      lcd.setCursor(0,3);
      lcd.print("Return: LR button");  
    }else{
      lcd.setCursor(0,0);
      lcd.print("Clutch calibration");
      lcd.setCursor(0,1);      
      lcd.print("No calibration found");
      lcd.setCursor(0,2);
      lcd.print("Start cal: UR button");  
      lcd.setCursor(0,3);
      lcd.print("Return: LR button");        
    }
  }else if (calval==1){
    lcd.setCursor(0,0);
    lcd.print("Select A/B with LSW");
    lcd.setCursor(0,1); 
    lcd.print("Adj open with UL/LL");
    lcd.setCursor(0,2);
    if (digitalRead(lsw)>0){
      lcd.print("A ");
    }else{
      lcd.print("B ");
    }
    lcd.print("Clutch opn: ");
    if (incoming.clutchpos>5){
      lcd.print(incoming.clutchpos-6); //the analog value has to be corrected since it is sent shifted (sent in the interval from 6 to 261 instead of 0 to 255)
    }
    lcd.print("   ");    
    lcd.setCursor(0,3);
    lcd.print("Conf.: UR; Ret.: LR");         
  }else if (calval==2){
    lcd.setCursor(0,0);
    lcd.print("Select A/B with LSW");
    lcd.setCursor(0,1); 
    lcd.print("Adj clsd with UL/LL");
    lcd.setCursor(0,2);
    if (digitalRead(lsw)>0){
      lcd.print("A; ");
    }else{
      lcd.print("B; ");
    }
    lcd.print("Clutch cls: ");
    if (incoming.clutchpos>5){
      lcd.print(incoming.clutchpos-6);
    }
    lcd.print("   ");    
    lcd.setCursor(0,3);
    lcd.print("Conf.: UR; Ret.: LR");            
  }else if (calval==3){
    lcd.setCursor(0,0); 
    lcd.print("Calibration done!");    
    lcd.setCursor(0,2);
    lcd.print("Confirm: UR button");  
    lcd.setCursor(0,3);
    lcd.print("Return: LR button");      
  }
}

void shifterCal(){ //follows the same logic like the clutch calibration but omits the calibration of a 2nd actuator
  /*
  Output code summary for the gear shifter:
  outgoing.gear=0 -> engage slow gear
  outgoing.gear=1 -> engage fast gear
  outgoing.gear=2 -> enter gear shifter calibration
  outgoing.gear=3 -> move gear shifter in dir 1
  outgoing.gear=4 -> move gear shifter in dir 0
  outgoing.gear=5 -> store current position as "fast gear engaged"
  outgoing.gear=6 -> store current position as "slow gear engaged"
  */
  outgoing.gear=2;
  if (btn4v==0){
    if (calval==1||calval==2){
      digitalWrite(led4, HIGH);
      outgoing.gear=3;
      while(incoming.shifterpos!=2){
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
      }      
    }
    btn4v=1;
  }
  if (btn1v==0){
    if (calval==1||calval==2){
      digitalWrite(led1, HIGH);
      outgoing.gear=4;
      while(incoming.shifterpos!=2){
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
      }        
    }
    btn1v=1;
  }  
  if (btn3v==0){
    digitalWrite(led3, HIGH);
    btn3v=1;
    calval=calval+1;
    preferences.putInt("ShifterCal",0);
    if (calval==2){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Input confirmed: ");
      lcd.setCursor(0,2);
      lcd.print("Fast gear");
      outgoing.gear=5;
      while(incoming.shifterpos!=4){
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
      }
      delay(1000);
    }else if (calval==3){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Input confirmed: ");
      lcd.setCursor(0,2);
      lcd.print("Slow gear");
      outgoing.gear=6;
      while(incoming.shifterpos!=5){
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
      }      
      delay(1000);
    }else if (calval==4){
      menu=1;
      calval=0;
      preferences.putInt("ShifterCal",1);
      digitalWrite(led2, HIGH);
    }
    lcd.clear();
  }
  if (btn2v==0){
    digitalWrite(led2, HIGH);
    btn2v=1;
    calval=calval-1;
    if (calval<0){
      menu=1;
      calval=0;
    }
    lcd.clear();
  }  
}

void shifterLCD(){ //follows the same logic like the clutch calibration but omits the calibration of a 2nd actuator
  digitalWrite(led1, LOW);
  if(menu!=1){
    digitalWrite(led2, LOW);
  }else{
    digitalWrite(led2, HIGH);
  }
  digitalWrite(led3, LOW);
  digitalWrite(led4, LOW);  
  if (calval==0){
    if (preferences.getInt("ShifterCal",0)>0){
      lcd.setCursor(0,0);
      lcd.print("Gearbox calibration");
      lcd.setCursor(0,1);
      lcd.print("Calibration found   ");
      lcd.setCursor(0,2);
      lcd.print("Override: UR button");  
      lcd.setCursor(0,3);
      lcd.print("Return: LR button");  
    }else{
      lcd.setCursor(0,0);
      lcd.print("Gearbox calibration");
      lcd.setCursor(0,1);      
      lcd.print("No calibration found");
      lcd.setCursor(0,2);
      lcd.print("Start cal: UR button");  
      lcd.setCursor(0,3);
      lcd.print("Return: LR button");        
    }
  }else if (calval==1){
    lcd.setCursor(0,0); 
    lcd.print("Adj fast with UL/LL");
    lcd.setCursor(0,1);
    lcd.print("Fast gear: ");
    if (incoming.shifterpos>5){
      lcd.print(incoming.shifterpos-6);
    }
    lcd.print("   ");    
    lcd.setCursor(0,2);
    lcd.print("Confirm: UR button");   
    lcd.setCursor(0,3);
    lcd.print("Return : LR button");       
  }else if (calval==2){
    lcd.setCursor(0,0); 
    lcd.print("Adj slow with UL/LL");
    lcd.setCursor(0,1);
    lcd.print("Slow gear: ");
    if (incoming.shifterpos>5){
      lcd.print(incoming.shifterpos-6);
    }
    lcd.print("   ");       
    lcd.setCursor(0,2);
    lcd.print("Confirm: UR button");   
    lcd.setCursor(0,3);
    lcd.print("Return : LR button");          
  }else if (calval==3){
    lcd.setCursor(0,0); 
    lcd.print("Calibration done!");    
    lcd.setCursor(0,2);
    lcd.print("Confirm: UR button");  
    lcd.setCursor(0,3);
    lcd.print("Return: LR button");      
  }
}

void diffCal(){ //follows the same logic like the clutch calibration but omits the calibration of a 2nd actuator
  /*
  Output code summary for the diff lock:
  outgoing.diff=0 -> open diff
  outgoing.diff=1 -> lock diff
  outgoing.diff=2 -> enter diff lock calibration
  outgoing.diff=3 -> move diff lock in dir 1
  outgoing.diff=4 -> move diff lock in dir 0
  outgoing.diff=5 -> store current position as "diff is open"
  outgoing.diff=6 -> store current position as "diff is locked"
  */
  outgoing.diff=2;
  if (btn4v==0){
    if (calval==1||calval==2){
      digitalWrite(led4, HIGH);
      outgoing.diff=3;
      while(incoming.diffpos!=2){
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
      }      
    }
    btn4v=1;
  }
  if (btn1v==0){
    if (calval==1||calval==2){
      digitalWrite(led1, HIGH);
      outgoing.diff=4;
      while(incoming.diffpos!=2){
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
      }        
    }
    btn1v=1;
  }  
  if (btn3v==0){
    digitalWrite(led3, HIGH);
    btn3v=1;
    calval=calval+1;
    preferences.putInt("DiffCal",0);
    if (calval==2){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Input confirmed: ");
      lcd.setCursor(0,2);
      lcd.print("Diff open");
      outgoing.diff=5;
      while(incoming.diffpos!=4){
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
      }
      delay(1000);
    }else if (calval==3){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Input confirmed: ");
      lcd.setCursor(0,2);
      lcd.print("Diff locked");
      outgoing.diff=6;
      while(incoming.diffpos!=5){
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoing, sizeof(outgoing));
      }      
      delay(1000);
    }else if (calval==4){
      menu=1;
      calval=0;
      preferences.putInt("DiffCal",1);
      digitalWrite(led2, HIGH);
    }
    lcd.clear();
  }
  if (btn2v==0){
    digitalWrite(led2, HIGH);
    btn2v=1;
    calval=calval-1;
    if (calval<0){
      menu=1;
      calval=0;
    }
    lcd.clear();
  }  
}

void diffLCD(){ //follows the same logic like the clutch calibration but omits the calibration of a 2nd actuator
  digitalWrite(led1, LOW);
  if(menu!=1){
    digitalWrite(led2, LOW);
  }else{
    digitalWrite(led2, HIGH);
  }
  digitalWrite(led3, LOW);
  digitalWrite(led4, LOW);  
  if (calval==0){
    if (preferences.getInt("DiffCal",0)>0){
      lcd.setCursor(0,0);
      lcd.print("Difflock calibration");
      lcd.setCursor(0,1);
      lcd.print("Calibration found   ");
      lcd.setCursor(0,2);
      lcd.print("Override: UR button");  
      lcd.setCursor(0,3);
      lcd.print("Return: LR button");  
    }else{
      lcd.setCursor(0,0);
      lcd.print("Difflock calibration");
      lcd.setCursor(0,1);      
      lcd.print("No calibration found");
      lcd.setCursor(0,2);
      lcd.print("Start cal: UR button");  
      lcd.setCursor(0,3);
      lcd.print("Return: LR button");        
    }
  }else if (calval==1){
    lcd.setCursor(0,0); 
    lcd.print("Adj open with UL/LL");
    lcd.setCursor(0,1);
    lcd.print("Open: ");
    if (incoming.diffpos>5){
      lcd.print(incoming.diffpos-6);
    }
    lcd.print("   ");    
    lcd.setCursor(0,2);
    lcd.print("Confirm: UR button");   
    lcd.setCursor(0,3);
    lcd.print("Return : LR button");       
  }else if (calval==2){
    lcd.setCursor(0,0); 
    lcd.print("Adj lock with UL/LL");
    lcd.setCursor(0,1);
    lcd.print("Locked: ");
    if (incoming.diffpos>5){
      lcd.print(incoming.diffpos-6);
    }
    lcd.print("   ");     
    lcd.setCursor(0,2);
    lcd.print("Confirm: UR button");   
    lcd.setCursor(0,3);
    lcd.print("Return : LR button");          
  }else if (calval==3){
    lcd.setCursor(0,0); 
    lcd.print("Calibration done!");    
    lcd.setCursor(0,2);
    lcd.print("Confirm: UR button");  
    lcd.setCursor(0,3);
    lcd.print("Return: LR button");      
  }
}

void steerCal(){ //calibration of the steering servo
  if (calval==1){ //the remote control transmits fixed values for the steering servo depending on the step of the calibration
    steering=leftlimit; //step one: left limit of the steering servo
  }else if (calval==2){
    steering=rightlimit; //step two: right limit of the steering servo
  }else{
    steering=(leftlimit+rightlimit)/2; //step three: center position of the steering servo
  }
  if (btn4v==0){ //button 4 (upper left button) increases the value of the limit that is currently adjusted
    if (calval==1){
      digitalWrite(led4, HIGH);
      if (leftlimit<596){
        leftlimit=leftlimit+5;
      }
    }else if (calval==2){
      digitalWrite(led4, HIGH);
      if (rightlimit<596){
        rightlimit=rightlimit+5;
      }
    }else if (calval==3){ //step 3 does fine tuning of the position by changing the values by smaller increments
      digitalWrite(led4, HIGH);
      if (leftlimit<600&&rightlimit<600){
        leftlimit=leftlimit+1;
        rightlimit=rightlimit+1;
      }
    }
    btn4v=1;
  }
  if (btn1v==0){ //button 1 (lower left button) decreases the value of the limit that is currently adjusted
    if (calval==1){
      digitalWrite(led1, HIGH);
      if (leftlimit>154){
        leftlimit=leftlimit-5;
      }
    }else if (calval==2){
      digitalWrite(led1, HIGH);
      if (rightlimit>154){
        rightlimit=rightlimit-5;
      }
    }else if (calval==3){
      digitalWrite(led1, HIGH);
      if (leftlimit>150&&rightlimit>150){
        leftlimit=leftlimit-1;
        rightlimit=rightlimit-1;
      }      
    }
    btn1v=1;
  }  
  if (btn3v==0){ //button 3 (upper right button): proceed to next step (save current value as the limit that is adjusted in this step)
    digitalWrite(led3, HIGH);
    btn3v=1;
    calval=calval+1;
    preferences.putInt("SteerCal",0);
    if (calval==2){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Value confirmed: ");
      lcd.setCursor(0,2);
      lcd.print(leftlimit);
      preferences.putInt("LeftLimit",leftlimit);
      delay(1000);
    }else if (calval==3){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Value confirmed: ");
      lcd.setCursor(0,2);
      lcd.print(rightlimit);
      preferences.putInt("RightLimit",rightlimit);
      delay(1000);
    }else if (calval==4){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Value confirmed: ");
      lcd.setCursor(0,2);
      lcd.print((leftlimit+rightlimit)/2);
      preferences.putInt("LeftLimit",leftlimit);
      preferences.putInt("RightLimit",rightlimit);
      delay(1000);
    }else if (calval==5){
      menu=1;
      calval=0;
      preferences.putInt("SteerCal",1); //store in flash memory that the calibration of the steering servo has been performed
      digitalWrite(led2, HIGH);
    }
    lcd.clear();
  }
  if (btn2v==0){ //button 2 (lower right button): go one step back in the calibration
    digitalWrite(led2, HIGH);
    btn2v=1;
    calval=calval-1;
    if (calval<0){
      menu=1;
      calval=0;
    }
    lcd.clear();
  }  
}

void steerLCD(){ //the LCD screen of the steering servo calibration is equal to the calibration screens of the actuators
  digitalWrite(led1, LOW);
  if(menu!=1){
    digitalWrite(led2, LOW);
  }else{
    digitalWrite(led2, HIGH);
  }
  digitalWrite(led3, LOW);
  digitalWrite(led4, LOW);  
  if (calval==0){
    if (preferences.getInt("SteerCal",0)>0){
      lcd.setCursor(0,0);
      lcd.print("Steering calibration");
      lcd.setCursor(0,1);
      lcd.print("Calibration found   ");
      lcd.setCursor(0,2);
      lcd.print("Override: UR button");  
      lcd.setCursor(0,3);
      lcd.print("Return: LR button");  
    }else{
      lcd.setCursor(0,0);
      lcd.print("Steering calibration");
      lcd.setCursor(0,1);      
      lcd.print("No calibration found");
      lcd.setCursor(0,2);
      lcd.print("Start cal: UR button");  
      lcd.setCursor(0,3);
      lcd.print("Return: LR button");        
    }
  }else if (calval==1){
    lcd.setCursor(0,0); 
    lcd.print("Adj left with UL/LL");
    lcd.setCursor(0,1);
    lcd.print("Left limit: ");
    lcd.print(leftlimit);
    lcd.print("  ");    
    lcd.setCursor(0,2);
    lcd.print("Confirm: UR button");   
    lcd.setCursor(0,3);
    lcd.print("Return : LR button");       
  }else if (calval==2){
    lcd.setCursor(0,0); 
    lcd.print("Adj right with UL/LL");
    lcd.setCursor(0,1);
    lcd.print("Right limit: ");
    lcd.print(rightlimit);
    lcd.print("  ");    
    lcd.setCursor(0,2);
    lcd.print("Confirm: UR button");   
    lcd.setCursor(0,3);
    lcd.print("Return : LR button");       
  }else if (calval==3){
    lcd.setCursor(0,0); 
    lcd.print("Adj center with UL/LL");    
    lcd.setCursor(0,1); 
    lcd.print("Center: ");    
    lcd.print((leftlimit+rightlimit)/2);
    lcd.setCursor(0,2);
    lcd.print("Confirm: UR button");  
    lcd.setCursor(0,3);
    lcd.print("Return: LR button");     
  }else if (calval==4){
    lcd.setCursor(0,0); 
    lcd.print("Calibration done!");    
    lcd.setCursor(0,2);
    lcd.print("Confirm: UR button");  
    lcd.setCursor(0,3);
    lcd.print("Return: LR button");      
  }
}

void setLight(){ //adjustment of the headlight LED brightness
  lightint = readAnalog(poti); //the speed poti is used to adjust the LED  intensity value
  if (lightint>17580){
    lightint=17580;
  }else if (lightint<20){
    lightint=20;
  }
  lightint = map(lightint,20,17580,26,255); //the intensity can be adjusted between 10 and 100% PWM duty cycle
  preferences.putInt("headlights",lightint);
  if (btn2v==0){ //the intensity adjustment is completed upon pressing the upper or lower right button
    digitalWrite(led2, HIGH);
    menu=1;
  }
  if (btn3v==0){
    digitalWrite(led3, HIGH);
    menu=1;
  }
  if (menu==1){
    btn2v=1;
    btn3v=1;
    digitalWrite(led2, HIGH);
    lcd.clear();
  }
}

void lightLCD(){ //LCD screen for headlight LED adjustment
  digitalWrite(led4, LOW);
  digitalWrite(led1, LOW);  
  lcd.setCursor(0,0);
  lcd.print("Headlight intensity");
  lcd.setCursor(0,1);
  lcd.print("    ");
  lcd.setCursor(4,1);  
  lcd.print(lightint);
  lcd.print("  ");
  lcd.setCursor(0,3);
  lcd.print("Use poti to adjust");
}

void RGBLCD(){ //LCD screen for adjusting the RGB LED running light color
  digitalWrite(led4, LOW);
  digitalWrite(led1, LOW);
  lcd.setCursor(0,0);
  lcd.print("Running light mode:");
  lcd.setCursor(0,1);
  lcd.print("    ");
  lcd.setCursor(4,1);
  switch (program){
    case 0:
      lcd.print("Off    ");
      break;
    case 1:
      lcd.print("Blue   ");
      break;  
    case 2:
      lcd.print("Cyan   ");
      break;
    case 3:
      lcd.print("Green  ");
      break;      
    case 4:
      lcd.print("Yellow ");
      break;
    case 5:
      lcd.print("Red    ");
      break;
    case 6:
      lcd.print("Violet ");
      break;      
    case 7:
      lcd.print("White  ");
      break;
    case 8:
      lcd.print("Rainbow");
      break;    
  }
  lcd.setCursor(0,2);
  lcd.print("RGB:"); //Not only the color, but also the RGB code of the color is displayed
  lcd.setCursor(5,2);
  lcd.print("[");
  if (outgoing.R<10){ //make sure that small numbers are displayed correctly by adding zeros if needed
    lcd.print("00");
  }else if (outgoing.R<100){
    lcd.print("0");
  }  
  lcd.print(outgoing.R);
  lcd.print(",");
  if (outgoing.G<10){
    lcd.print("00");
  }else if (outgoing.G<100){
    lcd.print("0");
  }  
  lcd.print(outgoing.G);
  lcd.print(",");
  if (outgoing.B<10){
    lcd.print("00");
  }else if (outgoing.B<100){
    lcd.print("0");
  }   
  lcd.print(outgoing.B);
  lcd.print("]");   
  lcd.setCursor(0,3);
  lcd.print("Adjust: left btns");
}

void RGBInput(){ //RGB LED running light color adjustment: Selection menu for the different programs
  if (btn4v==0){ //button 4 (lower left button): go back one program option
    digitalWrite(led4, HIGH);
    if (program==0){
      program=8;
    }else{
      program=program-1;
    }
    btn4v=1;
  }
  if (btn1v==0){ //button 1 (upper left button): go to next program option
    digitalWrite(led1, HIGH);
    if (program==8){
      program=0;
    }else{
      program=program+1;
    }
    btn1v=1;
  }  
  if (btn2v==0){ //buttons 2 and 3 (upper or lower right button) confirm the choice of the currently selected program; return to selection menu
    digitalWrite(led2, HIGH);
    menu=1;
  }
  if (btn3v==0){
    digitalWrite(led3, HIGH);
    menu=1;
  }
  if (menu==1){
    btn2v=1;
    btn3v=1;
    digitalWrite(led2, HIGH);
    lcd.clear();
  }
}

void RGBUpdate() { //Update function for the RGB LED running light
  if (rbr==0&&rbb>0){ //these variables correspond to RGB values for the rainbow mode of the RGB LED strip: rb equals rainbow, and the individual colors r, b, and g
    rbb = rbb-2; //the change of each value per iteration is timed based on the 25 ms delay in the main code that triggers calling this function
    rbg = rbg+2; //in total, this code snippet results in a rainbow color change from blue to green to red to blue and so on
  }else if(rbb==0&&rbg>0){
    rbg = rbg-2;
    rbr = rbr+2;
  }else if(rbg==0&&rbr>0){
    rbr = rbr-2;
    rbb = rbb+2;
  }      
  switch (program) {
    case 0: //program zero: running light off
      outgoing.R = 0;
      outgoing.G = 0;
      outgoing.B = 0;
      break;
    case 1: //program one: running light blue
      outgoing.R = 0;
      outgoing.G = 0;
      outgoing.B = 127;
      break;
    case 2: //program two: running light cyan
      outgoing.R = 0;
      outgoing.G = 127;
      outgoing.B = 127;
      break;
    case 3: //program three: running light green
      outgoing.R = 0;
      outgoing.G = 127;
      outgoing.B = 0;
      break;
    case 4: //program four: running light yellow
      outgoing.R = 127;
      outgoing.G = 127;
      outgoing.B = 0;
      break;
    case 5: //program five: running light red
      outgoing.R = 127;
      outgoing.G = 0;
      outgoing.B = 0;
      break;
    case 6: //program six: running light violet
      outgoing.R = 127;
      outgoing.G = 0;
      outgoing.B = 127;
      break;
    case 7: //program seven: running light white
      outgoing.R = 127;
      outgoing.G = 127;
      outgoing.B = 127;
      break;
    case 8: //program eight: running light rainbow
      outgoing.R = rbr;
      outgoing.G = rbg;
      outgoing.B = rbb;
      break;          
  }  
  preferences.putInt("Program",program);
}

void joyCal(){ //function for joystick calibration
  if (btn4v==0){ //adjustment of joystick tolerance value around center (region to be set to zero to account for tolerances) using upper left (4) and lower left (1) buttons
    if (calval==3){  //only done in the respective step of the calibration process
      digitalWrite(led4, HIGH);
      if (joyltol<64){ //maximum tolerance: 25%
        joyltol=joyltol+1;
      }
    }else if (calval==6){
      digitalWrite(led4, HIGH);
      if (joyrtol<64){
        joyrtol=joyrtol+1;
      }
    }else if (calval==7){ //change between normal and inverted joystick mode
      digitalWrite(led4, HIGH);
      if (joyrinv==0){
        joyrinv=1;
      }else{
        joyrinv=0;
      }
    }
    btn4v=1;
  }
  if (btn1v==0){
    if (calval==3){
      digitalWrite(led1, HIGH);
      if (joyltol>1){ //minimum tolerance: 1
        joyltol=joyltol-1;
      }
    }else if (calval==6){
      digitalWrite(led1, HIGH);
      if (joyrtol>1){
        joyrtol=joyrtol-1;
      }
    }else if (calval==7){
      digitalWrite(led1, HIGH);
      if (joyrinv==0){
        joyrinv=1;
      }else{
        joyrinv=0;
      }
    }
    btn1v=1;
  }  
  if (btn3v==0){ //upper right button (3): proceed to next step of the calibration (confirm current input)
    digitalWrite(led3, HIGH);
    btn3v=1;
    calval=calval+1;
    preferences.putInt("JoyCal",0);
    if (calval==1){
      joylcen = readAnalog(joyl); //the center values are recorded at the beginning to later on account for the two wiring options of the joystick poti
      joyrcen = readAnalog(joyr);
    }else if (calval==2){ //step one: left limit of left joystick
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Value confirmed: ");
      lcd.setCursor(0,2);
      lcd.print(map(readAnalog(joyl),0,17670,0,255));
      if (readAnalog(joyl)>joylcen){ //the left and right limits are recorded and slightly reduced to ensure that the maximum can be reached even if there is noise in the readings
        preferences.putInt("joyLleft",readAnalog(joyl)-50);
      }else{
        preferences.putInt("joyLleft",readAnalog(joyl)+50);      
      }
      delay(1000);
    }else if (calval==3){ //step two: right limit of left joystick
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Value confirmed: ");
      lcd.setCursor(0,2);
      lcd.print(map(readAnalog(joyl),0,17670,0,255));
      if (readAnalog(joyl)>joylcen){
        preferences.putInt("joyLright",readAnalog(joyl)-50);
      }else{
        preferences.putInt("joyLright",readAnalog(joyl)+50);      
      }
      delay(1000);
    }else if (calval==4){ //step three: center tolerance of left joystick
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Value confirmed: ");
      lcd.setCursor(0,2);
      lcd.print((double(joyltol)/double(255))*double(100),1);
      lcd.print(" %   ");
      preferences.putInt("joyLtol",joyltol);
      delay(1000);
    }else if (calval==5){ //step four: upper limit of right joystick
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Value confirmed: ");
      lcd.setCursor(0,2);
      lcd.print(map(readAnalog(joyr),0,17670,0,255));
      if (readAnalog(joyr)>joyrcen){
        preferences.putInt("joyRup",readAnalog(joyr)-50);
      }else{
        preferences.putInt("joyRup",readAnalog(joyr)+50);      
      }
      delay(1000);
    }else if (calval==6){ //step five: lower limit of right joystick
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Value confirmed: ");
      lcd.setCursor(0,2);
      lcd.print(map(readAnalog(joyr),0,17670,0,255));
      if (readAnalog(joyr)>joyrcen){
        preferences.putInt("joyRdown",readAnalog(joyr)-50);
      }else{
        preferences.putInt("joyRdown",readAnalog(joyr)+50);      
      }
      delay(1000);
    }else if (calval==7){ //step six: center tolerance of right joystick
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Value confirmed: ");
      lcd.setCursor(0,2);
      lcd.print((double(joyrtol)/double(255))*double(100),1);
      lcd.print(" %   ");
      preferences.putInt("joyRtol",joyrtol);
      delay(1000);
    }else if (calval==8){ //step seven: specify if joyR input shall be inverted
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Input confirmed: ");
      lcd.setCursor(0,2);
      lcd.print("Joystick ");
      if (joyrinv==0){
        lcd.print("default");
      }else{
        lcd.print("inverted");
      }
      preferences.putInt("joyRInv",joyrinv);
      delay(1000);
    }else if (calval==9){ //step eight: confirmation of calibration
      menu=1;
      calval=0;
      preferences.putInt("JoyCal",1);
      joyltol = preferences.getInt("joyLtol",20);
      joyll = preferences.getInt("joyLleft",0);
      joylr = preferences.getInt("joyLright",0);
      joyrtol = preferences.getInt("joyRtol",20);
      joyru = preferences.getInt("joyRup",0);
      joyrd = preferences.getInt("joyRdown",0);
      joyrinv = preferences.getInt("joyRInv",0);
      digitalWrite(led2, HIGH);
    }
    lcd.clear();
  }
  if (btn2v==0){ //lower right button: go back one step in the calibration
    digitalWrite(led2, HIGH);
    btn2v=1;
    calval=calval-1;
    if (calval<0){
      menu=1;
      calval=0;
    }
    lcd.clear();
  }  
}

void joyCalLCD(){ //LCD screen for joystick calibration
  digitalWrite(led1, LOW);
  if(menu!=1){
    digitalWrite(led2, LOW);
  }else{
    digitalWrite(led2, HIGH);
  }
  digitalWrite(led3, LOW);
  digitalWrite(led4, LOW);  
  if (calval==0){
    if (preferences.getInt("JoyCal",0)>0){
      lcd.setCursor(0,0);
      lcd.print("Joystick calibration");
      lcd.setCursor(0,1);
      lcd.print("Calibration found   ");
      lcd.setCursor(0,2);
      lcd.print("Override: UR button");  
      lcd.setCursor(0,3);
      lcd.print("Return: LR button");  
    }else{
      lcd.setCursor(0,0);
      lcd.print("Joystick calibration");
      lcd.setCursor(0,1);      
      lcd.print("No calibration found");
      lcd.setCursor(0,2);
      lcd.print("Start cal: UR button");  
      lcd.setCursor(0,3);
      lcd.print("Return: LR button");        
    }
  }else if (calval==1){
    lcd.setCursor(0,0); 
    lcd.print("Move JoyL left ");
    lcd.setCursor(0,1);
    lcd.print("JoyL value: ");
    lcd.print(map(readAnalog(joyl),0,17670,0,255));
    lcd.print("  ");    
    lcd.setCursor(0,2);
    lcd.print("Confirm: UR button");   
    lcd.setCursor(0,3);
    lcd.print("Return : LR button");       
  }else if (calval==2){
    lcd.setCursor(0,0); 
    lcd.print("Move JoyL right");
    lcd.setCursor(0,1);
    lcd.print("JoyL value: ");
    lcd.print(map(readAnalog(joyl),0,17670,0,255));
    lcd.print("  ");    
    lcd.setCursor(0,2);
    lcd.print("Confirm: UR button");   
    lcd.setCursor(0,3);
    lcd.print("Return : LR button");       
  }else if (calval==3){
    lcd.setCursor(0,0); 
    lcd.print("Joy L: tolerance");    
    lcd.setCursor(0,1);  
    if (joyltol>1&&joyltol<64){ //tolerance values are given in per cent
      lcd.print("Value: ");   
      lcd.print((double(joyltol)/double(255))*double(100),1);
      lcd.print(" %   ");
    }else{
      lcd.print("Limit reached! ");
    }
    lcd.setCursor(0,2);
    lcd.print("Confirm: UR button");  
    lcd.setCursor(0,3);
    lcd.print("Return: LR button");     
  }else if (calval==4){
    lcd.setCursor(0,0); 
    lcd.print("Move JoyR up ");
    lcd.setCursor(0,1);
    lcd.print("JoyR value: ");
    lcd.print(map(readAnalog(joyr),0,17670,0,255));
    lcd.print("  ");
    lcd.setCursor(0,2);
    lcd.print("Confirm: UR button");   
    lcd.setCursor(0,3);
    lcd.print("Return : LR button");       
  }else if (calval==5){
    lcd.setCursor(0,0); 
    lcd.print("Move JoyR down");
    lcd.setCursor(0,1);
    lcd.print("JoyR value: ");
    lcd.print(map(readAnalog(joyr),0,17670,0,255));
    lcd.print("  ");
    lcd.setCursor(0,2);
    lcd.print("Confirm: UR button");   
    lcd.setCursor(0,3);
    lcd.print("Return : LR button");       
  }else if (calval==6){
    lcd.setCursor(0,0); 
    lcd.print("Joy R: tolerance");    
    lcd.setCursor(0,1); 
    if (joyrtol>1&&joyrtol<64){
      lcd.print("Value: ");   
      lcd.print((double(joyrtol)/double(255))*double(100),1);
      lcd.print(" %   ");
    }else{
      lcd.print("Limit reached! ");
    }
    lcd.setCursor(0,2);
    lcd.print("Confirm: UR button");  
    lcd.setCursor(0,3);
    lcd.print("Return: LR button");     
  }else if (calval==7){
    lcd.setCursor(0,0);   
    lcd.print("JoyR mode: ");
    if (joyrinv==0){
      lcd.print("default ");
    }else{
      lcd.print("inverted");
    }
    lcd.setCursor(0,1);
    lcd.print("Adj: UL/LL button");
    lcd.setCursor(0,2);
    lcd.print("Confirm: UR button");   
    lcd.setCursor(0,3);
    lcd.print("Return : LR button");     
  }else if (calval==8){
    lcd.setCursor(0,0); 
    lcd.print("Calibration done!");    
    lcd.setCursor(0,2);
    lcd.print("Confirm: UR button");  
    lcd.setCursor(0,3);
    lcd.print("Return: LR button");       
  }
}

void menuInputs(){ //function for selection menu
  if (btn4v==0){ //upper left button (4): move cursor up by one entry
    digitalWrite(led4, HIGH);
    if (menuval>0){
      menuval = menuval-1;
    }else{
      menuval = 7;
    }
    btn4v=1;
  }
  if (btn1v==0){ //lower left button (1): move cursor down by one entry
    digitalWrite(led1, HIGH);
    if (menuval<7){
      menuval = menuval+1;
    }else{
      menuval = 0;
    }
    btn1v=1;
  }
  if (btn2v==0){ //return from menu to the selected menu entry by pressing the lwoer right button (2)
    btn1v = 1; //in case the buttons were pressed: return to default state
    btn2v = 1;
    btn3v = 1;
    btn4v = 1;
    if(menuval==0){ //the cursor of the selection menu has a different number than the menu entries
      menu=2;
    }else if(menuval==1){
      menu=3;
    }else if(menuval==2){
      menu=4;  
    }else if(menuval==3){
      menu=5;
    }else if(menuval==4){
      menu=6;
    }else if(menuval==5){
      menu=7;
    }else if(menuval==6){
      menu=8;
    }else if(menuval==7){
      menu=0;
    }
    digitalWrite(led2, LOW);
    lcd.clear();
  }   
}

void menuLCD(){ //LCD screen for the selection menu
  lcd.setCursor(1,0);
  lcd.print("Joysticks");
  lcd.setCursor(1,1);
  lcd.print("Steering ");
  lcd.setCursor(1,2);
  lcd.print("Gears    ");
  lcd.setCursor(1,3);
  lcd.print("Clutch   ");
  lcd.setCursor(11,0);
  lcd.print("Diff     ");  
  lcd.setCursor(11,1);
  lcd.print("Headlight"); 
  lcd.setCursor(11,2);
  lcd.print("RGB-Strip"); 
  lcd.setCursor(11,3);
  lcd.print("Back     "); 
  for (int i=0; i<=3; i++){ //show the current position of the cursor
    lcd.setCursor(0,i);
    if (menuval==i){
      lcd.write(1);
    }else{
      lcd.print(" ");
    }
  }
  for (int i=0; i<=3; i++){
    lcd.setCursor(10,i);
    if (menuval-4==i){
      lcd.write(1);
    }else{
      lcd.print(" ");
    }
  }
  digitalWrite(led4, LOW);
  digitalWrite(led3, LOW);
  digitalWrite(led1, LOW);
}

void digitalInputs(){ //check digital inputs (switches & buttons) as control inputs for the car
  if (btn3v==0){ //switching the headlights on and off
    btn3v = 1;
    if (lighton==0){
      lighton = 1;
    }else{
      lighton = 0;
    }
  }
  if (lighton==1&&batlow<11){
    outgoing.headlights = lightint; //simple transmission of an 8bit duty cycle (0-255) for the headlight intensity
    digitalWrite(led3, HIGH);
  }else{
    outgoing.headlights = 0;
    digitalWrite(led3, LOW);
  }  

  if (btn4v==0&&batlow<11){ //opening and closing the brake; only done when the battery low threshold has not been reached
    btn4v = 1;
    if (brakeon==0){
      brakeon = 1;
    }else{
      brakeon = 0;
    }
  }
  if (brakeon==1){
    outgoing.brake = 1;
    if (incoming.brake<3){
      digitalWrite(led4, HIGH);
    }
  }else{
    outgoing.brake = 0;
    digitalWrite(led4, LOW);
  }  

  if (btn1v==0&&batlow<11){ //opening and closing the diff lock; only done when the battery low threshold has not been reached
    btn1v = 1;
    if (difflockon==0){
      difflockon = 1;
    }else{
      difflockon = 0;
    }
  }
  if (preferences.getInt("DiffCal",0)>0&&incoming.diffpos!=3){ //the diff lock is only moved if its calibration has been performed
    if (difflockon==1){
      outgoing.diff = 1; 
      digitalWrite(led1, HIGH);
    }else{
      outgoing.diff = 0;
      digitalWrite(led1, LOW);
    }  
  }else{
    outgoing.diff=2; //value 2 indicates that the actuator has not been calibrated
  }

  if (incoming.clutchpos!=3&&preferences.getInt("ClutchCal",0)>0){ //the clutch is only moved if its calibration has been performed
    if (digitalRead(lsw)>0&&batlow<11){ //only done when the battery low threshold has not been reached
      outgoing.clutch=1; //open the clutch
    }else if(digitalRead(lsw)<1&&batlow<11){
      outgoing.clutch=0; //close the clutch
    }
    if (preferences.getInt("ShifterCal",0)>0&&incoming.shifterpos!=3){ //the gear shifter is only moved if its calibration has been performed
      if (digitalRead(rsw)>0&&incoming.shifterpos!=1&&batlow<11){ //gear shifting triggers opening the clutch; only done when the battery low threshold has not been reached
        outgoing.clutch=1; //this line overwrites the desired clutch position before sending it to the car; therefore, it forces to open the clutch (if it is not open yet) when a gear change is requested
        if (incoming.clutchpos==1){
          outgoing.gear=1;
        }
      }else if (digitalRead(rsw)<1&&incoming.shifterpos!=0&&batlow<11){
        outgoing.clutch=1;
        if(incoming.clutchpos==1){
          outgoing.gear=0;
        }
      }
    }else{
      outgoing.gear=2;
    }    
  }else{
    outgoing.clutch=2;
  }

  if (btn2v==0){ //call the selection menu using the lower right button
    btn2v = 1;
    menu=1;
    menuval = 0;
    digitalWrite(led1, LOW);
    digitalWrite(led2, HIGH);
    digitalWrite(led3, LOW);
    digitalWrite(led4, LOW);
    lcd.clear();
  }
}

void analogInputs(){ //check the joysticks as control inputs for the car
  steering = readAnalog(joyl);
  if (preferences.getInt("JoyCal")>0){
    if (joyll>joylr){ //the value of the steering joystick is capped at the limits specified during calibration (also accounts for inverted wiring of the joystick's poti)
      if (steering>joyll){
        steering = joyll;
      }else if (steering<joylr){
        steering = joylr;
      }
    }else{
      if (steering<joyll){
        steering = joyll;
      }else if (steering>joylr){
        steering = joylr;
      }
    }
    steering = map(steering,joyll,joylr,-255,255); 
  }else{
    steering = 0;
  }

  if (steering<joyltol&&steering>-joyltol){  //the tolerance range is specified during calibration; values within +-tolerance are set to 0 to avoid shivering around the center point of the joystick
    steering = 0;
  }else if (steering>=joyltol){
    steering=map(steering,joyltol,255,1,255); //values are re-mapped to make use of the full available range
  }else if (steering<=-joyltol){
    steering=map(steering,-joyltol,-255,-1,-255);
  }

  mainspeed = readAnalog(joyr);

  if (joyru>joyrd){ //the value of the speed joystick is capped at the limits specified during calibration (also accounts for inverted wiring of the joystick's poti)
    if (mainspeed>joyru){
      mainspeed = joyru;
    }else if (mainspeed<joyrd){
      mainspeed = joyrd;
    }
  }else{
    if (mainspeed<joyru){
      mainspeed = joyru;
    }else if (mainspeed>joyrd){
      mainspeed = joyrd;
    }    
  }

  if (joyrinv==0){ //adjust values in case of inverted joystick mode
    mainspeed = map(mainspeed,joyrd,joyru,-potival,potival); //re-map the values at the limits specified by the speed joystick to limit maximum speed
    if (mainspeed<-64){ //limit max speed in reverse: 25% PWM duty cycle
      mainspeed = -64;
    }    
  }else{
    mainspeed = map(mainspeed,joyrd,joyru,potival,-potival); //re-map the values at the limits specified by the speed joystick to limit maximum speed
    if (mainspeed>64){ //limit max speed in reverse: 25% PWM duty cycle
      mainspeed = 64;
    }
  }

  if (mainspeed<(double(joyrtol)/255)*potival&&mainspeed>-(double(joyrtol)/255)*potival){ //keep the center of the joystick numb
    mainspeed = 0;
  }

  if (incoming.clutchpos>1||(incoming.clutchpos==0&&incoming.brake>0)||incoming.shifterpos>2||batlow>10){ //prevent driving while moving the clutch or if the brake is active or if the battery is low
    mainspeed = 0;
  }
  if (batlow<11){
    steering = map(steering,-255,255,leftlimit,rightlimit); //map the steering values to the limits specified during calibration of the steering servo
  }else{
    steering = map(0,-255,255,leftlimit,rightlimit);
  }
}

void updateLCD(){ //LCD screen for the default operation
  lcd.setCursor(0,0);
  lcd.print("TX "); //Battery voltage of remote control
  if (batvolt>=6.8||lcdcounter>3){ //Indicate that the battery is low if the voltage is 6.8 V or less (<1.13V per cell)
    lcd.print(batvolt,2);
    lcd.print("V");  
  }else{
    lcd.print("Bt low");
  }
  lcd.setCursor(11,0); //show status of headlight LEDs
  if (lighton>0){
    lcd.print("Light on ");
  }else{
    lcd.print("Light off");
  }
  lcd.setCursor(0,1);
  lcd.print("RX "); //Battery voltage and current draw of the car
  if (signalquality>10&&batlow<11||lcdcounter>3&&signalquality>10){
    lcd.print(voltage,2);
    lcd.print("V ");
    if (currentflow!=0){ //only show current & power if the current flow direction was measured in the setup
      if(current>=10){
        lcd.print(current,1);
        lcd.print("A ");
      }else if(current<0){ //negative currents (=motor charges the battery) are not displayed
        lcd.print("<0");
        lcd.print("A   ");
      }else{
        lcd.print(current,1);
        lcd.print("A  ");
      }
      if (voltage*current>=100){ //show power consumption of the car
        lcd.print(voltage*current,0);
        lcd.print("W");    
      }else if (voltage*current>=10){
        lcd.print(voltage*current,0);
        lcd.print("W "); 
      }else if (voltage*current>0){
        lcd.print(voltage*current,0);
        lcd.print("W  "); 
      }else{
        lcd.print("0");
        lcd.print("W  "); 
      } 
    }else{
      lcd.print("        ");
    }
  }else if(signalquality>10&&batlow>10){ //Indicate that the battery is low if the threshold is reached
    lcd.print("battery low!    ");
  }else{
    lcd.print("no connection!  "); //Indicate if there is no connection between remote control and car
  }
  if (preferences.getInt("JoyCal")>0&&preferences.getInt("SteerCal")>0){ //information on actuators is only displayed if joysticks and steering servo have been calibrated
    lcd.setCursor(0,2);
    lcd.print("Brake ");
    if (incoming.brake==1){ //brake status
      lcd.print("eng");
    }else if(incoming.brake==2){
      lcd.print("...");
    }else if(incoming.brake==0){
      lcd.print("dis");
    }else{
      lcd.print("n.c");
    }
    lcd.setCursor(11,2);
    lcd.print("Diff ");
    if(incoming.diffpos==1){ //diff lock status
      lcd.print("lckd");
    }else if(incoming.diffpos==2){
      lcd.print("... ");
    }else if(incoming.diffpos==0){
      lcd.print("open");
    }else if(incoming.diffpos==3||preferences.getInt("DiffCal",0)==0){
      lcd.print("n.c.");
    }
    lcd.setCursor(0,3);
    lcd.print("Clutch ");
    if (incoming.clutchpos==1){ //clutch status
      lcd.print("opn");
    }else if(incoming.clutchpos==2){
      lcd.print("...");
    }else if(incoming.clutchpos==0){
      lcd.print("cls");
    }else if(incoming.clutchpos>=3||preferences.getInt("ClutchCal",0)==0){
      lcd.print("n.c");
    }
    lcd.setCursor(11,3);
    if(incoming.shifterpos==1){ //gear shifter status
      lcd.print("Fast gear");
    }else if(incoming.shifterpos==2){
      lcd.print("Shifting ");
    }else if(incoming.shifterpos==0){
      lcd.print("Slow gear");
    }else if(incoming.shifterpos>=3||preferences.getInt("ShifterCal",0)==0){
      lcd.print("Gear n.c.");
    }
  }else if(preferences.getInt("JoyCal")==0){
    lcd.setCursor(0,2);
    lcd.print("Calibrate joysticks ");
    lcd.setCursor(0,3);
    lcd.print("to get started!     ");
  }else if(preferences.getInt("SteerCal")==0){
    lcd.setCursor(0,2);
    lcd.print("Calibrate steering  ");
    lcd.setCursor(0,3);
    lcd.print("to get started!     ");    
  }
  lcdcounter=lcdcounter+1; //counter for indicating low battery
  if (lcdcounter>7){
    lcdcounter=0;
  }   
}
