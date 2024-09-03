//Include required libraries
#include <Preferences.h> //library for storing variables in flash memory

//pin definitions
#define brakesense 15
#define brakeled 16
#define sw1 17
#define sw2 18
#define sw3 19
#define brakein1 21
#define brakein2 22
#define brakeout 23
#define enpin 25
#define m0 27
#define m1 26
#define dirpin 32
#define stppin 33

//Global variables needed for the code
int brakecounter  = 0;
int brakelimit    = 0;
int brakedir      = 0;
int btn2debounce0 = 0;
int btn2debounce1 = 0;
int btn2v         = 1;
int lighton       = 0;
int lighttimer    = 0;
int status        = 0;
int calmenu       = 0;

//The preferences library allows you to store variables within the flash so that their value can be retrieved on startup. Used to store user-adjustable settings.
Preferences preferences;

//Interrupt function for reading one of the three PCB buttons (the center one)
void IRAM_ATTR btn2int(){
  btn2debounce1 = millis();
  if (btn2debounce1>btn2debounce0+500){ //debouncing: the button only reacts to new inputs if the last input was at least 0.5 s ago
    btn2v = 0;    
  }
  btn2debounce0 = btn2debounce1;
}

void setup() {
  //pin mode definitions
  pinMode(brakesense, INPUT_PULLUP);
  pinMode(brakeled, OUTPUT);
  digitalWrite(brakeled, LOW);
  pinMode(sw1, INPUT_PULLUP);
  pinMode(sw2, INPUT_PULLUP);
  pinMode(sw3, INPUT_PULLUP);
  pinMode(brakein1, OUTPUT);
  pinMode(brakein2, OUTPUT);
  pinMode(brakeout, INPUT_PULLUP);
  pinMode(m0, OUTPUT);
  pinMode(m1, OUTPUT);
  digitalWrite(m0, HIGH);  //Setting the M-pins of the stepper driver high means going incremental steps (1/8 steps: M0 high, M1 high, M2 low (M2 is always low, hard-wired by the PCB))
  digitalWrite(m1, HIGH);
  pinMode(enpin, OUTPUT);
  pinMode(dirpin, OUTPUT);
  pinMode(stppin, OUTPUT);
  digitalWrite(enpin, LOW);
  digitalWrite(dirpin, LOW);
  digitalWrite(stppin, LOW);
  attachInterrupt(sw2,btn2int,FALLING); //the interrupt has to be activated once; the trigger is "falling", meaning a change from high to low
  Serial.begin(115200); //left here in case debugging via the serial monitor is needed

  preferences.begin("Defaults", false); //retrieve values from flash memory upon startup
  brakelimit = preferences.getInt("limit",0);
  brakedir = preferences.getInt("dir",0);

  if(preferences.getInt("Cal",0)>0){ //if the brake is calibrated and not open during startup, the system homes the brake upon startup (=moves the brake to the open position)
    while (digitalRead(brakesense)==0){ //the brake sensor is low while the brake is not in open position
      digitalWrite(brakeled, HIGH); //led high means the brake is moving
      digitalWrite(enpin, HIGH); //driving the enable pin of the driver high means the stepper motor is powered on (constant current draw, no matter if moving or not!)
      if (brakedir==1){ //dir is specified during the calibration of the brake
        digitalWrite(dirpin, LOW); 
      }else{
        digitalWrite(dirpin, HIGH);
      }
      digitalWrite(stppin, HIGH); //driving the stepper motor is done by providing "high" pulses on the step input of the stepper driver
      delayMicroseconds(10);
      digitalWrite(stppin, LOW);
      delayMicroseconds(1990); //this interval (1.99 ms) was determined manually. It needs to match the stepper motor's characteristics (driving a stepper motor too fast means it won't move but only produce noises)
    }
    digitalWrite(enpin, LOW);
    digitalWrite(brakein1, LOW); //transmit the brake status to the main ESP32: 00 -> brake is open
    digitalWrite(brakein2, LOW);
  }else{
    digitalWrite(brakein1, LOW); //transmit the brake status to the main ESP32: 01 -> brake is not calibrated
    digitalWrite(brakein2, HIGH);    
  }

  delay(2000); //make sure the main ESP32 is running in its main loop prior to entering the main loop of the auxiliary ESP32 (to ensure the parallel data interface is ready)
}

void loop() {

  if (btn2v==0&&calmenu==0){ //pressing the center button (btn2) means starting the calibration
    calmenu=1;
    btn2v=1;
    status=0;
    brakedir=0;
  }

  //check status of brake, control stepper, interface & led, call calibration if requested
  if(preferences.getInt("Cal",0)>0&&calmenu==0){  //normal operation is only entered if calibration is present
    if(digitalRead(brakeout)==0&&digitalRead(brakesense)==1){ //brake is open and is supposed to be open
      if(millis()>lighttimer+2000){ //the led of the brake blinks slowly in this case
        if (lighton==1){
          lighton=0;
          digitalWrite(brakeled, HIGH);
        }else{
          lighton=1;
          digitalWrite(brakeled, LOW);
        }
        lighttimer = millis();      
      } 
      digitalWrite(brakein1, LOW); //transmit the brake status to the main ESP32: 00 -> brake is open
      digitalWrite(brakein2, LOW); 
      digitalWrite(enpin, LOW); //the stepper motor is disabled whenever the brake does not have to open or close (saves energy)   
      brakecounter = 0; //this variable is used to count steps; it is reset in the open position of the brake 
    }else if(digitalRead(brakeout)==1&&brakecounter==brakelimit){ //brake is closed and is supposed to be closed
      digitalWrite(brakeled, HIGH); //the brake led is on (no blinking) when the brake is closed
      digitalWrite(brakein1, HIGH); //transmit the brake status to the main ESP32: 11 -> brake is closed
      digitalWrite(brakein2, HIGH);
      digitalWrite(enpin, LOW);      
    }else if(digitalRead(brakeout)==0&&digitalRead(brakesense)==0){ //brake is not open but supposed to be open (=moving)
      if(millis()>lighttimer+100){ //fast blinking of the brake led when the brake is moving
        if (lighton==1){
          lighton=0;
          digitalWrite(brakeled, HIGH);
        }else{
          lighton=1;
          digitalWrite(brakeled, LOW);
        }
        lighttimer = millis();       
      }
      digitalWrite(brakein1, HIGH); //transmit the brake status to the main ESP32: 10 -> brake is moving
      digitalWrite(brakein2, LOW);
      digitalWrite(enpin, HIGH);  //stepper motor is enabled (powered on) because it has to move
      if (brakedir==1){
        digitalWrite(dirpin, LOW); 
      }else{
        digitalWrite(dirpin, HIGH);
      }
      digitalWrite(stppin, HIGH);
      delayMicroseconds(10);
      digitalWrite(stppin, LOW);
      delayMicroseconds(1790);  //this delay is slightly shorter than in the setup because the rest of the loop also takes some time to be executed                
    }else if(digitalRead(brakeout)==1&&brakecounter<brakelimit){ //brake is not closed but supposed to be closed (=moving)
      if(millis()>lighttimer+100){
        if (lighton==1){
          lighton=0;
          digitalWrite(brakeled, HIGH);
        }else{
          lighton=1;
          digitalWrite(brakeled, LOW);
        }
        lighttimer = millis();
      }  
      digitalWrite(brakein1, HIGH); //transmit the brake status to the main ESP32: 10 -> brake is moving
      digitalWrite(brakein2, LOW);
      digitalWrite(enpin, HIGH);  
      if (brakedir==1){
        digitalWrite(dirpin, HIGH); 
      }else{
        digitalWrite(dirpin, LOW);
      }
      digitalWrite(stppin, HIGH);
      delayMicroseconds(10);
      digitalWrite(stppin, LOW);
      delayMicroseconds(1790);   
      brakecounter = brakecounter + 1; //increases by one for each step of the brake until reaching the number of required steps according to the brake limit (in the if-condition)                 
    }
  }else{
    digitalWrite(brakein1, LOW); //transmit the brake status to the main ESP32: 01 -> brake is not calibrated
    digitalWrite(brakein2, HIGH);      
    if(calmenu==0){ //brake not calibrated
      if(millis()>lighttimer+500){ //medium frequency blinking of the brake led when the brake is not calibrated
        if (lighton==1){
          lighton=0;
          digitalWrite(brakeled, HIGH);
        }else{
          lighton=1;
          digitalWrite(brakeled, LOW);
        }
        lighttimer = millis();      
      }     
      digitalWrite(enpin, LOW); 
    }else{ //brake is being calibrated
      brakecal(); //the calibration is performed in a separate function     
    }
  }
}

void brakecal(){ //the calibration consists of two consecutive steps
  if (status==0){ //step one: move the brake to the "open" position (trigger the brake sensor)
    if(millis()>lighttimer+250){ //the brake led blinks with this quite fast frequency during step one
      if (lighton==1){
        lighton=0;
        digitalWrite(brakeled, HIGH);
      }else{
        lighton=1;
        digitalWrite(brakeled, LOW);
      }
      lighttimer = millis();       
    }    
    if (btn2v==0){
      brakecounter=0; //the step counter is reset when pressing the center button
      btn2v=1;
    }
    if(digitalRead(sw1)<1){ //pressing button one moves the brake in direction one (dir=1)
      digitalWrite(enpin, HIGH);
      digitalWrite(dirpin, HIGH);
      digitalWrite(stppin, HIGH);
      delayMicroseconds(10);
      digitalWrite(stppin, LOW);
      delayMicroseconds(2490); 
      if (digitalRead(brakesense)==0){
        brakedir=brakedir+1; //brakedir>0 -> dirpin high to open the brake
      }
    }else if(digitalRead(sw3)<1){ //pressing button one moves the brake in direction zero (dir=0)
      digitalWrite(enpin, HIGH);
      digitalWrite(dirpin, LOW);
      digitalWrite(stppin, HIGH);
      delayMicroseconds(10);
      digitalWrite(stppin, LOW);
      delayMicroseconds(2490); 
      if (digitalRead(brakesense)==0){
        brakedir=brakedir-1; //brakedir<0 -> dirpin low to open the brake
      }  
    }
    if(digitalRead(brakesense)==1&&abs(brakedir)>0){ //once the brake reaches the brake sensor, the brakedir counter is evaluated to determine the movement direction
      status=1; //the calibration then moves on to step 2
      digitalWrite(brakeled, HIGH);
      delay(1000);
      if (brakedir<0){ //the calibration requires the brake to start when the brake sensor is not triggered (brakedir!=0)
        brakedir=1;
        preferences.putInt("dir",brakedir);
      }else{
        brakedir=0;
        preferences.putInt("dir",brakedir);
      }
    }
    btn2v=1;
    brakecounter=0;
  }
  if (status==1){ //step two: specify the brake position that equals "brake is closed"
    if(millis()>lighttimer+100){ //the brake led blinks fast during this step
      if (lighton==1){
        lighton=0;
        digitalWrite(brakeled, HIGH);
      }else{
        lighton=1;
        digitalWrite(brakeled, LOW);
      }
      lighttimer = millis();       
    } 
    if(digitalRead(sw1)<1){ //the brake moves (closes) while keeping button one pressed
      digitalWrite(enpin, HIGH);
      if (brakedir==1){ //brakedir==1 -> dirpin low to open the brake/high to close the brake
        digitalWrite(dirpin, HIGH);
      }else{ //brakedir==0 -> dirpin high to open the brake/low to close the brake
        digitalWrite(dirpin, LOW);
      }
      digitalWrite(stppin, HIGH);
      delayMicroseconds(10);
      digitalWrite(stppin, LOW);
      delayMicroseconds(2490); 
      brakecounter=brakecounter+1; 
    }
    if (btn2v==0){ //pressing button two finishes the calibration: the number of steps done to close the brake are stored in flash memory and used from now on
      brakelimit=brakecounter;
      preferences.putInt("limit",brakelimit);
      preferences.putInt("Cal",1);
      btn2v=1;
      calmenu=0;
    }
  }
}
