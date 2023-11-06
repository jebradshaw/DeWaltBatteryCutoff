// RasPi pico SURFER Vessel Interface board
//https://github.com/earlephilhower/arduino-pico/tree/master/cores/rp2040
#include <EEPROM.h>
#include "pico/stdlib.h"
#include <PicoAnalogCorrection.h> // https://github.com/Phoenix1747/Arduino-Pico-Analog-Correction
#include <math.h>

const uint8_t VCC_PIN = A0; // VCC meas pin
const uint8_t GND_PIN = A1; // GND meas pin
const uint8_t VIN_PIN = A2; // Vin pin to read the high voltage

const uint8_t ADC_RES = 12; // ADC bits

const uint8_t BUZZER_PIN = D3; // PWM Buzzer Ouput pin
PicoAnalogCorrection pico(ADC_RES, 1, 4078);

// Terminal command sequences.
#define CLR_SCR       "\33[;H\33[2J"
#define MOVE_L_C      "\33[%d;%dH\33[K" //move to line:column
#define RED_FONT      "\33[0;31m"
#define GREEN_FONT    "\33[0;32m"
#define YELLOW_FONT   "\33[0;33m"
#define BLUE_FONT     "\33[0;34m"
#define MAGENTA_FONT  "\33[0;35m"
#define WHITE_FONT    "\33[97m"    // bright white
#define DEFAULT_FONT  "\33[0m"
#define ERASE_LINE    "\33[K"

#define CRIT_TIMOUT_SEC     10    //  Set timeout seconds before shutdown when critical voltage low level is reached

float gainCalc = 10.660;    //voltage scale time resistor voltage divider gain (10M/1M ohm)
float highAlarmVolts = 17.5;
float lowAlarmVolts = 17.0;
float criticalLowVolts = 16.6;
int criticalCounter = 0;
int critCntLastTime = 0;

static unsigned long millisLastLed = 0;
static unsigned long millisLedDelay = 100;
static unsigned long millisLastBuzzer = 0;
static unsigned long millisLastSerial = 0;
static unsigned long millisSerialDelay = 1000;

int led = LED_BUILTIN; // the PWM pin the LED is attached to
static int batteryState = 1;
static int buzzerState = 0;
static int buzzerTime = 100;
static int buzzerCrit = 0;

// start assuming battery is charged and going down
float battVolts, battVoltsLast;
float NoAlarmHysVolts = -0.1;
float NormAlarmHysVolts = -0.1;
int dir = 0; // negative is moving down, positive if moving up

float dVdT;
float dVdT_avg;

int eeAddress = 0;   //Location we want the data to be put.
struct MemObject {  
  char descriptionStr[20];
  float gainCalc;  
  float highAlarmVolts;
  float lowAlarmVolts;
};

void setup() {
    // Make D2 the output to turn off the P-Channel MOSFET when triggered
  //digitalWrite(D2, HIGH);
  pinMode(D2, OUTPUT);
  digitalWrite(D2, HIGH);    
  pinMode(led, OUTPUT);
  

  //delay(10000);
  analogWriteFreq(2000);     // set freq output to 2KHz
  for(int i = 0;i<3;i++){    
    analogWrite(BUZZER_PIN, 512);     // turn on PWM 
    digitalWrite(led, !digitalRead(led));
    delay(200); 
    analogWrite(BUZZER_PIN, 0);     // turn off PWM 
    digitalWrite(led, !digitalRead(led));
    delay(100);    
  }  
       
  MemObject eepromData;  // create an object for the eeprom data
  //delay(2000);
  //set_sys_clock_khz(10000, true); // Set System clock to 10000 kHz


//  Uncomment out for the first time programming
//    eeAddress = 0;
//    eepromData.highAlarmVolts = highAlarmVolts;
//    eepromData.lowAlarmVolts = lowAlarmVolts;
//    eepromData.gainCalc = gainCalc;
//    EEPROM.put(eeAddress, eepromData);
//    delay(20);
//    EEPROM.commit();
    
  //Serial.setTimeout(20000); // allow up to 20 seconds to receive command
  Serial.begin(115200);
  Serial.printf("PICO BATTERY MONITOR V1.1 - 20231106 J.Bradshaw\r\n");
  EEPROM.begin(256);          // must be called before using the 4K chunk of flash at the end of flash space 

  //analogReadResolution(12);
   analogWriteFreq(1000);     // set freq output to 1KHz

  pinMode(GND_PIN, INPUT);
  pinMode(VCC_PIN, INPUT);

  analogReadResolution(ADC_RES);  

    // find initial condition state
   battVolts = (float)(pico.analogCRead(VIN_PIN, 10)/4095.0 * 3.3) * gainCalc;  // 10M / 1M voltage divider * adc pin voltage
   battVoltsLast = battVolts;
   if(battVolts >= highAlarmVolts){
      dir = 1;
   }
   else if((battVolts < highAlarmVolts) && (battVolts >= criticalLowVolts)){
      dir = 0;
   }
   else if(battVolts < criticalLowVolts){
      dir = -1;
   }

  eeAddress = 0;

  Serial.printf("Reading EEPROM:\r\n");
  // read the eeprom memory
  EEPROM.get(eeAddress, eepromData);  
  delay(20);
  Serial.printf("DescStr=%s\r\n", eepromData.descriptionStr);  
  Serial.printf("Resistor Gain Factor= %7.2f\r\n", eepromData.gainCalc);
  Serial.printf("Alarm On Voltage= %7.2f\r\n", eepromData.highAlarmVolts);
  Serial.printf("Low Voltage Cutoff = %7.2f\r\n", eepromData.lowAlarmVolts);
  //delay(3000);
  //Serial.println(eepromData.absoluteCutoffVolts);
  
  // Has the eeprom ever been set?
//   if(!isinf(eepromData.highAlarmVolts) || !isinf(eepromData.lowAlarmVolts) || !isinf(eepromData.gainCalc)){
//    Serial.printf("Setting eeprom to default values");
//    delay(1000);
//    strcpy(eepromData.descriptionStr, "DeWalt Batt Tester\0"); // copies the second string passed to the function to the first function
//    eepromData.highAlarmVolts = highAlarmVolts;
//    eepromData.lowAlarmVolts = lowAlarmVolts;
//    eepromData.gainCalc = gainCalc;
//    EEPROM.put(eeAddress, eepromData);
//    delay(20);
//    EEPROM.commit();
//    delay(2000);
//    EEPROM.get(eeAddress, eepromData);  
//    delay(20);
//    Serial.printf("DescStr=%s\r\n", eepromData.descriptionStr);  
//    Serial.printf("Resistor Gain Factor= %7.2f\r\n", eepromData.gainCalc);
//    Serial.printf("Alarm On Voltage= %7.2f\r\n", eepromData.highAlarmVolts);
//    Serial.printf("Low Voltage Cutoff = %7.2f\r\n", eepromData.lowAlarmVolts);
//    delay(2000);
//  }
//  else{ // else update with eeprom stored value
//    highAlarmVolts = eepromData.highAlarmVolts;
//    lowAlarmVolts = eepromData.lowAlarmVolts;
//    gainCalc = eepromData.gainCalc;
//  }
  
}

// core 1
void setup1() {

}

void loop1() {

}

void loop() {
  int lv_cutoff_cnt = 0;
  
  //int adc0 = analogRead(VIN_PIN);
    float battVolts = 0.0;
    //battVolts = ((float)analogRead(VIN_PIN)/4095.0 * 3.256) * 11.0;  // 10M / 1M voltage divider * adc pin voltage
    //for(int i=0;i<100;i++){
      battVolts = (float)(pico.analogCRead(VIN_PIN, 10)/4095.0 * 3.3) * gainCalc;  // 10M / 1M voltage divider * adc pin voltage
    //}
    //battVolts /= 100.0;    

    // sort the condition based on the battery voltage
    if(battVolts >= highAlarmVolts){     
      NoAlarmHysVolts = -0.1;  // Add small negative hysteresis volts for detection until crossed        

      dir = 1;
      analogWrite(BUZZER_PIN, 0);     // turn off PWM 
      digitalWrite(D2, HIGH); // turn on the P-Channel Power FET (Source input voltage to the load)
      buzzerState = 0;
       buzzerCrit = 0;
       millisLedDelay = 300;
    }
    
    if((battVolts < (highAlarmVolts + NoAlarmHysVolts)) && (battVolts >= (lowAlarmVolts + NormAlarmHysVolts))){
      if(dir == 1){ // if the normal alarm volts is negative, going down
        NoAlarmHysVolts = .2; // add .1 volts to No Alarm detect hysteresis        
      }
      if(dir == -1){ //going up
        NormAlarmHysVolts = -.2; // add .1 volts to No Alarm detect hysteresis         
      }
      dir = 0;
//      NormAlarmHysVolts = .2;  // Add small negative hysteresis volts to Normal Alarm crossover detection
      analogWriteFreq(1000);     // set freq output to 1KHz
      buzzerCrit = 0;      
      millisLedDelay = 200;
      switch(buzzerState){
        case 1:
          analogWrite(BUZZER_PIN, 100);        
          millisLastBuzzer = millis();
          buzzerTime = 500;   //half second beep
          buzzerState = 2;
          break;
        case 2:
          if(millis() > millisLastBuzzer + buzzerTime){
              millisLastBuzzer = millis();
              analogWrite(BUZZER_PIN, 0);
              buzzerTime = (int)(((battVolts - lowAlarmVolts) / (highAlarmVolts - lowAlarmVolts)) * 5000); // wait percentage of 5 seconds
              buzzerState = 3;
          }
          break;
        case 3:
          if(millis() > millisLastBuzzer + buzzerTime){
              buzzerState = 1;
          }
          break;
       default:
        buzzerState = 1;
        break;
      }                      
    }
    
    if((battVolts < (lowAlarmVolts + NormAlarmHysVolts)) && (battVolts >= criticalLowVolts)){
      if(dir == 0){
        NormAlarmHysVolts = .2; // add .1 volts to No Alarm detect hysteresis 
      }
      dir = -1;
      //digitalWrite(D2, LOW); // turn off the P-Channel Power FET (Pinch off the voltage source to the load)
      analogWriteFreq(2000);     // set freq output to 2KHz      
      millisLedDelay = 100;
      buzzerCrit = 1;
      switch(buzzerState){
        case 4:
          analogWrite(BUZZER_PIN, 512);         // Duty Cycle for Loudness
          millisLastBuzzer = millis();
          buzzerTime = 500;   //half second beep
          buzzerState = 5;
          break;
        case 5:
          if(millis() > millisLastBuzzer + buzzerTime){
              millisLastBuzzer = millis();
              analogWrite(BUZZER_PIN, 0);
              buzzerTime = 1000; // wait 5 seconds
              buzzerState = 6;
          }
          break;
        case 6:
          if(millis() > millisLastBuzzer + buzzerTime){
              buzzerState = 4;
          }
          break;
       default:
        buzzerState = 4;
        break;
      }       
    }

  if(battVolts < criticalLowVolts){
    if(buzzerCrit != 2){
      Serial.printf("Critical Voltage level reached, starting counter!\r\n");
      buzzerCrit = 2;
      criticalCounter=0;
      critCntLastTime = millis();
    }
    else{      
      if(millis() - critCntLastTime > 1000){
        critCntLastTime = millis();
        criticalCounter++;
        analogWriteFreq(2000);
        analogWrite(BUZZER_PIN, 512);     // turn on PWM 
        digitalWrite(led, !digitalRead(led));
        delay(500); 
        analogWrite(BUZZER_PIN, 0);     // turn off PWM 
        digitalWrite(led, !digitalRead(led));        
        Serial.printf("\r\nCritical Timeout left in seconds = %d\r\n", CRIT_TIMOUT_SEC - criticalCounter);
        if(criticalCounter > CRIT_TIMOUT_SEC){ // seconds until timeout and shutoff
          //sleep
          //delay(2000);
          Serial.printf("\r\nPROGRAM TRAP\r\n");
          digitalWrite(D2, LOW); // turn off the P-Channel Power FET (Pinch off the voltage source to the load)
          analogWrite(BUZZER_PIN, 0);     // turn off PWM
          digitalWrite(led, LOW); //  turn off LED

          // ----------------------------------------------------------------------------------------
          // under normal operation the program would trap here by turning itself off, but if USB connected it will stay on
          int exitTrap = 1;
          buzzerCrit = 3;
          buzzerState = 5;
          while(exitTrap){            
            digitalWrite(led, HIGH); //  turn on LED
            delay(300);
            digitalWrite(led, LOW); //  turn off LED
            delay(700);   
            battVolts = (float)(pico.analogCRead(VIN_PIN, 10)/4095.0 * 3.3) * gainCalc;
            if(battVolts > criticalLowVolts){
              exitTrap = 0;
            }
            //Serial.printf("\r\nProgram trap ");
            Serial.printf("PICOBATMON,T:%d,dVdT:%.3f,Bv:%.2f,Bcrit:%d,Bstate:%d,buzzT:%d,Halarm:%.2f,Lalarm:%.2f\r\n", millis(), dVdT, battVolts, buzzerCrit, buzzerState, buzzerTime, highAlarmVolts, lowAlarmVolts);
          }
          //analogWriteFreq(200);     // set freq output to 2KHz
          analogWriteFreq(2000);
          for(int i = 0;i<3;i++){    
            analogWrite(BUZZER_PIN, 512);     // turn on PWM 
            digitalWrite(led, !digitalRead(led));
            delay(200); 
            analogWrite(BUZZER_PIN, 0);     // turn off PWM 
            digitalWrite(led, !digitalRead(led));
            delay(100);    
          } 
          // !!! If it did not turn itself off then the USB port is supplying power to the microcontroller and 
          //  the program will simply start at the top of the loop again...
        }
      }        
    }
    buzzerCrit=2;
  }

  // LED flash rate
  if(millis() > millisLastLed + millisLedDelay){
    millisLastLed = millis();          
    
    digitalWrite(led, !digitalRead(led));
  } 

  // Update the serial output
  if(millis() > millisLastSerial + millisSerialDelay){
    dVdT = (battVolts - battVoltsLast) * ((millis()-millisLastSerial)/1000.0);
    
    millisLastSerial = millis();
    battVoltsLast = battVolts;    

    //Serial.printf("%d dVdT=%7.3f Battery Voltage = %7.2f buzzerCrit=%d state=%d buzzTime=%d HighAlarmV=%7.2f LowAlarmV=%7.2f\r\n", millis(), dVdT, battVolts, buzzerCrit, buzzerState, buzzerTime, highAlarmVolts, lowAlarmVolts);
    Serial.printf("PICOBATMON,T:%d,dVdT:%.3f,Bv:%.2f,Bcrit:%d,Bstate:%d,buzzT:%d,Halarm:%.2f,Lalarm:%.2f\r\n", millis(), dVdT, battVolts, buzzerCrit, buzzerState, buzzerTime, highAlarmVolts, lowAlarmVolts);
  }
  
// check is a serial character was received
  if(Serial.available()>0){
    char str[30];

    if(Serial.peek() == '?'){ // ? display commands
        int dummy = Serial.read();  
        Serial.printf(" SINGLE CHARACTER COMMANDS  \r\n");
        Serial.printf("'?' - display this menu\r\n");
        delay(10);
        Serial.printf(" STRING COMMANDS (followed by carriage return '\\r') \r\n");
        Serial.printf("voltsIn F1\\r - When supplying a known voltage F1(float), auto calculates the gain from voltage divider\r\n");
        delay(10);
        Serial.printf("eepromr\\r - read the current values in the EEPROM\r\n");
        Serial.printf("eepromw S1 F1 F2\\r - write to EEPROM, S1 is string description, F1 high alarm volts float, F2 low alarm volts float\r\n");
        Serial.printf("setgain F1\\r - Set the Gain of the voltage divider (around 11.0)\r\n");
        Serial.printf("adccal\\r - Calibrate the ADC\r\n");
        Serial.printf("shutdown\\r - Shutdown the voltage output of the monitor. Requires a following 'y'.\r\n");

        // clar out any additional characters in the Serial buffer
        while(Serial.available()){
          char c = Serial.read();
        }

        // wait for single character press to exit menu
        while(!Serial.available()); 
        char c = Serial.read();       // expunge new character entered in buffer
      }

    // extract string commands
    Serial.readBytesUntil('\r', str, 30);
    
    if(strncmp(str, "voltsIn ", 8) == 0){
      float voltsIn;
      int numvals = sscanf(str, "voltsIn %f\r", &voltsIn);
      if(numvals == 1){                
        if(voltsIn < 0.0)
          voltsIn = 0.0;
        if(voltsIn > 100.0)
          voltsIn = 100.0;
          
        Serial.printf("\r\nVoltsIn = %7.3f\r\n", voltsIn);     
        gainCalc = voltsIn / ((float)(pico.analogCRead(VIN_PIN, 10)/4095.0) * 3.3);        
        Serial.printf("\r\nGain Calc = %7.3f\r\n", gainCalc);     
        delay(2000);
        MemObject eepromData;
        eeAddress = 0;   //Location we want the data to get
        eepromData.gainCalc = gainCalc;
        EEPROM.put(eeAddress, eepromData);

        EEPROM.commit();  // Writes the updated data to flash, so next reboot it will be readable
      }
      else{
        Serial.printf("ERROR - num parameters scanned is %d", numvals);
        delay(2000);
      }
    }// if received command was voltsIn

    if(strncmp(str, "eepromr", 7) == 0){      
        MemObject eepromData;
        eeAddress = 0;   //Location we want the data to get
        EEPROM.get(eeAddress, eepromData);
        //Serial.printf(MOVE_L_C, 20, 0);
        Serial.println(eepromData.descriptionStr);
        Serial.println(eepromData.gainCalc);
        Serial.println(eepromData.highAlarmVolts);
        Serial.println(eepromData.lowAlarmVolts);
        //Serial.println(eepromData.absoluteCutoffVolts);

        delay(3000);
      }// eeprom command

      // write eeprom command
      if(strncmp(str, "eepromw ", 8) == 0){
        char tempStr[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        //float tempgainCalc = 11.0;
        float temphighAlarmVolts = 16.5;
        float templowAlarmVolts = 16.0;
        //float tempabsoluteCutoffVolts = 15.6;
        
        int numvals = sscanf(str, "eepromw %s %f %f", tempStr, &temphighAlarmVolts, &templowAlarmVolts);

        Serial.printf("\r\nSerial read - eepromw %s %f %f\r\n", tempStr, temphighAlarmVolts, templowAlarmVolts);        

        Serial.printf("Old eeprom vals are :\r\n");
        eeAddress = 0;

        MemObject eepromDatar;  // create an object for the eeprom data
        EEPROM.get(eeAddress, eepromDatar);
        Serial.println(eepromDatar.gainCalc);    //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.
        Serial.println(eepromDatar.descriptionStr);       
        Serial.println(eepromDatar.highAlarmVolts);
        Serial.println(eepromDatar.lowAlarmVolts);        
        
        if(numvals == 3){ // Successfully scanned 3 values
          MemObject eepromData;  // create an object for the eeprom data
          
          strcpy(eepromData.descriptionStr, tempStr); // copies the second string passed to the function to the first function
          eepromData.gainCalc = gainCalc;
          eepromData.highAlarmVolts = temphighAlarmVolts;
          eepromData.lowAlarmVolts = templowAlarmVolts;
        
          EEPROM.put(eeAddress, eepromData);
  
          Serial.printf(MOVE_L_C, 35, 0);
          eeAddress = 0;   //Location we want the data to get
          EEPROM.get(eeAddress, eepromData);
          Serial.printf("Number of parameters parsed was %d\r\n", numvals);
          Serial.println(eepromData.gainCalc);    //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.
          Serial.println(eepromData.descriptionStr);       
          Serial.println(eepromData.highAlarmVolts);
          Serial.println(eepromData.lowAlarmVolts);

          EEPROM.commit();  // Writes the updated data to flash, so next reboot it will be readable
          
          highAlarmVolts = eepromData.highAlarmVolts;
          lowAlarmVolts = eepromData.lowAlarmVolts;
        }
        else{
          Serial.printf(MOVE_L_C, 35, 0);
          Serial.printf("Number of parameters parsed was %d", numvals);
          delay(2000);
        }
      } // eepromw command

      // set gain value (resistor voltage divider
      if(strncmp(str, "setgain ", 8) == 0){
        float tempSetGain = 0.0;
        int numvals = sscanf(str, "setgain %f", &tempSetGain);

        Serial.printf("\r\nSerial read - setgain %f\r\n", tempSetGain);

        Serial.printf("Old eeprom gain was :\r\n");
        eeAddress = 0;

        MemObject eepromDatar;  // create an object for the eeprom data
        EEPROM.get(eeAddress, eepromDatar);
        Serial.println(eepromDatar.gainCalc);    //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.

        MemObject eepromData;  // create an object for the eeprom data
          
        eepromData.gainCalc = tempSetGain;

        eeAddress = 0;
        EEPROM.put(eeAddress, eepromData);

        EEPROM.commit();  // Writes the updated data to flash, so next reboot it will be readable        
          
        eeAddress = 0;
        
        EEPROM.get(eeAddress, eepromDatar);
        Serial.println(eepromDatar.gainCalc);    //This may print 'ovf, nan' if the data inside the EEPROM is not a valid float.
        Serial.println(eepromDatar.descriptionStr);       
        Serial.println(eepromDatar.highAlarmVolts);
        Serial.println(eepromDatar.lowAlarmVolts); 

          gainCalc = tempSetGain;
      }
  // calibrate the on-board ADC
  if(strncmp(str, "adccal", 6) == 0){ 
      Serial.print("Offset Values: ");
      pico.returnCalibrationValues();
    
      Serial.print("Uncalibrated GND: ");
      Serial.println(pico.analogRead(GND_PIN));
      Serial.print("Calibrated GND: ");
      Serial.println(pico.analogCRead(GND_PIN,10));
    
      Serial.print("Uncalibrated VCC: ");
      Serial.println(pico.analogRead(VCC_PIN));
      Serial.print("Calibrated VCC: ");
      Serial.println(pico.analogCRead(VCC_PIN,10));
      Serial.println();
    
      delay(1000);
    // Calibrate ADC using an average of 5000 measurements
    pico.calibrateAdc(GND_PIN, VCC_PIN, 5000);
    
    PicoAnalogCorrection pico(ADC_RES, pico.analogCRead(GND_PIN,10), pico.analogCRead(VCC_PIN,10));    
  }        

// shutdown the output 
  if(strncmp(str, "shutdown", 8) == 0){
    Serial.print("Are you sure you want to shutdown?(y/n)");
    while(!Serial.available());
    char c = 0;
    c = Serial.read();

    if(c == 'y' || c == 'Y'){
      Serial.print("Shutting down in 30 seconds!\r\n");
      analogWriteFreq(2000);
      for(int i=30;i>0;i--){
        analogWrite(BUZZER_PIN, 512);     // turn on PWM 
        digitalWrite(led, !digitalRead(led));
        delay(200); 
        analogWrite(BUZZER_PIN, 0);     // turn off PWM
        delay(800);
        Serial.printf("Shutting down in %d seconds!\r\n", i);
      }
      
      digitalWrite(D2, LOW); // Shut OFF the whole system
      delay(500);
    }
    else{
      Serial.print("Exiting shutdown sequence\r\n");
    }
  }
  
    //clear out any garbage in buffer
   while(Serial.available()>0){
      char c = Serial.read();
   }   
  }//serial available

  delay(20);    
  
} // loop
