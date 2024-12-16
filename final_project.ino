/*
* Trevor Kaufman and Kathleen Scafidi
* Final Project - Water Cooler
* December 14 2024
*/

#include <Stepper.h>       //library for Stepper motor
#include <LiquidCrystal.h> //library for LCD
#include <RTClib.h>        //library for Real Time Clock
#include <DHT.h>           //library for DHT11 sensor

unsigned char* PORT_A = (unsigned char*) 0x22;
unsigned char* DDR_A = (unsigned char*) 0x21;
volatile unsigned char* PIN_A = (unsigned char*) 0x20;

unsigned char* PORT_C = (unsigned char*) 0x28;
unsigned char* DDR_C = (unsigned char*) 0x27;
volatile unsigned char* PIN_C = (unsigned char*) 0x26;

unsigned char* PORT_D = (unsigned char*) 0x2B;
unsigned char* DDR_D = (unsigned char*) 0x2A;
volatile unsigned char* PIN_D = (unsigned char*) 0x29;

unsigned char* PORT_E = (unsigned char*) 0x2E;
unsigned char* DDR_E = (unsigned char*) 0x2D;
volatile unsigned char* PIN_E = (unsigned char*) 0x2C;

unsigned char* PORT_G = (unsigned char*) 0x34;
unsigned char* DDR_G = (unsigned char*) 0x33;
volatile unsigned char* PIN_G = (unsigned char*) 0x32;

unsigned char* PORT_H = (unsigned char*) 0x102;
unsigned char* DDR_H = (unsigned char*) 0x101;
volatile unsigned char* PIN_H = (unsigned char*) 0x100;

//serial ports
#define TBE 0x20 
#define RDA 0x80
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;

//state definitions
enum system_state { DISABLED, IDLE, ERROR, RUNNING };
volatile system_state current_state = DISABLED;

//RTC object
RTC_DS3231 rtc;

//fan motor definition
int speed_control_pin = 5; //PE3
int dir1 = 4;//PG5
int dir2 = 3;//PE5
int motor_speed = 90;


//steps to complete a rotation
const int steps_per_rev = 200;
Stepper stepper_motor = Stepper(steps_per_rev, 8, 10, 9, 11);
unsigned long motor_step_duration = 5000; //step duration in milliseconds
unsigned long motor_step_start_time = 0;
bool motor_step_in_progress = false;

//define pins and devices
#define START_BUTTON_PIN 2 //interrupt
#define STOP_BUTTON_PIN 32
#define RESET_BUTTON_PIN 34
#define YELLOW_LED_PIN 22
#define GREEN_LED_PIN 26
#define BLUE_LED_PIN 24
#define RED_LED_PIN 28
#define WATER_SENSOR_PIN A5 //water level sensor
#define FAN_PIN 5 //fan motor 
#define TEMP_THRESHOLD 21.5 //threshold  
#define POTENTIOMETER_PIN A0 //potentiometer
#define DHTPIN 6      //define the pin number
#define DHTTYPE DHT11 //define the sensor type

DHT dht(DHTPIN, DHTTYPE);  

const int RS = 27, EN = 25, D4 = 37, D5 = 35, D6 = 33, D7 = 31;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7); //LCD pins

volatile unsigned long last_debounce = 0; //most recent ISR trigger
const unsigned long delay_debounce = 200; //delay in milliseconds

int water_level = 0;

//function decalrations
void pressStart();
void pressStop();
void pressReset();
void adc_init();
uint16_t adc_read(uint8_t channel);
void switchState();
void updateTemp();
void updateFan();
void motorControl();
void testInterrupt();


void setup() {

  U0init(9600);

  //fan motor
  *DDR_E |= 0x08; //set speed_control_pin(PE3) to output
  *DDR_G |= 0x20;  //set dir1(PG5) to output 
  *DDR_E |= 0x20; //set dir2(PE5) to output
  lcd.begin(16, 2); //columns and rows for LCD
  *DDR_E |= 0x08; //set FAN_PIN (PE3) to output

  *DDR_E &=0xEF; //set START_BUTTON_PIN (PE4) to input 
  *PORT_E |= 0x10; //enable pullup resistor 

  attachInterrupt(digitalPinToInterrupt(START_BUTTON_PIN), pressStart, CHANGE);

  *DDR_C &= 0xDF; //set STOP_BUTTON_PIN (PC5) to input 
  *PORT_C |= 0x20; //enable the pullup resistor
  *DDR_A |= 0x01; //set YELLOW_LED_PIN (PA0) to output  
  *DDR_A |= 0x10; //set GREEN_LED_PIN (PA4) to output
  *DDR_A |= 0x04; //set BLUE_LED_PIN (PA2) to output
  *DDR_A |= 0x40; //set RED_LED_PIN (PA6) to output
  *DDR_H |=  0x08; //sensor input PH 3
  *PORT_A |=0x01; //set YELLOW_LED_PIN (PA0) High
  *PORT_A &= 0xEF; //clear GREEN_LED_PIN (PA4) to low 
  *PORT_A &=0xFB; //clear BLUE_LED_PIN (PA2) Low
  *PORT_A &=0xBF; //clear RED_LED_PIN (PA6) Low
  *DDR_A |= 0x08;//set water sensor (PH3) to output

  dht.begin();
  lcd.begin(16, 2);
  lcd.print("System Ready");
  rtc.begin();
  adc_init();
}

void loop() {
  pressStop();
  pressReset();

  if (current_state == IDLE || current_state == RUNNING) {

    *PORT_H |=0x08; //set water sensor (PH3) HIGH
    water_level = adc_read(5);

    updateTemp();
    motorControl();
       
    if (water_level < 20) {
      current_state = ERROR;

      *PORT_A |= 0x40; //set RED_LED_PIN (PA6) HIGH
      *PORT_A &= 0xFE; //clear YELLOW_LED_PIN (PA0) LOW
      *PORT_A &= 0xEF; //clear GREEN_LED_PIN (PA4) LOW
      *PORT_A &= 0xFB;//clear BLUE_LED_PIN (PA2) LOW

      lcd.clear();
      lcd.print("ERROR: Low Water!");
    }

    *PORT_H &= 0xF7; //clear water sensor (PH3) LOW
  }

  //minimize bounce (no delay function used)
  unsigned long currentMillis = millis();
  static unsigned long previousMillis = 0;
  if (currentMillis - previousMillis >= 100) {
    previousMillis = currentMillis;
  }
}

//serial
void U0init(unsigned long U0baud)
{
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  *myUBRR0  = tbaud;
}

void pressStart() {
  static unsigned long lastInterruptTime = 0; //keep track of the last time the button was pressed
  unsigned long current_time = millis(); //get the current time

  if ((current_time - lastInterruptTime > delay_debounce) && ((*PIN_E & 0x10) == LOW)) { //check for debounce and button state
    current_state = IDLE; //change the state to IDLE

    *PORT_A |= 0x10; //set GREEN_LED_PIN (PA4) to HIGH
    *PORT_A &=0xFE; //clear YELLOW_LED_PIN (PA0) LOW
    *PORT_A &=0xBF; //clear RED_LED_PIN (PA6) LOW
    *PORT_A &=0xFB; //clear BLUE_LED_PIN (PA2) LOW

    lastInterruptTime = current_time; //update last interrupt time
  }
}

void pressStop() {
  static unsigned long lastInterruptTime = 0; //track the last time the button was pressed
  unsigned long current_time = millis(); //get the current time

  bool temp = (*PIN_C & 0x20); //read STOP_BUTTON_PIN (PC5)

  if ((current_time - lastInterruptTime > delay_debounce) && ((*PIN_C & 0x20) == LOW)) { //check for debounce and button state
    current_state = DISABLED; //change the state to IDLE

    *PORT_A &= 0xEF;//clear GREEN_LED_PIN (PA4) to LOW
    *PORT_A |= 0x01;//set YELLOW_LED_PIN (PA0) to HIGH
    *PORT_A &=0xBF; //clear RED_LED_PIN (PA6) LOW
    *PORT_A &=0xFB; //clear BLUE_LED_PIN (PA2) LOW

    lastInterruptTime = current_time; //update the last interrupt time
    lcd.clear();
    lcd.print("System Ready!");
  }
}

void pressReset() {
  static unsigned long lastInterruptTime = 0; //track the last time the button was pressed
  unsigned long current_time = millis(); //get the current time
  bool temp = (*PIN_C & 0x20);

  if ((current_time - lastInterruptTime > delay_debounce) && ((*PIN_C & 0x08) == LOW && current_state == ERROR)) { //check for debounce and button state
    current_state = IDLE; //change the state to IDLE

    *PORT_A |= 0x10;//set GREEN_LED_PIN (PA4) to HIGH
    *PORT_A &=0xFE; //clear YELLOW_LED_PIN (PA0) LOW
    *PORT_A &=0xBF; //clear RED_LED_PIN (PA6) LOW
    *PORT_A &=0xFB; //clear BLUE_LED_PIN (PA2) LOW
  
    DateTime now = rtc.now();
    lastInterruptTime = current_time; //update the last interrupt time
  }
}

void adc_init() {
  //set AVcc
  ADMUX = (1 << REFS0);
  //enable ADC and set prescaler to 128
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(uint8_t channel) {
  //select ADC channel with safety mask
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
  //start single conversion
  ADCSRA |= (1 << ADSC);
  //wait for the conversion
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

void switchState() {
  unsigned long current_time = millis();
  //check if the interrupt is too close to previous one
  if ((current_time - last_debounce) >= delay_debounce) {
    //toggle state once the delay passes
    if (current_state == DISABLED) {
      current_state = IDLE;
      *PORT_A &=0xFE; //clear YELLOW_LED_PIN (PA0) LOW
      *PORT_A |= 0x10;//set GREEN_LED_PIN (PA4) to HIGH
    } 
    else {
      current_state = DISABLED;
      *PORT_A |=0x01; //set YELLOW_LED_PIN (PA0) HIGH
      *PORT_A &= 0xEF;//clear GREEN_LED_PIN (PA4) to LOW
    }
    //update the last debounce time
    last_debounce = current_time;
  }
}

void testInterrupt() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 200) {
    lastInterruptTime = interruptTime;
  }
}

void updateTemp(){
  static unsigned long previousFanToggleTime = 0; //store the fan's most recent togle
  static unsigned long fanToggleInterval = 0; //store the fan's togle interval
  static bool fan_on = false; //fan state flag
  static unsigned long fanStartTime = 0; //fan's start time
  const unsigned long fanDuration = 5000; //fan's running time in milliseconds 

  float temperature = dht.readTemperature(); //temp in celsius
  float humidity = dht.readHumidity();

  //display all the values on the LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print(" C");

  lcd.setCursor(0,1);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print(" %");

  //check if the state should be IDLE or RUNNING
  if(current_state == IDLE && temperature > TEMP_THRESHOLD){
    current_state = RUNNING;

    *PORT_A &= 0xEF;//clear GREEN_LED_PIN (PA4) to LOW
    *PORT_A |=0x04; //set BLUE_LED_PIN (PA2) HIGH
    *PORT_A &=0xBF; //clear RED_LED_PIN (PA6) LOW
    *PORT_A &=0xFE; //clear YELLOW_LED_PIN (PA0) LOW

    //start the fan
    fan_on = true;
    *PORT_G &= 0xDF;//clear dir1 (PG5) to LOW
    *PORT_E |= 0x20;//set dir2 (PE5) to HIGH
  
  }
  else if (current_state == RUNNING && temperature <= TEMP_THRESHOLD) {
    current_state = IDLE;
    *PORT_A &=0xFB; //clear BLUE_LED_PIN (PA2) LOW
    *PORT_A &=0xBF; //clear RED_LED_PIN (PA6) LOW
    *PORT_A &=0xFE; //clear YELLOW_LED_PIN (PA0) LOW
    *PORT_A |= 0x10;//set GREEN_LED_PIN (PA4) to HIGH

    //stop the fan 
    fan_on = false;
  }

  if(fan_on){
    *PORT_E |= 0x08;//set speed_control_pin (PE3) to HIGH
  } else{
    *PORT_E &= 0xF7;//clear speed_control_pin (PE3) to LOW
  }

}

void motorControl() {
  if (current_state != DISABLED) { //see if system is disabled
    if (!motor_step_in_progress) {
      int potentiometer_val = adc_read(0); //get potentiometer value
      int motorSpeed = map(potentiometer_val, 0, 1023, 1, 15); //map potentiometer value to motor speed
      stepper_motor.setSpeed(motorSpeed); //set stepper motor speed
      stepper_motor.step(steps_per_rev); //rotate stepper motor
      stepper_motor.step(-steps_per_rev); //rotate stepper motor (opposite way)

      DateTime now = rtc.now();

      motor_step_start_time = millis();
      motor_step_in_progress = true;

    } else {
      unsigned long current_time = millis();
      if (current_time - motor_step_start_time >= motor_step_duration) {
        int potentiometer_val = adc_read(0); //get potentiometer value
        int motorSpeed = map(potentiometer_val, 0, 1023, 1, 10); //map potentiometer value to motor speed

        stepper_motor.setSpeed(motorSpeed); //set stepper motor speed
        stepper_motor.step(steps_per_rev); //keep rotating the stepper motor in the same direction
        stepper_motor.step(-steps_per_rev); //rotate stepper motor
        
        DateTime now = rtc.now();

        motor_step_start_time = current_time;
      }
    }
  } else {
    stepper_motor.setSpeed(0); //stop the motor when the system is disabled
  }
}