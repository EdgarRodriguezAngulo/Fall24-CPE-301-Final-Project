//Author: Edgar Rodriguez-Angulo
//Date Created: 11/30/2024
/*
Purpose: Final project for CPE 301; make a primitive swamp cooler that
can monitor water levels, environment temp and humidity, display the
temp and humidity to an LCD display, allow the user to adjust the 
angle of an output vent, allow the user to turn the cooler on or off,
and record the date and time when the fan motor is turned off
*/

#define RDA 0x80
#define TBE 0x20

//Pin manipulation
#define SETHIGH(address, pinNum) address |= (0x01 << pinNum);
#define SETLOW(address, pinNum) address &= ~(0x01 << pinNum);
#define PINREAD(address, pinNum) (address & (1 << pinNum)) != 0;

//Stepper
#include <Stepper.h>
const int stepsPerRevolution = 2038;
Stepper TheStepper = Stepper(stepsPerRevolution, 39, 41, 43, 45);
//1N1=39, 1N4=45

//LCD Display
#include <LiquidCrystal.h>
const int RS = 22, EN = 24, D4 = 3, D5 = 4, D6 = 5, D7 = 6;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
bool displayData=false;

//Date and time recording
#include <Wire.h>
#include <RTClib.h>
RTC_DS1307 rtc;
DateTime now;

String date_time_to_str(DateTime obj){
  String output = String(obj.year())+"-"+ 
                  String(obj.month())+"-"+ 
                  String(obj.day())+" "+ 
                  String(obj.hour())+":"+ 
                  String(obj.minute())+":"+ 
                  String(obj.second());
  return output;
}

//Global vars
bool startButton=false;
bool resetButton=false;
bool stopButton=false;
bool readData=true;
bool readButtons=true;

//Serial Transmission
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

//ADC Registers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//Timers
volatile unsigned char *myTCCR1A  = 0x80;
volatile unsigned char *myTCCR1B  = 0x81;
volatile unsigned char *myTCCR1C  = 0x82;
volatile unsigned char *myTIMSK1  = 0x6F;
volatile unsigned char *myTIFR1   = 0x36;
volatile unsigned int  *myTCNT1   = 0x84;

//GPIO
//Port H
volatile unsigned char* port_h = (unsigned char*) 0x102;
volatile unsigned char* ddr_h = (unsigned char*) 0x101;
volatile unsigned char* pin_h = (unsigned char*) 0x100;
//Port B
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23;
//Port C
volatile unsigned char* port_c = (unsigned char*) 0x28;
volatile unsigned char* ddr_c = (unsigned char*) 0x27;
volatile unsigned char* pin_c = (unsigned char*) 0x26;

//ISR
volatile unsigned long overflowCounter=0;
volatile unsigned long delayCounter=0;
volatile unsigned long buttonCounter=false;
const unsigned long overflowsPerMin=60000/4.1;
const unsigned long overflowsPer500ms=500/4.1;

const int startPin=11;
const int interruptNumber=digitalPinToInterrupt(startPin);

ISR(TIMER1_OVF_vect)
{
  overflowCounter++;
  delayCounter++;
  buttonCounter++;

  if(overflowCounter>=overflowsPerMin)
  {
    overflowCounter=0;
    readData=true;
  }
  
  if(buttonCounter>=overflowsPer500ms)
  {
    readButtons=true;
    buttonCounter=true;
  }
}

void customDelay(unsigned long duration)
{
  unsigned long delayThreshold=duration/4.1;
  delayCounter=0;
  while(delayCounter<delayThreshold){}
}

//Sensors
unsigned int tempHumidity;
unsigned int waterLevel;
static const unsigned waterThreshold=700;
static const unsigned tempHumidityThreshold=650;
unsigned int potPos;
unsigned int desiredPos;
unsigned int currentPos;
void moveToPosition()
{
  int sensitivity=100;
  if(currentPos < (desiredPos-sensitivity) || currentPos>(desiredPos+sensitivity))
  {
    stepperReport();
  }
  int stepsToMove=desiredPos-currentPos;
  TheStepper.step(stepsToMove);
  currentPos=desiredPos;
}
void controlFan(bool state)
{
  //Fan control:
  //IN4: 31 (port c, 6), IN3: 33 (port c, 4), ENB1: 35 (port c, 2)
  //IN3 HIGH, IN4 LOW=Forward
  //IN3 LOW, IN4 HIGH=Backwards
  //ENB HIGH=ON, ENB LOW=OFF
  if(state)
  {
    SETLOW(*port_c, 6); //IN4
    SETHIGH(*port_c, 4); //IN3
    SETHIGH(*port_c, 2); //ENB
  }
  else{
    SETLOW(*port_c, 2); //ENB
  }
}

//States
/*
Idle state = 1
Running state = 2
Disabled state = 3
Error state = 4
*/
//Red = 10 (port B, 4), Blue = 9 (port H, 6), Yellow = 8 (port H, 5), Green = 7 (port H, 4)
class State
{
  private:
    int currentState=0;
  public:
    const int getState()
    {
      return currentState;
    }
    
    void enterState(const int& state)
    {
      stateChangeReport(currentState, state);
      currentState=state;
      switch(state)
      {
        case 1: //Idle state
          SETHIGH(*port_h, 4); //Green
          SETLOW(*port_h, 5);
          SETLOW(*port_h, 6);
          SETLOW(*port_b, 4);
          
          displayData=true;
          readButtons=true;
        break;
        case 2: //Running state
          SETHIGH(*port_h, 6);//Blue
          SETLOW(*port_b, 4);
          SETLOW(*port_h, 5);
          SETLOW(*port_h, 4);

          displayData=true;
          controlFan(true);
          readButtons=true;
        break;
        case 3: //Disabled state
          SETHIGH(*port_h, 5); //Yellow
          SETLOW(*port_h, 4);
          SETLOW(*port_h, 6);
          SETLOW(*port_b, 4);

          displayData=false;
          readButtons=true;
          lcd.clear();
          controlFan(false);
        break;
        case 4: //Error state
          SETHIGH(*port_b, 4); //Red
          SETLOW(*port_h, 5);
          SETLOW(*port_h, 6);
          SETLOW(*port_h, 4);

          displayData=false;
          displayError();
          controlFan(false);
        break;
        default:
          SETLOW(*port_b, 4);
          SETLOW(*port_h, 4);
          SETLOW(*port_h, 5);
          SETLOW(*port_h, 6);

          //Serial.println("SUPER ERROR STATE");
        break;
      }
    }

    void update()
    {
      switch(currentState)
      {
        case 1: //Idle state
            moveToPosition();
            if(stopButton)
            {
              enterState(3);
            }
            else if(waterLevel<waterThreshold)
            {
              enterState(4);
            }
            else if(tempHumidity>tempHumidityThreshold)
            {
              enterState(2);
            }

        break;
        case 2: //Running state
          moveToPosition();
          if(stopButton)
          {
            enterState(3);
          }
          else if(tempHumidity<=tempHumidityThreshold)
          {
            enterState(1);
          }
          else if(waterLevel<waterThreshold)
          {
            enterState(4);
          }

        break;
        case 3: //Disabled state
          moveToPosition();
          if(startButton)
          {
            enterState(1);
          }

        break;
        case 4: //Error state
          if(stopButton)
          {
            enterState(3);
          }
          if(resetButton)
          {
            enterState(1);
          }

        break;
        default:
        /*
          Serial.println("NO UPDATE POSSIBLE");
          Serial.print("CURRENT STATE: ");
          Serial.println(currentState);
        */
        break;
        startButton=false;
        stopButton=false;
        resetButton=false;
      }
    }
};

State Cooler;

//Main functions
void setup() {
  U0init(9600);
  //Setting up UART
  adc_init();
  //Setting up the ADC
  lcd.begin(16, 2);
  //Initializing the LCD display with 16 columns and 2 rows
  gpio_init();
  isr_setup();
  setup_timer_regs();
  stepper_init();
  rtc_init();
  Cooler.enterState(3);
}

void loop() {
  //Data reading
  if(readData || overflowCounter>=overflowsPerMin)
  {
    tempHumidity=adc_read(0);//Read the Temp and humiditiy

    waterLevel=adc_read(1);//Read the water levels
    readData=false;
  }

  //Read potentiometer for the position of the "vent"
  potPos=adc_read(2);
  desiredPos=map(potPos, 0, 1023, 0, stepsPerRevolution);

  //Display
  if(displayData)
  {
    display(waterLevel, tempHumidity);
  }

  //Buttons
  //Reset the buttons
  if(readButtons || buttonCounter>=overflowsPer500ms)
  {
    //Leftmost/Start = 11 (port B, 5), Middle/Stop = 12 (port B, 6), Rightmost/Reset = 13 (port B, 7)
    startButton=PINREAD(*pin_b, 5)
    stopButton=PINREAD(*pin_b, 6);
    resetButton=PINREAD(*pin_b, 7);
    readButtons=false;
  }

  //Date and Time
  now=rtc.now();

  Cooler.update();

  customDelay(50);
}

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

void stepper_init()
{
  TheStepper.setSpeed(100);
  desiredPos=currentPos;
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void gpio_init()
{
  //Output LEDs
  //Red = 10 (port B, 4), Blue = 9 (port H, 6), Yellow = 8 (port H, 5), Green = 7 (port H, 4)
  SETHIGH(*ddr_b, 4); //Red
  SETHIGH(*ddr_h, 5); //Yellow
  SETHIGH(*ddr_h, 4); //Green
  SETHIGH(*ddr_h, 6); //Blue

  //Initialize all LEDs to off
  SETLOW(*port_b, 4); //Red
  SETLOW(*port_h, 5); //Yellow
  SETLOW(*port_h, 4); //Green
  SETLOW(*port_h, 6); //Blue

  //Input Buttons
  //Leftmost/Start = 11 (port B, 5), Middle/Stop = 12 (port B, 6), Rightmost/Reset = 13 (port B, 7)
  SETLOW(*ddr_b, 5); //Start
  SETLOW(*ddr_b, 6); //Stop
  SETLOW(*ddr_b, 7); //Reset

  //Initialize pull-up resistors
  SETHIGH(*port_b, 5); //Start
  SETHIGH(*port_b, 6); //Stop
  SETHIGH(*port_b, 7); //Reset

  //Fan output
  //Controls:
  //IN4 = 31 (port C, 6), IN3 = 33 (port C, 4), ON/OFF/SPEED = 35 (port C, 2)
  SETHIGH(*ddr_c, 6); //IN4
  SETHIGH(*ddr_c, 4); //IN3
  SETHIGH(*ddr_c, 2); //ON/OFF/SPEED

}

void isr_setup()
{
  pinMode(startPin, INPUT_PULLUP);
  attachInterrupt(interruptNumber, handleStartPress, FALLING);
}

void handleStartPress()
{
  startButton=true;
  //Serial.println("START BUTTON PRESSED");
}

void setup_timer_regs()
{
  //Disable global interrupts
  cli();

  //Timer control registers
  *myTCCR1A= 0x00;
  *myTCCR1B= 0X00;
  *myTCCR1C= 0x00;

  //Reset TOV flag
  *myTIFR1 |= 0x01;

  //Enable TOV interrupt
  *myTIMSK1 |= 0x01;

  //Start Timer
  *myTCCR1B |= 0x01;

  //Enable global interrupts
  sei();
}

void rtc_init()
{
  Wire.begin();
  rtc.begin();
  String rtcError="RTC IS NOT RUNNING!\n";
  if(!rtc.isrunning())
  {
    for(int i=0;i<rtcError.length();i++)
    {
      U0putchar(rtcError[i]);
    }
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

unsigned char U0getchar()
{
  return *myUDR0;
}

void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

void display(int waterLevel, int tempHumid)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Water: ");
  lcd.print(waterLevel);
  lcd.setCursor(0, 1);
  lcd.print("T/H: ");
  lcd.print(tempHumid);
}

void displayError()
{
  lcd.clear();
  lcd.setCursor(0 ,0);
  lcd.print("ERROR: WATER");
  lcd.setCursor(0, 1);
  lcd.print("LOW");
}

String createReport(String reporting, String dateTime)
{
  String report=reporting+" at "+dateTime;
  return report;
}

void stepperReport()
{
  String newPos="NEW STEPPER POS: "+String(desiredPos);
  String dateTime=date_time_to_str(now);
  String report=createReport(newPos, dateTime);
  serialReport(report);
}

void stateChangeReport(int current, int next)
{
  //States
  /*
  Idle state = 1
  Running state = 2
  Disabled state = 3
  Error state = 4
  */
  char* states[5]={"SUPER ERROR", "IDLE", "RUNNING", "DISABLED", "ERROR"};
  String currState=states[current];
  String nextState=states[next];
  String changeCurrent="STATE CHANGING FROM " + currState + " to " + nextState;
  String dateTime=date_time_to_str(now);
  String report=createReport(changeCurrent, dateTime);
  serialReport(report);
}

void serialReport(String report)
{
  for(int i=0;i<report.length();i++)
  {
    U0putchar(report[i]);
  }
  U0putchar('\n');
}