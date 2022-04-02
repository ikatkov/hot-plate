#include <Arduino.h>
#include <U8g2lib.h>
#include "Countimer.h"
#include <OneButton.h>
#include "PID.h"
#include <digitalWriteFast.h>
#include <EEPROM.h>
#include <NewTone.h>
#define TONE_PIN 4

static const uint8_t MAX6675_CS = 11;
static const uint8_t MAX6675_SO = 12;
static const uint8_t MAX6675_SCK = 10;
static const uint8_t SSR_PIN = 5;

static const uint8_t ARROW_UP = 0x47;     // arrow up
static const uint8_t ARROW_DOWN = 0x44;   // arrow down
static const uint8_t ARROW_RIGH = 0x46;   // arrow right
static const uint8_t ARROW_REFLOW = 0x56; // reflow

static const uint8_t MIN_TEMPERATURE = 0;
static const uint8_t MAX_TEMPERATURE = 255;

const char NUMBER[] PROGMEM = "%d";

static const int MENU_TIMEOUT = 5000;

static const uint8_t IDLE_STATE = 0;

static const uint8_t HEATING_STATE = 1;
static const uint8_t SOAKING_STATE = 2;
static const uint8_t REFLOW_STATE = 3;

static const uint8_t TEMP_SET_STATE = 4;
static const uint8_t PROFILE_SET_STATE = 5;
static const uint8_t THERMOCOUPLE_ERROR = 6;

const typedef struct ReflowProfile_t
{
  uint8_t preheatTemp;
  uint8_t preheatDurationSec;

  uint8_t soakTemp;
  uint8_t soakDurationSec;

  uint8_t reflowTemp;
  uint8_t reflowDurationSec;
} ReflowProfile;

// only one profile for now, for mg chemicals solder paste t3
// https://www.mgchemicals.com/downloads/tds/tds-4902p.pdf
ReflowProfile reflowProfiles[1] = {
    // t    s    t    s    t    s
   // {110, 100, 145, 155, 185, 195}}; // this is original T3 mgchemical
     {135, 120, 170, 200, 210, 250}}; //+25C from normal profile to compensate for temp gradient plate/pcb, +extra time 

static const uint8_t REFLOW_PROFILES_LEN = 1;
static const uint8_t MANUAL_PROFILE = 0;

uint8_t deviceState = IDLE_STATE;
uint8_t deviceProfile = MANUAL_PROFILE;
long menuStateAge = 0;

//=================================
// UI
//=================================
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
Countimer timer;

OneButton upButton = OneButton(8, true, true);
OneButton downButton = OneButton(7, true, true);
OneButton timerButton = OneButton(9, true, true);
OneButton xButton = OneButton(6, true, true);

//=================================
// PID settinga
//=================================
static const uint8_t PID_FUNCTIONAL_RANGE = 2;
double targetTemperature = 100;
double currentTemperature = 0;
double pidOutput;
double farKp = 5, farKi = 0, farKd = 7;
double farHOTKp = 8, farHOTKi = 0, farHOTKd = 14;
double nearKp = 10, nearKi = 1, nearKd = 8;
PID myPID(&currentTemperature, &pidOutput, &targetTemperature, farKp, farKi, farKd, DIRECT);
double profileTempSlope;
double profilePhaseStartingTemp;
uint8_t profileCompletedPhasesSec;
uint8_t profilePhaseMaxTemp;

//==================================
// Slow PWM
//==================================
volatile uint8_t pwmDutyCycle;
volatile uint8_t pwmCycleCounter = 0;

void eepromWrite();

void reDrawProfileSelectionScreen()
{
  Serial.print(F("reDrawProfileSelectionScreen"));
  char buffer[18];
  display.firstPage();
  do
  {
    // current profile
    if (deviceProfile == MANUAL_PROFILE)
    {
      display.setFont(u8g2_font_7x14B_tr);
      snprintf_P(buffer, 18, PSTR("Heat to %d C"), (int)targetTemperature);
      display.drawStr(25, 14, buffer);
      display.setCursor(25, 30);
      display.print(F("Manual control"));
    }
    else
    {
      display.setFont(u8g2_font_6x10_tf);
      snprintf_P(buffer, 18, PSTR("Heat   %d   %ds"), reflowProfiles[deviceProfile - 1].preheatTemp, reflowProfiles[deviceProfile - 1].preheatDurationSec);
      display.drawStr(25, 10, buffer);
      display.drawCircle(88, 4, 2);
      snprintf_P(buffer, 18, PSTR("Soak   %d   %ds"), reflowProfiles[deviceProfile - 1].soakTemp, reflowProfiles[deviceProfile - 1].soakDurationSec);
      display.drawStr(25, 21, buffer);
      display.drawCircle(88, 15, 2);
      snprintf_P(buffer, 18, PSTR("Reflow %d   %ds"), reflowProfiles[deviceProfile - 1].reflowTemp, reflowProfiles[deviceProfile - 1].reflowDurationSec);
      display.drawStr(25, 32, buffer);
      display.drawCircle(88, 26, 2);
    }
    display.setFont(u8g2_font_open_iconic_arrow_2x_t);
    display.drawGlyph(0, 16, ARROW_UP);
    display.drawGlyph(0, 32, ARROW_DOWN);
  } while (display.nextPage());
}

void reDrawTempSelectionScreen()
{
  char buffer[10];
  display.firstPage();
  do
  {
    // current temp
    display.setFont(u8g2_font_logisoso26_tn);
    snprintf_P(buffer, 7, NUMBER, int(targetTemperature));
    u8g2_uint_t position = display.drawStr(40, 30, buffer);
    display.drawCircle(40 + position + 5, 8, 3);

    display.setFont(u8g2_font_open_iconic_arrow_2x_t);
    display.drawGlyph(0, 16, ARROW_UP);
    display.drawGlyph(0, 32, ARROW_DOWN);
  } while (display.nextPage());
}

void reDrawThermocoupleError()
{
  // error beep
  NewTone(TONE_PIN, 1000, 50);
  delay(100);
  NewTone(TONE_PIN, 100, 50);
  delay(100);
  NewTone(TONE_PIN, 750, 100);

  display.firstPage();
  do
  {
    display.setFont(u8g2_font_7x14B_tr);
    display.setCursor(0, 10);
    display.print(F("Temperature sensor"));
    display.setCursor(20, 26);
    display.print(F("disconnected"));
  } while (display.nextPage());
}

void reDrawRegularScreen()
{
  char buffer[10];
  display.firstPage();
  do
  {
    if (deviceProfile != MANUAL_PROFILE)
    {
      // current profile
      display.setDrawColor(1);
      display.drawRBox(60, 20, 16, 12, 3);
      display.setDrawColor(0);
      display.setFont(u8g2_font_7x14B_tr);
      snprintf_P(buffer, 3, PSTR("P%d"), deviceProfile);
      display.drawStr(62, 31, buffer);
    }

    display.setDrawColor(1);
    // current temp
    display.setFont(u8g2_font_logisoso26_tn);
    snprintf_P(buffer, 7, NUMBER, int(round(currentTemperature)));
    u8g2_uint_t position = display.drawStr(0, 30, buffer);
    display.drawCircle(position + 5, 8, 3);

    // target temp
    display.setFont(u8g2_font_7x14B_tr);
    snprintf_P(buffer, 4, NUMBER, int(targetTemperature));
    position = display.drawStr(80, 14, buffer);
    display.drawCircle(80 + position + 2, 5, 2);

    if (deviceState == HEATING_STATE && currentTemperature < targetTemperature)
    {
      display.setFont(u8g2_font_open_iconic_arrow_2x_t);
      display.drawGlyph(111, 16, ARROW_UP);
    }
    else if (deviceState == HEATING_STATE && currentTemperature == targetTemperature)
    {
      display.setFont(u8g2_font_open_iconic_arrow_2x_t);
      display.drawGlyph(111, 16, REFLOW_STATE);
    }
    else if ((deviceState == HEATING_STATE || deviceState == IDLE_STATE) && currentTemperature > targetTemperature)
    {
      display.setFont(u8g2_font_open_iconic_arrow_2x_t);
      display.drawGlyph(111, 16, ARROW_DOWN);
    }
    else if (deviceState == SOAKING_STATE)
    {
      display.setFont(u8g2_font_open_iconic_arrow_2x_t);
      display.drawGlyph(111, 16, ARROW_RIGH);
    }
    else if (deviceState == REFLOW_STATE)
    {
      display.setFont(u8g2_font_open_iconic_arrow_2x_t);
      display.drawGlyph(111, 16, ARROW_REFLOW);
    }

    // second row
    display.setFont(u8g2_font_7x14B_tr);
    if (deviceState == IDLE_STATE)
    {
      display.setCursor(80, 30);
      display.print(F("Idle"));
    }
    else
    {
      // timer
      snprintf_P(buffer, 6, PSTR("%02d:%02d"), timer.getCurrentMinutes(), timer.getCurrentSeconds());
      display.drawStr(88, 30, buffer);
    }

  } while (display.nextPage());
}

void reDrawScreen()
{
  // beep every minute
  if (timer.getTotalSeconds() > 30 && timer.getTotalSeconds() % 60 == 0 && deviceState != IDLE_STATE)
  {
    NewTone(TONE_PIN, 500, 50);
  }

  if (timer.getCurrentMinutes() > 30)
  {
    // turn off beep
    NewTone(TONE_PIN, 1000, 200);
    deviceState = IDLE_STATE;
    pwmDutyCycle = 0;
  }

  if (deviceState == THERMOCOUPLE_ERROR)
  {
    reDrawThermocoupleError();
  }
  else if (deviceState == TEMP_SET_STATE && millis() - menuStateAge > MENU_TIMEOUT)
  {
    deviceState = IDLE_STATE;
    pwmDutyCycle = 0;
    deviceProfile = MANUAL_PROFILE;
    eepromWrite();
    reDrawRegularScreen();
  }
  else if (deviceState == PROFILE_SET_STATE && millis() - menuStateAge > MENU_TIMEOUT)
  {
    deviceState = IDLE_STATE;
    pwmDutyCycle = 0;
    targetTemperature = reflowProfiles[deviceProfile - 1].preheatTemp;
    eepromWrite();
    reDrawRegularScreen();
  }
  else if (deviceState == TEMP_SET_STATE)
  {
    reDrawTempSelectionScreen();
  }
  else if (deviceState == PROFILE_SET_STATE)
  {
    reDrawProfileSelectionScreen();
  }
  else
  {
    reDrawRegularScreen();
  }
}

void onUpButtonDoubleClick()
{
  Serial.println(F("onUpButtonDoubleClick"));
  NewTone(TONE_PIN, 750, 50);
  if (deviceState == TEMP_SET_STATE)
  {
    menuStateAge = millis();
    targetTemperature += 10;
    if (targetTemperature > MAX_TEMPERATURE)
      targetTemperature = MAX_TEMPERATURE;
    reDrawTempSelectionScreen();
  }
}

void onDownButtonDoubleClick()
{
  Serial.println(F("onDownButtonDoubleClick"));
  NewTone(TONE_PIN, 750, 50);
  if (deviceState == TEMP_SET_STATE)
  {
    menuStateAge = millis();
    targetTemperature -= 10;
    if (targetTemperature < MIN_TEMPERATURE)
      targetTemperature = MIN_TEMPERATURE;
    reDrawTempSelectionScreen();
  }
}

void onUpButtonPressed()
{
  Serial.println(F("onUpButtonPressed"));
  NewTone(TONE_PIN, 750, 50);
  if (deviceState == IDLE_STATE || deviceState == HEATING_STATE || deviceState == SOAKING_STATE || deviceState == REFLOW_STATE)
  {
    deviceState = TEMP_SET_STATE;
    menuStateAge = millis();
    reDrawTempSelectionScreen();
  }
  else if (deviceState == TEMP_SET_STATE)
  {
    menuStateAge = millis();
    targetTemperature += 5;
    if (targetTemperature > MAX_TEMPERATURE)
      targetTemperature = MAX_TEMPERATURE;
    reDrawTempSelectionScreen();
  }
  else if (deviceState == PROFILE_SET_STATE)
  {
    menuStateAge = millis();
    deviceProfile = (deviceProfile + 1) % (REFLOW_PROFILES_LEN + 1);
    reDrawProfileSelectionScreen();
  }
}

void onDownButtonPressed()
{
  Serial.println(F("onDownButtonPressed"));
  NewTone(TONE_PIN, 750, 50);
  if (deviceState == IDLE_STATE || deviceState == HEATING_STATE || deviceState == SOAKING_STATE || deviceState == REFLOW_STATE)
  {
    deviceState = TEMP_SET_STATE;
    menuStateAge = millis();
    reDrawTempSelectionScreen();
  }
  else if (deviceState == TEMP_SET_STATE)
  {
    menuStateAge = millis();
    targetTemperature -= 5;
    if (targetTemperature < MIN_TEMPERATURE)
      targetTemperature = MIN_TEMPERATURE;
    reDrawTempSelectionScreen();
  }
  else if (deviceState == PROFILE_SET_STATE)
  {
    menuStateAge = millis();
    deviceProfile--;
    if (deviceProfile < 0)
      deviceProfile = REFLOW_PROFILES_LEN;
    reDrawProfileSelectionScreen();
  }
}

void onTimerButtonPressed()
{
  Serial.println(F("onTimerButtonPressed"));
  NewTone(TONE_PIN, 750, 50);
  timer.restart();
}

void onXButtonPressed()
{
  Serial.println(F("onXButtonPressed"));
  NewTone(TONE_PIN, 750, 50);
  if (deviceState == IDLE_STATE || deviceState == TEMP_SET_STATE || deviceState == PROFILE_SET_STATE)
  {
    eepromWrite();
    deviceState = HEATING_STATE;
    timer.restart();
    reDrawScreen();
  }
  else if (deviceState == HEATING_STATE || deviceState == SOAKING_STATE || deviceState == REFLOW_STATE)
  {
    pwmDutyCycle = 0;
    deviceState = IDLE_STATE;
    targetTemperature = 0;
    profileTempSlope = 0;
    profilePhaseStartingTemp = 0;
    profileCompletedPhasesSec = 0;
    profilePhaseMaxTemp = 0;
    timer.restart();
    reDrawScreen();
  }
}

void onXButtonLongPressed()
{
  Serial.println(F("onXButtonLongPressed"));
  NewTone(TONE_PIN, 750, 50);
  if (deviceState == IDLE_STATE)
  {
    deviceState = PROFILE_SET_STATE;
    menuStateAge = millis();
  }
  reDrawScreen();
}

double readThermocouple()
{

  uint16_t v;
  digitalWrite(MAX6675_CS, LOW);
  delay(1);

  // Read in 16 bits,
  //  15    = 0 always
  //  14..2 = 0.25 degree counts MSB First
  //  2     = 1 if thermocouple is open circuit
  //  1..0  = uninteresting status

  v = shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  v <<= 8;
  v |= shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);

  digitalWrite(MAX6675_CS, HIGH);
  if (v & 0x4)
  {
    // Bit 2 indicates if the thermocouple is disconnected
    return NAN;
  }

  // The lower three bits (0,1,2) are discarded status bits
  v >>= 3;

  // The remaining bits are the number of 0.25 degree (C) counts
  return v * 0.25;
}

inline void profileTransitionBeep()
{
  NewTone(TONE_PIN, 1200, 500);
}

inline void setTempSlope(uint8_t temp, uint8_t durationSec, uint8_t elapsedTimeSec)
{
  profilePhaseMaxTemp = temp;
  profileCompletedPhasesSec = elapsedTimeSec;
  double tempDelta = temp - currentTemperature;
  profileTempSlope = tempDelta / (durationSec - elapsedTimeSec - 5); //5second look ahead
  profilePhaseStartingTemp = currentTemperature;
  // Serial.print(F("elapsedTimeSec: "));
  // Serial.println(elapsedTimeSec);
  // Serial.print(F("profilePhaseMaxTemp: "));
  // Serial.println(profilePhaseMaxTemp);
  // Serial.print(F("tempDelta: "));
  // Serial.println(tempDelta);
  // Serial.print(F("profileTempSlope: "));
  // Serial.println(profileTempSlope);
  // Serial.print(F("profilePhaseStartingTemp: "));
  // Serial.println(profilePhaseStartingTemp);
}

//===============================================
// PID processing at ~2Hz rate
//===============================================
void processPID()
{
  currentTemperature = readThermocouple();
  if (currentTemperature != NAN)
  {
    uint8_t elapsedPhaseTimeSec = timer.getTotalSeconds() - profileCompletedPhasesSec;
    if (deviceState == HEATING_STATE || deviceState == SOAKING_STATE || deviceState == REFLOW_STATE)
    {
      if (deviceProfile != MANUAL_PROFILE)
      {
        if (profileTempSlope == 0)
        {
          // preheat
          setTempSlope(reflowProfiles[deviceProfile - 1].preheatTemp, reflowProfiles[deviceProfile - 1].preheatDurationSec, 0);
        }

        if (deviceState == HEATING_STATE && timer.getTotalSeconds() >= reflowProfiles[deviceProfile - 1].preheatDurationSec)
        {
          Serial.println(F("->SOAKING_STATE"));
          profileTransitionBeep();
          deviceState = SOAKING_STATE;
          setTempSlope(reflowProfiles[deviceProfile - 1].soakTemp, reflowProfiles[deviceProfile - 1].soakDurationSec, reflowProfiles[deviceProfile - 1].preheatDurationSec);
        }
        if (deviceState == SOAKING_STATE && timer.getTotalSeconds() >= reflowProfiles[deviceProfile - 1].soakDurationSec)
        {
          Serial.println(F("->REFLOW_STATE"));
          profileTransitionBeep();
          deviceState = REFLOW_STATE;
          setTempSlope(reflowProfiles[deviceProfile - 1].reflowTemp, reflowProfiles[deviceProfile - 1].reflowDurationSec, reflowProfiles[deviceProfile - 1].soakDurationSec);
        }

        if (deviceState == REFLOW_STATE && timer.getTotalSeconds() >= reflowProfiles[deviceProfile - 1].reflowDurationSec)
        {
          profileTransitionBeep();
          deviceState = IDLE_STATE;
          pwmDutyCycle = 0;
          targetTemperature = 0;
          profileTempSlope = 0;
          profilePhaseStartingTemp = 0;
        }
        else
        {
          targetTemperature = min(profilePhaseMaxTemp, profileTempSlope * elapsedPhaseTimeSec + profilePhaseStartingTemp);
        }
      }

      // here below is an ugly number matching, be careful changing PID weights, as it ruin this logic
      bool farMode = (myPID.GetKd() == farKd);
      bool farHotMode = farMode && (myPID.GetKp() == farHOTKp);
      double gap = abs(targetTemperature - currentTemperature);
      if (gap < PID_FUNCTIONAL_RANGE && farMode)
      {
        myPID.reset();
        myPID.SetTunings(nearKp, nearKi, nearKd);
      }
      else if (gap > PID_FUNCTIONAL_RANGE && !farMode && targetTemperature < 200)
      {
        myPID.reset();
        myPID.SetTunings(farKp, farKi, farKd);
      }
      else if (gap > PID_FUNCTIONAL_RANGE && !farMode && !farHotMode && targetTemperature >= 200)
      {
        // TODO test it
        // heater is not powerful enough, need to increase Kp at the high end of tep range
        myPID.reset();
        myPID.SetTunings(farHOTKp, farHOTKi, farHOTKd);
      }

      myPID.Compute();
      pwmDutyCycle = pidOutput;
    }
  }
  else
  {
    pwmDutyCycle = 0;
    targetTemperature = 0;
    Serial.println(F("Thermocouple error"));
    deviceState = THERMOCOUPLE_ERROR;
    NewTone(TONE_PIN, 1000, 5000);
  }
}

// timer for PWM + divider
//=================================
// PID tuning
// For 60 Hz AC power, the maximum delay is 1/2 cycle or 8.33 ms.
// Considering that our PWM duty cycle ranges from 0-255, a single bit of resolution at 8.33 ms
// would imply a full range PWM switching period of 256 * 8.33 ms = 2.133 sec i.e. 0.46875Hz
// MAX6675 IC can read temp every 200ms ~4Hz
// i.e. every 266.67ms or 32 AC cycles we process PID algo and decide on PWM duty cycle
// PWM cycle is 266.67ms that we split in 2^5 intervals, therefore we have 2^5 control bits
// PID output variable will range from 0 to 32;
//
// Timer2 is set for interrupts at 120Hz rate (because half-cycles)
//=================================
void setupTimer2()
{
  noInterrupts();
  // Clear registers
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  // set compare match register for 120Hz increments
  OCR2A = 64; // = (8Mhz) / (120Hz*1024) - 1 =~ 64
  // turn on CTC mode - (Clear timer on compare match) mode
  TCCR2A |= (1 << WGM21);
  // Prescaler 1024
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
  // Enable "Output Compare Match" interrupt
  TIMSK2 |= (1 << OCIE2A);
  interrupts();
}

void eepromWrite()
{
  EEPROM.write(0, (uint8_t)targetTemperature);
  EEPROM.write(1, (uint8_t)deviceProfile);
}

void eepromRead()
{
  targetTemperature = EEPROM.read(0);
  deviceProfile = EEPROM.read(1);
}

void setup()
{
  Serial.begin(38400);

  while (!Serial)
  {
    delay(1);
  }

  Serial.println(F("booting up"));
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  display.begin();

  timer.setCounter(0, 30, 0, CountType::COUNT_UP, NULL);
  timer.setInterval(reDrawScreen, 1000);
  timer.start();

  upButton.attachClick(onUpButtonPressed);
  upButton.attachDoubleClick(onUpButtonDoubleClick);
  upButton.setClickTicks(100);
  downButton.attachClick(onDownButtonPressed);
  downButton.attachDoubleClick(onDownButtonDoubleClick);
  downButton.setClickTicks(100);
  xButton.attachClick(onXButtonPressed);
  xButton.setClickTicks(100);
  xButton.attachLongPressStart(onXButtonLongPressed);
  timerButton.attachClick(onTimerButtonPressed);
  timerButton.setClickTicks(100);

  // turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 32);
  // TODO change to 264 in the PID.cpp
  // myPID.SetSampleTime(266);

  pinModeFast(SSR_PIN, OUTPUT);
  digitalWriteFast(SSR_PIN, LOW);

  pinMode(MAX6675_CS, OUTPUT);
  pinMode(MAX6675_SO, INPUT);
  pinMode(MAX6675_SCK, OUTPUT);

  setupTimer2();
  eepromRead();

  char buffer[50];
  snprintf_P(buffer, 50, PSTR("far %.2f, %.2f, %.2f"), farKp, farKi, farKd);
  Serial.println(buffer);
  snprintf_P(buffer, 50, PSTR("near %.2f, %.2f, %.2f"), nearKp, nearKi, nearKd);
  Serial.println(buffer);
  Serial.println(PID_FUNCTIONAL_RANGE);

  reDrawRegularScreen();

  // ready beep
  NewTone(TONE_PIN, 750, 50);
  delay(100);
  NewTone(TONE_PIN, 1000, 50);
}

void loop()
{
  timer.run();
  upButton.tick();
  downButton.tick();
  timerButton.tick();
  xButton.tick();
  delay(50);
}

ISR(TIMER2_COMPA_vect)
{
  // every 120Hz/8 i.e 266.67ms, process PID
  if (pwmCycleCounter == 0)
  {
    processPID();
  }

  if (pwmDutyCycle == 0)
  {
    digitalWriteFast(SSR_PIN, LOW);
  }
  else if (pwmCycleCounter == 0)
  {
    digitalWriteFast(SSR_PIN, HIGH);
  }
  else if (pwmCycleCounter == pwmDutyCycle && pwmDutyCycle != 32)
  {
    digitalWriteFast(SSR_PIN, LOW);
  }

  pwmCycleCounter = (pwmCycleCounter + 1) % 32;
}