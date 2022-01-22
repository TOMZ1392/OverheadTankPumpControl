#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile


//RH_ASK driver(2000,12,11);//TX on pin0
//datapin must be arduino digital pin 11

//debug
//#define LEVEL_VAR_TEST
#define LCD_PRINT_MSG

#ifdef LCD_PRINT_MSG
#define LCD_UPDATE_INTERVAL_MS  4000
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 column and 2 rows
#endif


#define MOTOR_RELAY_PIN             9
#define MOTOR_TOGGLE_BUTTON_PIN     10 // --- test button pin 8 on dev board

#define ERROR_LAMP_PIN              13 // 6 to enable debug lamp
#define POWER_LATCH_PIN             99 // not used


#define MAX_RCV_ERRORS               5
#define TANK_FULL_LEVEL             12
#define TANK_EMPTY_LEVEL            50
#define TANK_ERROR_LEVEL            255
#define MOTOR_BTN_STATE_OFF          0
#define MOTOR_BTN_STATE_ON           1
#define BTN_ANTI_DEBOUNCE_PERIOD    800
#define AVERAGE_SAMPLE_SIZE          3
#define ERROR_LAMP_DLY_STEP         400

#define LEVEL_HISTORY_RECORD_SIZE                5
#define LEVEL_PLAUSIBILITY_PROCESSING            1
#define LEVEL_PLAUSIBILITY_IN_TREND              2
#define LEVEL_PLAUSIBILITY_ERROR                 3
#define SPORADIC_ERROR_MAX                       6 // MAX LEVEL ERROR BEFORE SHUTDOWN
#define SEC_TICK_UPDATE_STEP_USER_MOTOR_REQ_MS  4000 // update second ticks in 4000ms step for user motor request
#define MOTOR_RUNTIME_ON_USER_REQUEST_SEC       420 // motor runtime on user request in seconds (eg:motor runs for 300s(5mins) when button pressed)


RH_ASK wirelesComDriver;
volatile float rx_data = 0;
uint8_t rx_data_buflen = 4;
uint8_t receptionErrCtr = 0;
volatile uint8_t isSignalGood = 0;
volatile uint32_t starTimeRXLastRcpt = 0;
volatile uint32_t starTimeBtnPush = 0;
volatile uint8_t buttonPushedState_se = 0; //state exchange variable

volatile uint8_t motorState = MOTOR_BTN_STATE_OFF;
volatile uint8_t onUserDmdMotorRequestState = 0; // for idenifying pushbutton pump dmd
volatile uint32_t userMotorRequestStartTime = 0;


typedef enum {
  NO_ERROR = 0,
  RX_SIGNAL_LOST,
  MOTOR_RELAY_PIN_FAILURE,
  PUMP_FLOW_ERROR,
  LEVEL_SENSOR_SPORADIC,



} sysErr_t;

sysErr_t sysError;

void Task_getInputControls();
void Task_motorDriver();
void Task_setSysStatusLamp();
void setError(sysErr_t err);
void clearError(sysErr_t err);
uint8_t fluidLevelPlausibilityTest(uint8_t level, uint8_t motorState);
void shutdownSysOnCriticalError();

#ifdef LCD_PRINT_MSG
void  intiLcd()
{
  lcd.init(); // initialize the lcd
  lcd.backlight();

  lcd.setCursor(0, 0);         // move cursor to   (0, 0)
  lcd.print("Pump Controller");        // print message at (0, 0)
  lcd.setCursor(0, 1);         // move cursor to   (2, 1)
  lcd.print("initializing.."); // print message at (2, 1)
  delay(2000);
  lcd.clear();
  //lcd.noBacklight();
}

void lcdPrintLines(String line1, String line2)
{
  lcd.clear();
  lcd.setCursor(0, 0);         // move cursor to   (0, 0)
  lcd.print(line1);        // print message at (0, 0)
  lcd.setCursor(0, 1);         // move cursor to   (2, 1)
  lcd.print(line2); // print message at (2, 1)
}

#endif

void setup()
{
  Serial.begin(9600);  // Debugging only
  pinMode(MOTOR_RELAY_PIN, OUTPUT);
  pinMode(ERROR_LAMP_PIN, OUTPUT);
  // pinMode(POWER_LATCH_PIN, OUTPUT);
  pinMode(MOTOR_TOGGLE_BUTTON_PIN, INPUT);
  digitalWrite(MOTOR_RELAY_PIN, LOW);
  delay(200);
  //digitalWrite(POWER_LATCH_PIN, HIGH);
  Serial.println("Starting...");
  if (!wirelesComDriver.init())
    Serial.println("RX init failed..!!");
  else
  {
    Serial.println("RX init ready..");
  }

#ifdef LCD_PRINT_MSG
  intiLcd();
#endif;
}





void loop()
{
  Task_getInputControls();
  Task_motorDriver();
  Task_setSysStatusLamp();
}





// function to set status of user motor request, and store startime
void setUserMotorRequest(uint8_t state) {

  if (state == MOTOR_BTN_STATE_ON)
  {
    userMotorRequestStartTime = millis();
  }
  else {
    userMotorRequestStartTime = 0;
  }

  onUserDmdMotorRequestState = state;
}





uint32_t getUserMotorRequestState() {
  return onUserDmdMotorRequestState;
}




// note: never leave button pin floating!
void Task_getInputControls()
{
  if (digitalRead(MOTOR_TOGGLE_BUTTON_PIN) == HIGH) {
    if (starTimeBtnPush > 0) {
      uint32_t pastime = millis() - starTimeBtnPush;
      if (pastime > BTN_ANTI_DEBOUNCE_PERIOD)
      {
        buttonPushedState_se = 1;
        if (motorState == MOTOR_BTN_STATE_ON)
        {
          motorState = MOTOR_BTN_STATE_OFF;
          Serial.println(" Button pressed: OFF");
          setUserMotorRequest(MOTOR_BTN_STATE_OFF);
        }
        else
        {
          motorState = MOTOR_BTN_STATE_ON;
          Serial.println(" Button pressed: ON");
          setUserMotorRequest(MOTOR_BTN_STATE_ON);
        }
        starTimeBtnPush = 0;
      }
    }
    else if (starTimeBtnPush == 0)
    {
      buttonPushedState_se = 0;
      starTimeBtnPush = millis();
    }
    else
    {
    }
  }
  else
  {
  }
}




void Task_motorDriver()
{

  static uint16_t sampleCtr;
  static uint32_t avg_sum;
  static uint8_t avgLevel = 255;

  // level testing block
#ifdef LEVEL_VAR_TEST
  if (Serial.available())
  {
    rx_data = (float)((Serial.readString()).toInt());
  }
  static uint32_t ct;
  ct++;
  if (ct % 1000 == 0)
#else
  //Serial.println("RCV CODE ON");
  if (wirelesComDriver.recv((uint8_t*)&rx_data, &rx_data_buflen)) // Non-blocking
#endif
  {

    Serial.print("Level: ");
    Serial.print(rx_data);
    Serial.println(" cm");
    receptionErrCtr = 0;
    isSignalGood = 1;
    starTimeRXLastRcpt = millis();
    clearError(RX_SIGNAL_LOST);


    if ((float)(TANK_FULL_LEVEL - 5) < rx_data && rx_data < (float)(TANK_EMPTY_LEVEL + 10)) // take samples only if value within range
    {
      sampleCtr++;
      avg_sum += rx_data;
    }


    // avg_sum += rx_data;
    if (sampleCtr == AVERAGE_SAMPLE_SIZE)
    {
      static uint8_t levelErrorCtr;
      sampleCtr = 0;
      avgLevel = (uint8_t)(avg_sum / AVERAGE_SAMPLE_SIZE);
      avg_sum = 0;
      Serial.print("Level Avg: ");
      Serial.print(avgLevel);
      Serial.println(" cm");
      switch (fluidLevelPlausibilityTest(avgLevel, motorState))
      {
        case LEVEL_PLAUSIBILITY_IN_TREND :
          isSignalGood = 1;
          clearError(PUMP_FLOW_ERROR);
          levelErrorCtr = 0;
          Serial.println("Level variation good!");
          break;
        case LEVEL_PLAUSIBILITY_ERROR :
          //isSignalGood = 0;
          //motorState = MOTOR_BTN_STATE_OFF;
          setError(PUMP_FLOW_ERROR);/*define new error for sporadic level*/
          //isSignalGood=0;
          levelErrorCtr++;
          Serial.println("Level variation sporadic!");
          if (levelErrorCtr == SPORADIC_ERROR_MAX && motorState)
          {
            Serial.println("Level error on pump run.. shuting down..!");
            delay(1000);
            shutdownSysOnCriticalError();
          }
          break;
        case LEVEL_PLAUSIBILITY_PROCESSING :
        default:
          //Serial.println(" Plausibility check running..");
          break;
      }
    }


  }
  else
  {
    isSignalGood = 0;//must be moved to a getter setter
    uint32_t pastime = millis() - starTimeRXLastRcpt;
    static uint16_t userReqMotorOnTimeSec;
    if (pastime >= 15000)
    {
      receptionErrCtr = receptionErrCtr >= MAX_RCV_ERRORS ? MAX_RCV_ERRORS : receptionErrCtr + 1;
      rx_data = (double)TANK_ERROR_LEVEL;


      //Serial.println("Motor shut off due to error");

    }

    if (getUserMotorRequestState() == MOTOR_BTN_STATE_ON)
    {
      // since long delays are involved, chances of timer overflow leading to incorrect time calculation is possible
      // so a seconds counder is introduced
      if ( (millis() - userMotorRequestStartTime) > SEC_TICK_UPDATE_STEP_USER_MOTOR_REQ_MS)
      {
        userReqMotorOnTimeSec += (uint16_t)( SEC_TICK_UPDATE_STEP_USER_MOTOR_REQ_MS / 1000);
        userMotorRequestStartTime = millis(); // reload timer tick
      }
      if (userReqMotorOnTimeSec >= MOTOR_RUNTIME_ON_USER_REQUEST_SEC)
      {

        setUserMotorRequest(MOTOR_BTN_STATE_OFF);
        digitalWrite(MOTOR_RELAY_PIN, LOW);
        Serial.println("Motor runtime on user request elapsed, turning off!");
        Serial.print(userReqMotorOnTimeSec);
        userReqMotorOnTimeSec = 0;
        motorState = MOTOR_BTN_STATE_OFF;

      }
    }
    else if (receptionErrCtr == MAX_RCV_ERRORS)
    {
      digitalWrite(MOTOR_RELAY_PIN, LOW);
      Serial.println("Default turn off motor on signal lost");

    }
  }

  if (receptionErrCtr == MAX_RCV_ERRORS)
  {
    //Serial.println("RX signal lost!");
    setError(RX_SIGNAL_LOST);
  }

  //ON DEMAND BUTTON PUSH. !sporadic when button pin floating!!!
  if (buttonPushedState_se)
  {
    if (isSignalGood && motorState)
    {
      digitalWrite(MOTOR_RELAY_PIN, HIGH);
      Serial.println("Motor running on demand! on signal good");

    }
    else if (isSignalGood && !motorState)
    {
      digitalWrite(MOTOR_RELAY_PIN, LOW);
      Serial.println("Motor shutoff on demand!on signal good");
      // on deman shutoff should override level sensor value
    }
    else if (!isSignalGood)
    {

      if (motorState)
      {
        digitalWrite(MOTOR_RELAY_PIN, HIGH);
        Serial.println("Motor running on demand! during signal lost");
      }
      else {
        if (receptionErrCtr == MAX_RCV_ERRORS) { // use case for button push start on not tested 16122021
          digitalWrite(MOTOR_RELAY_PIN, LOW);
          Serial.println("Motor shutoff on demand! during signal lost");
        }
      }
    }

  }

  if (isSignalGood)
  {
    if ((int)rx_data > TANK_EMPTY_LEVEL)
    {
      motorState = MOTOR_BTN_STATE_ON;
      digitalWrite(MOTOR_RELAY_PIN, HIGH);
      Serial.println("Motor on automatic");
    }
    else if ((int)rx_data < TANK_FULL_LEVEL)
    {
      motorState = MOTOR_BTN_STATE_OFF;
      digitalWrite(MOTOR_RELAY_PIN, LOW);
      Serial.println("Motor off automatic");
    }
    else
    {
    }
  }
  //  static uint64_t lcdUpdElapsed;
  //  if (millis() - lcdUpdElapsed > 2000) {
  //    uint8_t percLevel = (uint8_t)(uint32_t)((1 - ( (float)(55 - TANK_FULL_LEVEL) / (TANK_EMPTY_LEVEL - TANK_FULL_LEVEL))) * 100);
  //    Serial.println(percLevel);
  //
  //    lcdUpdElapsed = millis();
  //  }
#ifdef LCD_PRINT_MSG
  // Serial.println("LCD");
  static uint64_t lcdUpdElapsed;
  if (millis() - lcdUpdElapsed > LCD_UPDATE_INTERVAL_MS)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Level:");
    lcd.setCursor(7, 0);
    float percLevel = 1 - ( (float)(avgLevel - TANK_FULL_LEVEL) / (TANK_EMPTY_LEVEL - TANK_FULL_LEVEL));
    if (avgLevel > TANK_EMPTY_LEVEL)
    {
      percLevel = 0;
    }
    lcd.print((uint8_t)(uint32_t)(percLevel * 100));

    lcd.setCursor(10, 0);
    lcd.print('%');



    switch (sysError) {
      case RX_SIGNAL_LOST:
        lcd.setCursor(0, 1);
        lcd.print("Error:");
        lcd.setCursor(6, 1);
        lcd.print("TX_LOST");
        break;

      case MOTOR_RELAY_PIN_FAILURE:
        lcd.setCursor(0, 1);
        lcd.print("Error:");
        lcd.setCursor(6, 1);
        lcd.print("RLY_PIN_F");
        break;

      case PUMP_FLOW_ERROR:
        lcd.setCursor(0, 1);
        lcd.print("Error:");
        lcd.setCursor(6, 1);
        lcd.print("PMP_F");
        break;

      case LEVEL_SENSOR_SPORADIC:
        lcd.setCursor(0, 1);
        lcd.print("Error:");
        lcd.setCursor(6, 1);
        lcd.print("SNSR_F");
        break;

      default:

        lcd.setCursor(0, 1);
        lcd.print(avgLevel);
        lcd.setCursor(3, 1);
        lcd.print("cm");
        break;
    }
    lcdUpdElapsed = millis();
  }
#endif
}





void Task_setSysStatusLamp()
{
  static uint16_t blinkDelay;
  static uint32_t blinkStrtime; // best rutime error illustration when its uint16_t
  static uint8_t blinkStart;
  static uint8_t blinkState;
  switch (sysError) {
    case RX_SIGNAL_LOST:
      blinkDelay = RX_SIGNAL_LOST * ERROR_LAMP_DLY_STEP;
      break;
    case MOTOR_RELAY_PIN_FAILURE:
      blinkDelay = MOTOR_RELAY_PIN_FAILURE * ERROR_LAMP_DLY_STEP;
      break;

    case PUMP_FLOW_ERROR:
      blinkDelay = PUMP_FLOW_ERROR * ERROR_LAMP_DLY_STEP;

    default:
      blinkDelay = 0;
      break;
  }
  if (sysError != NO_ERROR && !blinkStart)
  {
    blinkStrtime = millis();
    blinkStart = 1;
  }
  else if (sysError == NO_ERROR) {
    blinkStart = 0;
    digitalWrite(ERROR_LAMP_PIN, LOW);
  }
  else
  {
  }

  if (blinkStart)
  {
    if ((millis() - blinkStrtime) > blinkDelay)
    {
      if (blinkState) {
        digitalWrite(ERROR_LAMP_PIN, LOW);
        blinkState = 0;
      }
      else {
        digitalWrite(ERROR_LAMP_PIN, HIGH);
        blinkState = 1;
      }
      blinkStrtime = millis();

    }
  }

}





void setError(sysErr_t err)
{
  sysError = err;
  //Serial.println("Error Set!!");
}




void clearError(sysErr_t err)
{
  switch (sysError) {
    case RX_SIGNAL_LOST:
      if (err == RX_SIGNAL_LOST) {
        sysError = NO_ERROR;
      }
      break;
    case MOTOR_RELAY_PIN_FAILURE:
      if (err == MOTOR_RELAY_PIN_FAILURE) {
        sysError = NO_ERROR;
      }
      break;
    default:

      sysError = NO_ERROR;

      break;
  }
}






uint8_t fluidLevelPlausibilityTest(uint8_t level, uint8_t motorState)
{
  static uint8_t onStateCtr, offStateCtr;
  static uint8_t onStateLevelHist[LEVEL_HISTORY_RECORD_SIZE];
  static uint8_t offStateLevelHist[LEVEL_HISTORY_RECORD_SIZE];
  uint8_t retStat = LEVEL_PLAUSIBILITY_PROCESSING;
  uint8_t onStateTrueCtr = 0, offStateTrueCtr = 0;
  if (motorState == MOTOR_BTN_STATE_ON)
  {
    onStateLevelHist[onStateCtr] = level;
    onStateCtr++;
    if (onStateCtr == LEVEL_HISTORY_RECORD_SIZE)
    {
      onStateCtr = 0;
      uint8_t i = 0;
      for (i = 0; i < LEVEL_HISTORY_RECORD_SIZE; i++)
      {
        uint8_t j = (i <= (LEVEL_HISTORY_RECORD_SIZE - 2)) ? i : i - 1;
        if (onStateLevelHist[j] > onStateLevelHist[j + 1])
        {
          onStateTrueCtr++;
        }
      }
      Serial.println(onStateTrueCtr);
      if (onStateTrueCtr == LEVEL_HISTORY_RECORD_SIZE )
      {
        retStat = LEVEL_PLAUSIBILITY_IN_TREND;
      }
      else
      {
        retStat = LEVEL_PLAUSIBILITY_ERROR;
      }
    }
  }
  else
  {
    offStateLevelHist[offStateCtr] = level;
    offStateCtr++;
    if (offStateCtr == LEVEL_HISTORY_RECORD_SIZE)
    {
      offStateCtr = 0;
      uint8_t i = 0;
      for (i = 0; i < LEVEL_HISTORY_RECORD_SIZE; i++)
      {
        uint8_t j = (i <= (LEVEL_HISTORY_RECORD_SIZE - 2)) ? i : i - 1;
        if (offStateLevelHist[j] <= offStateLevelHist[j + 1])
        {
          offStateTrueCtr++;
        }
      }
      Serial.println(offStateTrueCtr);
      if (offStateTrueCtr == LEVEL_HISTORY_RECORD_SIZE )
      {
        retStat = LEVEL_PLAUSIBILITY_IN_TREND;
      }
      else
      {
        retStat = LEVEL_PLAUSIBILITY_ERROR;
      }
    }
  }
  return retStat;
}




void shutdownSysOnCriticalError()
{
#ifdef LCD_PRINT_MSG
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("CRITICAL_ERR!!");
#endif
  digitalWrite(MOTOR_RELAY_PIN, LOW);
  //while (1);
  //digitalWrite(POWER_LATCH_PIN, LOW);
}
