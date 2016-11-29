
#include <SPI.h>
#include <RH_NRF24.h>

#define CW  1
#define CCW 0

#define CLIENT_ADDRESS 0x02
#define SERVER_ADDRESS 0x01


//PinMapping
const byte pinA = 3;
const byte pinB = 2;
const byte pinButton = 4;






//States
volatile byte stateA = HIGH;
volatile byte stateB = HIGH;
byte stateButton = HIGH;

unsigned long buttonTimeOut = millis();



enum LedSettingState
{
  SET_BRIGHTNESS,
  SET_DIFFERENCE
};

LedSettingState ledSetState = SET_BRIGHTNESS;
unsigned long buttonDoublePushTimer = 0;





//Rotary encoder StateEngine
enum EncoderState
{
  WAIT_ALL,

  //CLOCKWISE STATES
  CW_WAIT_B_LOW,
  CW_WAIT_A_HIGH,
  CW_WAIT_B_HIGH,

  //COUNTERCLOCKWISE STATES
  CCW_WAIT_A_LOW,
  CCW_WAIT_B_HIGH,
  CCW_WAIT_A_HIGH

};

volatile EncoderState eState = WAIT_ALL;


RH_NRF24 nrf24;


void setup()
{
  Serial.begin(9600);

  while (!Serial)
    ; // wait for serial port to connect. Needed for Leonardo only
  if (!nrf24.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");


  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  pinMode(pinButton, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pinA), HandleRotaryA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), HandleRotaryB, CHANGE);
}



void loop()
{
  ReadButton();
}




void ReadButton()
{
  if (millis() - buttonTimeOut > 80)
  {
    byte currentButtonState = digitalRead(pinButton);
    if (stateButton == LOW && currentButtonState == HIGH)
      ButtonPushed();

    stateButton = currentButtonState;

    buttonTimeOut = millis();
  }
}


void ButtonPushed()
{
  LedSettingState lst = ledSetState;
  switch (ledSetState)
  {
    case SET_BRIGHTNESS:
      ledSetState = SET_DIFFERENCE;
      break;

    case SET_DIFFERENCE:
      ledSetState = SET_BRIGHTNESS;
      break;
  }

  if (millis() - buttonDoublePushTimer < 400)
  {
    Serial.println("Double button");
    uint8_t data[2] = {SERVER_ADDRESS, 0xAA};
    nrf24.send(data, sizeof(data));
  }
  else
    buttonDoublePushTimer = millis();

  Serial.println("BUTTON");
}




void Rotate(int dir)
{
  uint8_t data[2];
  data[0] = SERVER_ADDRESS;

  eState = WAIT_ALL;

  if (dir == CW)
  {
    Serial.println("CW");
    if (ledSetState == SET_DIFFERENCE)
      data[1] = 0xBB;

    if (ledSetState == SET_BRIGHTNESS)
      data[1] = 0xDD;
  }
  else if (dir == CCW)
  {
    Serial.println("CCW");
    if (ledSetState == SET_DIFFERENCE)
      data[1] = 0xCC;

    if (ledSetState == SET_BRIGHTNESS)
      data[1] = 0xEE;
  }

  nrf24.send(data, sizeof(data));
}




void HandleRotary(int pin, byte state)
{
  if (pin == pinA)
  {
    switch (eState)
    {
      case WAIT_ALL:
        if (state == LOW)
          eState = CW_WAIT_B_LOW;
        break;

      case CW_WAIT_B_LOW:
        if (state == HIGH)
          eState = WAIT_ALL;
        break;

      case CW_WAIT_A_HIGH:
        if (state == HIGH)
          eState = CW_WAIT_B_HIGH;
        break;

      case CW_WAIT_B_HIGH:
        if (state == LOW)
          eState = CW_WAIT_A_HIGH;
        break;

      case CCW_WAIT_A_LOW:
        if (state == LOW)
          eState = CCW_WAIT_B_HIGH;
        break;

      case CCW_WAIT_B_HIGH:
        if (state == HIGH)
          CCW_WAIT_A_LOW;
        break;

      case CCW_WAIT_A_HIGH:
        if (state == HIGH)
          Rotate(CCW);
        break;
    }
  }

  else if (pin == pinB)
  {
    switch (eState)
    {
      case WAIT_ALL:
        if (state == LOW)
          eState = CCW_WAIT_A_LOW;
        break;

      case CW_WAIT_B_LOW:
        if (state == LOW)
          eState = CW_WAIT_A_HIGH;
        break;

      case CW_WAIT_A_HIGH:
        if (state == HIGH)
          eState = CW_WAIT_B_LOW;
        break;

      case CW_WAIT_B_HIGH:
        if (state == HIGH)
          Rotate(CW);
        break;

      case CCW_WAIT_A_LOW:
        if (state == HIGH)
          eState = WAIT_ALL;
        break;

      case CCW_WAIT_B_HIGH:
        if (state == HIGH)
          eState = CCW_WAIT_A_HIGH;
        break;

      case CCW_WAIT_A_HIGH:
        if (state == LOW)
          eState = CCW_WAIT_B_HIGH;
        break;
    }

  }

  if (stateA == HIGH && stateB == HIGH)
    eState = WAIT_ALL;
}




void HandleRotaryA()
{
  byte newState = digitalRead(pinA);
  if (stateA != newState)
  {
    stateA = newState;
    HandleRotary(pinA, stateA);
  }
}




void HandleRotaryB()
{
  byte newState = digitalRead(pinB);
  if (stateB != newState)
  {
    stateB = newState;
    HandleRotary(pinB, stateB);
  }
}


