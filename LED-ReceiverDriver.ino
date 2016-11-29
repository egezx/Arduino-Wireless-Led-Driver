
#include <RH_NRF24.h>
#include <SPI.h>

#define CLIENT_ADDRESS 0x02
#define SERVER_ADDRESS 0x01
#define PAIR_ADDRESS 0x0A

RH_NRF24 nrf24;




//LED PARAMS
bool ledPowerOn = false;

const float ledMaxValue = 255.0;
const float ledStepSize = 0.05;
float ledBrightness = 1.0;
float ledBrightnessDifference = 0.5;

const byte ledPin1 = 3;
const byte ledPin2 = 5;
const byte pairPin = 4;

bool pairButtonDown = false;
unsigned long pairButtonTimer = millis();
bool pairing = false;

unsigned int pairTimeout = 2000;

void setup()
{
  Serial.begin(9600);
  while (!Serial); // wait for serial port to connect. Needed for Leonardo only
  if (!nrf24.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");

  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(pairPin, INPUT_PULLUP);
}


void loop()
{
  PairButtonCheck();

  if (pairing)
  {
    uint8_t data[2] = {PAIR_ADDRESS, SERVER_ADDRESS};
    nrf24.send(data, sizeof(data));

    delay(500);
  }

  else
  {
    if (nrf24.available())
    {
      uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);

      if (nrf24.recv(buf, &len))
      {
        if (buf[0] == SERVER_ADDRESS)
        {
          Serial.println(buf[1], HEX);
          switch (buf[1])
          {
            case 0xAA:
              ledPowerOn = !ledPowerOn;
              break;
            case 0xBB:
              UpdateLedParameter(&ledBrightnessDifference, ledStepSize);
              break;
            case 0xCC:
              UpdateLedParameter(&ledBrightnessDifference, ledStepSize * (-1.0));
              break;
            case 0xDD:
              UpdateLedParameter(&ledBrightness, ledStepSize);
              break;
            case 0xEE:
              UpdateLedParameter(&ledBrightness, ledStepSize * (-1.0));
              break;
          }

        }
        /* Serial.print("Brightness Dirfference: ");
         Serial.println(ledBrightnessDifference);
         Serial.print("Brightness: ");
         Serial.println(ledBrightness);*/

        if (ledPowerOn)
        {
          analogWrite(ledPin1, (ledBrightnessDifference * ledBrightness * ledMaxValue));
          analogWrite(ledPin2, ((1.0 - ledBrightnessDifference) * ledBrightness * ledMaxValue));
        }
        else
        {
          analogWrite(ledPin1, 0);
          analogWrite(ledPin2, 0);
        }
      }
    }
  }
}

void PairButtonCheck()
{
  if (pairPin == LOW)
  {
    if (pairButtonDown)
    {
      if (millis() - pairButtonTimer > pairTimeout)
        pairing = true;
    }
    else
    {
      pairButtonDown = true;
      pairButtonTimer = millis();
    }
  }
  else
  {
    pairing = false;
    pairButtonDown = false;
  }
}


void UpdateLedParameter( float* t_param, float t_val)
{
  float nextVal = *t_param + t_val;
  if (nextVal >= -0.001 && nextVal <= 1.001)
  {
    /* Serial.print("Current Value (*param): ");
     Serial.println(*t_param);
     Serial.print("Value change: ");
     Serial.println(t_val);

     Serial.print("Next value: ");
     Serial.println(fabs(nextVal));
     */
    *t_param = fabs(nextVal);
  }
}

