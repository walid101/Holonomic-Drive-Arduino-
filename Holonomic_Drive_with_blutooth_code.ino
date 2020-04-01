/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];
 
  int m1b=2;
  int m2b=A1;
  int m3b=5;
  int m4b=9;
 
  int m1f=3;
  int m2f=A2;
  int m3f=6;
  int m4f=10;
/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
 
 
  pinMode(m1f,OUTPUT);
  pinMode(m2f,OUTPUT);
  pinMode(m3f,OUTPUT);
  pinMode(m4f,OUTPUT);
 
  pinMode(m1b,OUTPUT);
  pinMode(m2b,OUTPUT);
  pinMode(m3b,OUTPUT);
  pinMode(m4b,OUTPUT);
}

/**************************************************************************/
/*
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  /* Got a packet! */
  // printHex(packetbuffer, len);

  // Color
  if (packetbuffer[1] == 'C') {
    uint8_t red = packetbuffer[2];
    uint8_t green = packetbuffer[3];
    uint8_t blue = packetbuffer[4];
    int newRed = (int)(red);
    int newGreen = (int)(green);
    int newBlue = (int)(blue);
  if(!(newRed > 255 && newBlue > 255 && newGreen > 255) && (newRed != 255 && newBlue != 255 && newGreen != 255))
  {
    int trueAngle = findAngle(newRed,newGreen,newBlue);
    double v1 = vel1(255.0,trueAngle);
    double v2 = vel2(255.0,trueAngle);
    Move(v2,v1);
  }
  else
  {
        analogWrite(m1f,0);
        analogWrite(m2f,0);
        analogWrite(m3f,0);
        analogWrite(m4f,0);

        analogWrite(m1b,0);
        analogWrite(m2b,0);
        analogWrite(m3b,0);
        analogWrite(m4b,0);
  }
   
    Serial.print("Angle: ");
    Serial.println(trueAngle);
    Serial.print("V1: ");
    Serial.println(v1);
    Serial.print("V2: ");
    Serial.println(v2);
    Serial.print ("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);

   
 
  }

  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    ////////////
    if (pressed) {
      if(buttnum==5)//GO UP TESTING
      {
        analogWrite(m1f,255);
        analogWrite(m2f,0);
       
        analogWrite(m3f,255);
        analogWrite(m4f,0);

        analogWrite(m1b,0);
        analogWrite(m2b,255);
        analogWrite(m3b,0);
        analogWrite(m4b,255);
      }
      if(buttnum == 6)//GO DOWN TESTING
      {
        analogWrite(m1f,0);
        analogWrite(m2f,255);
       
        analogWrite(m3f,0);
        analogWrite(m4f,255);

        analogWrite(m1b,255);
        analogWrite(m2b,0);
        analogWrite(m3b,255);
        analogWrite(m4b,0);
      }
      else if(buttnum == 8)//GO RIGHT TESTING
      {
        analogWrite(m1f,180);
        analogWrite(m2f,180);
       
        analogWrite(m3f,0);
        analogWrite(m4f,0);

        analogWrite(m1b,0);
        analogWrite(m2b,0);
        analogWrite(m3b,180);
        analogWrite(m4b,180);
      }
      else if(buttnum == 7)//GO LEFT
      {
        analogWrite(m1f,0);
        analogWrite(m2f,0);
       
        analogWrite(m3f,180);
        analogWrite(m4f,180);

        analogWrite(m1b,180);
        analogWrite(m2b,180);
        analogWrite(m3b,0);
        analogWrite(m4b,0);
      }
      else if(buttnum = 1)//left Diagonal
      {
        analogWrite(m1f,0);
        analogWrite(m2f,0);
       
        analogWrite(m3f,180);
        analogWrite(m4f,0);

        analogWrite(m1b,0);
        analogWrite(m2b,180);
        analogWrite(m3b,0);
        analogWrite(m4b,0);
      }
      else if(buttnum == 2)
      {
        analogWrite(m1f,180);
        analogWrite(m2f,0);
       
        analogWrite(m3f,0);
        analogWrite(m4f,0);

        analogWrite(m1b,0);
        analogWrite(m2b,0);
        analogWrite(m3b,0);
        analogWrite(m4b,180);
      }
      else if(buttnum==4)//TURN RIGHT TESTING
      {
        analogWrite(m1f,100);
        analogWrite(m2f,100);
        analogWrite(m3f,100);
        analogWrite(m4f,100);

        analogWrite(m1b,0);
        analogWrite(m2b,0);
        analogWrite(m3b,0);
        analogWrite(m4b,0);
      }
      else if(buttnum == 3)//TURN ALL OFF
      {
        analogWrite(m1f,0);
        analogWrite(m2f,0);
        analogWrite(m3f,0);
        analogWrite(m4f,0);

        analogWrite(m1b,0);
        analogWrite(m2b,0);
        analogWrite(m3b,0);
        analogWrite(m4b,0);
      }
      else
      {
        analogWrite(m1f,0);
        analogWrite(m2f,0);
        analogWrite(m3f,0);
        analogWrite(m4f,0);

        analogWrite(m1b,0);
        analogWrite(m2b,0);
        analogWrite(m3b,0);
        analogWrite(m4b,0);
      }
     //else if(buttnum==7)
     //{
         
     //}
     // Serial.println(" pressed");
    } else {
      Serial.println(" released");
    }}
  }
int findAngle(double R, double G, double B)
{
 
  double rMod = R/255;
  double gMod = G/255;
  double bMod = B/255;

  double Min = findMin(rMod,gMod,bMod);
  double Max = findMax(rMod,gMod,bMod);
  /**
  Serial.println("Min: ");
  Serial.println(Min);
  Serial.println("");

  Serial.println("Max: ");
  Serial.println(Max);
  Serial.println("");
  **/

  double hue = (gMod - bMod)/(Max - Min);
 
  int Angle = 0;
  if(Min!=Max)
  {
    if(gMod == Max)
    {
      hue = 2.0 + (bMod-rMod)/(Max-Min);
    }
    else if(bMod == Max)
    {
      hue = 4.0 + (rMod-gMod)/(Max-Min);
    }
  }
  Angle = hue * 60;
  if(hue<0)
  {
    Angle = Angle + 360;
  }
  return Angle;

 
}
double findMax(double R, double G, double B)
{
  if(R>G && R > B)
  {
    return R;
  }
  else if(G>B && G>R)
  {
    return G;
  }
  return B;
}
double findMin(double R, double G, double B)
{
  if(R<G && R<B)
  {
    return R;
  }
  else if(G<R && G<B)
  {
    return G;
  }
  return B;
}
double vel1(double Max, double angle)//to go to the Right
{
  return Max*(-1*sin(convertToRad(angle) + PI/4));
}
double vel2(double Max, double angle)// to go to the left
{
  return Max*(cos(convertToRad(angle) + PI/4));
}
double convertToRad(double angle)
{
  return (angle/180)*(PI);
}
void Move(double v1, double v2) //NOTE THAT RIGHT AND LEFT NEED TO BE SWITCHED HERE
{
  //THERE ARE FOUR MOTORS:
  //m1,m2,m3,m4
    /**frontL == -v2
    frontR == v1
    backR == v2
    backL == -v1
    **/
    /**
     * GO UP:
     *
     * **/
     //IF I WANT TO GO UP: v1 = -; v2 = -; ==> YES MOTORS ARE SETUP CORRECTLY NOW
    //m1 ==> -v2 ==> if v2 is + then wheel goes back, if v2 == - then wheel goes forward
    //m2 ==> v1 ==> if v1 is + then wheel goes forward, if v1 is - then wheel goes backward
    //m3 ==> -v1 ==>  if v1 is - then wheel goes forward, if v1 is + then wheel goes backward
    //m4 ==> v2 ==> if v2 is - then wheel goes back, if v2 == +then wheel goes forward
    if(!(v1 == 0 && v2 == 0))
    {
      if(v1 >= 0)
      {
       
        analogWrite(m2f,v1);
        analogWrite(m3f,0);
       

     
        analogWrite(m2b,0);
        analogWrite(m3b,v1);
       
      }
      else if(v1<=0)
      {
       
        analogWrite(m2f,0);
        analogWrite(m3f,-1*v1);
       
        analogWrite(m2b,-1*v1);
        analogWrite(m3b,0);
       
      }
   
    //m2 ==> v1 ==> if v1 is + then wheel goes forward, if v1 is - then wheel goes backward
    //m3 ==> -v1 ==>  if v1 is - then wheel goes forward, if v1 is + then wheel goes backward
    //m4 ==> v2 ==> if v2 is - then wheel goes back, if v2 == + then wheel goes forward
    //m1 ==> -v2 ==> if v2 is + then wheel goes back, if v2 == - then wheel goes forward
      if(v2 >= 0)
      {
        analogWrite(m1f,0);
        analogWrite(m4f,v2);

        analogWrite(m1b,v2);
        analogWrite(m4b,0);
      }
      else if(v2 <= 0)
      {
        analogWrite(m1f,-1*v2);
        analogWrite(m4f,0);

        analogWrite(m1b,0);
        analogWrite(m4b,-1*v2);
      }
    }
    else
    {
        analogWrite(m1f,0);
        analogWrite(m2f,0);
        analogWrite(m3f,0);
        analogWrite(m4f,0);

        analogWrite(m1b,0);
        analogWrite(m2b,0);
        analogWrite(m3b,0);
        analogWrite(m4b,0);
    }
   
   
 
 
}
