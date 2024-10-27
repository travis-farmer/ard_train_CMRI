//  MCP23017 code -- some parts translated from Portugese
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <CMRI.h>
#include <Auto485.h>
#include <Adafruit_MCP23X17.h>

#define RS485Serial Serial2

#define CMRI_ADDR 1
#define DE_PIN 2

int Tbit[16];           // number of servos
uint8_t servonum = 0;   // servo counter
uint8_t lastservo = 16;

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);    //setup the board address 0
//Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);    //setup the board address 1
Auto485 bus(DE_PIN,-1,Serial2); // Arduino pin 2 -> MAX485 DE and RE pins
CMRI cmri(CMRI_ADDR, 24, 48, bus); // addr, input bits, output bits, bus

// Initialize I2C MCP23017
Adafruit_MCP23X17 mcp0;
Adafruit_MCP23X17 mcp1;
Adafruit_MCP23X17 mcp2;

byte UpdateOutputValue=1;
byte OutputValue[6];   //Each byte is 8 bits - 8x6 48 bits
byte InputValue[3]; //Each byte is 8 bits - 8x3 24 bits
unsigned long TimerLoadInput;


void setup()
{
    Serial.begin(9600);
    bus.begin(9600, SERIAL_8N2);
    pwm1.begin();
    //pwm2.begin();
    pwm1.setPWMFreq(60);
    //pwm2.setPWMFreq(60);

    // Define MCP23017 modules
    // addresses from 0x20 - 0x27
    mcp0.begin_I2C(0x20); // 16-Inputs
    mcp1.begin_I2C(0x21); // 16-Outputs
    mcp2.begin_I2C(0x22); // 16-Outputs

    InitializePortsMCP23017();

    Serial.println("setup1 complete");
}

void loop()
{
//  Serial.println(bus.read());
    cmri.process();

    // Load Input Values
    if (millis()-TimerLoadInput>500)
    {
        LoadInputValues();
        TimerLoadInput = millis();
    }

    for(servonum = 0 ; servonum < lastservo; servonum ++)
    {

        Tbit[servonum] = (cmri.get_bit(servonum));
        // Serial.println(servonum, Tbit[servonum]);
        if(servonum <16)
        {
            if (Tbit[servonum] == 1)
            {
                pwm1.setPWM(servonum, 0, 100);
                //Serial.println("throw 1");
            }
            if (Tbit[servonum] == 0)
            {
                pwm1.setPWM(servonum, 0, 400);
                //Serial.println("close 1");
            }
        }
        /*
          if(servonum >=16 && servonum <= 31){
              if (Tbit[servonum] == 1){
                  pwm2.setPWM(servonum - 16, 0, 100);
                  //Serial.println("throw2");
                }
              if (Tbit[servonum] == 0){
                  pwm2.setPWM(servonum - 16, 0, 400);
                  //Serial.println("close 2");
                }
            }
          */
        // add additional lines for additional boards

    }

    // Check for change in output values
    for(int i = 0; i < 6; i++)
    {
        byte NewValue;
        NewValue = cmri.get_byte(i);
        //  Serial.println(NewValue);
        if (NewValue != OutputValue[i])
        {
            CompareOutputBits(NewValue, i);
        }
    }

    // Update of Output Values
    if (UpdateOutputValue>0) UpdateOutput();
}

void CompareOutputBits(byte tmpValue, byte bBlock)
{
    //Compare the New Value bits
    for(byte i = 0; i < 8; i++)
    {
        if (bitRead(tmpValue, i) != bitRead(OutputValue[bBlock],i))
        {
            if (bBlock<2)
            {
                i = i;  // This is where the servo code was called
                // -- review could improve efficiency
                // -- also a reason for only using one pca9685 pwm board
                //   NovoValueServo(bBlock * 8 + i, bitRead(tmpValue, i));
            }
            else
            {
                UpdateOutputValue=1;
            }
        }
    }
    OutputValue[bBlock] = tmpValue;
}

void LoadInputValues()
{
    for (byte i = 0; i < 2; i++)
    {
        // Input Values MCP23017
        byte NewValue = ReturnValueEnteredMCP23017(i);

        // Check value change
        if (NewValue != InputValue[i])
        {
            cmri.set_byte(i, ~NewValue);
            InputValue[i] = NewValue;
        }
    }
}

byte ReturnValueEnteredMCP23017(byte bInput)
{
    byte bValue;
    // Input ports 0 - MCP23017 0 / Port A
    if (bInput==0) bValue=mcp0.readGPIO(0);
    // Input ports 1 - MCP23017 0 / Port B
    if (bInput==1) bValue=mcp0.readGPIO(1);
    /*int inpPins[16] = {27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42};
    int iVal = 0;
    for (int i=0; i<8; i++) {
      if (bInput == 0) iVal = i;
      else if (bInput == 1) iVal = (i+8);
      bitWrite(bValue,i,digitalRead(inpPins[iVal])? 1:0);
    }
    // Returns bValue*/
    return bValue;
}

void InitializePortsMCP23017()
{
    //Module 0 - Input, Hardware Pins
    for (int p=0; p<16; p++)
    {
        mcp0.pinMode(p, INPUT_PULLUP);
    }
    //Module 1 - Output, MCP23017 Pins
    for (int p=0; p<16; p++)
    {
        mcp1.pinMode(p, OUTPUT);
    }
    //Module 2 - Output, hardware pins
    for (int p=0; p<=16; p++)
    {
        mcp2.pinMode(p, OUTPUT);
    }
}

void UpdateOutput()
{
    int tmpAB;

    // Output ports 2 and 3 (17 to 32) - MCP23017 1
    tmpAB = OutputValue[3];
    tmpAB <<= 8;
    tmpAB |= OutputValue[2];
    mcp1.writeGPIOAB(tmpAB);

    // Output ports 4 and 5 (32 to 48)- MCP23017 2
    tmpAB = OutputValue[5];
    tmpAB <<= 8;
    tmpAB |= OutputValue[4];
    mcp2.writeGPIOAB(tmpAB);
    /*int pinsA[8] = {3,4,5,6,7,8,9,10};
    int pinsB[8] = {11,12,13,22,23,24,25,26};
    for (int i=0; i<8; i++) {
      digitalWrite(pinsA[i],bitRead(OutputValue[5],i)? HIGH:LOW);
    }
    for (int i=0; i<8; i++) {
      digitalWrite(pinsB[i],bitRead(OutputValue[4],i)? HIGH:LOW);
    }*/
}
