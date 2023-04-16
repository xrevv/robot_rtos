#include <Arduino.h>
#include <M5Core2.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>

// Initial positions
#define servo01235InitPos 90;
#define servo4InitPos 0;

// Max memory size
#define memory 50

// Timing for degtoms
const uint16_t minUs = 1000;
const uint16_t maxUs = 2000;

// Number of servos
const int servoNum = 6;

// PWM driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Bluetooth
BluetoothSerial Bluetooth;

// Servo screen positions
int servoPosScreen[] = {20, 60, 100, 140, 180, 220};

// Servo max positions
int servoPosMax[] = {180, 180, 180, 180, 130, 180};

// Servo positions
int servoPos[servoNum];

// Servo previous positions
int servoPPos[servoNum];

// Servo saved positions
int servoSPos[servoNum][memory];

// Position in saved positions
int indexS = 0;

// Received BT data
String dataIn = "";

// Function prototypes
void moveServo(int whichServo, int PosServo);
void setMem();
bool checkPos(int pos);
int mapper(int value);
void updateScreen(int servo);
void updateText();

void setup()
{
    M5.begin(true, false, true, true);

    Serial.begin(115200);

    // Initializing PWM driver
    pwm.begin();
    pwm.setPWMFreq(60);

    // Setting initial state
    setMem();
    for (size_t i = 0; i < servoNum; i++)
    {
        servoPos[i] = servoSPos[i][0];
        updateScreen(i);
    }
    updateText();

    // Initializing Bluetooth
    Bluetooth.begin("M5Core2");
}

void loop()
{

    while (!Bluetooth.available()) // Check BT
    {
        // Nothing to do
    }

    dataIn = Bluetooth.readStringUntil('\n'); // Receive BT data
    Serial.println(dataIn);

    if (dataIn.startsWith("s")) // Servo position?
    {
        // Which servo?
        String dataInServo = dataIn.substring(1, 2);
        int SelectServo = dataInServo.toInt() - 1;

        // Which position?
        String dataInServoPos = dataIn.substring(2, dataIn.length());
        int PosServo = dataInServoPos.toInt();

        moveServo(SelectServo, PosServo);
        updateScreen(SelectServo);
    }
    else if (dataIn.startsWith("c")) // Command?
    {
        // Which command?
        String dataInFunc = dataIn.substring(1, 2);
        int SelectFunc = dataInFunc.toInt();

        switch (SelectFunc)
        {

        case 1: // Save

            // Save all current servo positions
            if (indexS < memory)
            {
                for (size_t i = 0; i < servoNum; i++)
                {
                    servoSPos[i][indexS] = servoPos[i];
                }
                indexS++;
            }

            break;

        case 2: // Play

            // Repeat until Stop command
            while (!dataIn.startsWith("c3"))
            {
                for (int j = 0; j < indexS; j++)
                {

                    dataIn = Bluetooth.readString(); // Receive BT data
                    if (dataIn.startsWith("c3"))     // Stop?
                    {
                        break;
                    }

                    // Move all servos simultaneously
                    while (checkPos(j)) // Check if all servos are in posiotion
                    {
                        for (size_t i = 0; i < servoNum; i++)
                        {
                            if (servoPos[i] > servoSPos[i][j])
                            {
                                pwm.writeMicroseconds(i, mapper(--servoPos[i]));
                            }
                            else if (servoPos[i] < servoSPos[i][j])
                            {
                                pwm.writeMicroseconds(i, mapper(++servoPos[i]));
                            }
                            updateScreen(i);
                        }
                    }
                }
            }

            break;

        case 4: // Reset

            setMem(); // Setting initial positions

            break;
        }
    }
}

void moveServo(int whichServo, int PosServo)
{
    servoPos[whichServo] = PosServo;

    // Movement speed adjustment
    int offset = 2;

    if (whichServo <= 2)
    {
        offset = 30;
    }

    if (servoPos[whichServo] > servoPPos[whichServo])
    {
        for (int i = servoPPos[whichServo]; i <= servoPos[whichServo]; i += offset)
        {
            pwm.writeMicroseconds(whichServo, mapper(i));
        }
        pwm.writeMicroseconds(whichServo, mapper(servoPos[whichServo]));
    }
    else if (servoPos[whichServo] < servoPPos[whichServo])
    {
        for (int i = servoPPos[whichServo]; i >= servoPos[whichServo]; i -= offset)
        {
            pwm.writeMicroseconds(whichServo, mapper(i));
        }
        pwm.writeMicroseconds(whichServo, mapper(servoPos[whichServo]));
    }

    servoPPos[whichServo] = servoPos[whichServo];
}
bool checkPos(int pos)
{
    for (size_t i = 0; i < servoNum; i++)
    {
        if (servoPos[i] - servoSPos[i][pos] != 0)
        {
            return 1;
        }
    }
    return 0;
}

void setMem()
{
    // Setting initial positions
    for (size_t j = 0; j < memory; j++)
    {
        for (size_t i = 0; i < servoNum; i++)
        {
            servoSPos[i][j] = servo01235InitPos;
        }
        servoSPos[4][j] = servo4InitPos;
    }
    indexS = 0;
}

int mapper(int value)
{
    return map(value, 0, 180, minUs, maxUs);
}

void updateScreen(int servo)
{
    int pos = (servoPos[servo] / float(servoPosMax[servo])) * 320;
    M5.Lcd.fillRect(0, servoPosScreen[servo], pos, 20, RED);
    M5.Lcd.fillRect(pos, servoPosScreen[servo], 320, 20, BLACK);
}

void updateText()
{
    M5.Lcd.drawString("Grip", 0, 2, 2);
    M5.Lcd.drawString("Wrist Pitch", 0, 42, 2);
    M5.Lcd.drawString("Wrist Roll", 0, 82, 2);
    M5.Lcd.drawString("Elbow", 0, 122, 2);
    M5.Lcd.drawString("Waist", 0, 162, 2);
    M5.Lcd.drawString("Rotation", 0, 202, 2);
}