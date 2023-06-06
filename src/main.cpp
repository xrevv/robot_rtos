#include <Arduino.h>
#include <M5Core2.h>
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>

// Task stack size
#define STACK 2048

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
const int servoPosScreen[] = {20, 60, 100, 140, 180, 220};

// Servo max positions
const int servoPosMax[] = {180, 180, 180, 180, 130, 180};

// Servo positions
int servoPos[servoNum];

// Servo previous positions
int servoPPos[servoNum];

// Servo saved positions
int servoSPos[servoNum][memory];

// Position in saved positions
int indexS = 0;

enum Mode
{
    move = 0,
    play = 1,
    stop = 2
};

struct MoveCommand
{
    int whichServo;
    int posServo;
    Mode mode;
};

// Queue
static QueueHandle_t xQueueData;
static QueueHandle_t xQueueMove;
static QueueHandle_t xQueueDraw;

// Prototypes
// Tasks
void taskBT(void *param);
void taskParser(void *param);
void taskMoveServo(void *param);
void taskUpdateScreen(void *param);

// Functions
void setMem();
void saveMem();
bool checkPos(int pos);
int mapper(int value);
void updateText();

// Tasks

void setup()
{
    disableCore0WDT();

    M5.begin(true, true, true, true);
    M5.Lcd.drawJpgFile(SD, "/logo.jpg");

    Serial.begin(115200);

    // Initializing PWM driver
    pwm.begin();
    pwm.setPWMFreq(60);

    // Initializing Bluetooth
    Bluetooth.begin("M5Core2");

    // Setting initial state
    setMem();
    for (size_t i = 0; i < servoNum; i++)
    {
        servoPos[i] = servoSPos[i][0];
        pwm.writeMicroseconds(i, mapper(servoPos[i]));
    }

    while (!Bluetooth.available()) // Check BT
    {
        // Nothing to do
    }

    // Creating queues
    xQueueData = xQueueCreate(10, sizeof(String));
    xQueueMove = xQueueCreate(10, sizeof(MoveCommand));
    xQueueDraw = xQueueCreate(10, sizeof(int));

    // Initializing the screen
    M5.Lcd.clear(BLACK);
    for (size_t i = 0; i < servoNum; i++)
    {
        if (xQueueSend(xQueueDraw, (void *)&i, 10) != pdTRUE)
        {
            Serial.println("ERROR: Could not put item on draw queue.");
        }
    }
    updateText();

    // Creating tasks
    xTaskCreate(taskBT, "BT", STACK, NULL, 1, NULL);
    xTaskCreate(taskParser, "Parser", STACK, NULL, 2, NULL);
    xTaskCreate(taskMoveServo, "MoveServo", STACK, NULL, 3, NULL);
    xTaskCreate(taskUpdateScreen, "updateScreen", STACK, NULL, 4, NULL);

    vTaskDelete(NULL);
}

void loop() {}

void taskBT(void *param)
{
    String data;

    while (1)
    {
        while (!Bluetooth.available()) // Check BT
        {
            vTaskDelay(pdMS_TO_TICKS(100)); // Nothing to do
        }

        data = Bluetooth.readStringUntil('n'); // Receive BT data
        Serial.println(data);
        if (xQueueSend(xQueueData, (void *)&data, 10) != pdTRUE)
        {
            Serial.println("ERROR: Could not put item on data queue.");
        }
    }
}

void taskParser(void *param)
{
    String data;
    MoveCommand command;

    while (1)
    {
        if (xQueueReceive(xQueueData, (void *)&data, 0) == pdTRUE)
        {
            if (data.startsWith("s")) // Servo position?
            {
                command.whichServo = data.substring(1, 2).toInt() - 1;
                command.posServo = data.substring(2, data.length()).toInt();
                command.mode = move;

                xQueueSend(xQueueMove, (void *)&command, 10);
                xQueueSend(xQueueDraw, (void *)&command.whichServo, 10);
            }
            else if (data.startsWith("c")) // Command?
            {
                // Which command?
                int SelectFunc = data.substring(1, 2).toInt();

                switch (SelectFunc)
                {
                case 1: // Save
                    // Save all current servo positions
                    saveMem();
                    break;

                case 2:                  // Play
                    command.mode = play; // Repeat until Stop command
                    xQueueSend(xQueueMove, (void *)&command, 10);
                    break;

                case 3:                  // Stop
                    command.mode = stop; // Stop command
                    xQueueSend(xQueueMove, (void *)&command, 10);
                    break;

                case 4:       // Reset
                    setMem(); // Setting initial positions
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
    }
}

void taskMoveServo(void *param)
{
    MoveCommand command;
    while (1)
    {
        if (xQueueReceive(xQueueMove, (void *)&command, 0) == pdTRUE)
        {
            switch (command.mode)
            {
            case move:
            {
                int whichServo = command.whichServo;
                int posServo = command.posServo;

                servoPos[whichServo] = posServo;

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
                break;
            }

            case play: // Repeat until Stop command
            {
                while (command.mode != stop)
                {
                    for (int j = 0; j < indexS; j++)
                    {
                        xQueueReceive(xQueueMove, (void *)&command, 10);
                        if (command.mode == stop) // Stop?
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
                                xQueueSend(xQueueDraw, (void *)&i, 10);
                            }
                        }
                    }
                }
                break;
            }

            default:
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void taskUpdateScreen(void *param)
{
    int servo;
    while (1)
    {
        if (xQueueReceive(xQueueDraw, (void *)&servo, 0) == pdTRUE)
        {
            int pos = (servoPos[servo] / float(servoPosMax[servo])) * 320;
            M5.Lcd.fillRect(0, servoPosScreen[servo], pos, 20, RED);
            M5.Lcd.fillRect(pos, servoPosScreen[servo], 320, 20, BLACK);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Functions

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

void saveMem()
{
    if (indexS < memory)
    {
        for (size_t i = 0; i < servoNum; i++)
        {
            servoSPos[i][indexS] = servoPos[i];
        }
        indexS++;
    }
}

int mapper(int value)
{
    return map(value, 0, 180, minUs, maxUs);
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