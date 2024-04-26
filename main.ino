#include <Wire.h>
#include <VL6180X.h>
#include <Adafruit_Sensor.h>
#include <NewPing.h>
// #include <Adafruit_MPU6050.h>

#define maxDistance 50
#define PING_INTERVAL 35 // ping interval must be high (30 is not enough) change later todo Nowfn
#define SONAR_NUM 4
#define MOTOR_DELAY 30

const int trigPin1 = 22; // front left
const int echoPin1 = 23;

const int trigPin2 = 24; // front right
const int echoPin2 = 25;

const int trigPin3 = 28; // left side
const int echoPin3 = 29;

const int trigPin4 = 26; // right side
const int echoPin4 = 27;

NewPing sonar[SONAR_NUM] = {                  // Sensor object array.
    NewPing(trigPin1, echoPin1, maxDistance), // Each sensor's trigger pin, echo pin, and max distance to ping.
    NewPing(trigPin2, echoPin2, maxDistance),
    NewPing(trigPin3, echoPin3, maxDistance),
    NewPing(trigPin4, echoPin4, maxDistance)};

const int frontLeftIR = 2;
const int frontRightIR = 3;
const int backLeftIR = 12;
const int backRightIR = 13;

const int enaPin = 6;
const int in1Pin = 7;
const int in2Pin = 8;

const int enbPin = 11;
const int in3Pin = 9;
const int in4Pin = 10;

VL6180X tofSensor;
// Adafruit_MPU6050 mpu; gyroscope broke,

unsigned long pingTimer[SONAR_NUM];
unsigned int ultrasonic_status[SONAR_NUM];
int currentSensor = 0;

int lineSensorStatus = 0;
bool tofStatus = 0;

bool turnDirection = true;
bool searchMode;
unsigned long motorStopDelay;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(trigPin1, OUTPUT);
    pinMode(trigPin2, OUTPUT);
    pinMode(trigPin3, OUTPUT);
    pinMode(trigPin4, OUTPUT);

    pinMode(echoPin1, INPUT);
    pinMode(echoPin2, INPUT);
    pinMode(echoPin3, INPUT);
    pinMode(echoPin4, INPUT);

    pinMode(enaPin, OUTPUT);
    pinMode(enbPin, OUTPUT);
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    pinMode(in3Pin, OUTPUT);
    pinMode(in4Pin, OUTPUT);

    tofSensor.init();
    tofSensor.configureDefault();
    tofSensor.setScaling(3);

    tofSensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
    tofSensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);

    tofSensor.setTimeout(500);

    tofSensor.stopContinuous();
    delay(300);
    tofSensor.startRangeContinuous(90);

    matchStart();
    for (int i = 1; i < SONAR_NUM; i++)
        pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void loop()
{
    for (int i = 0; i < 4; i++)
    {
        if (millis() >= pingTimer[i])
        {
            pingTimer[i] += PING_INTERVAL * SONAR_NUM;
            if (i == 0 && currentSensor == SONAR_NUM - 1)
            {
                oneSensorCycle();
            }
            if (i == 3)
            {
                frontSensorStatus();
            }
            sonar[currentSensor].timer_stop();
            currentSensor = i;
            ultrasonic_status[currentSensor] = 0;
            sonar[currentSensor].ping_timer(echoCheck);
        }
    }
    sensorCheck();

    switch (lineSensorStatus)
    {
    case 1:
        setMotorSpeed(-255, -185);
        turnDirection(true);
        break;
    case 2:
        setMotorSpeed(-185, -255);
        turnDirection(false);
        break;
    case 3:
        setMotorSpeed(255, 185);
        turnDirection(true);
        break;
    case 4:
        setMotorSpeed(185, 255);
        turnDirection(false);
        break;
    }
    
    if (searchMode) {
        digitalWrite(LED_BUILTIN, HIGH);
    }
    else {
        digitalWrite(LED_BUILTIN, LOW);
    }
}

void echoCheck()
{
    if (sonar[currentSensor].check_timer())
        ultrasonic_status[currentSensor] = 1;
}

void sensorCheck()
{
    if (digitalRead(frontLeftIR))
    {
        lineSensorStatus = 1;
    }
    else if (digitalRead(frontRightIR))
    {
        lineSensorStatus = 2;
    }
    else if (digitalRead(backLeftIR))
    {
        lineSensorStatus = 3;
    }
    else if (digitalRead(backRightIR))
    {
        lineSensorStatus = 4;
    }
    else
    {
        lineSensorStatus = 0;
    }
}

void matchStart()
{
    int distance_front = tofSensor.readRangeContinuousMillimeters();
    while (distance_front < 300)
    {
        distance_front = tofSensor.readRangeContinuousMillimeters();
    }
}

void oneSensorCycle()
{
    if (ultrasonic_status[0])
    {
        if (searchMode)
        {
            setMotorSpeed(0, 0);
            motorStopDelay = millis() + MOTOR_DELAY;
            searchMode = false;
        }

        if (millis() > motorStopDelay)
        {
            if (tofStatus)
            {
                if (ultrasonic_status[1])
                { // enemy is directly in front
                    setMotorSpeed(255, 255);
                }
                else
                {
                    setMotorSpeed(200, 255); // enemy is to the front left of the robot, go slight left but forwards
                }
            }
            else
            {
                // enemy is to the left of the robot, go left harder  but also a bit forwards
                setMotorSpeed(185, 255);
            }
        }
    }
    else if (tofStatus)
    {
        searchMode = false;
        if (ultrasonic_status[1])
        {
            setMotorSpeed(255, 200); // enemy is to the front right of the robot, go slight right but forwards
        }
        else
        { // enemy is straight on, go forwards
            setMotorSpeed(255, 255);
        }
    }
    else if (ultrasonic_status[1])
    {
        if (searchMode)
        {
            setMotorSpeed(0, 0);
            motorStopDelay = millis() + MOTOR_DELAY;
            searchMode = false;
        }

        if (millis() > motorStopDelay)
        {
            // enemy is to the right of the robot, go right harder, but also a bit forwards
            setMotorSpeed(255, 185);
        }
    }
    else
    { // search mode
        searchMode = true;
        setMotorSpeed(75, 75);
        if (ultrasonic_status[2])
        {
            turnOnTheSpot(true);
            turnDirection = true;
        }
        else if (ultrasonic_status[3])
        {
            turnOnTheSpot(false);
            turnDirection = false;
        }
        else
        {
            turnOnTheSpot(turnDirection);
        }
    }

    pingTimer[0] = millis() + 10;
    for (int i = 1; i < SONAR_NUM; i++)
        pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void frontSensorStatus()
{
    int distance = tofSensor.readRangeContinuousMillimeters();
    if (distance != 765)
    {
        tofStatus = 1;
    }
    else
    {
        tofStatus = 0;
    }
}

void turnOnTheSpot(bool direction)
{
    if (direction)
    {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);

        digitalWrite(in3Pin, HIGH);
        digitalWrite(in4Pin, LOW);
    }
    else
    {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);

        digitalWrite(in3Pin, LOW);
        digitalWrite(in4Pin, HIGH);
    }
}

void setMotorSpeed(int speedB, int speedA)
{
    if (speedA < 0)
    {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
        speedA = abs(speedA);
    }
    else
    {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
    }
    if (speedB < 0)
    {
        digitalWrite(in3Pin, HIGH);
        digitalWrite(in4Pin, LOW);
        speedB = abs(speedB);
    }
    else
    {
        digitalWrite(in3Pin, LOW);
        digitalWrite(in4Pin, HIGH);
    }

    analogWrite(enaPin, speedA);
    analogWrite(enbPin, speedB);
}