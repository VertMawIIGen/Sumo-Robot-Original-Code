#include <Wire.h>
#include <VL6180X.h>
#include <Adafruit_Sensor.h>
#include <NewPing.h>
// #include <Adafruit_MPU6050.h>

#define maxDistance 85
#define PING_INTERVAL 30
#define SONAR_NUM 4

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

const int frontLeftIR = 1;
const int frontRightIR = 2;
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

unsigned long pingTimer[SONAR_NUM];        // Holds the times when the next ping should happen for each sensor.
unsigned int ultrasonic_status[SONAR_NUM]; // Where the ping distances are stored.
int currentSensor = 0;

int lineSensorStatus = 0;
bool tofStatus = 0;

void setup()
{
    Serial.begin(9600);
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

    tofSensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 25);
    tofSensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);

    tofSensor.setTimeout(500);

    tofSensor.stopContinuous();
    delay(300);
    tofSensor.startRangeContinuous(50);

    matchStart();
    for (int i = 1; i < SONAR_NUM; i++) pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
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
            if (i == 3) {
                frontSensorStatus();
            }                           
            sonar[currentSensor].timer_stop();          
            currentSensor = i;                          
            ultrasonic_status[currentSensor] = 0;                      
            sonar[currentSensor].ping_timer(echoCheck); 
        }
    }
    sensorCheck();

    Serial.println("This ran.");

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

void oneSensorCycle() {
    Serial.println(tofStatus);
    Serial.print(ultrasonic_status[0]);
    Serial.print(", ");
    Serial.print(ultrasonic_status[1]);
    Serial.print(", ");
    Serial.print(ultrasonic_status[2]);
    Serial.print(", ");
    Serial.println(ultrasonic_status[3]);
}

void frontSensorStatus() {
    int distance = tofSensor.readRangeContinuousMillimeters();
    if (distance != 765) {
        tofStatus = 1;
    }
    else {
        tofStatus = 0;
    }
}