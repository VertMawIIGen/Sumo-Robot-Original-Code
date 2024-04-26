#include <Wire.h>
#include <VL6180X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

const int left_trig = 10;
const int left_echo = 11;
const int right_trig = 9;
const int right_echo = 8;

const int ena_pin = 7; // output left motor
const int in1_pin = 6;
const int in2_pin = 5;

const int enb_pin = 2; // output right motor
const int in3_pin = 4;
const int in4_pin = 3;

VL6180X sensor;
Adafruit_MPU6050 mpu;

const int frontRightIR = A1;
const int frontLeftIR = A0;
const int backRightIR = A2;

int valueLineChecking = 0;
int valueFrontChecking = 6;
float biggestAcceleration = 0; // change data type for efficiency

void setup()
{
    Serial.begin(9600);
    Wire.begin();
    sensor.init();
    sensor.configureDefault();
    sensor.setScaling(3);

    sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
    sensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);

    sensor.setTimeout(500);

    sensor.stopContinuous();
    delay(300);
    sensor.startInterleavedContinuous(100);

    if (!mpu.begin())
    {
        while (1)
        {
            delay(10);
        }
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    mpu.setGyroStandby(true, true, true);
    mpu.setAccelerometerStandby(true, false, true);

    delay(100);

    pinMode(left_trig, OUTPUT);
    pinMode(left_echo, INPUT);
    pinMode(right_trig, OUTPUT);
    pinMode(right_echo, INPUT);

    pinMode(ena_pin, OUTPUT);
    pinMode(in1_pin, OUTPUT);
    pinMode(in2_pin, OUTPUT);

    pinMode(enb_pin, OUTPUT);
    pinMode(in3_pin, OUTPUT);
    pinMode(in4_pin, OUTPUT);

    pinMode(frontLeftIR, INPUT);
    pinMode(frontRightIR, INPUT);

    digitalWrite(in1_pin, HIGH); // forward left motor
    digitalWrite(in2_pin, LOW);

    digitalWrite(in3_pin, LOW);
    digitalWrite(in4_pin, HIGH); // forward right motor

    matchStart(); // confirmed to be working
}

void loop()
{
    valueLineChecking = lineChecking();
    Serial.println("Line checking!");

    if (lineChecking() != 0)
    {
        Serial.println("Line found!");
        do
        {
            if (valueLineChecking = 1)
            { // front left
                backwardsMotor();
                setLeftMotorSpeed(255);
                setRightMotorSpeed(230);
                Serial.println("Line checking value is 1!");
            }
            else if (valueLineChecking = 2)
            { // front right
                backwardsMotor();
                setLeftMotorSpeed(230);
                setRightMotorSpeed(255);
                Serial.println("Line checking value is 2!");
            }
            else if (valueLineChecking = 3)
            { // back right
                forwardMotor();
                setLeftMotorSpeed(255);
                setRightMotorSpeed(230);
                Serial.println("Line checking value is 3!");
            }
            valueLineChecking = lineChecking();
        } while (valueLineChecking != 0);
        Serial.println("Line checking loop has ended.");
    }
    else
    {
        Serial.println("Front checking!");

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        float sideAcceleration = a.acceleration.y;
        if (abs(sideAcceleration) > abs(biggestAcceleration))
        {
            biggestAcceleration = sideAcceleration;
        }

        if (abs(sideAcceleration) > 4)
        { // got hit on the side
            Serial.println("Side acceleration loop has started!");
            do
            {
                if (sideAcceleration < 0)
                { // got hit on the right side
                    setLeftMotorSpeed(100);
                    setRightMotorSpeed(80);
                }
                else
                { // got hit on the left side
                    setLeftMotorSpeed(80);
                    setRightMotorSpeed(100);
                }
                delay(100);
                setBothMotorSpeeds(0);

                valueLineChecking = lineChecking();
                valueFrontChecking = frontDetection();
                Serial.println("Side acceleration looping.");
            } while ((valueLineChecking = 0) && (valueFrontChecking != 0));
            biggestAcceleration = 0;
            Serial.println("Side acceleration loop ended.");
        }
        else
        {
            valueFrontChecking = frontDetection();
            Serial.println("Front checking is going.");

            if (valueFrontChecking = 1)
            {
                setLeftMotorSpeed(100);
                setRightMotorSpeed(70);
                Serial.println("Front checking value is 1");
            }
            else if (valueFrontChecking = 2)
            {
                setRightMotorSpeed(100);
                setLeftMotorSpeed(70);
                Serial.println("Front checking value is 2");
            }
            else if (valueFrontChecking = 3)
            {
                forwardMotor();
                setBothMotorSpeeds(100);
                Serial.println("Front checking value is 3");
            }
            else if (valueFrontChecking = 4)
            {
                turnLeftOnTheSpot();
                setBothMotorSpeeds(75);
                Serial.println("Front checking value is 4");
            }
            else if (valueFrontChecking = 5)
            {
                turnRightOnTheSpot();
                setBothMotorSpeeds(75);
                Serial.println("Front checking value is 5");
            }
            else if (valueFrontChecking = 6)
            {
                Serial.println("Front checking value is 6");
                if (biggestAcceleration < 0)
                {
                    turnRightOnTheSpot();
                    setBothMotorSpeeds(70);
                    Serial.println("Turning right!");
                }
                else
                {
                    turnLeftOnTheSpot();
                    setBothMotorSpeeds(70);
                    Serial.println("Turning right!");
                }
            }
        }
    }
}

int ultrasonicDistance(int trigPin, int echoPin)
{ // distance in cm is duration / 58.31
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    int duration = pulseIn(echoPin, HIGH, 5900);

    return duration;
}

void forwardMotor()
{
    digitalWrite(in1_pin, HIGH); // forward left motor
    digitalWrite(in2_pin, LOW);

    digitalWrite(in3_pin, LOW);
    digitalWrite(in4_pin, HIGH); // forward right motor
    delayMicroseconds(10);
}

void backwardsMotor()
{
    digitalWrite(in1_pin, LOW); // backward left motor
    digitalWrite(in2_pin, HIGH);

    digitalWrite(in3_pin, HIGH);
    digitalWrite(in4_pin, LOW); // backward right motor
    delayMicroseconds(10);
}

void turnLeftOnTheSpot()
{
    digitalWrite(in1_pin, LOW); // backward left motor
    digitalWrite(in2_pin, HIGH);

    digitalWrite(in3_pin, LOW); // forward right
    digitalWrite(in4_pin, HIGH);
    delayMicroseconds(10);
}

void turnRightOnTheSpot()
{
    digitalWrite(in1_pin, HIGH); // forward left motor
    digitalWrite(in2_pin, LOW);

    digitalWrite(in3_pin, HIGH);
    digitalWrite(in4_pin, LOW); // backward right motor
    delayMicroseconds(10);
}

void setBothMotorSpeeds(int speed)
{
    analogWrite(ena_pin, speed);
    analogWrite(enb_pin, speed);
    delayMicroseconds(10);
}

void setLeftMotorSpeed(int speed)
{
    analogWrite(ena_pin, speed);
    delayMicroseconds(10);
}

void setRightMotorSpeed(int speed)
{
    analogWrite(enb_pin, speed);
    delayMicroseconds(10);
}

void matchStart()
{
    int distance_front = sensor.readRangeContinuousMillimeters();
    while (distance_front < 300)
    {
        delayMicroseconds(10);
        distance_front = sensor.readRangeContinuousMillimeters();
    }
}

int frontDetection() // check the logic later
{
    int distance_front = sensor.readRangeContinuousMillimeters();
    int left_time = ultrasonicDistance(left_trig, left_echo);
    int right_time = ultrasonicDistance(right_trig, right_echo);

    if (distance_front < 300)
    {
        if ((left_time < 1749) && (right_time < 1749))
        {
            return 3;
        }
        else if (left_time < 1749)
        {
            return 2;
        }
        else if (right_time < 1749)
        {
            return 1;
        }
        else
        {
            return 6;
        }
    }
    else
    {
        if (left_time < 1749)
        {
            return 4;
        }
        else if (right_time < 1749)
        {
            return 5;
        }
        else
        {
            return 0;
        }
    }
}

int lineChecking()
{
    int irfrontLeft = analogRead(frontLeftIR);
    int irfrontRight = analogRead(frontRightIR);
    int irbackRight = analogRead(backRightIR);

    if (irfrontLeft > 430)
    {
        return 1;
    }
    else if (irfrontRight > 430)
    {
        return 2;
    }
    else if (irbackRight > 550)
    {
        return 3;
    }
    else
    {
        return 0;
    }
}