#include <Wire.h>
#include <VL6180X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

const int left_trig = 10;
const int left_echo = 11;
const int right_trig = 9;
const int right_echo = 8;

const float speedOfSound = 0.034282;

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

float biggestAcceleration = 0;

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

    matchStart(); //confirmed to be working
    Serial.print("THE MATCH BEGINS.");
}

void loop()
{
    Serial.println("A loop has started again.");
    int irfrontLeft = analogRead(frontLeftIR);
    int irfrontRight = analogRead(frontRightIR);
    int irbackRight = analogRead(backRightIR);
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    float sideAcceleration = a.acceleration.y;
    if (abs(sideAcceleration) > abs(biggestAcceleration))
    {
        biggestAcceleration = sideAcceleration;
    }
    

    if ((irfrontLeft > 500) || (irfrontRight > 500) || (irbackRight > 500)) // above 500 is black enough to be considered a line.
    {
        Serial.println("Line has been detected");
        do
        {
            if (irfrontLeft > 500)
            {
                backwardsMotor();
                setLeftMotorSpeed(255);
                setRightMotorSpeed(230);
                Serial.println("LINE DETECTED FRONT LEFT.");
            }
            else if (irfrontRight > 500)
            {
                backwardsMotor();
                setLeftMotorSpeed(230);
                setRightMotorSpeed(255);
                Serial.println("LINE DETECTED FRONT RIGHT.");
            }
            else if (irbackRight > 500)
            {
                forwardMotor();
                setLeftMotorSpeed(255);
                setRightMotorSpeed(230);
                Serial.println("LINE DETECTED BACK RIGHT.");
            }
        } while (lineChecking());
        setBothMotorSpeeds(0);
        Serial.println("Black line loop has ended.");
    }
    
    
    else if (abs(sideAcceleration) > 4)
    { // got hit on the side
        Serial.println("Got hit on the side loop has STARTED.");
        do
        {
            if (biggestAcceleration < 0)
            { // got hit on the right side
                setLeftMotorSpeed(255);
                setRightMotorSpeed(200);
                Serial.println("GOT HIT ON THE RIGHT.");
            }
            else
            { // got hit on the left side
                setLeftMotorSpeed(200);
                setRightMotorSpeed(255);
                Serial.println("GOT HIT ON THE LEFT.");
            }
            delay(100);
            setBothMotorSpeeds(0);
            Serial.println("Set motor speed 0.");
        } while (frontDetection() && lineChecking());
        biggestAcceleration = 0;
        Serial.println("SIDE ACCELERATION LOOP ENDED.");
    }
    
    else
    {
        int distance_front = sensor.readRangeContinuousMillimeters();
        int left_time = ultrasonicDistance(left_trig, left_echo);
        int right_time = ultrasonicDistance(right_trig, right_echo);

        Serial.println("Reading Distance Sensors");

        if (distance_front < 300)
        {
            Serial.println("DISTANCE FRONT LESS THEN 300!");
            forwardMotor();
            setBothMotorSpeeds(255);
            if ((left_time < 1749) || (right_time < 1749))
            {
                if (right_time < 1749)
                {
                    setLeftMotorSpeed(255);
                    setRightMotorSpeed(230);
                    Serial.println("Slight Right!");
                }
                else if (left_time < 1749)
                {
                    setRightMotorSpeed(255);
                    setLeftMotorSpeed(230);
                    Serial.println("Slight LEFT!");
                }
                else
                {
                    forwardMotor();
                    setBothMotorSpeeds(255);
                    Serial.println("ALL SENSORS COVERED!");
                }
            }
        }
        else if (left_time < 1749)
        {
            turnLeftOnTheSpot();
            setBothMotorSpeeds(180);
            Serial.println("Left Sensor ONLY!");
        }
        else if (right_time < 1749)
        {
            turnRightOnTheSpot();
            setBothMotorSpeeds(180);
            Serial.println("RIGHT Sensor ONLY!");
        }
        else
        {
            Serial.println("SEarch MODE!");
            if (biggestAcceleration < 0)
            {
                turnRightOnTheSpot();
                setBothMotorSpeeds(90);
                Serial.println("TURNING RIGHT!");
            }
            else
            {
                turnLeftOnTheSpot();
                setBothMotorSpeeds(90);
                Serial.println("TURNING LEFT!");
            }
        }
    }
}

long ultrasonicDistance(int trigPin, int echoPin)
{ // 1000 is 20cm roughly
    long duration;

    do
    {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(2);
        digitalWrite(trigPin, LOW);

        duration = pulseIn(echoPin, HIGH);
    } while (duration == 0);

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

bool frontDetection() {
    int distance_front = sensor.readRangeContinuousMillimeters();
    int left_time = ultrasonicDistance(left_trig, left_echo);
    int right_time = ultrasonicDistance(right_trig, right_echo);


    if ((distance_front < 300) || (left_time < 1749) || (right_time < 1749)) {
        Serial.println("FRONT DETECTION RETURNS FALSE");
        return false;
    }
    else {
        Serial.println("FRONT DETECTIOn returning TRUE!!!");
        return true;
    }
}

bool lineChecking() {
    int irfrontLeft = analogRead(frontLeftIR);
    int irfrontRight = analogRead(frontRightIR);
    int irbackRight = analogRead(backRightIR);

    if ((irfrontLeft > 600) || (irfrontRight > 600) || (irbackRight > 600)) {
        Serial.println("LINE CHECKING TRUE!!!");
        return true;
    }
    else {
        Serial.println("LINE CHECKING FALSE!");
        return false;
    }
}
