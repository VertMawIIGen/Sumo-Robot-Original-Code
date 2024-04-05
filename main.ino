#include <Wire.h>
#include <VL6180X.h>

const int left_trig = 27;
const int left_echo = 26;
const int right_trig = 25;
const int right_echo = 24;

const float speedOfSound = 0.034282;

const int ena_pin = 7; // output left motor
const int in1_pin = 6;
const int in2_pin = 5;

const int enb_pin = 2; // output right motor
const int in3_pin = 4;
const int in4_pin = 3;

VL6180X sensor;

const int frontIR1 = A0;

void setup() {
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

    pinMode(frontIR1, INPUT);
    
    digitalWrite(in1_pin, HIGH); // forward left motor
    digitalWrite(in2_pin, LOW);

    digitalWrite(in3_pin, LOW);
    digitalWrite(in4_pin, HIGH); // forward right motor

    //matchstart();
}

void loop() {
    // int frontLine = analogRead(frontIR1);
    int distance_front = sensor.readRangeContinuousMillimeters();
    int left_time = ultrasonicDistance(left_trig, left_echo);
    int right_time = ultrasonicDistance(right_trig, right_echo);

   /* if (frontLine > 300) { // if there is a line
        backwardsMotor();
        setMotorSpeed(255);
        delay(250);
        setMotorSpeed(0);
        turnRightOnTheSpot();
        setMotorSpeed(255);
        delay(250);
        forwardMotor();
    } */

    if (distance_front < 200) {
        if (left_time < 1000) {
            turnLeftOnTheSpot();
            setMotorSpeed(85);
        }
        else if (right_time < 1000) {
            turnRightOnTheSpot();
            setMotorSpeed(85);
        }
        else {
            setMotorSpeed(0);
        }
    }
    else if (left_time < 1000) {
        turnLeftOnTheSpot();
        setMotorSpeed(100);
    }
    else if (right_time < 1000) {
        turnRightOnTheSpot();
        setMotorSpeed(100);
    }
    else {
        setMotorSpeed(0);
    }
}

long ultrasonicDistance(int trigPin, int echoPin) { // 1000 is 20cm roughly 
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

void forwardMotor() {
    digitalWrite(in1_pin, HIGH); // forward left motor
    digitalWrite(in2_pin, LOW);

    digitalWrite(in3_pin, LOW);
    digitalWrite(in4_pin, HIGH); // forward right motor
}

void backwardsMotor() {
    digitalWrite(in1_pin, LOW); // backward left motor
    digitalWrite(in2_pin, HIGH);

    digitalWrite(in3_pin, HIGH);
    digitalWrite(in4_pin, LOW); // backward right motor
}

void turnLeftOnTheSpot() {
    digitalWrite(in1_pin, LOW); // backward left motor
    digitalWrite(in2_pin, HIGH);

    digitalWrite(in3_pin, LOW); // forward right
    digitalWrite(in4_pin, HIGH);
}

void turnRightOnTheSpot() {
    digitalWrite(in1_pin, HIGH); // forward left motor
    digitalWrite(in2_pin, LOW);

    digitalWrite(in3_pin, HIGH);
    digitalWrite(in4_pin, LOW); // backward right motor
}

void setMotorSpeed(int speed) {
    analogWrite(ena_pin, speed);
    analogWrite(enb_pin, speed);
}