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
    pinMode(backRightIR, INPUT);

    digitalWrite(in1_pin, HIGH); // forward left motor
    digitalWrite(in2_pin, LOW);

    digitalWrite(in3_pin, LOW);
    digitalWrite(in4_pin, HIGH); // forward right motor
    Serial.println("This ran.");
}

void loop()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  int distance_front = sensor.readRangeContinuousMillimeters();

  int left_time = ultrasonicDistance(left_trig, left_echo);
  int right_time = ultrasonicDistance(right_trig, right_echo);

  int irfrontLeft = analogRead(frontLeftIR);
  int irfrontRight = analogRead(frontRightIR);
  int irbackRight = analogRead(backRightIR);

  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");
  
  Serial.print("TOF sensor: ");
  Serial.println(distance_front);

  Serial.print("Ultrasonic sensors: ");
  Serial.print(left_time);
  Serial.print(", ");
  Serial.println(right_time);

  Serial.print("Line sensors: ");
  Serial.print(irfrontLeft);
  Serial.print(", ");
  Serial.print(irfrontRight);
  Serial.print(", ");
  Serial.println(irbackRight);
  Serial.println("");
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