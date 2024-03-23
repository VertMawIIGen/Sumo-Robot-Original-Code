const int ena_pin = 7; // output left motor
const int in1_pin = 6; // forward
const int in2_pin = 5;

const int enb_pin = 2; // output right motor
const int in3_pin = 4;
const int in4_pin = 3;

const int trig = 8; // ultrasonic front
const int echo = 9;
const float speedOfSound = 0.034282;

void setup() {
    Serial.begin(9600);
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);

    pinMode(ena_pin, OUTPUT);
    pinMode(in1_pin, OUTPUT);
    pinMode(in2_pin, OUTPUT);

    pinMode(enb_pin, OUTPUT);
    pinMode(in3_pin, OUTPUT);
    pinMode(in4_pin, OUTPUT);
}

void loop() {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
    digitalWrite(in3_pin, LOW); // Forward.
    digitalWrite(in4_pin, HIGH);

    int distance = getUltrasoundDistance(trig, echo);
    if (distance < 10) {
        analogWrite(ena_pin, 0);
		analogWrite(enb_pin, 0);
    }
    else {
        analogWrite(ena_pin, 100);
		analogWrite(enb_pin, 100);
    }

}

long getUltrasoundDistance(int trigPin, int echoPin)
{
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

    long cm = (duration / 2) * speedOfSound;

    return cm;
}