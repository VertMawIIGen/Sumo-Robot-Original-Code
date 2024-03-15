const int left_trig = 13;
const int left_echo = 12;
const int right_trig = 11;
const int right_echo = 10;

const float speedOfSound = 0.034282;

const int ena_pin = 7; // output left motor
const int in1_pin = 6;
const int in2_pin = 5;

const int enb_pin = 2; // output right motor
const int in3_pin = 4;
const int in4_pin = 3;

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

void setup()
{
    Serial.begin(9600);
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
}

void loop()
{
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
    digitalWrite(in3_pin, LOW);
    digitalWrite(in4_pin, HIGH);

    int distance_right = getUltrasoundDistance(left_trig, left_echo);
    delay(10);
    int distance_left = getUltrasoundDistance(right_trig, right_echo);

    if (distance_left < 100)
    {
        if (distance_right < 100)
        {
            Serial.println("Center.");
            Serial.println(distance_left);
            Serial.println(distance_right);
            analogWrite(ena_pin, 100);
		    analogWrite(enb_pin, 100);
        }
        else
        {
            Serial.println("Left.");
            Serial.println(distance_left);
            Serial.println(distance_right);
            analogWrite(ena_pin, 100);
        }
    }
    else if (distance_right < 100)
    {
        Serial.println("Right.");
        Serial.println(distance_left);
        Serial.println(distance_right);
		analogWrite(enb_pin, 100);
    }
    else {
      Serial.println("Neither");
      Serial.println(distance_left);
      Serial.println(distance_right);
      analogWrite(ena_pin, 0);
	  analogWrite(enb_pin, 0);
    }
}