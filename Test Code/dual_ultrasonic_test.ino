const int left_trig = 13;
const int left_echo = 12;
const int right_trig = 3;
const int right_echo = 2;

const float speedOfSound = 0.034282;

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
}

void loop()
{
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
        }
        else
        {
            Serial.println("Left.");
            Serial.println(distance_left);
            Serial.println(distance_right);
        }
    }
    else if (distance_right < 100)
    {
        Serial.println("Right.");
        Serial.println(distance_left);
        Serial.println(distance_right);
    }
    else {
      Serial.println("Neither");
      Serial.println(distance_left);
      Serial.println(distance_right);
    }
}