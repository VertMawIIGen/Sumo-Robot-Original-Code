const int ena_pin = 1; // output left motor
const int in1_pin = 2;
const int in2_pin = 3;

const int enb_pin = 4; // output right motor
const int in3_pin = 5;
const int in4_pin = 6;

void setup() {
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
    digitalWrite(in3_pin, HIGH);
    digitalWrite(in4_pin, LOW);

    analogWrite(ena_pin, 100);
    
}
