const int ena_pin = 7; // output left motor
const int in1_pin = 6;
const int in2_pin = 5;

const int enb_pin = 2; // output right motor
const int in3_pin = 4;
const int in4_pin = 3;

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
    digitalWrite(in3_pin, LOW);
    digitalWrite(in4_pin, HIGH);

    for (int i = 0; i < 256; i++) {
		analogWrite(ena_pin, i);
		analogWrite(enb_pin, i);
		delay(20);
    }
	
    for (int i = 255; i >= 0; --i) {
      analogWrite(ena_pin, i);
      analogWrite(enb_pin, i);
      delay(20);
    }
    
}
