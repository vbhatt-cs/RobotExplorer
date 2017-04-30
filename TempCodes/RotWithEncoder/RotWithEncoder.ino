#define LPWM D8
#define LDIR1 D7
#define LDIR2 D6

#define RPWM D5
#define RDIR1 D4
#define RDIR2 D0

#define ENCODER D3

const int RIGHT_SS_SPEED = 300;
const int LEFT_SS_SPEED = 300;
int curDistance = 0;

void setup() {
  // put your setup code here, to run once:
    pinMode(ENCODER, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER), encISR, FALLING);

    pinMode(RPWM, OUTPUT);
    pinMode(RDIR1, OUTPUT);
    pinMode(RDIR2, OUTPUT);

    pinMode(LPWM, OUTPUT);
    pinMode(LDIR1, OUTPUT);
    pinMode(LDIR2, OUTPUT);

    delay(5000);
    
    digitalWrite(RDIR2, LOW);
    digitalWrite(RDIR1, HIGH);
    digitalWrite(LDIR2, LOW);
    digitalWrite(LDIR1, HIGH);
    analogWrite(RPWM, RIGHT_SS_SPEED);
    analogWrite(LPWM, LEFT_SS_SPEED);
}

void loop() {
  // put your main code here, to run repeatedly:
    Serial.println(curDistance);
    if(curDistance > 25)
    {
        digitalWrite(LDIR1, LOW);
        digitalWrite(LDIR2, LOW);
        analogWrite(LPWM, 0);
        digitalWrite(RDIR1, LOW);
        digitalWrite(RDIR2, LOW);
        analogWrite(LPWM, 0);
    }
}

void encISR()
{
    curDistance++;
}
