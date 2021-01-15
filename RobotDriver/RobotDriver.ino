const int motor1fwdPin = 5;
const int motor1revPin = 6;

const int motor2fwdPin = 8;
const int motor2revPin = 9;

int x = 0;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(motor1fwdPin, OUTPUT);
  pinMode(motor1revPin, OUTPUT);

  pinMode(motor2fwdPin, OUTPUT);
  pinMode(motor2revPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(x == 0){
    //Move forward 1m
    digitalWrite(motor1fwdPin, LOW);
    digitalWrite(motor1revPin, HIGH);
    digitalWrite(motor2fwdPin, HIGH);
    digitalWrite(motor2revPin, LOW);
  }else if(x == 1){
    //Cleaning cycle
    for(int i = 0; i < 5; i++){
      digitalWrite(motor1fwdPin, LOW);
      digitalWrite(motor1revPin, HIGH);
      digitalWrite(motor2fwdPin, LOW);
      digitalWrite(motor2revPin, HIGH);
      delay(600);
      digitalWrite(motor1fwdPin, HIGH);
      digitalWrite(motor1revPin, LOW);
      digitalWrite(motor2fwdPin, HIGH);
      digitalWrite(motor2revPin, LOW);
      delay(600);
    }
    digitalWrite(motor1fwdPin, LOW);
    digitalWrite(motor1revPin, HIGH);
    digitalWrite(motor2fwdPin, LOW);
    digitalWrite(motor2revPin, LOW);
    delay(3000);
    digitalWrite(motor1fwdPin, LOW);
    digitalWrite(motor1revPin, LOW);
  }
  x++;
  delay(2150);

}
