int in1 = 6;
int in2 = 5;
int in3 = 4;
int in4 = 3;
int enA = 9;
int enB = 10;

void setup() {
  // put your setup code here, to run once:
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(enA, 200);
  analogWrite(enB, 200);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  delay(10000);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  delay(100);

}
