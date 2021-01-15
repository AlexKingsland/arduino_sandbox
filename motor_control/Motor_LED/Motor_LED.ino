const int analogPin = A0;   // the pin that the potentiometer is attached to
const int ledCount = 8;    // the number of LEDs in the bar graph

int ledPins[] = {
  2, 3, 4, 5, 6, 7, 8, 9, 
};   // an array of pin numbers to which LEDs are attached

int fwdPin = 11;
int revPin = 12;
int controlPin1 = 13;
int controlPin2 = 10;

void setup() {
  // loop over the pin array and set them all to output:
  for (int thisLed = 0; thisLed < ledCount; thisLed++) {
    pinMode(ledPins[thisLed], OUTPUT);
  }

  pinMode(fwdPin, OUTPUT);
  pinMode(revPin, OUTPUT);

  pinMode(controlPin1, INPUT_PULLUP);
  pinMode(controlPin2, INPUT_PULLUP);
}

void loop() {
  // read the potentiometer:
  int sensorReading = analogRead(analogPin);
  // map the result to a range from 0 to the number of LEDs:
  int ledLevel = map(sensorReading, 0, 1023, 0, ledCount);

  // loop over the LED array:
  for (int thisLed = 0; thisLed < ledCount; thisLed++) {
    // if the array element's index is less than ledLevel,
    // turn the pin for this element on:
    if (thisLed < ledLevel) {
      digitalWrite(ledPins[thisLed], HIGH);
    }
    // turn off all pins higher than the ledLev:
    else {
      digitalWrite(ledPins[thisLed], LOW);
    }
  }
  
  //Motor control code
  //Divide analog input by 4 so it can be used in snslogWrite function
  int output = sensorReading / 4;
  if(digitalRead(controlPin1) == HIGH){
    analogWrite(fwdPin, output);
  }else if(digitalRead(controlPin2) == HIGH){
    analogWrite(revPin, output);
  }
  

  delay(25);
}
