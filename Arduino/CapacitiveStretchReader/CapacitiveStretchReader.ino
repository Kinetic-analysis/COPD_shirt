/*
  ReadAnalogVoltage

  Reads an analog input on pin 0, converts it to voltage, and prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/ReadAnalogVoltage
*/
float a = 0;
float b = 0;
float timingMillis = 0;
float timingMillis2 = 0;
float c = 0;
float capacity = 0;
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
}

// the loop routine runs over and over again forever:
void loop() {
  timingMillis = micros();
  digitalWrite(5, HIGH);
  a = 1;
  while(a == 1)
  {
    if(analogRead(A0) >= 647)
    {
      timingMillis2 = micros();
      b = timingMillis2-timingMillis;
      c = b*1000;
      capacity = c/1050000;
      Serial.println(capacity);
      digitalWrite(5, LOW);
      a = 0;
    }
  }
  delay(2000);
}
