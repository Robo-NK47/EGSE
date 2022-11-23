int analogPin0 = A0;
int analogPin1 = A1;
int analogPin2 = A2;
int analogPin3 = A3;
int analogPin4 = A4;
int analogPin5 = A5;
int analogPin6 = A6;
int analogPin7 = A7;
int analogPin8 = A8;
int analogPin9 = A9;
int analogPin10 = A10;
int analogPin11 = A11;
int analogPin12 = A12;
int analogPin13 = A13;
int analogPin14 = A14;
int analogPin15 = A15;


void setup(void)
{
  Serial.begin(115200); // initialization of serial communication
}

void loop()
{
  sendStrainGageSignals();
  sendThermistorSignals();
}


void sendStrainGageSignals()
{
  float array1[] = {2.0, (analogRead(analogPin0) / 1023.0), (analogRead(analogPin1) / 1023.0), (analogRead(analogPin2) / 1023.0), (analogRead(analogPin3) / 1023.0), 
  (analogRead(analogPin4) / 1023.0), (analogRead(analogPin5) / 1023.0), (analogRead(analogPin6) / 1023.0), (analogRead(analogPin7) / 1023.0)};
  byte *p = (byte*)array1;
  for(byte i = 0; i < sizeof(array1); i++)
  {
    Serial.write(p[i]);
  }
}

void sendThermistorSignals()
{
  float array1[] = {3.0, (analogRead(analogPin8) / 1023.0), (analogRead(analogPin9) / 1023.0), (analogRead(analogPin10) / 1023.0), (analogRead(analogPin11) / 1023.0), 
  (analogRead(analogPin12) / 1023.0), (analogRead(analogPin13) / 1023.0), (analogRead(analogPin14) / 1023.0), (analogRead(analogPin15) / 1023.0)};
  byte *p = (byte*)array1;
  for(byte i = 0; i < sizeof(array1); i++)
  {
    Serial.write(p[i]);
  }
}
