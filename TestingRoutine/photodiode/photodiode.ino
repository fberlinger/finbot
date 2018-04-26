void setup()
{
  Serial.begin(9600);
}

void loop()
{
  unsigned int sample;
  sample = analogRead(6);
  Serial.println(sample);
}

