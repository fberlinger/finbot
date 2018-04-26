int side_1 = 6;
int side_2 = 9;

void setup()
{
  pinMode(side_1, OUTPUT);
  pinMode(side_2, OUTPUT);
}

void loop()
{
  static int dir = 0;
  static uint32_t last_changed = 0;

  if (millis() > last_changed + 500)
  {
    if (dir)
    {
      digitalWrite(side_1, LOW);
      digitalWrite(side_2, HIGH);
    }
    else
    {
      digitalWrite(side_2, LOW);
      digitalWrite(side_1, HIGH);
    }

    dir = 1 - dir;

    last_changed = millis();
  }
}

