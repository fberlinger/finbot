void initialize_photodiode_thresholds(uint16_t& threshold_sampled)
// turns until minimum and maximum light intensity are found, then sets thresholds light detection
{
  static EMA light_threshold(0.1);
  uint16_t sample_max = 0;
  uint16_t sample_min = 1022;
  uint16_t sample_current = 0;
  uint16_t sample_start = 0;

  sample_current = analogRead(photodiode);
  sample_current = light_threshold.update_filter(sample_current);
  sample_start = sample_current;

  uint32_t t_0 = millis();
  //while (millis()-t_0 < 20000) // time based, 20s for 360 degree rotation
  while (sample_current < 1.1 * sample_min || sample_current > 0.9 * sample_max || sample_min > 0.9 * sample_start || sample_max < 1.1 * sample_start) // sample based
  {
    left = 1; // turn

    // find highest light intensity
    if (sample_current > sample_max)
    {
      sample_max = sample_current;
    }
    // find lowest light intensity
    if (sample_current < sample_min)
    {
      sample_min = sample_current;
    }

    sample_current = analogRead(photodiode);
    sample_current = light_threshold.update_filter(sample_current);
  }

  // set threshold at half the peak
  threshold_sampled = sample_min + (sample_max - sample_min) / 2;

  Serial.print("light threshold sampled = ");
  Serial.println(threshold_sampled);
  blueBot.print("light threshold sampled = ");
  blueBot.println(threshold_sampled);
  blueBot.flush();
}

void read_photodiode(uint16_t& light_intensity)
// reads the photodiode and returns a filtered value
{
  static EMA light_sampling(0.1);
  uint16_t sample;
  uint16_t sample_filtered;
  sample = analogRead(photodiode);
  light_intensity = light_sampling.update_filter(sample);
}

void homing_to_light()
// turns until robot finds led and then swims towards it
{
  static bool align_caudal_fin = 0;

  if (light < pd_thresh)
  // searching for light
  // switch caudal fin and LED off, turn and sample intensity
  {
    forward = 0;
    backward = 0;
    right = 0;
    blink_LED = 0;
    left = 1;

    if (align_caudal_fin == 0)
      // bring caudal fin into favorable position for turning
    {
      digitalWrite(caudal_1, HIGH);
      digitalWrite(caudal_2, LOW);
      align_caudal_fin = 1;
    }
  }

  else
  // swim towards the LED
  {
    left = 0;
    forward = 1;
    blink_LED = 1;

    align_caudal_fin = 0;
  }
}

