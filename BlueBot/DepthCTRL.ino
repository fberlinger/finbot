void initialize_depth_ctrl()
// measure surface pressure and dive to target depth
{   
  uint16_t p_surface = 0;
  static EMA surface_pressure(0.1);
    
  for (uint8_t i=0; i<250; i++)
  {
    read_pressure(pres);
    p_surface = surface_pressure.update_filter(pres);
  }
  
  target_depth += p_surface; // set target depth based on surface pressure

  Serial.print("surface pressure = ");
  Serial.println(p_surface);
  blueBot.print("surface pressure = ");
  blueBot.println(p_surface);
  blueBot.flush();
}

void maintain_target_depth()
// commands dorsal fin based on set target depth
{
  static EMA pressure(0.5);
  uint16_t p_filtered = pressure.update_filter(pres);

  uint16_t target_depth_margin = 25; // {mm}, target_depth +/- 25mm
  if (p_filtered > (target_depth + target_depth_margin))
  {
    dive = 0;
  }
  else if (p_filtered < (target_depth - target_depth_margin))
  {
    dive = 1;
  }
}

