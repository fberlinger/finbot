// for switching dorsal fin on (1) and off (0)
bool dorsal_fin = 0;

// measured surface pressure at the beginning of the experiment
double surface_pressure;

// control thresholds (98.1Pa = 1cmH20)
double thresh_low;
double thresh_high;


// TAKE PRESSURE MEASUREMENTS AT SURFACE TO SET BANG-BANG THRESHOLDS AND DIVE TO STARTING DEPTH
void initialize_depth_control(uint32_t& thresh_diving)
{ 
  static EMA surface_pressure_filter(0.1);
    
  for (int i = 0; i < 250; i++)
  {
    read_pressure_sensor(0x77, temp_left, pres_left);
    surface_pressure = surface_pressure_filter.update_filter(pres_left);
  }

  dataFile.print("surface pressure = ");
  dataFile.println(surface_pressure);
  dataFile.println();
  
  thresh_low = surface_pressure+6671; // 68cm
  thresh_high = surface_pressure+6965; // 71cm

  thresh_diving = thresh_high;
  maneuver = 3;
}


void update_diving_depth(uint32_t& thresh_diving)
{
  thresh_low = thresh_low - 1668; // 17cm
  thresh_high = thresh_high - 1668; // 17cm

  thresh_diving = thresh_high + 196; // + 2cm margin
}


// DEPTH CONTROL BASED ON PRESSURE READINGS (BANG-BANG WITH HYSTERESIS)
void depth_control(uint32_t& depth)
{ 
  read_pressure_sensor(0x77, temp_left, pres_left);
  double y = depth_control_filter.update_filter(pres_left);
  depth = y;
  
  if (dorsal_fin == 0)
  {
    if (y < thresh_low)
    {
      dorsal_fin = 1;
    }
  }
  else
  {
    if (y > thresh_high)
    {
      dorsal_fin = 0;
      dorsal = 0;
      digitalWrite(dorsal_1, LOW);
      digitalWrite(dorsal_2, LOW);  
    }
  }
  
  if (dorsal_fin == 1)
  {
    dorsal = 1;
  }
}


