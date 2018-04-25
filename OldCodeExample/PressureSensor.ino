// The sensor's I2C address is 0x76 or 0x77 depending on whether 
// CSB (pin 3) is connected to VDD or GND respectively.
uint8_t I2C_ADDRESS = 0x77;

const uint16_t RESET_DELAY_MS = 10;
const uint16_t CONVERT_DELAY_MS = 1;

const uint8_t CMD_ADC_READ = 0x00;
const uint8_t CMD_RESET = 0x1E;
const uint8_t CMD_CONVERT_D1 = 0x40;
const uint8_t CMD_CONVERT_D2 = 0x50;
const uint8_t CMD_PROM_BASE = 0xA0;

uint16_t C[8];  // Sensor calibration parameters


// INITIALIZE BOTH PRESSURE SENSORS
uint8_t initialize_pressure_sensors()
{
  Serial.println("init_pressure_sensors");
  
  initialize_pressure_sensor(0x76);
  initialize_pressure_sensor(0x77);
}


// INITIALIZE SINGLE PRESSURE SENSOR
uint8_t initialize_pressure_sensor(uint8_t I2C_ADDRESS)
{ 
  // Reset the sensor
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(CMD_RESET);
  Wire.endTransmission();
  
  delay(RESET_DELAY_MS);

  // Read calibration parameters
  for (int i = 0; i < 8; ++i)
  {
    uint8_t CMD_PROM_ACTUAL = CMD_PROM_BASE + (2 * i);
    
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(CMD_PROM_ACTUAL);
    Wire.endTransmission();
    
    Wire.requestFrom(I2C_ADDRESS, (uint8_t) 2);

    uint16_t upper_byte = Wire.read();
    uint16_t lower_byte = Wire.read();
    C[i] = (upper_byte << 8) | lower_byte;
  }
}


// READ BOTH PRESSURE SENSORS AND RETURN CONVERTED DATA IN °C AND Pa
void read_pressure_sensors(int32_t& T_l, int32_t& P_l, int32_t& T_r, int32_t& P_r)
{
  read_pressure_sensor(0x77, T_l, P_l);
  // read_pressure_sensor(0x76, T_r, P_r); this fish has only one pressure sensor
}


// READ SINGLE PRESSURE SENSOR
void read_pressure_sensor(uint8_t I2C_ADDRESS, int32_t& Temp_out, int32_t& Pres_out)
{ 
uint32_t D1, D2;
  read_raw_values(D1, D2);

  long double dT = D2 - (C[5] * pow(2, 8));
  double TEMP = 2000 + ((dT * C[6]) / pow(2, 23));

  long double OFF = (C[2] * pow(2, 17)) + (C[4] * dT / pow(2, 6));
  long double SENS = (C[1] * pow(2, 16)) + (C[3] * dT / pow(2, 7));

  if (TEMP < 2000)
  {
    long double T2 = pow(dT, 2) / pow(2, 31);
    long double OFF2 = 61 * pow(TEMP - 2000, 2) / pow(2, 4);
    long double SENS2 = 2 * pow(TEMP - 2000, 2);

    if (TEMP < -1500)
    {
      OFF2 = OFF2 + (20 * pow(TEMP + 1500, 2));
      SENS2 = SENS2 + (12 * pow(TEMP + 1500, 2));
    }

    TEMP = TEMP - T2;
    OFF = OFF - OFF2;
    SENS = SENS - SENS2;
  }

  double P = ((D1 * SENS / pow(2, 21)) - OFF) / pow(2, 15);
  
  Temp_out = TEMP;
  Pres_out = P;
  
//  Serial.print("TEMP = ");
//  Serial.print((int32_t) TEMP);
//  Serial.print("\tP = ");
//  Serial.println((int32_t) P);
}


// READ RAW VALUES FROM DEVICE
void read_raw_values(uint32_t& D1, uint32_t& D2)
{
  // Read D1 (Temperature)
  
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(CMD_CONVERT_D1);
  Wire.endTransmission();

  delay(CONVERT_DELAY_MS);

  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(CMD_ADC_READ);
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDRESS, (uint8_t) 3);

  uint32_t byte_2 = Wire.read();
  uint32_t byte_1 = Wire.read();
  uint32_t byte_0 = Wire.read();

  D1 = (byte_2 << 16) | (byte_1 << 8) | byte_0;

  // Read D2 (Pressure)

  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(CMD_CONVERT_D2);
  Wire.endTransmission();

  delay(CONVERT_DELAY_MS);

  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(CMD_ADC_READ);
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDRESS, (uint8_t) 3);

  byte_2 = Wire.read();
  byte_1 = Wire.read();
  byte_0 = Wire.read();

  D2 = (byte_2 << 16) | (byte_1 << 8) | byte_0;
}
