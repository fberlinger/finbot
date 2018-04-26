// ----------------------------------------------------------------------------------
String psensor_model = "BA01"; // [BA01, BA02], specify model of pressure sensor here
// ----------------------------------------------------------------------------------

const double Pa_mmH20_conversion = 0.10197162129779283;

const uint8_t I2C_ADDRESS = 0x77; // CSB (pin 3) on VDD (0x76) or GND (0x77)

const uint16_t RESET_DELAY_MS = 10;
const uint16_t CONVERT_DELAY_MS = 1;

const uint8_t CMD_ADC_READ = 0x00;
const uint8_t CMD_RESET = 0x1E;
const uint8_t CMD_CONVERT_D1 = 0x40;
const uint8_t CMD_CONVERT_D2 = 0x50;
const uint8_t CMD_PROM_BASE = 0xA0;

uint16_t C[8]; // sensor calibration parameters

uint8_t initialize_pressure_sensor()
// initialize pressure sensor
{
  // reset the sensor
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(CMD_RESET);
  Wire.endTransmission();

  delay(RESET_DELAY_MS);

  // read calibration parameters
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

void read_pressure_sensor(uint16_t& p_mmH20)
// read pressure sensor based on specified model
// returns data in mmH20
{
  uint32_t D1, D2; // temp, pressure
  read_raw_values(D1, D2);

  long double dT = D2 - (C[5] * pow(2, 8));
  double TEMP = 2000 + ((dT * C[6]) / pow(2, 23));

  long double OFF = 0;
  long double SENS = 0;
  long double T2 = 0;
  long double OFF2 = 0;
  long double SENS2 = 0;

  if (psensor_model == "BA01")
  {
    OFF = (C[2] * pow(2, 16)) + (C[4] * dT / pow(2, 7));
    SENS = (C[1] * pow(2, 15)) + (C[3] * dT / pow(2, 8));

    if (TEMP < 2000)
    {
      long double T2 = pow(dT, 2) / pow(2, 31);
      long double OFF2 = 3 * pow(TEMP - 2000, 2);
      long double SENS2 = 7 * pow(TEMP - 2000, 2)  / pow(2, 3);

      if (TEMP < -1500)
      {
        SENS2 = SENS2 + (2 * pow(TEMP + 1500, 2));
      }
    }
  }
  else if (psensor_model == "BA02")
  {
    OFF = (C[2] * pow(2, 17)) + (C[4] * dT / pow(2, 6));
    SENS = (C[1] * pow(2, 16)) + (C[3] * dT / pow(2, 7));

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
    }
  }

  TEMP = TEMP - T2;
  OFF = OFF - OFF2;
  SENS = SENS - SENS2;

  double P = ((D1 * SENS / pow(2, 21)) - OFF) / pow(2, 15);
  p_mmH20 = P * Pa_mmH20_conversion;
}

void read_raw_values(uint32_t& D1, uint32_t& D2)
// read raw values from device
{
  // read D1 (temperature)
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

  // read D2 (pressure)
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

