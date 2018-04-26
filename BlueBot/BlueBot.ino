// -----------------------------------------------------------------------
String logfile = "180501_1.txt"; // adjust file name for data logging here
// -----------------------------------------------------------------------

// LIBRARIES
#include <TimerOne.h> // time robot ctrl
#include <Wire.h> // I2C communication
#include <SPI.h> // SPI communication
#include <SD.h> // SD card

// INPUT PINS (SENSORS)
const uint8_t photodiode = 6;

// SENSOR VARIABLES
uint16_t pres; // {mmH20}, pressure
// int16_t ax, ay, az, gx, gy, gz, mx, my, mz; // IMU
// photodiode

// OUTPUT PINS (ACTUATORS)
const uint8_t caudal_1 = 6; // 5
const uint8_t caudal_2 = 9; // 6
const uint8_t dorsal_1 = 4;
const uint8_t dorsal_2 = 5; // 7
const uint8_t pectoral_left_1 = 14;
const uint8_t pectoral_left_2 = 15;
const uint8_t pectoral_right_1 = 16;
const uint8_t pectoral_right_2 = 17;
const uint8_t LED = 7; // 9

// MOTION STATES
bool forward = 0;
bool backward = 0;
bool dive = 0;
bool left = 0;
bool right = 0;
bool blink_LED = 0;

// VARIABLES FOR CONTROL ALGORITHMS
uint32_t counter = 0; // keep track of time
const uint8_t robot_ctrl_freq = 10; // {Hz}, robot ctrl frequency
String fin_mode = "step_fct"; // [step_fct, sine_fct], fin mode for caudal fin
const float cauddors_freq = 4.0; // [1-4]{Hz}, default frequency for caudal and dorsal fin
const float pect_freq = 8.0; // [2-8]{Hz}, default frequency for pectoral fins

// PREPARE SD CARD FOR DATA LOGGING
const uint8_t SDcard = 8;
File blueBot;

// CLASSES
class Actuation
// switches fins on or off
// instances have their own frequency, pin numbers, and actuation mode
{
  public:
    Actuation(const float freq, const uint8_t side_1, const uint8_t side_2, String mode)
    {
      m_half_period_step = 1000000/(2*freq); // {us}
      m_period_sine = 1000000/(256*freq); // {us}
      m_side_1 = side_1;
      m_side_2 = side_2;
      m_mode = mode;
      m_dir = 0;
      m_last_changed = 0;
      m_sine_x = 0;
      m_sine_y = 0;
      m_pi = 3.14159265;
    }

    void on()
    {
      if (m_mode == "step_fct")
      // step input to fins
      {
        if (micros() > m_last_changed + m_half_period_step)
        {
          if (m_dir)
          {
            digitalWrite(m_side_1, LOW);
            digitalWrite(m_side_2, HIGH);
          }
          else
          {
            digitalWrite(m_side_2, LOW);
            digitalWrite(m_side_1, HIGH);
          }
          m_dir = 1 - m_dir;
          m_last_changed = micros();
        }
      }
      
      if (m_mode == "sine_fct")
      // sinusoidal input to caudal fin
      // does not work for cork fish if timer 1 is on
      {
        if (micros() > m_last_changed + m_period_sine)
        {
          if (m_sine_x > 128)
          {
            m_sine_y = abs(255*sin(m_sine_x*2*m_pi/256));
            digitalWrite(m_side_1, LOW);
            analogWrite(m_side_2, round(m_sine_y));
          }
          else
          {
            m_sine_y = abs(255*sin(m_sine_x*2*m_pi/256));
            digitalWrite(m_side_2, LOW);
            analogWrite(m_side_1, round(m_sine_y));
          }
          m_sine_x = m_sine_x + 1;

          if (m_sine_x == 255)
          {
            m_sine_x = 0;
          }
          m_last_changed = micros();
        }
      }

      if (m_mode == "LED")
      // blink the LED
      {
        if (micros() > m_last_changed + m_half_period_step)
        {
          if (m_dir)
          {
            digitalWrite(m_side_1, LOW);
          }
          else
          {
            digitalWrite(m_side_1, HIGH);
          }
          m_dir = 1 - m_dir;
          m_last_changed = micros();
        }
      }
    }

    void off()
    {
      digitalWrite(m_side_1, LOW);
      digitalWrite(m_side_2, LOW);
    }

  private:
    uint32_t m_half_period_step;
    uint32_t m_period_sine;
    uint8_t m_side_1;
    uint8_t m_side_2;
    String m_mode;
    bool m_dir;
    uint32_t m_last_changed;
    uint8_t m_sine_x;
    uint8_t m_sine_y;
    float m_pi;
};

class EMA
// exponential moving average filter
// instances have their own alpha
{
  public:
    EMA(float alpha_arg)
    {
      alpha = alpha_arg;
    }

    float update_filter(float Y)
    {
      if (initialized == 0)
      {
        S = Y;
        initialized = 1;
      }
      else
      {
        S = alpha * Y + (1.0 - alpha) * S;
      }
      return S;
    }

    float get_output()
    {
      return S;
    }

    void reset_filter()
    {
      initialized = 0;
    }

  private:
    float alpha;
    bool initialized = 0;
    float S = 0;
};


void setup()
// executed once to initialize processes
{
  // INITIALIZE SERIAL COMMUNICATION
  Serial.begin(9600);

  // START DATA LOGGING
  pinMode(SDcard, OUTPUT);
  SD.begin(SDcard);
  blueBot = SD.open(logfile, FILE_WRITE);
  blueBot.println();
  blueBot.println("-----------------------------");
  blueBot.println("NEW EXPERIMENT STARTING HERE!");
  blueBot.println("-----------------------------");
  blueBot.println();
  blueBot.flush();

  // INITIALIZE SENSORS
  initialize_pressure_sensor(); // pressure
  // IMU
  // photodiode

  // ARM MOTORS
  pinMode(caudal_1, OUTPUT);
  pinMode(caudal_2, OUTPUT);
  pinMode(dorsal_1, OUTPUT);
  pinMode(dorsal_2, OUTPUT);
  pinMode(pectoral_left_1, OUTPUT);
  pinMode(pectoral_left_2, OUTPUT);
  pinMode(pectoral_right_1, OUTPUT);
  pinMode(pectoral_right_2, OUTPUT);
  pinMode(LED, OUTPUT); // LED
  pinMode(10, OUTPUT); // SS pin has to be output
  digitalWrite(caudal_1, LOW);
  digitalWrite(caudal_2, LOW);
  digitalWrite(dorsal_1, LOW);
  digitalWrite(dorsal_2, LOW);
  digitalWrite(pectoral_left_1, LOW);
  digitalWrite(pectoral_left_2, LOW);
  digitalWrite(pectoral_right_1, LOW);
  digitalWrite(pectoral_right_2, LOW);
  digitalWrite(LED, LOW); // LED

  // OPTICAL STARTING SIGNAL USING LED
  digitalWrite(LED, HIGH);
  delay(333);
  digitalWrite(LED, LOW);
  delay(333);
  digitalWrite(LED, HIGH);
  delay(333);
  digitalWrite(LED, LOW);

  // INITIALIZE THE INTERRUPT SERVICE ROUTINES (has to be at the end of the setup as they start interrupting immediately)
  uint32_t robot_ctrl_period = 1000000 / robot_ctrl_freq; // 1000000us/10Hz = 0.1
  Timer1.initialize(robot_ctrl_period);
  Timer1.attachInterrupt(robot_ctrl); // calls robot_ctrl() at robot_ctrl_freq
}


void loop()
// runs endlessly as fast as possible to control fins
// precedences: backward>left,right; left>right 
{
  static Actuation caudal(cauddors_freq, caudal_1, caudal_2, fin_mode);
  static Actuation dorsal(cauddors_freq, dorsal_1, dorsal_2, "step_fct");
  static Actuation pectoral_l(pect_freq, pectoral_left_1, pectoral_left_2, "step_fct");
  static Actuation pectoral_r(pect_freq, pectoral_right_1, pectoral_right_2, "step_fct");
  static Actuation led(1, LED, LED, "LED");

  if (dive == 1)
  {
    dorsal.on();
  }
  else
  {
    dorsal.off();
  }
  
  if (forward == 1)
  {
    caudal.on();
  }
  else
  {
    caudal.off();
  }

  if (backward == 1)
  {
    left = 0;
    right = 0;
    pectoral_r.on();
    pectoral_l.on();
  }
  else
  {
    if (left == 0)
    {
      pectoral_r.off();
    }
    if (right == 0)
    {
      pectoral_l.off();
    }
  }

  if (left == 1)
  {
    right = 0;
    pectoral_r.on();
  }
  else
  {
    if (backward == 0)
    {
      pectoral_r.off();
    }
  }

  if (right == 1)
  {
    pectoral_l.on();
  }
  else
  {
    if (backward == 0)
    {
      pectoral_l.off();
    }
  }

  if (blink_LED == 1)
  {
    led.on();
  }
  else
  {
    led.off();
  }
}


void robot_ctrl()
// runs endlessly at robot_ctrl_freq to update the robot behavior
{
  uint32_t time_start = micros();
  sei(); // enable interrupts within interrupts
  counter++;

  // READ SENSORS
  read_pressure_sensor(pres); // {mmH20}, pressure
  // IMU
  // photodiode

  // UPDATE ACTUATION
  dive = 0; 
  forward = 1;
  backward = 0;
  left = 0;
  right = 0;
  blink_LED = 1;

  // LOGGING TO SD CARD
  //blueBot.print(counter);
  //blueBot.print(",");
  //blueBot.println();
  //blueBot.flush();

  // PRINT TO SERIAL MONITOR
  Serial.print(counter);
  Serial.print(",");
  Serial.print(dive);
  Serial.print(",");
  Serial.print(forward);
  Serial.print(",");
  Serial.print(backward);
  Serial.print(",");
  Serial.print(left);
  Serial.print(",");
  Serial.print(right);
  Serial.print(",");
  Serial.print(pres);
  Serial.print(","); 
  uint32_t loop_duration = micros() - time_start;
  Serial.println(loop_duration);
}

