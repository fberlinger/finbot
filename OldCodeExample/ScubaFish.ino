#include <TimerOne.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

const int chipSelect = 10; // for SD card
File dataFile;

// ARDUINO PINS FOR MICs ON HANS
int caudal_1 = 3;
int caudal_2 = 4;
int dorsal_1 = 5;
int dorsal_2 = 6;
int pectoral_left_1 = 14;
int pectoral_left_2 = 15;
int pectoral_right_1 = 16;
int pectoral_right_2 = 17;

// BOOLS FOR FIN FUNCTIONS
bool caudal = 0;
bool dorsal = 0;
bool pectoral_left = 0;
bool pectoral_right = 0;

// VARIABLES FOR PRESSURE, IMU, AND PHOTODIODE READINGS
int32_t temp_left, pres_left, temp_right, pres_right;
int16_t ax, ay, az, gx, gy, gz, mx, my, mz;

// VARIABLES FOR CONTROL ALGORITHMS
uint32_t t = 0; // local timer
uint32_t t_start_fwd = 0; // starting time for local forward timer
uint32_t t_start_turn = 0; // starting time for local turning timer
uint16_t last_turn = 0;
bool stat_turn = 0; // turning status: 0 = left x; 1 = right
bool stat_turn_copy = 0;
bool stat_dir = 0; // facing status: 0 = searching; 1 = found
bool stat_diving = 0; // diving status: 0 = hovering; 1 = diving
bool init_drift = 1; // 1: drift model uncomplete; 0: drift model complete
bool gain_depth = 0;
uint32_t counter = 0;
uint32_t counter_turning = 0;
uint32_t thresh = 40; // empirical threshold for light signal
uint32_t thresh_diving = 0; // diving threshold
int s = 100; // 10s, sample time for drift model
int maneuver = 1; // 1 = fwd; 2 = turn; 3 = diving
uint32_t depth = 0; // diving depth

// yaw parameters
double yaw_vel_turn = 0.0; // gyro readings
double yaw_pos_turn_uncorrected = 0.0; // raw values
double yaw_pos_turn = 0.0; // corrected values with drift model
double yaw_drift = 0.0; // modeled drift from setup

// linear regression for yaw: y = mx + q, where x = time and y = angle
double y_yaw = 0.0;
double m_yaw = 0.0;
double q_yaw = 0.0;


// FILTER FUNCTION
class EMA
{
private:
  double alpha;
  bool initialized = 0;
  double S = 0;
  
public:
  EMA(double alpha_arg)
  {
    alpha = alpha_arg;  
  }
  
  double update_filter(double Y)
  {
    if (initialized == 0)
    {
      S = Y;
      initialized = 1;
    }
    else
    {
      S = alpha*Y + (1.0-alpha)*S;
    }
    return S;
  }

  double get_output()
  {
    return S;
  }
  
  void reset_filter()
  {
    initialized = 0;
  }
};

EMA depth_control_filter(0.1);


void setup()
{
  // INITIALIZE SERIAL COMMUNICATION
  //Serial.begin(9600);

  // START DATA LOGGING
  pinMode(10, OUTPUT);
  SD.begin(chipSelect);
  dataFile = SD.open("012717_6.txt", FILE_WRITE);
  dataFile.println();
  dataFile.println("-----------------------------");
  dataFile.println("NEW EXPERIMENT STARTING HERE!");
  dataFile.println("-----------------------------");
  dataFile.println();

  // ARM MOTORS
  pinMode(dorsal_1, OUTPUT);
  pinMode(dorsal_2, OUTPUT);
  pinMode(caudal_1, OUTPUT);
  pinMode(caudal_2, OUTPUT);
  pinMode(pectoral_left_1, OUTPUT);
  pinMode(pectoral_left_2, OUTPUT);
  pinMode(pectoral_right_1, OUTPUT);
  pinMode(pectoral_right_2, OUTPUT);
  pinMode(9, OUTPUT); // LED

  // INITIALIZE ELECTRONICS
  initialize_pressure_sensor(0x77); // HANS
  initialize_IMU();
  
  // INITIALIZE DEPTH CONTROL
  initialize_depth_control(thresh_diving); // for submerged experiments only

  // GET ZERO VALUES FOR DRIFT
  read_IMU(ax, ay, az, gx, gy, gz, mx, my, mz);
  q_yaw = -gy;   
    
  // OPTICAL STARTING SIGNAL USING LED
  digitalWrite(9, HIGH);
  delay(333);
  digitalWrite(9, LOW);
  delay(333);
  digitalWrite(9, HIGH);
  delay(333);
  digitalWrite(9, LOW);

  // INITIALIZE THE INTERRUPT SERVICE ROUTINE
  Timer1.initialize(100000); // 100000 microseconds or 0.1 sec = 10Hz
  Timer1.attachInterrupt( robot_ctrl ); // ISR function
}


// MAIN LOOP DOING HIGH-FREQUENCY FIN CONTROL
void loop()
{
  if (caudal == 1)
  {
    caudal_fin_function();
  }

  if (dorsal == 1)
  {
    dorsal_fin_function();
  }

  if (pectoral_left == 1)
  {
    pectoral_left_fin_function(1000);
  }

  if (pectoral_right == 1)
  {
    pectoral_right_fin_function(1000);
  }
}


// 10Hz CONTROL LOOP DEFINING ROBOTS BEHAVIOR
void robot_ctrl()
{
  // ENABLE INTERRUPTS WITHIN INTERRUPTS
  sei();

  // UPDATE COUNTER
  counter++;

  // INITIALIZE FILTER FUNCTIONS
  static EMA diving_control_filter(0.1);
  
  // INITIALIZE DRIFT MODEL (tbd once only)
  if (init_drift == 1)
  {
    initialize_drift();
  }
  
  else
  { 
    // LOG DATA --------------- tbd
    dataFile.print(counter);
    dataFile.print(",");
    dataFile.print(maneuver);
    dataFile.print(",");
    dataFile.print(yaw_pos_turn);
    dataFile.print(",");
    dataFile.println(depth);
    dataFile.flush();

    // ADJUST TRAJECTORY   
    switch (maneuver) 
    {
      // SWIM FORWARD
      case 1:
      { 
        // KEEP DESIGNATED DIVING DEPTH IN SUBMERGED EXPERIMENTS
        depth_control(depth);
    
        if ((counter-t_start_fwd) < 75) // for 3D experiments
        {
          // run caudal fin for forward propulsion
          caudal = 1;
        }
        else
        {
          caudal = 0;
          // bring caudal fin into favorable position for turning
          digitalWrite(caudal_1, LOW);
          digitalWrite(caudal_2, HIGH);

          dorsal = 0;
          digitalWrite(dorsal_1, LOW);
          digitalWrite(dorsal_2, LOW);
          
          maneuver = 2;
          t_start_turn = counter;
          stat_turn = 0;
        }
      }
      break;
      
      // TURN
      case 2:
      {
        // update depth readings manually because depth control is disabled while turning
        read_pressure_sensor(0x77, temp_left, pres_left);
        depth = depth_control_filter.update_filter(pres_left);

        if (counter_turning < 3)
        {
          if (yaw_pos_turn < 107500)
          {
            if (stat_turn == 0)
            {
              digitalWrite(caudal_1, LOW);
              digitalWrite(caudal_2, HIGH);
            }
            else
            {
              digitalWrite(caudal_1, LOW);
              digitalWrite(caudal_2, LOW);
            }
            
            stat_turn = 1;
            
            // run pectoral left fin for yaw turning
            pectoral_left = 1; // f = 1000/1000 = 1.0 [Hz]
        
            // update timer and drift model
            t = counter - t_start_turn;
            yaw_drift = m_yaw*t + q_yaw;
                  
            // update imu readings and angle turned
            read_IMU(ax, ay, az, gx, gy, gz, mx, my, mz);
            yaw_vel_turn = -gy;
            yaw_pos_turn_uncorrected += yaw_vel_turn;
            yaw_pos_turn = yaw_pos_turn_uncorrected - yaw_drift;
        
            // signal that you are turning
            digitalWrite(9, digitalRead(9) ^ 1 ); // blink signal
          }
  
          else
          {
            digitalWrite(9, LOW);
                
            pectoral_left = 0;
            digitalWrite(pectoral_left_1, LOW);
            digitalWrite(pectoral_left_2, LOW);
            
            // reset all turning values for next section  
            yaw_vel_turn = 0.0;
            yaw_pos_turn_uncorrected = 0.0;
            yaw_drift = 0.0;
            yaw_pos_turn = 0.0;
      
            // for 3d only in order to incremenatlly dive after each full square
            counter_turning++;
            maneuver = 1;
            t_start_fwd = counter;
          }
        }

        else if (counter_turning == 3)
        {
          if (yaw_pos_turn < 53600)
          {
            if (stat_turn == 0)
            {
              digitalWrite(caudal_1, LOW);
              digitalWrite(caudal_2, HIGH);
            }
            else
            {
              digitalWrite(caudal_1, LOW);
              digitalWrite(caudal_2, LOW);
            }
            
            stat_turn = 1;
            
            // run pectoral left fin for yaw turning
            pectoral_left = 1; // f = 1000/1000 = 1.0 [Hz]
        
            // update timer and drift model
            t = counter - t_start_turn;
            yaw_drift = m_yaw*t + q_yaw;
                  
            // update imu readings and angle turned
            read_IMU(ax, ay, az, gx, gy, gz, mx, my, mz);
            yaw_vel_turn = -gy;
            yaw_pos_turn_uncorrected += yaw_vel_turn;
            yaw_pos_turn = yaw_pos_turn_uncorrected - yaw_drift;
        
            // signal that you are turning
            digitalWrite(9, digitalRead(9) ^ 1 ); // blink signal
          }
  
          else
          {
            digitalWrite(9, LOW);
                
            pectoral_left = 0;
            digitalWrite(pectoral_left_1, LOW);
            digitalWrite(pectoral_left_2, LOW);
            
            // reset all turning values for next section  
            yaw_vel_turn = 0.0;
            yaw_pos_turn_uncorrected = 0.0;
            yaw_drift = 0.0;
            yaw_pos_turn = 0.0;
      
            // for 3d only in order to incremenatlly dive after each full square
            counter_turning = 0;
            maneuver = 3;
            update_diving_depth(thresh_diving);
          }
        }  
      }
      break;
          
      // GAIN DEPTH
      case 3:
      { 
        depth_control(depth);
        
        read_pressure_sensor(0x77, temp_left, pres_left);
        double y = diving_control_filter.update_filter(pres_left);

        if (gain_depth == 0)
        {
          if (y > thresh_diving)
          {
            maneuver = 1;
            t_start_fwd = counter;
            gain_depth = 1;
          }
        }
        else if (y < thresh_diving)
        {
          maneuver = 1;
          t_start_fwd = counter;
        }
      }
      break;
    }
  } 
}


// PROPULSION FUNCTIONS
void caudal_fin_function()
{
  static int dir = 0;
  static uint32_t last_changed = 0;

  if (millis() > last_changed + 300) // f = 1000/(2*300) = 2.2 [Hz]
  {
    if (dir)
    {
      digitalWrite(caudal_1, HIGH);
      digitalWrite(caudal_2, LOW);
    }
    else
    {
      digitalWrite(caudal_1, LOW);
      digitalWrite(caudal_2, HIGH);
    }

    dir = 1 - dir;

    last_changed = millis();
  }
}

void dorsal_fin_function()
{
  static int dir = 0;
  static uint32_t last_changed = 0;

  if (millis() > last_changed + 180) // f = 1000/360 = 2.8 [Hz]
  {
    if (dir)
    {
      digitalWrite(dorsal_1, HIGH);
      digitalWrite(dorsal_2, LOW);
    }
    else
    {
      digitalWrite(dorsal_1, LOW);
      digitalWrite(dorsal_2, HIGH);
    }

    dir = 1 - dir;

    last_changed = millis();
  }   
}

void pectoral_left_fin_function(int period)
{
  static int dir = 0;
  static uint32_t last_changed = 0;

  if (millis() > last_changed + period/2)
  {
    if (dir)
    {
      digitalWrite(pectoral_left_1, HIGH);
      digitalWrite(pectoral_left_2, LOW);
    }
    else
    {
      digitalWrite(pectoral_left_1, LOW);
      digitalWrite(pectoral_left_2, HIGH);
    }

    dir = 1 - dir;

    last_changed = millis();
  }
}

void pectoral_right_fin_function(int period)
{
  static int dir = 0;
  static uint32_t last_changed = 0;

  if (millis() > last_changed + period/2)
  {
    if (dir)
    {
      digitalWrite(pectoral_right_1, HIGH);
      digitalWrite(pectoral_right_2, LOW);
    }
    else
    {
      digitalWrite(pectoral_right_1, LOW);
      digitalWrite(pectoral_right_2, HIGH);
    }

    dir = 1 - dir;

    last_changed = millis();
  }
}


// DRIFT INITIALIZATION
void initialize_drift()
{
  // sample at s for yaw angle
  if (counter < s)
  {
    read_IMU(ax, ay, az, gx, gy, gz, mx, my, mz);
    yaw_vel_turn = -gy;
    yaw_pos_turn_uncorrected += yaw_vel_turn;
  }
  else
  {
    y_yaw = yaw_pos_turn_uncorrected;
    m_yaw = (y_yaw-q_yaw)/s;

    yaw_vel_turn = 0.0;
    yaw_pos_turn_uncorrected = 0.0;

    init_drift = 0;
    t_start_fwd = counter;
  }
}


