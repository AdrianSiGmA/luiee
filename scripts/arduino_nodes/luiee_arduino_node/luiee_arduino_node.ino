// 5.25 ratio of the gears

// Libraries:
#include <Wire.h>
/* Library for the small stepper motor. Used to control the position of the plane mirror */
#include <Stepper.h>
/* Add library and create an instance of the sensor object */
#include <FaBo9Axis_MPU9250.h>
FaBo9Axis fabo_9axis;         //  CONSIDER MAKING THE OBJECT LOCAL

#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include <FastLED.h>
#define NUM_LEDS 24
#define LEDring_DELAY 30

CRGBArray<NUM_LEDS> leds;

int STEPS = 2038;
#define SPEED 15   // recommendation between 1 and 15 [min, max]
Stepper stepper(STEPS, 36, 32, 34, 30);
/*  Order of Modes: Nav_Mode,Cent_Mode,Extr_Mode,Stor_Mode,Rec_Mode,Man_Mode,Stan_Mode,Safe_Mode
 *  
 */

// --------------- PIN DEFINITION ---------------------
const int mot_pin[4] = {2,3,4,5};               // {right_pwm_f,right_pwm_b,left_pwm_f,left_pwm_b}
const int LEDring_pin = 13;                      // LEDring_pwm
const int buzzer_pin = 6;
const int linAct_vapChamb_pin[6] = {8,7,9,10,11,12};  // {linAct1_pwm_f,linAct1_pwm_b,linAct2_pwm_f,linAct2_pwm_b,linAct3_pwm_f,linAct3_pwm_b} 
const int LM_pin[8] = {22,24,26,28,30,32,34,36};      // {Rstp1_dir,Rstp1_step,Pstp2_dir,Pstp2_step,PMstp3_1,PMstp3_2,PMstp3_3,PMspt3_4}
const int pump_and_fans[3] = {38,40,42};            // {pump,fan1, fan2}

// Sensors:
const float sensors_pin[10] = {A0,A1,A2,A3,A4,A5,A6,A7,A8,A9};  // (below the pinout)
/*
const int temp_sens_pin = A0;
const int voltage_sens_pin[2] = {A1, A2};
const int strain_gauges_pin[3] = {A3, A4, A5};
const int phototrans_pin[8] = {A6, A7, A8, A9};*/

// ------------------------ Variables definition ---------------------------
int mot_data[4] = {0,0,0,0};
int mot_pwm_right = 0;
int mot_pwm_left = 0;
int LEDring_data = 0;
int buzzer_data = 0;
int linAct_vapChamb_data[6]= {0,0,0,0,0,0};
int linAct1_pwm = 0;
int linAct2_pwm = 0;
int linAct3_pwm = 0;
int LM_data[3] = {0,0,0};
int calculated_heading;
int n_sensors = 12;
short R = 0;
short G = 0;
short B = 0;
//Serial.println("Mag_hard_iron_calibration has begun...");
float Factor = 0.15;
/* Max and min X/Y */
float max_x = -1000.0;
float max_y = -2000.0;
float min_x =  1100.0;
float min_y =  2200.0;
// PENDING PUMP AND FANS DATA

// Warning! you can only use a maximum of 12 sensors if you are using a GUI to publish 1 data array per push button
// to avoid losing data...
float sensors_data[12] = {0,0,0,0,0,0,0,0,0,0,0,0};     // (below the definition)
// {volt1,volt2,load1,load2,load3,ldr1,ldr2,ldr3,ldr4,heading,pitch,pm_angle}

// VARIABLES FOR THE MAGNETOMETER ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
/* Variables used for determination of the sun vector */
#define DEG_TO_RAD 0.01745329
#define PI 3.141592654
#define TWOPI 6.28318531
float Solar_vector_elevation; // 1st output of the sun vector algorithm ;
float Solar_vector_azimuth;   // 2nd output of the sun vector algorithm ;

/* Magnetometer variables:
   Variables that store magnetic field values along x,y and z axes */
static float mx,my,mz;
static float ax,ay,az;
static float Heading = 0;  // Actual heading read by magnetometer. Compared agains the sun vector.
/* Buffer holding 25 x, y and z readings for the magnetometer moving average */
//float values_x[25];
//float values_y[25];
//float values_z[25];
/* Holds the sum of 25 measurements */
float average_temp_x;
float average_temp_y;
float average_temp_z;
/* Variable used to overwrite old values in the buffer */
int counter = 0;
/* Axillary variables for the low pass filter */
float filter; 
float SmoothData_x, SmoothData_y, SmoothData_z; // Readings after the low pass filter is applied ; 
float RawData_x, RawData_y, RawData_z; // Raw readings. Factory compensation values have to be applied ;
float LPF_Beta = 0.0025; // Determines the cut-off frequency which decreases as the value -> 0 ;
float pitch;             // Pitch for magnetometer tilt compensation ;
float roll;              // Roll for magnetometer tilt compensation ;
/* Magnetometer variables for calibration */
float Offset_x;       // Offset for the initial hard iron calibration x ;
float Offset_y;       // Offset for the initial hard iron calibration  y ; 
float Offset_x_final; // Offset for the ginal hard iron calibration x ;
float Offset_y_final; // Offset for the final hard iron calibration y ;
float theta_rot;      // Angle of rotation. Used for the soft iron calibration ;
float Mag_x_scale;    // Scale for the magnetic field sensor x;
float Mag_y_scale;    // Scale for the magnetic field sensor y;


/* Additional variables used to store the accelerometer readings */
float Accel_total_vector ;       // Total magnitude of the gravitational vector ;
float Accel_x, Accel_y, Accel_z; // Hold unbiased accelerometer readings ;
float Accel_pitch;               // Holds pitch angle. Used for tilt compensation ;
float Accel_roll;                // Holds roll angle. Used for tilt compensation ;
float bias_x = 0;                // Offset for the accelerometer x axis ;
float bias_y = 0;                // Offset for the accelerometer y axis ;
float bias_z = 0;                // Offset for the accelerometer z axis ;

/* Magnetometer factory compensating values for the sensitivity adjustment
   were obtained with the following commands:
   Wire.beginTransmission(0x0C);                        // Open session with the magnetometer AK8963
   Wire.write(0x10);                                    // Point to AK8963 fuse ROM register
   Wire.endTransmission();
   Wire.requestFrom(0x0C, 3);                           // Request 3 bytes    
   ASAX = (Wire.read() - 128) * 0.5 / 128 + 1;          // Adjust readings
   ASAY = (Wire.read() - 128) * 0.5 / 128 + 1;          // Adjust readings
   ASAZ = (Wire.read() - 128) * 0.5 / 128 + 1;          // Adjust readings
// Returned values: ASAX = 1.21094; ASAY = 1.21484; ASAZ=1.16797 */
float ASAX = 1.21094;
float ASAY = 1.21484; 
float ASAZ = 1.16797;

unsigned long timer_hs_prev = 0;
unsigned long timer_hs = 0;
unsigned long timer_hs_difference = 0;
boolean activated_hs_flag = false;
int miliSeconds_hs_stepper = 100;   // min 10, max 200
int counter_hs_steps = 0;
int desired_hs_steps = 0;
float current_hs_angle = 0;
boolean miliSeconds_reached_flag_hs = false;
float k_hs = 1.8;

unsigned long timer_ps_prev = 0;
unsigned long timer_ps = 0;
unsigned long timer_ps_difference = 0;
boolean activated_ps_flag = false;
int miliSeconds_ps_stepper = 10;   // min 10, max 200
int counter_ps_steps = 0;
int desired_ps_steps = 0;
float current_ps_angle = 0;
boolean miliSeconds_reached_flag_ps = false;
float k_ps = 1.8;

/* Number of full steps per revolution: 200*/
unsigned long timer_prev_pm = 0;
unsigned long timer_pm = 0;
unsigned long timer_difference_pm = 0;
int counter_steps_pm = 0;
int desired_steps_pm = 0;
float current_pm_angle = 0;
float desired_pm_angle = 0;
boolean miliSeconds_reached_flag_pm = false;
float k_pm = 5.66;

unsigned long timer_prev_LEDring = 0;
unsigned long timer_LEDring = 0;
unsigned long timer_difference_LEDring = 0;
short i = 0;
boolean LEDring_flag = false;

boolean ROS_connection_flag = false;
short LM_calibration_flag = 0;
short LM_data_flag = 0;

// Subscriptor: IMPORTANT: ALWAYS USE THE IF CONDITIONALS HERE AND NOT IN THE LOOP ONE
void messageCb(const std_msgs::Int32MultiArray& luiee_msg)    // BE CAREFUL! ALWAYS USE INT32 HERE, NOT FLOAT32
{
  mot_pwm_right = luiee_msg.data[0];
  mot_pwm_left = luiee_msg.data[1];
  LEDring_data = luiee_msg.data[2];
  buzzer_data = luiee_msg.data[3];
  linAct1_pwm = luiee_msg.data[4];
  linAct2_pwm = luiee_msg.data[5];
  linAct3_pwm = luiee_msg.data[6];
  LM_data[0] = luiee_msg.data[7];
  LM_data[1] = luiee_msg.data[8];
  LM_data[2] = luiee_msg.data[9];
  LM_calibration_flag = luiee_msg.data[10];
  LM_data_flag = luiee_msg.data[11];

  ROS_connection_flag = true;
  timer_hs_prev = millis();
  counter_hs_steps = 0;
  counter_ps_steps = 0;
  counter_steps_pm = 0;

  if(mot_pwm_right > 0)
  { 
    mot_data[0] = mot_pwm_right;
    mot_data[1] = 0;
  }
  else
  {
    mot_data[0] = 0;
    mot_data[1] = abs(mot_pwm_right);
  }
  if(mot_pwm_left > 0)
  {
    mot_data[2] = mot_pwm_left;
    mot_data[3] = 0;
  }
  else
  {
    mot_data[2] = 0;
    mot_data[3] = abs(mot_pwm_left);
  }

  if(linAct1_pwm > 0)
  {
    linAct_vapChamb_data[0] = linAct1_pwm;
    linAct_vapChamb_data[1] = 0;
  }
  else
  {
    linAct_vapChamb_data[0] = 0;
    linAct_vapChamb_data[1] = abs(linAct1_pwm);
  }
  if(linAct2_pwm > 0)
  {
    linAct_vapChamb_data[2] = linAct2_pwm;
    linAct_vapChamb_data[3] = 0;
  }
  else
  {
    linAct_vapChamb_data[2] = 0;
    linAct_vapChamb_data[3] = abs(linAct2_pwm);
  }
  if(linAct3_pwm > 0)
  {
    linAct_vapChamb_data[4] = linAct3_pwm;
    linAct_vapChamb_data[5] = 0;
  }
  else
  {
    linAct_vapChamb_data[4] = 0;
    linAct_vapChamb_data[5] = abs(linAct3_pwm);
  }
  
  if(LEDring_flag == false)
  {
    leds[i] = CHSV(LEDring_data*50,255,150);
    //leds[i] = CRGB::Black;                   
    leds(NUM_LEDS/2,NUM_LEDS-1) = leds(0,NUM_LEDS/2 - 1 );
    FastLED.delay(1);
  }
  else
  {
    leds[i] = CHSV(0,0,0);               
    leds(NUM_LEDS/2,NUM_LEDS-1) = leds(0,NUM_LEDS/2 - 1 );
    FastLED.delay(1);
  }
  if(i < NUM_LEDS/2)
  {
    i++;
  }
  else
  {
    i = 0;
    LEDring_flag = !LEDring_flag;
  }

  if(LM_calibration_flag == 0 && LM_data_flag == 1)
    magnetic_sensor_read();
}

// Node creation
ros::NodeHandle nh;

//Crea un arreglo de tipo multif32 de la libreria std_msgs
std_msgs::Float32MultiArray data_console;

// Publisher creation
ros::Publisher pub("/sensors_topic", &data_console);

// Subscriber creation
ros::Subscriber<std_msgs::Int32MultiArray> sub("/luiee_topic", &messageCb);
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void setup()
{
  fabo_9axis.begin();
  FastLED.addLeds<NEOPIXEL,LEDring_pin>(leds, NUM_LEDS);
  static uint8_t hue;
  
  //Inicializa el puerto a 115200 baudios, para buen funcionamiento de acelerometro con el filtro
  //nh.getHardware()->setBaud(115200);
  
  for(int i = 0; i < 4; i++)
    pinMode(mot_pin[i], OUTPUT);
  pinMode(LEDring_pin, OUTPUT);
  for(int i = 0; i < 6; i++)
    pinMode(linAct_vapChamb_pin[i], OUTPUT);
  pinMode(buzzer_pin,OUTPUT);
  for(int i = 0; i < 8; i++)
    pinMode(LM_pin[i], OUTPUT);
  for(int i = 0; i < 3; i++)
    pinMode(pump_and_fans[i], OUTPUT);

  // Node initialization
  nh.initNode();

  // Subscrite node to sub
  nh.subscribe(sub);

  // Let the node know that we are going to publish
  nh.advertise(pub);
 
  // Declare the array size to be published
  data_console.data_length = n_sensors;

  timer_prev_LEDring = millis();
  // Pre - calibrated values for the accelerometer
  bias_x = 0.05060;
  bias_y = 0.09364;
  bias_z = -0.17084;
}

// IMPORTANT: NEVER USE IF CONDITIONLS OR SWITCH OR SIMILAR IN THE LOOP TO MANAGE VARIABLES, ONLY IN messageCb
void loop()
{
  if(!nh.connected())
  {
    nodeHandle_notConnected();
  }
  else
  {
    nodeHandle_Connected();
  }
  nh.spinOnce();
  delay(1);
}
