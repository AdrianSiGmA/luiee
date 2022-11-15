// Libraries:
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

// --------------- PIN DEFINITION ---------------------
const int mot_pin[4] = {5,3,6,9};               // {right_pwm_f,right_pwm_b,left_pwm_f,left_pwm_b}
const int LED_pin[3] = {10,11,12};              // {green, red, blue}
const int buzzer_pin = 13;

// Sensors:
const float sensor_pin = A0;

// ------------------------ Variables definition ---------------------------
int mot_data[4] = {0,0,0,0};
int mot_pwm_right = 0;
int mot_pwm_left = 0;
int LED_data[3] = {0,0,0};
int buzzer_data = 0;
int n_sensors = 2;
float sensors_data[2] = {0,0};

// Subscriptor: IMPORTANT: ALWAYS USE THE IF CONDITIONALS HERE AND NOT IN THE LOOP ONE
void messageCb(const std_msgs::Int32MultiArray& mini_luiee_msg)    // BE CAREFUL! ALWAYS USE INT32 HERE, NOT FLOAT32
{
  mot_pwm_right = mini_luiee_msg.data[0];
  mot_pwm_left = mini_luiee_msg.data[1];
  LED_data[0] = mini_luiee_msg.data[2];
  LED_data[1] = mini_luiee_msg.data[3];
  LED_data[2] = mini_luiee_msg.data[4];
  buzzer_data = mini_luiee_msg.data[5];

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
}

// Node creation
ros::NodeHandle nh;

//Crea un arreglo de tipo multif32 de la libreria std_msgs
std_msgs::Float32MultiArray data_console;

// Publisher creation
ros::Publisher pub("/sensor_topic", &data_console);

// Subscriber creation
ros::Subscriber<std_msgs::Int32MultiArray> sub("/mini_luiee_topic", &messageCb);
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void setup()
{
  for(int i = 0; i < 4; i++)
    pinMode(mot_pin[i], OUTPUT);
  for(int i = 0; i < 3; i++)
     pinMode(LED_pin[i], OUTPUT);
  pinMode(buzzer_pin,OUTPUT);

  // Node initialization
  nh.initNode();

  // Subscrite node to sub
  nh.subscribe(sub);

  // Let the node know that we are going to publish
  nh.advertise(pub);
 
  // Declare the array size to be published
  data_console.data_length = n_sensors;
}

void loop()
{  
  sensors_data[0] = analogRead(sensor_pin);
  sensors_data[1] = 0;
  
  for(int i = 0; i < 4; i++)
    analogWrite(mot_pin[i], mot_data[i]);
  for(int i = 0; i < 3; i++)
    analogWrite(LED_pin[i], LED_data[i]);

  data_console.data = sensors_data;   // it passes the obtained sensor data to the publisher message
  pub.publish(&data_console);

  nh.spinOnce();
  delay(1);
}
