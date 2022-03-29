#include <Servo.h>
#include <PID_v1.h>

//Declare variables ------------------------
Servo x_servo, y_servo;
int x_servo_pin = 10, y_servo_pin = 11;
double x,y;
double active_area_x = 21.6, active_area_y = 16.4; 
double min_sensor_value_x = 174.0, min_sensor_value_y = 163.0;
double max_sensor_value_x = 856.0, max_sensor_value_y = 858.0;
int upper_right = 5, upper_left = 3;
int bottom_right = 4, bottom_left = 2;
double x_proposed, y_proposed;
double change, change_proposed;
double max_step_size = 0.5;
const int buffer_size = 5;
int x_index = 0, y_index = 0;
double buffer_values_x[buffer_size]; double buffer_values_y[buffer_size];
double set_point[2] = {10.8,8.2};
double output_x, output_y;
double Kp=1, Ki=1, Kd=0; 

PID PID_x(&x, &output_x, &set_point[0], Kp, Ki, Kd, DIRECT);
PID PID_y(&y, &output_y, &set_point[1], Kp, Ki, Kd, DIRECT);


//------------------------------------------

// Buffer system --------------------------------
double mean(double* vector){
  double temp = 0.0;
  for (int i = 0; i < sizeof(vector); i++)
  {
    temp += vector[i];
  }
  return temp/sizeof(vector);}

double scale_x(int sensor_input){
  return ((sensor_input-min_sensor_value_x)*active_area_x/(max_sensor_value_x-min_sensor_value_x));
  }

double scale_y(int sensor_input){
  return ((sensor_input-min_sensor_value_y)*active_area_y/(max_sensor_value_y-min_sensor_value_y));
  }

double read_X(int reads = 1){
  //Prepare to read x
  digitalWrite(upper_right, HIGH);
  digitalWrite(bottom_right, HIGH);
  digitalWrite(upper_left, LOW);
  digitalWrite(bottom_left, LOW);

  for (int i = 0; i < reads; i++)
  {
    x_proposed = scale_x(analogRead(A0));
    change_proposed = x_proposed-x;
    if(abs(change_proposed)>max_step_size){
      change = (change_proposed)/abs(change_proposed);
      x_proposed = x + max_step_size*change;
      }
    buffer_values_x[x_index] = x_proposed;
    x_index ++ ;
    x_index = x_index%buffer_size;
  }

  //Read x value
  return mean(buffer_values_x);
  }
  
double read_Y(int reads = 1){
  //Prepare to read y
  digitalWrite(upper_right, HIGH);
  digitalWrite(bottom_right, LOW);
  digitalWrite(upper_left, HIGH);
  digitalWrite(bottom_left, LOW);
  
  for (int i = 0; i < reads; i++)
  {
    y_proposed = scale_y(analogRead(A0));
    change_proposed = y_proposed-y;
    if(abs(change_proposed)>max_step_size){
      change = (change_proposed)/abs(change_proposed);
      y_proposed = y + max_step_size*change;
      }
    buffer_values_y[y_index] = y_proposed;
    y_index ++ ;
    y_index = y_index%buffer_size;
  }

  //Read x value
  return mean(buffer_values_y);
  }


// Buffer system --------------------------------

void setup() {
  // put your setup code here, to run once:
  pinMode(A0, INPUT);
  Serial.begin(9600);
  x_servo.attach(x_servo_pin);
  y_servo.attach(y_servo_pin);
  PID_x.SetMode(AUTOMATIC);
  PID_x.SetTunings(Kp,Ki,Kd);
  PID_y.SetMode(AUTOMATIC);
  PID_y.SetTunings(Kp,Ki,Kd);
  pinMode(upper_right, OUTPUT);
  pinMode(bottom_right, OUTPUT);
  pinMode(upper_left, OUTPUT);
  pinMode(bottom_left, OUTPUT);
  digitalWrite(upper_right, HIGH);
  digitalWrite(bottom_left, LOW);
}


void loop() {

  //Obtain positions
  x = read_X(5);
  y = read_Y(5);

  //Calculate the PID target positons
  PID_x.Compute();
  PID_y.Compute();

  //Move servos
  x_servo.write(output_x);
  y_servo.write(output_y);

  Serial.print(output_x);
  Serial.print(", ");
  Serial.println(output_y);

  delay(1);
}


