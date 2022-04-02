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
double max_step_size = 1;
const int buffer_size = 2;
int x_index = 0, y_index = 0;
double buffer_values_x[buffer_size]; double buffer_values_y[buffer_size];
double output_x, output_y, set_point_x = 10.7, set_point_y = 8.2;
double Kp_x=2.0, Ki_x=0.0002, Kd_x=500.0; 
double Kp_y=2.0, Ki_y=0.0002, Kd_y=500.0; 
long currentTime_x, previousTime_x, elapsedTime_x;
long currentTime_y, previousTime_y, elapsedTime_y;
double error_x, cumError_x, rateError_x, lastError_x;
double error_y, cumError_y, rateError_y, lastError_y;
double servo_x_home = 91.0, servo_y_home = 95.0;


//PID PID_x(&x, &output_x, &set_point_x, Kp_x, Ki_x, Kd_x, DIRECT);
//PID PID_y(&y, &output_y, &set_point_y, Kp_y, Ki_y, Kd_y, DIRECT);


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
//-----------------------------

//Touchplate -----------------------------
double read_X(int reads = 1){
  //Prepare to read x
  //digitalWrite(upper_right, HIGH);
  digitalWrite(bottom_right, HIGH);
  digitalWrite(upper_left, LOW);
  //digitalWrite(bottom_left, LOW);

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
  //digitalWrite(upper_right, HIGH);
  digitalWrite(bottom_right, LOW);
  digitalWrite(upper_left, HIGH);
  //digitalWrite(bottom_left, LOW);
  
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


double map_error_to_angle(double inp_err, double offset, double center, double max_in = 40, double min_in = -40){
    return (min(max(inp_err,min_in),max_in) + max_in) * (2*offset) / (max_in - min_in) + center - offset;

  }

//---------------------------



// PID --------------------------------

double computePID_x(double& inp){     
        currentTime_x = millis();                //get current time
        elapsedTime_x = (double)(currentTime_x - previousTime_x);        //compute time elapsed from previous computation
        
        error_x = set_point_x - inp;                                // determine error
        cumError_x += error_x * elapsedTime_x;                // compute integral
        rateError_x = (error_x - lastError_x)/elapsedTime_x;   // compute derivative
        
        lastError_x = error_x;                                //remember current error
        previousTime_x = currentTime_x;                        //remember current time
 
        return Kp_x*error_x + Ki_x*cumError_x + Kd_x*rateError_x;                                        //have function return the PID output
}

double computePID_y(double& inp){     
        currentTime_y = millis();                //get current time
        elapsedTime_y = (double)(currentTime_y - previousTime_y);        //compute time elapsed from previous computation
        
        error_y = set_point_y - inp;                                // determine error
        cumError_y += error_y * elapsedTime_y;                // compute integral
        rateError_y = (double)(error_y - lastError_y)/elapsedTime_y;   // compute derivative
            
 
        lastError_y = error_y;                                //remember current error
        previousTime_y = currentTime_y;                        //remember current time
 
        return Kp_y*error_y + Ki_y*cumError_y + Kd_y*rateError_y;                                      //have function return the PID output
}

//-------------------------------------

//Program execution-------------------

void setup() {
  //Begin communication
  Serial.begin(9600);
  
  // Prepare A0 for reading position on plate
  pinMode(A0, INPUT);

  //Prepare servos
  x_servo.attach(x_servo_pin);
  y_servo.attach(y_servo_pin);
  x_servo.write(servo_x_home);
  y_servo.write(servo_y_home);

  //Prepare PID
  /*
  PID_x.SetMode(AUTOMATIC);
  PID_x.SetTunings(Kp_x,Ki_x,Kd_x);
  PID_y.SetMode(AUTOMATIC);
  PID_y.SetTunings(Kp_y,Ki_y,Kd_y);
  PID_x.SetOutputLimits(20, 160);
  PID_y.SetOutputLimits(20, 160);
  PID_x.SetSampleTime(50);
  PID_y.SetSampleTime(50);
  */

  // Prepare the resistive plate
  pinMode(upper_right, OUTPUT);
  pinMode(bottom_right, OUTPUT);
  pinMode(upper_left, OUTPUT);
  pinMode(bottom_left, OUTPUT);
  
  digitalWrite(upper_right, HIGH); //These are always the same
  digitalWrite(bottom_left, LOW); 

  //Pause before the loop starts
  delay(1000);
  previousTime_y = millis();
  previousTime_x = millis();
}


void loop() {

  //Obtain positions
  x = read_X(2);
  y = read_Y(2);

  //Calculate the target angles by use of the PID 
  //PID_x.Compute();
  //PID_y.Compute();
  output_x = computePID_x(x);
  output_y = computePID_y(y);

  //Move servos
  x_servo.write(map_error_to_angle(output_x, 35, servo_x_home));
  y_servo.write(map_error_to_angle(output_y, 35, servo_y_home));

  //Print to screen
  /*
  Serial.print(Ki_x*cumError_x);
  Serial.print(", ");
  Serial.print(Kp_x*error_x);
  Serial.print(", ");
  Serial.println(Kd_x*rateError_x);
  */
  
}
