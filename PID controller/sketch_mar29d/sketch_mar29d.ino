#include <Servo.h>

//Declare variables ------------------------
Servo x_servo, y_servo;
int x_servo_pin = 10, y_servo_pin = 11;
double x, y;
double active_area_x = 21.6, active_area_y = 16.4; 
double min_sensor_value_x = 174.0, min_sensor_value_y = 163.0;
double max_sensor_value_x = 856.0, max_sensor_value_y = 858.0;
int upper_right = 5, upper_left = 3;
int bottom_right = 4, bottom_left = 2;
double x_proposed, y_proposed;
double change, change_proposed;
double max_step_size = 3.0;
const int buffer_size = 5;
int x_index = 0, y_index = 0;
double buffer_values_x[buffer_size]; double buffer_values_y[buffer_size];
double output_x, output_y, set_point_x = 10.7, set_point_y = 8.2;
double Kp_x=1.7, Ki_x=0.00001, Kd_x=500.0, Ks_x=0.5;
double Kp_y=1.96, Ki_y=0.00001, Kd_y=700.0, Ks_y=0.5;
long currentTime_x, previousTime_x, elapsedTime_x;
long currentTime_y, previousTime_y, elapsedTime_y;
double error_x, cumError_x, rateError_x, lastError_x;
double error_y, cumError_y, rateError_y, lastError_y;
double servo_x_home = 93.0, servo_y_home = 95.0;
int led_pin_normal = 9, led_pin_circle = 8, led_pin_wait = 7, led_pin_cal = 6;
int state_button_pin = 12, calibrate_button_pin = 13;
int count = 0;
bool led_states[4];
int stable_count = 0, stable_break = 10; 
int square_index;
double x_point_list[4]={12.7,12.7,8.7,8.7}, y_point_list[4] = {6.2,9.2,9.2,6.2};


enum state_enum{normal, cal, square1, circle};
enum state_enum state;

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
 
        return Kp_x*error_x +Ks_x*error_x*abs(error_x) + Ki_x*cumError_x + Kd_x*rateError_x;                                        //have function return the PID output
}

double computePID_y(double& inp){     
        currentTime_y = millis();                //get current time
        elapsedTime_y = (double)(currentTime_y - previousTime_y);        //compute time elapsed from previous computation
        
        error_y = set_point_y - inp;                                // determine error
        cumError_y += error_y * elapsedTime_y;                // compute integral
        rateError_y = (double)(error_y - lastError_y)/elapsedTime_y;   // compute derivative
            
 
        lastError_y = error_y;                                //remember current error
        previousTime_y = currentTime_y;                        //remember current time
 
        return Kp_y*error_y+Ks_y*error_y*abs(error_y) + Ki_y*cumError_y + Kd_y*rateError_y;                                      //have function return the PID output
}

//-------------------------------------

void write_led(){ //States = {normal, circle, wait, cal}
    digitalWrite(led_pin_normal, LOW);
    digitalWrite(led_pin_circle, LOW);
    digitalWrite(led_pin_wait, LOW);
    digitalWrite(led_pin_cal, LOW);
    
    if(led_states[0]){
      digitalWrite(led_pin_normal, HIGH);
      }
     if(led_states[1]){
      digitalWrite(led_pin_circle, HIGH);
      }
    if(led_states[2]){
      digitalWrite(led_pin_wait, HIGH);
      }
    if(led_states[3]){
      digitalWrite(led_pin_cal, HIGH);
      }
  }

void reset_leds(){ //Resets the led state-array to 0
  for(int i; i < 4; i++){
    led_states[i] = 0;
    }
  }

bool check_button_input(int& button_pin){ //Simple button check
  if (digitalRead(button_pin) == HIGH){
    return 1;
    }
  else {
    return 0;
    }
  }


void set_home_position(){ //Sets new home/target position
  
  //Set led to wait mode
  reset_leds(); led_states[2] = 1; write_led();

  //Declare vectors for measuring new home position
  double x_buffer[100];
  double y_buffer[100];

  //Write servos to neutral state
  x_servo.write(servo_x_home);
  y_servo.write(servo_y_home);
  delay(1600);

  //Update LEDS to signal that it is ready for sampling
  reset_leds(); led_states[2] = 1; led_states[3] = 1; write_led();

  //Check for signal to start sampling
  while(!check_button_input(calibrate_button_pin)){
    delay(100);
    }
    
  //Signal that sampling is underway
  reset_leds(); led_states[3] = 1; write_led();

  //Copy the previous step size
  double temp = max_step_size;
  max_step_size = 40.0;
  
  //Obtain samples
  for(int i = 0; i < 100; i++){
    x_buffer[i] = read_X(buffer_size);
    y_buffer[i] = read_Y(buffer_size);
    }
  max_step_size = temp;

  //Set new home point and go back to state 1
  set_point_x = mean(x_buffer);
  set_point_y = mean(y_buffer);
  x = set_point_x;
  y = set_point_y;
  rateError_x = 0.0; error_x = 0.0; cumError_x=0.0; lastError_x = 0;
  rateError_y = 0.0; error_y = 0.0; cumError_y=0.0; lastError_y = 0;
  delay(1000);
  
  reset_leds(); led_states[0] = 1; write_led();
  state = normal;
  stable_count = stable_break;
  }

//Set square pos
void set_square_position(){
  set_point_x = x_point_list[square_index];
  set_point_y = y_point_list[square_index]; 
  }

double variance(double* arr){
  double mean_val = mean(arr);
  double temp = 0;
  for(int i = 0; i < sizeof(arr); i++){
    temp += pow(arr[i] - mean_val, 2);
    }
  return temp/sizeof(arr);
  }

void check_stable(){
  if(abs(x - set_point_x)<2.5 && abs(y - set_point_y)<2.5){
    stable_count ++;
    }
  else{
    stable_count = 0;
    
    }
  }


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

  // Prepare the resistive plate
  pinMode(upper_right, OUTPUT);
  pinMode(bottom_right, OUTPUT);
  pinMode(upper_left, OUTPUT);
  pinMode(bottom_left, OUTPUT);
  
  digitalWrite(upper_right, HIGH); //These are always the same regardless of measuring x of y
  digitalWrite(bottom_left, LOW); 

  //Prepare LEDs
  pinMode(led_pin_normal, OUTPUT);
  pinMode(led_pin_circle, OUTPUT);
  pinMode(led_pin_wait, OUTPUT);
  pinMode(led_pin_cal, OUTPUT);

  //Prepare buttons
  pinMode(calibrate_button_pin, INPUT);
  pinMode(state_button_pin, INPUT);
  
  //Pause before the loop starts
  delay(1000);

  //set_home_position();
  state = square1;
  //Get some staring times for the PID
  previousTime_y = millis();
  previousTime_x = millis();
}


void loop() {

  //Count of loops for the state machine
  count = 0;
  double fac=1;
  
  if(state == normal){
    //Set LEDS to normal mode
    reset_leds(); led_states[0] = 1; write_led();
    }
  
  //Normal mode
  while(state == normal){
    
      //increment
      count ++;
      //Every 100 loops, check for button input
      if(count == 5){
        count = 0; //Reset
        
        if(check_button_input(calibrate_button_pin)){ //If theres button input, then get new home position
          set_home_position();
          }
        else if(check_button_input(state_button_pin)){//If button to switch state is pushed, go to next state
          //state++;
          //state = state%2;
          state=square1;
          }
        }
        
      //Obtain positions
      x = read_X(5);
      y = read_Y(5);

      //Calculate the target angles by use of the PID 
      output_x = computePID_x(x);
      output_y = computePID_y(y);

      //Move servos
      x_servo.write(map_error_to_angle(output_x, 35*fac, servo_x_home));
      y_servo.write(map_error_to_angle(output_y, 35*fac, servo_y_home)); 
   

      check_stable();
     
        //Print to screen
      if(stable_count > stable_break){
        Kp_x=1.4/fac; Ki_x=0.00002/fac; Kd_x=500.0/fac; Ks_x=0.5/fac;
        Kp_y=1.5/fac; Ki_y=0.00002/fac; Kd_y=700.0/fac; Ks_y=0.5/fac;
        delay(130);
        }
      else {
          cumError_x = 0; cumError_y = 0;
          Kp_x=1.9/fac; Ki_x=0.0; Kd_x=1200.0/fac; Ks_x=0.6/fac;
          Kp_y=1.9/fac; Ki_y=0.0; Kd_y=1200.0/fac; Ks_y=0.6/fac;
          delay(80);
          }
        }
        
    if(state==square1){
      //Set LEDS to square mode
      reset_leds(); led_states[1] = 1; write_led();
      long start_time = millis();
      cumError_x = 0; cumError_y = 0;
      Kp_x=1.0/fac; Ki_x=0.0001/fac; Kd_x=800.0/fac; Ks_x=0/fac;
      Kp_y=1.0/fac; Ki_y=0.0001/fac; Kd_y=900.0/fac; Ks_y=0/fac;
      
      set_square_position();
      while(state == square1){
        if(millis() - start_time > 5000){
          square_index ++;
          square_index %= 4;
          set_square_position();
          start_time = millis();
          }
          
        //increment
        count ++;
        //Every 100 loops, check for button input
        if(count == 5){
          count = 0; //Reset
          
          if(check_button_input(calibrate_button_pin)){ //If theres button input, then get new home position
            set_home_position();
            }
          else if(check_button_input(state_button_pin)){//If button to switch state is pushed, go to next state
            //state+
            //state = state%2;
            state=normal;
            }
          }
          
        //Obtain positions
        x = read_X(5);
        y = read_Y(5);


        //Calculate the target angles by use of the PID 
        output_x = computePID_x(x);
        output_y = computePID_y(y);
  
        //Move servos
        x_servo.write(map_error_to_angle(output_x, 35*fac, servo_x_home));
        y_servo.write(map_error_to_angle(output_y, 35*fac, servo_y_home)); 
     
  
        delay(80);
        }
      }
        


  





















    
    }
















    
