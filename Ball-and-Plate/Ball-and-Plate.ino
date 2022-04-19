//Include the servo library
#include <Servo.h>

//Declare global variables ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Servo x_servo, y_servo;
int x_servo_pin = 10, y_servo_pin = 11;                                                                   //Pins used for the servo motors
double x, y;                                                                                              //current position on the x and y axis
double active_area_x = 21.6, active_area_y = 16.4;                                                        //The size of the active area of the plate
double min_sensor_value_x = 170.0, min_sensor_value_y = 170.0;                                            //Minimum measured sensor value from the plate in x and y
double max_sensor_value_x = 840.0, max_sensor_value_y = 840.0;                                            //Maximum measured sensor value from the plate in x and y
const int upper_right = 5, upper_left = 3;                                                                //Pins for setting up a potential gradient for the touch plate
const int bottom_right = 4, bottom_left = 2;
double x_proposed, y_proposed;                                                                            //Proposed value for x and y. Used in buffer system
double change, change_proposed;                                                                           //Keeps track of the change in position.
double max_step_size = 5.0;                                                                               //Limits how much the position in x and y may change in a single update
const int buffer_size = 2;                                                                                //Size of the buffer used for measuring x and y
int buffer_read = 5;                                                                                      //Number of reads performed for each measurement of x and y
int x_index = 0, y_index = 0;                                                                             //Used for keeping track of indicies in the buffer
double buffer_values_x[buffer_size]; double buffer_values_y[buffer_size];                                 //Array for keeping the buffered measurement values of x and y
double output_x, output_y, set_point_x = 10.7, set_point_y = 8.2;                                         //Declare the variables which contain the PID output and the desired position
double Kp_x=1.7, Ki_x=0.01, Kd_x=0.5, Ks_x=0.5;                                                      //PID coefficients for the x-servo
double Kp_y=1.96, Ki_y=0.01, Kd_y=0.7, Ks_y=0.5;                                                     //PID coefficients for the y-servo
long currentTime_x, previousTime_x;                                                                       //Keeps track of time between measurements in x
long currentTime_y, previousTime_y;                                                                       //Keeps track of time between measurements in y
double elapsedTime_x, elapsedTime_y;                                                                      //The elapsed time between PID calculations. Used for calculating the integration and derivation error
double error_x, cumError_x, rateError_x, lastError_x;                                                     //Keep track of error in x
double error_y, cumError_y, rateError_y, lastError_y;                                                     //Keep track of error in y
double servo_x_home = 89.0, servo_y_home = 95.0;                                                          //Home positions for the servo so the plate is approximately flat
int led_pin_normal = 9, led_pin_square = 8 ,led_pin_circle = 7, led_pin_wait = 1, led_pin_cal = 6;        //Define pins for LEDs for the different states
int state_button_pin = 12, calibrate_button_pin = 13;                                                     //Define pins for the buttons
int count = 0;                                                                                            //Loop counter. Used for limiting how many reads performed for the buttons
bool led_states[5];                                                                                       //Keeps track of the boolean states (off/on = 0/1) for all the LEDS. States = {normal, square, circle, wait, cal}
int stable_count = 0, stable_break = 10;                                                                  //Keeps track of how many times the system is close enough to rest and define a break point where the sytem enters a stable-mode
int square_index;                                                                                         //Keeps track of where in the square positional sequence we are
double x_point_list[4]={12.7,12.7,8.7,8.7}, y_point_list[4] = {6.2,9.2,9.2,6.2};                          //Stores the positions used in the square mode
double radius = 3, angle, center_x = 10.7, center_y = 8.2;                                                //Variables used for circle mode
int switch_time, switch_wait=500;                                                                         //Used in the transition between states so that a button push is not read twice
enum state_enum{normal, sqr, circle};                                                                     //Enum used for the different states
enum state_enum state;                                                                                    //Current state

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Buffer system --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
double mean(double* vector){//Calculates mean of given vector
  double temp = 0.0;
  for (int i = 0; i < sizeof(vector); i++)
  {
    temp += vector[i];
  }
  return temp/sizeof(vector);
  }

double scale_x(int sensor_input){ //Scales the sensor input in x to be centimeters
  return ((sensor_input-min_sensor_value_x)*active_area_x/(max_sensor_value_x-min_sensor_value_x));
  }

double scale_y(int sensor_input){ //Scales the sensor input in y to be centimeters
  return ((sensor_input-min_sensor_value_y)*active_area_y/(max_sensor_value_y-min_sensor_value_y));
  }
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//Touchplate -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
double read_X(int reads = 1){ //Reads the position along the x axis
  //Prepare to read x
  //digitalWrite(upper_right, HIGH); Is already in place
  digitalWrite(bottom_right, HIGH);
  digitalWrite(upper_left, LOW);
  //digitalWrite(bottom_left, LOW); Is already in place

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
  
double read_Y(int reads = 1){ //Reads the position along the y axis
  //Prepare to read y
  //digitalWrite(upper_right, HIGH); Is already in place
  digitalWrite(bottom_right, LOW);
  digitalWrite(upper_left, HIGH);
  //digitalWrite(bottom_left, LOW); Is already in place
  
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


double map_error_to_angle(double inp_err, double offset, double center, double max_in = 40, double min_in = -40){ //Maps the error of the system to an angle for the servos
    return (min(max(inp_err,min_in),max_in) + max_in) * (2*offset) / (max_in - min_in) + center - offset;

  }

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



// PID --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

double computePID_x(double& inp){ //Calculates the output of the PID on the x axis     
        currentTime_x = millis();                                                             //get current time
        elapsedTime_x = (double)(currentTime_x - previousTime_x);                             //compute time elapsed from previous computation
        
        error_x = set_point_x - inp;                                                          // determine error
        cumError_x += error_x * elapsedTime_x/1000;                                           // compute integral. /1000 cause ms to s
        rateError_x = (double) 1000*(error_x - lastError_x)/elapsedTime_x;                    // compute derivative. *1000 becuse ms to s
        
        lastError_x = error_x;                                                                //remember current error
        previousTime_x = currentTime_x;                                                       //remember current time
 
        return Kp_x*error_x +Ks_x*error_x*abs(error_x) + Ki_x*cumError_x + Kd_x*rateError_x;  //have function return the PID output
}

double computePID_y(double& inp){ //Calculates the output of the PID on the y axis  
        currentTime_y = millis();                                                             //get current time
        elapsedTime_y = (double)(currentTime_y - previousTime_y);                             //compute time elapsed from previous computation
        
        error_y = set_point_y - inp;                                                          // determine error
        cumError_y += error_y * elapsedTime_y/1000;                                          // compute integral
        rateError_y = (double) 1000*(error_y - lastError_y)/elapsedTime_y;                    // compute derivative
            
 
        lastError_y = error_y;                                                                //remember current error
        previousTime_y = currentTime_y;                                                       //remember current time
 
        return Kp_y*error_y+Ks_y*error_y*abs(error_y) + Ki_y*cumError_y + Kd_y*rateError_y;   //have function return the PID output
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// LED functions ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void write_led(){ //States = {normal, square, circle, wait, cal}. Resets the state vector to all off, then lights up LEDS based on what is stored in led_states
    digitalWrite(led_pin_normal, LOW);
    digitalWrite(led_pin_square, LOW);
    digitalWrite(led_pin_circle, LOW);
    digitalWrite(led_pin_wait, LOW);
    digitalWrite(led_pin_cal, LOW);
    
    if(led_states[0]){
      digitalWrite(led_pin_normal, HIGH);
      }
     if(led_states[1]){
      digitalWrite(led_pin_square, HIGH);
      }
    if(led_states[2]){
      digitalWrite(led_pin_circle, HIGH);
      }
    if(led_states[3]){
      digitalWrite(led_pin_wait, HIGH);
      }
    if(led_states[4]){
      digitalWrite(led_pin_cal, HIGH);
      }
  }

void reset_leds(){ //Resets the led state-array to 0 (all off)
  for(int i=0; i < 5; i++){
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
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//Position update functions ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void set_home_position(){ //Sets new home/target position. Press the setpoint button to confirm new placement
  
  //Set led to wait mode
  reset_leds(); led_states[3] = 1; write_led();

  //Declare vectors for measuring new home position
  double x_buffer[100];
  double y_buffer[100];

  //Write servos to neutral state
  x_servo.write(servo_x_home);
  y_servo.write(servo_y_home);
  delay(1600);

  //Update LEDS to signal that it is ready for sampling
  reset_leds(); led_states[3] = 1; led_states[4] = 1; write_led();

  //Check for signal to start sampling
  while(!check_button_input(calibrate_button_pin)){
    delay(100);
    }
    
  //Signal that sampling is underway
  reset_leds(); led_states[4] = 1; write_led();

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

void set_square_position(){ //Set position based on the index of the positional sequence
  set_point_x = x_point_list[square_index];
  set_point_y = y_point_list[square_index]; 
  }

void set_circle_position(){ //Set position based on the previous angle
  angle += 0.2;
  set_point_x = center_x + radius*cos(angle);
  set_point_y = center_y + radius*sin(angle);
  }

void check_stable(){ //Check if the ball is close enough to the target position. If so, increment the stable count. Else reset it
  if(abs(x - set_point_x)<2.5 && abs(y - set_point_y)<2.5){
    stable_count ++;
    }
  else{
    stable_count = 0;
    }
  }

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//Program execution-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup() {
  
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

  //These are always the same regardless of measuring x of y
  digitalWrite(upper_right, HIGH); 
  digitalWrite(bottom_left, LOW); 

  //Prepare LEDs
  pinMode(led_pin_normal, OUTPUT);
  pinMode(led_pin_circle, OUTPUT);
  pinMode(led_pin_wait, OUTPUT);
  pinMode(led_pin_cal, OUTPUT);
  pinMode(led_pin_square, OUTPUT);

  //Prepare buttons
  pinMode(calibrate_button_pin, INPUT);
  pinMode(state_button_pin, INPUT);
  
  //Pause before the loop starts
  delay(1000);

  //Start by requesing a target position from the user
  set_home_position();

  //Get some staring times for the PID and buttons
  previousTime_y = millis();
  previousTime_x = millis();
  switch_time = millis();
}


void loop() {

  //Count of loops for the state machine
  count = 0;
  
  if(state == normal){ //Normal mode. Resists changes to target position. Simple balance
    //Set LEDS to normal mode
    reset_leds(); led_states[0] = 1; write_led();
    }
  
  //Normal mode
  while(state == normal){
    
      //increment
      count ++;
      
      //Every 5 loops, check for button input
      if(count == 5){
        count = 0; //Reset
        
        if(check_button_input(calibrate_button_pin)){ //If theres button input, then get new home position
          set_home_position();
          }
        else if(check_button_input(state_button_pin) && millis()-switch_time > switch_wait){//If button to switch state is pushed, go to next state. Put target position to center in the meantime
            state=sqr;
            switch_time = millis();
            set_point_x = center_x, set_point_y = center_y;
          }
        }

      //Obtain positions
      x = read_X(2);
      y = read_Y(2);

      //Calculate the target angles by use of the PID 
      output_x = computePID_x(x);
      output_y = computePID_y(y);

      //Move servos
      x_servo.write(map_error_to_angle(output_x, 35, servo_x_home));
      y_servo.write(map_error_to_angle(output_y, 35, servo_y_home)); 
   
      //Check if the system is stable
      check_stable();
     
      //Check whether to use the stable mode coefficients or not
      if(stable_count > stable_break){
        Kp_x=1.4; Ki_x=0.02; Kd_x=0.5; Ks_x=0.5;
        Kp_y=1.5; Ki_y=0.02; Kd_y=0.7; Ks_y=0.5;
        delay(130);
        }
      else {
          cumError_x = 0; cumError_y = 0; //Reset cumulative error
          Kp_x=1.9; Ki_x=0.0; Kd_x=1.0; Ks_x=0.6;
          Kp_y=2.5; Ki_y=0.0; Kd_y=1.2; Ks_y=0.6;
          delay(90);
          }  
        }
        
        
    if(state==sqr){ //Square mode. Moves the ball in a rectangular pattern
      //Set LEDS to square mode
      reset_leds(); led_states[1] = 1; write_led();
      long start_time = millis();
      cumError_x = 0; cumError_y = 0;
      Kp_x=2.5; Ki_x=0.2; Kd_x=0.8; Ks_x=0.3;
      Kp_y=2.5; Ki_y=0.2; Kd_y=0.9; Ks_y=0.3;
      
      set_square_position(); //Obtain the position in the sequence for the current index
      
      while(state == sqr){
        if(millis() - start_time > 5000){ //Every 5 seconds, move the target position to the next position in the positional sequence
          square_index ++;
          square_index %= 4;
          set_square_position();
          start_time = millis();
          }
          
        //increment
        count ++;
        //Every 5 loops, check for button input
        if(count == 5){
          count = 0; //Reset
          
          if(check_button_input(calibrate_button_pin)){ //If theres button input, then get new home position
            set_home_position();
            }
          else if(check_button_input(state_button_pin) && millis()-switch_time > switch_wait){//If button to switch state is pushed, go to next state

            state = circle;
            switch_time = millis();
            }
          }
          
        //Obtain positions
        x = read_X(buffer_read);
        y = read_Y(buffer_read);


        //Calculate the target angles by use of the PID 
        output_x = computePID_x(x);
        output_y = computePID_y(y);
  
        //Move servos
        x_servo.write(map_error_to_angle(output_x, 35, servo_x_home));
        y_servo.write(map_error_to_angle(output_y, 35, servo_y_home)); 
        
        delay(80);
        }
      }

      if (state==circle){ //Circle mode. Moves the ball in a circular pattern.

        //Obtain the angle at which the ball is at. Use for staring point for the pattern
        angle = atan2((y-center_y),(x-center_x));
        if(angle<0){
          angle+=6.28;
          }
        x = radius*cos(angle)+center_x;
        y = radius*sin(angle)+center_y;

        //Reset LEDS and write for the new state.
        reset_leds(); led_states[2] = 1; write_led();

        //Reset error and update coefficients
        cumError_x = 0; cumError_y = 0;
        Kp_x=2.0; Ki_x=0.0; Kd_x=1.0; Ks_x=0.6;
        Kp_y=2.0; Ki_y=0.0; Kd_y=1.0; Ks_y=0.6;
        
        while(state==circle){

          //Update the target position
          set_circle_position();
          
          //increment
          count ++;
          //Every 5 loops, check for button input
          if(count == 5){
            count = 0; //Reset
            
            if(check_button_input(calibrate_button_pin)){ //If theres button input, then get new home position
              set_home_position();
              }
            else if(check_button_input(state_button_pin) && millis()-switch_time > switch_wait){//If button to switch state is pushed, go to next state
              state = normal;
              switch_time = millis();
              set_point_x = center_x, set_point_y = center_y;
              }
            }
          
          //Obtain positions
          x = read_X(buffer_read);
          y = read_Y(buffer_read);
  
  
          //Calculate the target angles by use of the PID 
          output_x = computePID_x(x);
          output_y = computePID_y(y);
    
          //Move servos
          x_servo.write(map_error_to_angle(output_x, 35, servo_x_home));
          y_servo.write(map_error_to_angle(output_y, 35, servo_y_home)); 
      
          delay(80);
          }
        }
    }
















    
