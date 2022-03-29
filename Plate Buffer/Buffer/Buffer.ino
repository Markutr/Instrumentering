double x;
double y;
double active_area_x = 21.6; //In cm
double active_area_y = 16.4;
double min_sensor_value_x = 174.0;
double max_sensor_value_x = 856.0;
double min_sensor_value_y = 163.0;
double max_sensor_value_y = 858.0;
int upper_right = 5;
int bottom_right = 4;
int upper_left = 3;
int bottom_left = 2;

double bufferValuesX[10];
double bufferValuesY[10];

void setup() {
  // put your setup code here, to run once:
  pinMode(A0, INPUT);
  Serial.begin(9600);
  pinMode(upper_right, OUTPUT);
  pinMode(bottom_right, OUTPUT);
  pinMode(upper_left, OUTPUT);
  pinMode(bottom_left, OUTPUT);
  digitalWrite(upper_right, HIGH);
  digitalWrite(bottom_left, LOW);
}

void loop() {
  for (int i = 0; i < 10; i++)
  {
    bufferValuesX[i] = read_X();
    bufferValuesY[i] = read_Y();
  }
  x = mean(bufferValuesX);
  y = mean(bufferValuesY);
  Serial.print(x);
  Serial.print(", ");
  Serial.println(y);
  delay(1);
}

double mean(double* vector){
  double temp = 0.0;
  for (int i = 0; i < sizeof(vector); i++)
  {
    temp += vector[i];
  }
  return temp/sizeof(vector);
}



double read_X(){
  //Prepare to read x
  digitalWrite(upper_right, HIGH);
  digitalWrite(bottom_right, HIGH);
  digitalWrite(upper_left, LOW);
  digitalWrite(bottom_left, LOW);

  //Read x value
  return ((analogRead(A0)-min_sensor_value_x)*active_area_x/(max_sensor_value_x-min_sensor_value_x));
  }
  
double read_Y(){
  //Prepare to read y
  digitalWrite(upper_right, HIGH);
  digitalWrite(bottom_right, LOW);
  digitalWrite(upper_left, HIGH);
  digitalWrite(bottom_left, LOW);
  
  //Read y value
  return((analogRead(A0)-min_sensor_value_y)*active_area_y/(max_sensor_value_y-min_sensor_value_y));
  }
