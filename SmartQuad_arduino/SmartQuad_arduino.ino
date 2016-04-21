  #include <Wire.h> // I2C library, gyroscope
#include <Servo.h>
#include <EEPROM.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID konstanty
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_stable_gain_roll = 0;		      // Konstanta pro stabilizaci I kontroleru (roll)
float pid_p_gain_roll = 0;			      //Konstanta pro gyroskop P kontroleru (roll)
float pid_i_gain_roll = 0;			      //Konstanta pro gyroskop I kontroleru (roll)
float pid_d_gain_roll = 0;	              //Konstanta pro gyroskop D kontroleru (roll)
int pid_max_roll = 65;				      //Maximální výstupní hodnota PID algorytmu (+/-)

float pid_stable_gain_pitch = pid_stable_gain_roll;//Konstanta pro stabilizaci I kontroleru (pitch)
float pid_p_gain_pitch = pid_p_gain_roll;  //Konstanta pro gyroskop P kontroleru (pitch)
float pid_i_gain_pitch = pid_i_gain_roll;  //Konstanta pro gyroskop I kontroleru (pitch)
float pid_d_gain_pitch = pid_d_gain_roll;  //Konstanta pro gyroskop D kontroleru (pitch)
int pid_max_pitch = pid_max_roll;          //Maximální výstupní hodnota PID algorytmu (+/-)

float pid_p_gain_yaw = 0.2;                //Konstanta pro gyroskop P kontroleru (yaw)
float pid_i_gain_yaw = 0.02;               //Konstanta pro gyroskop I kontroleru (yaw)
float pid_d_gain_yaw = 0;                  //Konstanta pro gyroskop D kontroleru  (yaw)
int pid_max_yaw = 45;                      //Maximální výstupní hodnota PID algorytmu (+/-)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Accelerometer ADXL345
#define ACC (0xA7>>1)    //ADXL345 ACC adresa
#define A_TO_READ (6)        //Počet bajtů přijatých od Acc

int L3G4200D_Address = 104; //I2C adresa gyroskopu

int gx;
int gy;
int gz; 

unsigned long last_dt;

int cal_int;

byte lowByte, highByte;

double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;

int esc_1, esc_2, esc_3, esc_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int throttle;

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

//Kalman Filter
double q_angle = 0.001, q_bias = 0.003, r_measure = 0.4;
double angleX = 0, biasX = 0, rateX;
double pX[2][2] = { {0,0},{0,0} }, kX[2], yX, sX;

double angleY = 0, biasY = 0, rateY;
double pY[2][2] = { {0,0},{0,0} }, kY[2], yY, sY;

bool first = true;

// motory
Servo e1;
Servo e2;
Servo e3;
Servo e4;



float		angle_roll;
float		angle_pitch;

double    base_x_accel = 0;
double    base_y_accel = 0;
double    base_z_accel = 0;

float voltage;

void setup()
{
receiver_input_channel_1 = receiver_input_channel_2 = receiver_input_channel_3 = receiver_input_channel_4 = 0;
Serial.begin(115200);                                     //začátek komunikace po sběrnici UART (raspberry pi)
Wire.begin();                                           //začátek komunikace po I2C (gyroskop, acc)
initAcc();                                              //nastavení registrů pro Acc
delay(100); 
setupL3G4200D();										//nastavení registrů pro Gyroskop
delay(100);
bool calibrated;
EEPROM.get(0, calibrated);                              //získání hodnoty z paměti, zda již byl gyroskop a Acc kalibrován
if(calibrated){                                         //získávání kalibračních hodnot z paměti
  EEPROM.get(1, base_x_accel); 
  EEPROM.get(5, base_y_accel);
  EEPROM.get(9, base_z_accel);
  EEPROM.get(13, gyro_roll_cal);
  EEPROM.get(17, gyro_pitch_cal);
  EEPROM.get(21, gyro_yaw_cal);
  }
else{ //kalibrace (2000 hodnot)
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){              
    double acc[3];
    getGyroValues();                                           
    getAccelerometerData(acc);
    base_x_accel += acc[0];
    base_y_accel += acc[1];
    base_z_accel += acc[2];
    gyro_roll_cal += gyro_roll;                                
    gyro_pitch_cal += gyro_pitch;                              
    gyro_yaw_cal += gyro_yaw;                                  
    delay(3);                                                  
  }
  gyro_roll_cal /= 2000;                                       
  gyro_pitch_cal /= 2000;                                      
  gyro_yaw_cal /= 2000;                                        
  base_x_accel /= 2000;
  base_y_accel /= 2000;
  base_z_accel /= 2000;
  EEPROM.put(0, true);                                  //uložení do paměti
  EEPROM.put(1, base_x_accel);
  EEPROM.put(5, base_y_accel);
  EEPROM.put(9, base_z_accel);
  EEPROM.put(13, gyro_roll_cal);
  EEPROM.put(17, gyro_pitch_cal);
  EEPROM.put(21, gyro_yaw_cal);
}
//nastavení výstupů pro motory
e1.attach(4);
e2.attach(5);
e3.attach(6);
e4.attach(7);
e1.write(0);
e2.write(0);
e3.write(0);
e4.write(0);
cal_int = 2000;                                         //řeší problém při čtení kalibračních hodnot z paměti (začátek kalibrace vstupních hodnot z gyroskopu a Acc)
}
 
unsigned long last_pid_t = 0;

float last_pitch, last_roll, last_yaw;
unsigned long dt_last = 0;
void loop()
{
double dT;
double acc[3];
getAccelerometerData(acc);                              //získání hodnot z Accelerometru do pole acc
getGyroValues();                                        //získání hodnot z Gyroskopu
readData();                                             //získání dat ze sběrnice UART (raspberry pi)

unsigned long t_now = millis();
float gyro_x = gyro_pitch / 14.286;
float gyro_y = gyro_roll / 14.286;
float gyro_z = gyro_yaw / 14.286;
double dt = (t_now - last_dt)/1000.0f;
float kalman_pitch = kalman_get_angle_X(acc[0], gyro_x, dt);
float kalman_roll = kalman_get_angle_Y(acc[1], gyro_y, dt);
angle_pitch = (angle_pitch * 0.8) + (kalman_pitch * 0.2);
angle_roll = (angle_roll * 0.8) + (kalman_roll * 0.2);

voltage = analogRead(A0)*5.0 / 1024.0;

//pokud hondota pro výšku se nerovná nule
if(receiver_input_channel_4 != 0 && voltage > 3.0){
gyro_yaw_input = (gyro_yaw_input * 0.8) + ((gyro_yaw/14.286) * 0.2);

//dosazení vstupních hodnot do proměnných pro PID algorytmus (dvojnásoběk kvůli prudšímu záběru)
pid_roll_setpoint = constrain(receiver_input_channel_1*4, -20, 20);
pid_pitch_setpoint = constrain(receiver_input_channel_2*4, -20, 20);
pid_yaw_setpoint = receiver_input_channel_3;

throttle = receiver_input_channel_4;
calculate_pid(); //spuštění PID algorytmu

//výstupní hodnoty pro každý motor zvlášť
esc_1 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Kalkulace pulzu pro esc 1 (front-left - CW)
esc_2 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Kalkulace pulzu pro esc 2 (front-right - CCW)
esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Kalkulace pulzu pro esc 3 (rear-left - CCW)
esc_4 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Kalkulace pulzu pro esc 4 (rear-right - CW)


if(esc_1 > 179) esc_1 = 179;
if(esc_1 < 0) esc_1 = 0;
if(esc_2 > 179) esc_2 = 179;
if(esc_2 < 0) esc_2 = 0;
if(esc_3 > 179) esc_3 = 179;
if(esc_3 < 0) esc_3 = 0;
if(esc_4 > 179) esc_4 = 179;
if(esc_4 < 0) esc_4 = 0;

//spuštění pulzu pro ESC -> motory
e1.write(esc_1);
e2.write(esc_2);
e3.write(esc_3);
e4.write(esc_4);
 
}
else{                                                     //nulace algorytmu při vypnutých motorech
pid_i_mem_roll = 0;
pid_last_roll_d_error = 0;
pid_i_mem_pitch = 0;
pid_last_pitch_d_error = 0;
pid_i_mem_yaw = 0;
pid_last_yaw_d_error = 0;
gyro_roll_input = 0;
gyro_pitch_input = 0;
gyro_yaw_input = 0;
last_pid_t = 0;
e1.write(0);
e2.write(0);
e3.write(0);
e4.write(0);
  }
  //odesílání hodnot pro raspberry (každá hodnota 2 bajty)
  //1. bajt 0x00 = kladné | 0xFE = záporné
  //2. bajt absolutní hodnota
      Serial.print(char(0xFE));
      Serial.print(char(0xFE));
      Serial.print(char(0xFE));
      Serial.print(char(0xFE));
      if(acc[0] < 0) Serial.print(char(0xFF));
      else Serial.print(char(0x00));
      Serial.print(char((int)abs(angle_pitch)));
      if(acc[1] < 0) Serial.print(char(0xFF));
      else Serial.print(char(0x00));
      Serial.print(char((int)abs(angle_roll)));
      if(acc[2] < 0) Serial.print(char(0xFF));
      else Serial.print(char(0x00));
      Serial.print(char((int)abs(acc[2])));
    if(gyro_x < 0) Serial.print(char(0xFF));
    else Serial.print(char(0x00));
    Serial.print(char(abs((int)(constrain(gyro_x, -255, 255)))));
    if(gyro_y < 0) Serial.print(char(0xFF));
    else Serial.print(char(0x00));
    Serial.print(char(abs((int)(constrain(gyro_y, -255, 255)))));
    if(gyro_z < 0) Serial.print(char(0xFF));
    else Serial.print(char(0x00));
    Serial.print(char(abs((int)constrain(gyro_z , -255, 255))));
	last_dt = dt;
}

void calculate_pid(){
  
  unsigned long pid_t = millis();
  float pid_dt = (pid_t - last_pid_t)/1000.0f;
  if(last_pid_t == 0 || pid_dt > 1){
      last_pid_t = pid_t;
      return;
    }
  last_pid_t = pid_t;
  //Roll kalkulace
  pid_error_temp = (pid_roll_setpoint - angle_roll)*pid_stable_gain_roll - (gyro_roll/14.286);                //vypočtení erroru
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp * pid_dt;                                                  //přičtení erroru vynásobeným konstantou
  pid_i_mem_roll = constrain(pid_i_mem_roll, pid_max_roll*-1, pid_max_roll); //pokud je hodnota větší než max, vrátí max hodnotu, pokud menší než min, vrátí min
  
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error); //sečtení hodnot do jednoho výstupu
  pid_output_roll = constrain(pid_output_roll, pid_max_roll*-1, pid_max_roll);
  pid_output_roll *= -1;
  
  pid_last_roll_d_error = pid_error_temp;                                                                     //uložení erroru do proměnné D kontroleru
  
  //Pitch kalkulace
  pid_error_temp = (pid_pitch_setpoint - angle_pitch)*pid_stable_gain_pitch - (gyro_pitch/14.286);          //vypočtení erroru
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp * pid_dt;                                              //přičtení erroru vynásobeným konstantou
  pid_i_mem_pitch = constrain(pid_i_mem_pitch, pid_max_pitch*-1, pid_max_pitch);                             //pokud je hodnota větší než max, vrátí max hodnotu, pokud menší než min, vrátí min
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error); //sečtení hodnot do jednoho výstupu
  pid_output_pitch = constrain(pid_output_pitch, pid_max_pitch*-1, pid_max_pitch);
  pid_output_pitch *= -1;
    
  pid_last_pitch_d_error = pid_error_temp;
    
  //Yaw kalkulace (pouze pomocí gyroskopu)
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;                                                         //vypočtení erroru
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;                                                           //přičtení errrou vynásobeným konstantou 
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error); //sečtení hodnot do jednoho výstupu
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
    
  pid_last_yaw_d_error = pid_error_temp;
}




//funkce pro zápis po I2C sběrnici
void writeTo(int DEVICE, byte address, byte val) {
 Wire.beginTransmission(DEVICE);
 Wire.write(address);
 Wire.write(val);
 Wire.endTransmission();
}

//funkce pro čtení z I2C sběrnice
void readFrom(int DEVICE, byte address, int num, byte buff[]) {
Wire.beginTransmission(DEVICE);
Wire.write(address);
Wire.endTransmission();

Wire.beginTransmission(DEVICE);
Wire.requestFrom(DEVICE, num);
int i = 0;
while(Wire.available())
{ 
  buff[i] = Wire.read();
  i++;
}
Wire.endTransmission();
}

//funkce pro čtení dat z UART sběrnice (raspberry pi)
void readData(){
  if(!Serial.available()) return;
  while(Serial.available() > 15){byte tmp = Serial.read();}
  while(Serial.available()){
    byte tmp = Serial.read();
    if(tmp == 0xFE){
        tmp = Serial.read();
        if(tmp == 0x00 && Serial.available() > 6){
          //hodnoty pro motory
          receiver_input_channel_1 = Serial.read();
          if(Serial.read() == 0xFF) receiver_input_channel_1 *= -1;
          receiver_input_channel_2 = Serial.read();
          if(Serial.read() == 0xFF) receiver_input_channel_2 *= -1;
          receiver_input_channel_3 = Serial.read();
          if(Serial.read() == 0xFF) receiver_input_channel_3 *= -1;
          receiver_input_channel_4 = Serial.read();
          Serial.flush();
          return;
        }
        else if(tmp == 0x01 && Serial.available() > 3){
          //hodnoty pro PID
          pid_stable_gain_roll = Serial.read()/10;
          pid_p_gain_roll = (float)Serial.read()/100;
          pid_i_gain_roll = (float)Serial.read()/100;
          pid_d_gain_roll = (float)Serial.read()/100;
          pid_stable_gain_pitch = pid_stable_gain_roll;
          pid_p_gain_pitch = pid_p_gain_roll;
          pid_i_gain_pitch = pid_i_gain_roll;
          pid_d_gain_pitch = pid_d_gain_roll;
          return;
          }
          Serial.flush();
      }
    }
  }

//funkce pro nastavení registru Accelerometru
void initAcc() {
writeTo(ACC, 0x2D, 1<<3);
writeTo(ACC, 0x31, 0x0B);
writeTo(ACC, 0x2C, 0x09);
}

//čtení dat z accelerometru
void getAccelerometerData(double * result) {
int regAddress = 0x32;
byte buff[A_TO_READ];
readFrom(ACC, regAddress, A_TO_READ, buff);
double ax = (((int)buff[1]) << 8) | buff[0];   
double ay = (((int)buff[3])<< 8) | buff[2];
double az = (((int)buff[5]) << 8) | buff[4];
result[0] = atan(ax / (sqrt(square(ay) + square(az))));
result[1] = atan(ay / (sqrt(square(ax) + square(az))));
result[2] = atan(sqrt(square(ax) + square(ay)) / az);

result[0] *= 57.2957795131; result[1] *= 57.2957795131; result[2] *= 57.2957795131;
if(cal_int == 2000){
    result[0] -= base_x_accel;
    result[1] -= base_y_accel;
    result[2] -= base_z_accel;	
  }
}

int setupL3G4200D(){
  writeTo(L3G4200D_Address, 0x20, 0x0F);
  writeTo(L3G4200D_Address, 0x23, 0xA0);
}

//čtení dat z Gyroskopu
void getGyroValues(){
  Wire.beginTransmission(L3G4200D_Address);                    
  Wire.write(168);                                             
  Wire.endTransmission();                                      
  Wire.requestFrom(L3G4200D_Address, 6);                       
  while(Wire.available() < 6);                                 
  lowByte = Wire.read();                                       
  highByte = Wire.read();                                      
  gyro_roll = ((highByte<<8)|lowByte);            
  if(cal_int == 2000)gyro_roll -= gyro_roll_cal;               
  lowByte = Wire.read();                                       
  highByte = Wire.read();                                      
  gyro_pitch = ((highByte<<8)|lowByte);                        
  gyro_pitch *= -1;                                            
  if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal;             
  lowByte = Wire.read();                                      
  highByte = Wire.read();                                      
  gyro_yaw = ((highByte<<8)|lowByte);                           
  gyro_yaw *= -1;                                              
  if(cal_int == 2000)gyro_yaw -= gyro_yaw_cal;                 
  gx = (int)gyro_roll;
  gy = (int)gyro_pitch;
  gz = (int)gyro_yaw;
}

float kalman_get_angle_X(float newAngle, float newRate, double dt) {
	rateX = newRate - biasX;
	angleX += dt * rateX;

	pX[0][0] += dt * (dt*pX[1][1] - pX[0][1] - pX[1][0] + q_angle);
	pX[0][1] -= dt * pX[1][1];
	pX[1][0] -= dt * pX[1][1];
	pX[1][1] += q_bias * dt;

	sX = pX[0][0] + r_measure;

	kX[0] = pX[0][0] / sX;
	kX[1] = pX[1][0] / sX;

	yX = newAngle - angleX;
	angleX += kX[0] * yX;
	biasX += kX[1] * yX;

	pX[0][0] -= kX[0] * pX[0][0];
	pX[0][1] -= kX[0] * pX[0][1];
	pX[1][0] -= kX[1] * pX[0][0];
	pX[1][1] -= kX[1] * pX[0][1];

	return angleX;
}

float kalman_get_angle_Y(float newAngle, float newRate, double dt) {
	rateY = newRate - biasY;
	angleY += dt * rateY;

	pY[0][0] += dt * (dt*pY[1][1] - pY[0][1] - pY[1][0] + q_angle);
	pY[0][1] -= dt * pY[1][1];
	pY[1][0] -= dt * pY[1][1];
	pY[1][1] += q_bias * dt;

	sY = pY[0][0] + r_measure;

	kY[0] = pY[0][0] / sY;
	kY[1] = pY[1][0] / sY;

	yY = newAngle - angleY;
	angleY += kY[0] * yY;
	biasY += kY[1] * yY;

	pY[0][0] -= kY[0] * pY[0][0];
	pY[0][1] -= kY[0] * pY[0][1];
	pY[1][0] -= kY[1] * pY[0][0];
	pY[1][1] -= kY[1] * pY[0][1];

	return angleY;
}