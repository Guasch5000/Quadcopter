//
// Save data motors and angle using DataFlash
//

#include <FastSerial.h>
#include <SPI.h>
#include <I2C.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_IMU.h>
#include <AP_DCM.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_Compass.h>
#include <DataFlash.h>
#include <APM_RC.h>
#include <GCS_MAVLink.h>
#include <AP_GPS.h>
#include <Servo.h>

FastSerialPort(Serial, 0);

Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess  scheduler;

Servo der_servo;
Servo izq_servo;
Servo arr_servo;
Servo aba_servo;

AP_Compass_HIL     compass;
AP_ADC_ADS7844          adc;
AP_InertialSensor_Oilpan ins( &adc );

static GPS         *g_gps;

AP_IMU_INS imu( &ins);
AP_DCM  dcm(&imu, g_gps);

//Definicion de pines y variables para el IMU
# define A_LED_PIN        37
# define C_LED_PIN        35
# define LED_ON           HIGH
# define LED_OFF          LOW
# define MAG_ORIENTATION  AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD

//Deficinion variables DataFlash
#define HEAD_BYTE1 0xA3
#define HEAD_BYTE2 0x95
DataFlash_APM1	DataFlash;

static void flash_leds(bool on)
{
  digitalWrite(A_LED_PIN, on?LED_OFF:LED_ON);
  digitalWrite(C_LED_PIN, on?LED_ON:LED_OFF);
}

//Variables a definir
float angulo_x;
float angulo_y;
uint16_t angulo_z;
unsigned long time;
int dif_time;

float dif_motor_now;
float dif_motor_before;

float dif_motor_x_now;
float dif_motor_x_before;

float dif_angle_x_now;
float dif_angle_x_before;

float dif_angle_y_now;
float dif_angle_y_before;

float dif_angle_z_now;
float dif_angle_z_before;

float error_y_accu;
float error_x_accu;

//Iniciamos variables
Vector3f accel = imu.get_accel();
Vector3f gyro  = imu.get_gyro();


void setup(void)
{
  time=micros();
//Definimos/asociamos cada pin para cada servo
  der_servo.attach(2);
  izq_servo.attach(3);
  arr_servo.attach(7);
  aba_servo.attach(8);

//Iniciamos motores al 0%-->PWM
  der_servo.write(45);
  izq_servo.write(45);
  arr_servo.write(45);
  aba_servo.write(45);

//Definimos puerto de comunicacion
  Serial.begin(115200);
  Serial.println("Starting up...");

  isr_registry.init();
  scheduler.init(&isr_registry);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);

  imu.init(IMU::COLD_START, delay, flash_leds, &scheduler);
  imu.init_accel(delay, flash_leds);

  compass.set_orientation(MAG_ORIENTATION);
  
//Configuracion Dataflash
  DataFlash.Init(); //Iniciamos Dataflash
  DataFlash.StartWrite(1); //Definimos punto de inicio de escritura
  
  //Definir valores (default)
  
  error_y_accu = 0;
  error_x_accu = 0;
}

static void save_data() //Escribimo s en DataFlash
{
  int data_motor, data_angulo_y;  
  dif_time=(int)(micros()-time);
  data_motor = (int)(dif_motor_now*100.00);
  data_angulo_y = (int)(dif_angle_y_now*100.00);
  DataFlash.WriteInt(data_motor);  //Escribimos diferencia motores
  DataFlash.WriteInt(data_angulo_y);  //Escribimos angulo final
  DataFlash.WriteInt(dif_time);  //Escribimos angulo final
  time=micros();
}

static void get_data() //Obtencin de aceleracion, velocidad angular y los angulos de inlcinacion
{
  compass.read();
  compass.calculate(dcm.get_dcm_matrix());
  dcm.update_DCM();

  gyro  = imu.get_gyro();
  accel = imu.get_accel();

  dif_angle_y_now = dcm.roll_sensor/100.00;
  dif_angle_x_now = dcm.pitch_sensor / 100.00;
  
  dif_angle_z_now = (uint16_t)dcm.yaw_sensor / 100;

}

static void control_axis_y()
{
  float def_vel;
  float def_angle_y;
  float vel_der;
  float vel_izq;
  float velmin_der;
  float velmin_izq;  
  def_vel = 60.00;
  def_angle_y = 0.00;
  
  velmin_der=63.00;
  velmin_izq=60.00;
  
  dif_motor_now = 0.48 * dif_angle_y_now + 16.00 * (dif_angle_y_now-dif_angle_y_before) + 0.0004 * error_y_accu;
  
  dif_motor_before = dif_motor_now;
  dif_angle_y_before = dif_angle_y_now;
  error_y_accu = error_y_accu + dif_angle_y_now;

  vel_izq = def_vel - dif_motor_now;
  vel_der = def_vel + dif_motor_now;
  
  if(vel_izq<velmin_izq){
    vel_izq=velmin_izq;
  }
  
  if(vel_der<velmin_der){
    vel_der=velmin_der;
  }
  
//Parada de emergencia
  if(  (abs(dif_angle_x_now) > 50.00) || (abs(dif_angle_y_now) > 50.00)) {
    vel_izq = 45.00;
    vel_der = 45.00;
  }

//Envio de datos al servo
  der_servo.write((int)vel_der);
  izq_servo.write((int)vel_izq);
}

static void control_axis_x()
{
  float def_vel;
  float def_angle_x;
  float vel_arr;
  float vel_aba;
  float velmin_arr;
  float velmin_aba;
  
  
  def_vel = 62.00;
  def_angle_x = 0.00;
  
  velmin_arr = 60.00;
  velmin_aba = 60.00;
  
  dif_motor_x_now = 0.46 * dif_angle_x_now + 16.00 * (dif_angle_x_now-dif_angle_x_before) + 0.0001 * error_x_accu;
  
  dif_motor_x_before = dif_motor_x_now;
  dif_angle_x_before = dif_angle_x_now;
  error_x_accu = error_x_accu + dif_angle_x_now;

  vel_aba = def_vel + dif_motor_x_now;
  vel_arr = def_vel - dif_motor_x_now;
  
  if(vel_aba < velmin_aba){
    vel_aba = velmin_aba;
  }
  
  if(vel_arr < velmin_arr){
    vel_arr = velmin_arr;
  }
  
//Parada de emergencia
  if( (abs(dif_angle_x_now) > 50.00) || (abs(dif_angle_y_now) > 50.00)) {
    vel_aba = 45.00;
    vel_arr = 45.00;
  }
  //if(dif_motor_now > 30.00){
    //dif_motor_now = 30.00;
  //}º

//Envio de datos al servo
  arr_servo.write((int)vel_arr);
  aba_servo.write((int)vel_aba);
}

void loop()
{
  if(millis()<22000){}//22 Segundos de espera para la confirmacin de los variadores
  else{
    
//    if(millis()<60000){
      get_data();//Actualizamos datos
      control_axis_y();
      control_axis_x();
      //save_data();
//    }
//    else{
//      arr_servo.write(45);
//      aba_servo.write(45);
//      der_servo.write(45);
//      izq_servo.write(45);
//    }
  }
}


