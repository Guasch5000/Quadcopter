
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

FastSerialPort(Serial, 0);

Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess  scheduler;
APM_RC_APM1 APM_RC;

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
float def_angle_y;
float def_angle_x;
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

float angle_x_now;
float angle_x_before;

float angle_y_now;
float angle_y_before;

float angle_z_now;
float angle_z_before;

float error_y_accu;
float error_x_accu;

float Kp_x;
float Kd_x;
float Ki_x;

float Kp_y;
float Kd_y;
float Ki_y;

float vel_arr;
float vel_aba;

float vel_der;
float vel_izq;
//Iniciamos variables
Vector3f accel = imu.get_accel();
Vector3f gyro  = imu.get_gyro();


void setup(void)
{  
  time=micros();
  
  isr_registry.init();
  APM_RC.Init(&isr_registry);	 // APM Radio initialization
  
  APM_RC.enable_out(CH_1);
  APM_RC.enable_out(CH_2);
  APM_RC.enable_out(CH_3);
  APM_RC.enable_out(CH_4);
  APM_RC.enable_out(CH_5);
  APM_RC.enable_out(CH_6);
  APM_RC.enable_out(CH_7);
  APM_RC.enable_out(CH_8);

  APM_RC.OutputCh(4,1030);
  APM_RC.OutputCh(5,1030);
  APM_RC.OutputCh(6,1030);
  APM_RC.OutputCh(7,1030);
  
//Definimos puerto de comunicacion
  Serial.begin(115200);
  Serial.println("Starting up...");
  
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
  error_y_accu = 0.0;
  error_x_accu = 0.0;
  
  Kp_x = 0.67;
  Kd_x = 36.50;
  //Kd_x = 33.50;
  Ki_x = 0.00015;
  
  Kp_y = 0.67;
  Kd_y = 48.00;
  //Kd_y = 45.00;
  Ki_y = 0.00015;
  
  
  def_angle_y = 0.00;
  def_angle_x = 0.00;
}

static void save_data() //Escribimo s en DataFlash
{
  int deff_angle_y, data_angulo_y, deff_angle_x, data_angulo_x;  
  
  dif_time=(int)(micros()-time);
  
  def_angle_y = def_angle_y * 100.00;
  deff_angle_y = (int)def_angle_y;
  
  angle_y_now = angle_y_now * 100.00;
  data_angulo_y = (int)angle_y_now;
  
  def_angle_x = def_angle_x * 100.00;
  deff_angle_x = (int)def_angle_x;
  
  angle_x_now = angle_x_now * 100.00;
  data_angulo_x = (int)angle_x_now;
  
  DataFlash.WriteInt(deff_angle_y);  //Escribimos diferencia motores
  DataFlash.WriteInt(data_angulo_y);  //Escribimos angulo final
  
  DataFlash.WriteInt(deff_angle_x);  //Escribimos diferencia motores
  DataFlash.WriteInt(data_angulo_x);  //Escribimos angulo final
  
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

  angle_y_now = dcm.roll_sensor/100.00;
  angle_x_now = dcm.pitch_sensor / 100.00;
  
  angle_z_now = (uint16_t)dcm.yaw_sensor / 100;
}

static void get_radio()
{
  int aux_int;
  float aux_float;
  
  aux_float = (((float)APM_RC.InputCh(6))*10.00)/500.00-30.00;
  aux_int = (int)aux_float;
  def_angle_x = (float)aux_int*2.00;
  
  aux_float = (((float)APM_RC.InputCh(7))*10.00)/500.00-30.00;
  aux_int = (int)aux_float;
  def_angle_y = (float)aux_int*2.00;
}

static void control_axis_y()
{
  float def_vel;
  float velmin_der;
  float velmin_izq;  
  
  def_vel = 60.00+((float)APM_RC.InputCh(5)-1040.0)/150.0;
  velmin_der = 63.00;
  velmin_izq = 60.00;
  
  dif_angle_y_now = angle_y_now - def_angle_y;
  
  dif_motor_now = Kp_y * dif_angle_y_now + Kd_y * (dif_angle_y_now-dif_angle_y_before) + Ki_y * error_y_accu;
  
  dif_angle_y_before = dif_angle_y_now;
  
  if(APM_RC.InputCh(5)>1040){
    error_y_accu = error_y_accu + dif_angle_y_now;
  }else{
    error_y_accu=0.0;
  }

  vel_izq = def_vel - dif_motor_now;
  vel_der = def_vel + dif_motor_now;
  
  if(vel_izq < velmin_izq){
    vel_izq = velmin_izq;
  }
  
  if(vel_der < velmin_der){
    vel_der = velmin_der;
  }
  
//Parada de emergencia
  if(  (abs(angle_x_now) > 50.00) || (abs(angle_y_now) > 50.00)) {
    vel_izq = 45.00;
    vel_der = 45.00;
  }
}

static void control_axis_x()
{
  float def_vel;
  float velmin_arr;
  float velmin_aba;
  
  def_vel = 62.00+((float)APM_RC.InputCh(5)-1040.0)/150.0;
  velmin_arr = 60.00;
  velmin_aba = 60.00;
  
  dif_angle_x_now = angle_x_now - def_angle_x;
  
  dif_motor_x_now = Kp_x * dif_angle_x_now + Kd_x * (dif_angle_x_now-dif_angle_x_before) + Ki_x * error_x_accu;
  
  dif_angle_x_before = dif_angle_x_now;
  if(APM_RC.InputCh(5)>1040){
    error_x_accu = error_x_accu + dif_angle_x_now;
  }else{
    error_x_accu=0.0;
  }
  vel_aba = def_vel + dif_motor_x_now;
  vel_arr = def_vel - dif_motor_x_now;
  
  if(vel_aba < velmin_aba){
    vel_aba = velmin_aba;
  }
  
  if(vel_arr < velmin_arr){
    vel_arr = velmin_arr;
  }
  
//Parada de emergencia
  if( (abs(angle_x_now) > 50.00) || (abs(angle_y_now) > 50.00)) {
    vel_aba = 45.00;
    vel_arr = 45.00;
  }
}

void order2servos()
{
  int output_arr;
  int output_aba;
  int output_der;
  int output_izq;
  
  output_arr = ( ( (int)((vel_arr - 45.0)*10.9 + 1030.0) ) /5 ) *5;
  output_aba = ( ( (int)((vel_aba - 45.0)*10.9 + 1030.0) ) /5 ) *5;
  output_der = ( ( (int)((vel_der - 45.0)*10.9 + 1030.0) ) /5 ) *5;
  output_izq = ( ( (int)((vel_izq - 45.0)*10.9 + 1030.0) ) /5 ) *5;
  if(APM_RC.InputCh(5)>1040){    
    APM_RC.OutputCh(7,output_der); //derecho
    APM_RC.OutputCh(6,output_izq); //izquierdo
    APM_RC.OutputCh(5,output_arr); //arriba
    APM_RC.OutputCh(4,output_aba); //abajo
  }else{
    APM_RC.OutputCh(7,1030); //derecho
    APM_RC.OutputCh(6,1030); //izquierdo
    APM_RC.OutputCh(5,1030); //arriba
    APM_RC.OutputCh(4,1030); //abajo
  }
}



void loop()
{
  if(millis()<22000){}//22 Segundos de espera para la confirmacin de los variadores
  else{
    get_data();//Actualizamos datos
    get_radio();
    control_axis_y();
    control_axis_x();
    order2servos();
    save_data();
  }
}

