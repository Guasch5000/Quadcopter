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
int angulo_x;
int angulo_y;
int diff_motor;
unsigned long time;
int dif_time;
uint16_t angulo_z;

//Iniciamos variables
Vector3f accel = imu.get_accel();
Vector3f gyro  = imu.get_gyro();


void setup(void)
{
  time=micros();
//Definimos cada pin para cada servo
  der_servo.attach(2);
  izq_servo.attach(3);
  arr_servo.attach(7);
  aba_servo.attach(8);

//Iniciamos motores al 0%
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

}

static void save_data() //Escribimo s en DataFlash
{
  dif_time=(int)(micros()-time);
//DataFlash.WriteInt(diff_motor);  //Escribimos diferencia motores
  DataFlash.WriteInt(angulo_y);  //Escribimos angulo final
  DataFlash.WriteInt(dif_time);  //Escribimos angulo final
  time=micros();
}

static void try_servos()  //Funcion para localizar cada servo
{
  delay(500);
  der_servo.write(135);
  izq_servo.write(45);
  arr_servo.write(45);
  aba_servo.write(45);

  delay(500);
  der_servo.write(135);
  izq_servo.write(135);
  arr_servo.write(45);
  aba_servo.write(45);

  delay(500);
  der_servo.write(135);
  izq_servo.write(135);
  arr_servo.write(135);
  aba_servo.write(45);

  delay(500);
  der_servo.write(135);
  izq_servo.write(135);
  arr_servo.write(135);
  aba_servo.write(135);

  delay(500);
  der_servo.write(45);
  izq_servo.write(45);
  arr_servo.write(45);
  aba_servo.write(45);
}

static void get_data() //Obtencin de aceleracion, velocidad angular y los angulos de inlcinacion
{
  compass.read();
  compass.calculate(dcm.get_dcm_matrix());
  dcm.update_DCM();

  gyro  = imu.get_gyro();
  accel = imu.get_accel();

  angulo_y = (int)dcm.roll_sensor / 100,
  angulo_x = (int)dcm.pitch_sensor / 100,
  angulo_z = (uint16_t)dcm.yaw_sensor / 100;

  //Serial.printf_P(PSTR("r:%4d  p:%4d  y:%3d \n"), angulo_x, angulo_y, angulo_z);
}

static void get_stable() //Funcion para mantener estable el quadcopter
{
  int vel_hold = 65; //Velocidad normal de los motores
  int vel_der;
  int vel_izq;
  
//Deficion de constantes P y D
  int num_Kp = 20;
  int den_Kp = 100;
  int num_Kd = 30;
  int den_Kd = 100;

//Calculo salida P y D del controlador
  int vel_proportional = (num_Kp * angulo_y) / den_Kp;
  int vel_derivative = (num_Kd * (int)gyro.y) / den_Kd;

//Asignacion velocidad a cada motor segun el control PD
  vel_der=vel_hold + vel_proportional + vel_derivative;
  vel_izq=vel_hold - vel_proportional - vel_derivative;

//Saturacion de los motores
  if(vel_der>100) vel_der=100; //maximo 135 (emisora)
  if(vel_der<45) vel_der=45;
  if(vel_izq>100) vel_izq=100;
  if(vel_izq<45) vel_izq=45;

//Parada de emergencia
  if( abs(angulo_y) > 60) {
    vel_izq=45;
    vel_der=45;
  }

//Guardamos diferencia
  diff_motor = vel_der - vel_izq;

//Envio de datos al servo
  der_servo.write(vel_der);
  izq_servo.write(vel_izq);
}

void loop()
{
  //if(millis()<22000){}//22 Segundos de espera para la confirmacin de los variadores
  //else{
    get_data();//Actualizamos datos
    //get_stable();//Estabilizamos
    save_data();
  //}
  //try_servos();


}


