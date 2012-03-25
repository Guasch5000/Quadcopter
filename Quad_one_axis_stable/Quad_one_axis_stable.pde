//
// Stable one axis... (not yet)
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

# define A_LED_PIN        37
# define C_LED_PIN        35
# define LED_ON           HIGH
# define LED_OFF          LOW
# define MAG_ORIENTATION  AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD

static void flash_leds(bool on)
{
  digitalWrite(A_LED_PIN, on?LED_OFF:LED_ON);
  digitalWrite(C_LED_PIN, on?LED_ON:LED_OFF);
}

//Variables a definir
int angulo_x;
int angulo_y;
uint16_t angulo_z;

Vector3f accel = imu.get_accel();
Vector3f gyro  = imu.get_gyro();


void setup(void)
{


  der_servo.attach(2);
  izq_servo.attach(3);
  arr_servo.attach(7);
  aba_servo.attach(8);

  der_servo.write(45);
  izq_servo.write(45);
  arr_servo.write(45);
  aba_servo.write(45);

  Serial.begin(115200);
  Serial.println("Starting up...");

  isr_registry.init();
  scheduler.init(&isr_registry);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);

  imu.init(IMU::COLD_START, delay, flash_leds, &scheduler);
  imu.init_accel(delay, flash_leds);

  compass.set_orientation(MAG_ORIENTATION);


}

static void try_servos()
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

static void get_data()
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

static void get_stable()
{
  int vel_hold = 65;
  int vel_der;
  int vel_izq;

  int num_Kp = 20;
  int den_Kp = 100;

  int num_Kd = 30;
  int den_Kd = 100;

  int vel_proportional = (num_Kp * angulo_y) / den_Kp;
  int vel_derivative = (num_Kd * (int)gyro.y) / den_Kd;

  vel_der=vel_hold + vel_proportional + vel_derivative;
  vel_izq=vel_hold - vel_proportional - vel_derivative;

  //  if ((angulo_y > 1) || (angulo_y < -1)) {
  //    vel_der=vel_hold + vel_proportional + vel_derivative;
  //    vel_izq=vel_hold - vel_proportional - vel_derivative;
  //    
  //  }else {
  //    vel_der = vel_hold;
  //    vel_izq = vel_hold;
  //  }

  if(vel_der>100) vel_der=100; //maximo 135 (emisora)
  if(vel_der<45) vel_der=45;
  if(vel_izq>100) vel_izq=100;
  if(vel_izq<45) vel_izq=45;

  if( abs(angulo_y) > 60) {
    vel_izq=45;
    vel_der=45;
  }

  der_servo.write(vel_der);
  izq_servo.write(vel_izq);
}

void loop()
{
  if(millis()<22000){
  }
  else{
    get_data();
    get_stable();
  }
  //try_servos();


}


