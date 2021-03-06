// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// Simple test for the AP_DCM library
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

FastSerialPort(Serial, 0);

Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess  scheduler;

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

void setup(void)
{
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

void loop(void)
{
  compass.read();
  compass.calculate(dcm.get_dcm_matrix());
  dcm.update_DCM();


  Vector3f accel = imu.get_accel();
  Vector3f gyro  = imu.get_gyro();
  
  gyro  = imu.get_gyro();
  accel = imu.get_accel();

  Serial.printf_P(PSTR("r:%4d  p:%4d  y:%3d \n"),
      (int)dcm.roll_sensor / 100,
      (int)dcm.pitch_sensor / 100,
      (uint16_t)dcm.yaw_sensor / 100);
 }

