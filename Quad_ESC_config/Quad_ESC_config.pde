
#include <Servo.h> 
 
Servo der_servo;
Servo izq_servo;
Servo arr_servo;
Servo aba_servo;
int x;
 
void setup() 
{ 
        der_servo.attach(2);
        izq_servo.attach(3);
        arr_servo.attach(7);
        aba_servo.attach(8);
        
        //Empezamos la calibracion de ESC//
        
        //Indicamos que estamos en modo configuracion//
        
        x = 135;
        servo();
        delay(5000);
              
        x = 45;
        servo();
        delay(5000);
        
        //Brake setting
        x = 45;
        servo();
        delay(5000);
        
        //Motor Poles
        x = 145;
        servo();
        delay(5000);
        
        //Batery Proteccion
        x = 45;
        servo();
        delay(5000);
        
        //Plane Mode
        x = 90;
        servo();
        delay(5000);
        
        //Throttle Response
        x = 45;
        servo();
        delay(5000);
} 
 
static void servo()
{
        der_servo.write(x);
        izq_servo.write(x);
        arr_servo.write(x);
        aba_servo.write(x);
}
void loop() 
{ 
  x = 45;
} 
