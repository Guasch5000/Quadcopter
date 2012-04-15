// Libraries
#include <SPI.h>
#include <DataFlash.h>


DataFlash_APM1	DataFlash;

int data;
long int i;

void setup()
{
	Serial.begin(115200);
	DataFlash.Init();
	
	Serial.println("Clear Quadcopter data");
        DataFlash.StartWrite(1);
}

void loop()
{
        DataFlash.WriteInt(9999);
 
}
