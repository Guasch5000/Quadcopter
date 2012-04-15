// Libraries
#include <SPI.h>
#include <DataFlash.h>


DataFlash_APM1	DataFlash;

int data;

void setup()
{
	Serial.begin(115200);
	DataFlash.Init();
	
	Serial.println("Reading Quadcopter data");
        Serial.println();
        Serial.println("Motor Difference,Angle Y, Time Difference");
        DataFlash.StartRead(1);
        
        data = DataFlash.ReadInt();
        while(data != 9999)
        {
	  Serial.print(data);
	  Serial.print(",");
    	  data = DataFlash.ReadInt();
    	  Serial.print(data);
          Serial.print(",");
    	  data = DataFlash.ReadInt();
    	  Serial.print(data);
    	  Serial.println();
          data = DataFlash.ReadInt();
        }
        
        
        
}

void loop()
{
//	data = DataFlash.ReadInt();
//        while(data !=
//	Serial.print(data);
//	Serial.print(",");
//	data = DataFlash.ReadInt();
//	Serial.print(data);
//	Serial.println();
}
