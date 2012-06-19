// Libraries
#include <SPI.h>
#include <DataFlash.h>


DataFlash_APM1	DataFlash;

int data;
float data2;

void setup()
{
	Serial.begin(115200);
	DataFlash.Init();

	Serial.println("Reading Quadcopter data");
        Serial.println();
        Serial.println("Radio Y, Angle Y, Radio X, Angle X, Time Difference");
        DataFlash.StartRead(1);
        
        data = DataFlash.ReadInt(); //Radio Y
        while(data != 9999)
        {
	  data2 = (float)data / 100;
          Serial.print(data2);
          Serial.print(",");

    	  data = DataFlash.ReadInt(); //Angle Y
    	  data2 = (float)data / 100;
          Serial.print(data2);
          Serial.print(",");
          
    	  data = DataFlash.ReadInt(); //Radio X
    	  data2 = (float)data / 100;
          Serial.print(data2);
    	  Serial.print(","
    );
    
          data = DataFlash.ReadInt(); //Angle X
          data2 = (float)data / 100;
          Serial.print(data2);
    	  Serial.println();
    
          data = DataFlash.ReadInt(); //Time difference
          Serial.print(data);
    	  Serial.println();
        }
        
        
        
}

void loop()
{
}
