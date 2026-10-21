#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    Serial.println("Starting VL53L0X...");

    while(!lox.begin())
    {
        Serial.println("VL53L0X not detected!");
        for (int i = 0;i < 10000;i ++);
       
    }

    Serial.println("VL53L0X detected!");
}

void loop()
{
    VL53L0X_RangingMeasurementData_t measure;

    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4)
    {
        Serial.print("Distance: ");
        Serial.print(measure.RangeMilliMeter);
        Serial.println(" mm");
    }
    else
    {
        Serial.println("Out of range");
    }

    delay(100);
}