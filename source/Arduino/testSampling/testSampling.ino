#include "measure.h"

Measure meas = Measure(6, 5, 0.0, 60.0, 2.0, 3.0);
RunningMedian medianTemp = RunningMedian(6);

uint8_t counterMin = 0;

uint8_t counterBig = 0;

void setup()
{
    Serial.begin(57600);
    while(!Serial){}

    delay(1000);

    Serial.println(F("setup ready..."));
    Serial.println(F("Start loop"));
}

void loop()
{

    meas.addMeasure(10.0);
    counterMin++;

    if(counterMin >= 6)
    {
        counterMin = 0;
        counterBig++;

        meas.calcLastMin();

        if(counterBig >= 5)
        {
            counterBig = 0;
            String result = meas.getAverageQI();

            Serial.println(result);
        }

    }



    delay(10000);


}
