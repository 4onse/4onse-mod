#define WSPEED 3
#define WDIR A0
#define RAIN 2

volatile unsigned long rainlast;
volatile unsigned long windlast;

void rainIRQ()
{
    if ((unsigned long)(millis() - rainlast) > 10) // ignore switch-bounce glitches less than 10mS after initial edge
    {
        Serial.println("Rain tick");
        rainlast = millis(); // set up for next event
    }
}

void windIRQ()
{
    if ((unsigned long)(millis() - windlast) > 10) // ignore switch-bounce glitches less than 10mS after initial edge
    {
        Serial.println("Wind tick");
        windlast = millis(); // set up for next event
    }
}


short get_wind_direction()
{
    unsigned short adc;
    adc = analogRead(WDIR); // get the current reading from the sensor

    // https://www.sparkfun.com/datasheets/Sensors/Weather/Weather%20Sensor%20Assembly.pdf
    // using 10K pull-up resistor
    Serial.print(adc);

    if (adc < 100) return (90);
    if (adc < 200) return (135);
    if (adc < 300) return (180);
    if (adc < 500) return (45);
    if (adc < 660) return (225);
    if (adc < 800) return (0);
    if (adc < 890) return (315);
    if (adc < 950) return (270);
    return -1;
}


void setup() {
  Serial.begin(9600);
  Serial.println("Test rain and wind clicks");

  // set pin mode
  pinMode(RAIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RAIN), rainIRQ, FALLING); // FALLING
  pinMode(WSPEED, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WSPEED), windIRQ, FALLING);

}

void loop() {
  // put your main code here, to run repeatedly:
  float winddir = get_wind_direction();
  Serial.print("WIND DIR: ");
  Serial.println(winddir);
  delay(5000);

}
