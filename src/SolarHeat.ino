// This #include statement was automatically added by the Particle IDE.
#include "statsd.h"

// This #include statement was automatically added by the Particle IDE.
#include "TSL2561.h"
#include "TMP102.h"

#define FAN1 D2 // D2 is TIM3_CH2 GPIOB

IPAddress stats_ip(54, 85, 171, 11);

TMP102 sensor0(0x48);
TMP102 sensor1(0x49);

SFE_TSL2561 light;

// PWM
//SystemCoreClock = 120MHz, period = 4000 @ 30Khz
#define PWM_FREQ 30000 // in Hertz (SET YOUR FREQUENCY)

#define MIN_FAN 35

double topTemp, bottomTemp, lux;

int fanSpeed;

bool gain;
unsigned int ms;

StatsD statsd_client(stats_ip, 8125);


void setup() {

    //Register variables
    Particle.variable("TopTemp", topTemp);
    Particle.variable("BottomTemp", bottomTemp);
    Particle.variable("Lux", lux);
    Particle.variable("FanSpeed", fanSpeed);

    // Start serial at 9600 baud
    Serial.begin(9600);


    statsd_client.begin();

    // Basic output setup
    pinMode(FAN1, OUTPUT);
    analogWrite(FAN1, 0);

    Serial.println("Started up!");
    delay(2000); // wait for human on the serial monitor


    sensor0.begin();  // Join I2C bus
    sensor1.begin();
    light.begin();

    // set the Conversion Rate (how quickly the sensor gets a new reading)
    //0-3: 0:0.25Hz, 1:1Hz, 2:4Hz, 3:8Hz
    sensor0.setConversionRate(2);
    sensor1.setConversionRate(2);
    unsigned char ID;

    if (light.getID(ID))
    {
        Serial.print("Got factory ID: 0X");
        Serial.print(ID,HEX);
        Serial.println(", should be 0X5X");
    }
    // Most library commands will return true if communications was successful,
    // and false if there was a problem. You can ignore this returned value,
    // or check whether a command worked correctly and retrieve an error code:
    else
    {
        byte error = light.getError();
        //printError(error);
    }

    // The light sensor has a default integration time of 402ms,
    // and a default gain of low (1X).

    // If you would like to change either of these, you can
    // do so using the setTiming() command.

    // If gain = false (0), device is set to low gain (1X)
    // If gain = high (1), device is set to high gain (16X)

    gain = 0;

    // If time = 0, integration will be 13.7ms
    // If time = 1, integration will be 101ms
    // If time = 2, integration will be 402ms
    // If time = 3, use manual start / stop to perform your own integration

    unsigned char time = 2;

    // setTiming() will set the third parameter (ms) to the
    // requested integration time in ms (this will be useful later):

    Serial.println("Set timing...");
    light.setTiming(gain,time,ms);

    // To start taking measurements, power up the sensor:

    Serial.println("Powerup...");
    light.setPowerUp();

    setFanSpeed(75);



    // The sensor will now gather light during the integration time.
    // After the specified time, you can retrieve the result from the sensor.
    // Once a measurement occurs, another integration period will start.

    Serial.println("Finished setup ");

}

float readTemp(TMP102 &sensor){
    float temp;
    sensor.wakeup();
    temp = sensor.readTempF();
    sensor.sleep();
    return temp;
}

bool readLight(double &lux, SFE_TSL2561 &light_sensor){
    unsigned int data0, data1;

    if (light.getData(data0,data1))
    {

        // getData() returned true, communication was successful

        //Serial.print("data0: ");
        //Serial.print(data0);
        //Serial.print(" data1: ");
        //Serial.print(data1);

        // To calculate lux, pass all your settings and readings
        // to the getLux() function.

        // The getLux() function will return 1 if the calculation
        // was successful, or 0 if one or both of the sensors was
        // saturated (too much light). If this happens, you can
        // reduce the integration time and/or gain.
        // For more information see the hookup guide at: https://learn.sparkfun.com/tutorials/getting-started-with-the-tsl2561-luminosity-sensor

        boolean good;  // True if neither sensor is saturated

        // Perform lux calculation:

        int gain = 1;
        int ms = 10;

        good = light.getLux(gain,ms,data0,data1,lux);

        // Print out the results:

        //Serial.print(" lux: ");
        //Serial.print(lux);
        //if (good) Serial.println(" (good)"); else Serial.println(" (BAD)");
        return good;
    }
    else
    {
        // getData() returned false because of an I2C error, inform the user.

        byte error = light.getError();
        Serial.println(error);
    }
    return false;
}
void setFanSpeed(int percent){
  SINGLE_THREADED_BLOCK() {
    fanSpeed = percent;
    float pct = (float)percent/100.0;
    int duty = pct * 255;
    analogWrite(FAN1, duty, PWM_FREQ);
  }
}
void loop() {
  statsd_client.gauge("solarheat.fanspeed", fanSpeed);
  float temperature1;
  float temperature0;
  double lux;


  temperature0 = readTemp(sensor0);
  temperature1 = readTemp(sensor1);

    // Print temperature and alarm state
  Serial.print("Temperature: ");
  Serial.print(temperature0);
  Serial.print(", ");
  Serial.println(temperature1);

  statsd_client.gauge("solarheat.baseTemp", temperature0);
  statsd_client.gauge("solarheat.topTemp", temperature1);




    readLight(lux, light);

    statsd_client.gauge("solarheat.lux", lux);


    Serial.print("Light Measurement: ");
    Serial.println(lux);
    // May as well slow down the loop
    delay(1000);
}
