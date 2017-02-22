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

// Loop variables
int percent = 10, increase = 2;
unsigned long lastTime = 0UL;

// Timer variables
uint16_t period = 0;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //Time base structure for timer 3
NVIC_InitTypeDef        NVIC_InitStructure;    //Nested Vector Interrupt Controller Initialisation Structure
TIM_OCInitTypeDef       TIM_OCInitStructure;   //Output compare init structure for PWM

bool gain;
unsigned int ms;

StatsD statsd_client(stats_ip, 8125);


void setup() {

    //Setup statsd


    // Start serial at 9600 baud
    Serial.begin(9600);

    statsd_client.begin();

    // Basic output setup
    pinMode(FAN1, OUTPUT);
    analogWrite(FAN1, 0);

    Serial.println("Started up!");
    delay(2000); // wait for human on the serial monitor

    /* Compute the period value to generate a clock freq of 30KHz, there are other schools that suggest (SystemCoreClock/2)/30000-1 is the way to go since the STM32F2 has a 60Mhz timer clock */
    // With some experimentation, a period of 4000 worked well with my fan
    period = (uint16_t)(((SystemCoreClock) / PWM_FREQ)-1);
    /* Compute the pulse value to generate a PWM signal with variable duty cycle */
    int pulse = period; // Just set it to the period to start off, which should be 0 duty cycle effectively

    //period = (uint16_t)(SystemCoreClock / 2000)-1;

    /* GPIOB clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* Initialise GPIO for PWM function */
    GPIO_InitTypeDef        GPIO_InitStructure;    //GPIO Initialisation Structure
    /* Initialize D2/PB5, Alternative Function, 100Mhz, Output, Push-pull*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOB, &GPIO_InitStructure); // Note the GPIOB.
    // Consult https://docs.particle.io/datasheets/photon-datasheet/#pin-out-diagrams for pinout, pay attention to the STM32 Pin column, ie PB5
    // the 'B' in PB5 means you should use the 'B' GPIO, GPIOB.  PA4 would use GPIOA.

    // Map the pin to the timer
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
    /* TIMER 3 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* Timer Base configuration */
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    //TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3; // symmetrical PWM
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = period;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); // TIM3 = Timer 3

    //Initialise Output Compare Structure

    //pulse = (uint16_t) ((period-1)/(100.0/dutyCycle));

    /* Timer 3 Channel 2 PWM output configuration */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; // can't find a decent reference to say whats the diff between PWM1 and PWM2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = pulse;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    // Channel 2 init timer 3
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    //TIM_ARRPreloadConfig(TIM3, ENABLE); // don't seem to need this

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

void loop() {

    // What is the time mr wolf?
    unsigned long now = millis();

    // Every 5 seconds sample the pit temp
    if (now-lastTime>=5000UL) {
        // Record time for loop
        lastTime = now;

        // Increase the duty percent
        percent += increase;
        // Reset if over 100
        if (percent > 100){
            percent = 0;
        }

        updateOC(FAN1,percent);

        //analogWrite2(D0, percent);

        Serial.print("fanSpeed is: ");
        Serial.print(percent);
        statsd_client.gauge("solarheat.fanspeed", percent);

    }
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

void updateOC(int pin, int percent)
{
    Serial.print("IN update for pin ");
    Serial.println(pin);

    int pulse = 0;

    if (percent){
        // Lower duty cycles need a kick start to spin the motor
        if (30 > percent){

            // Set the spin up pulse to be 75%
            pulse = (uint16_t) (period - (period+1) * 75 / 100 - 1 );

            Serial.print("kick start pulse ");
            Serial.println(pulse);
            // Set the CCR to new pulse value
            updateCCR(pin, pulse);
            // Delay time for the spin up
            delay (300);
        }

        // Interestingly, with the current clock settings 0 = full duty, pulse = period is 0 duty cycle
        // Its like its counting down instead of up
        // Set the pulse based on percent argument
        pulse = (uint16_t) (period - (period+1) * percent / 100 - 1 );
    }
    else {
        // pulse = period is 0 duty cycle, strange but true
        pulse = period;
    }

    Serial.print("pulse is: ");
    Serial.println(pulse);

    // Set the CCR to new pulse value
    updateCCR(pin, pulse);
}

void updateCCR(int pin, int dutyCycle){
    STM32_Pin_Info* PIN_MAP = HAL_Pin_Map();

    TIM_OCInitStructure.TIM_Pulse = dutyCycle;

    Serial.println("modifying TIM2");
    PIN_MAP[pin].timer_peripheral-> CCR2 = TIM_OCInitStructure.TIM_Pulse;


}
