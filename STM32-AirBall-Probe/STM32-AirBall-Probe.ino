/** AirBall Probe Firmware
    Created by Kenneth C. Budd - phreakmonkey@gmail.com
**/

/** Build-time Config Values 
    LED = pin for LED status indicator.
    SENSOR1, SENSOR2, SENSOR3 = Analog input pin assignments
    TIMERA = how often (in ms) to run timerA routines
    TIMERB = how often (in ms) to run timerB routines
**/

#define LED PC13
#define SENSOR1 PB1
#define SENSOR2 PB0
#define SENSOR3 PA7

#define TIMERA 50
#define TIMERB 200

/** End Config Section **/

uint16_t sensor1avg;
uint16_t sensor2avg;
uint16_t sensor3avg;

uint32_t timerA = 0;
uint32_t timerB = 0;
uint8_t smoothness = 4;

void setup() {
    // Set up the built-in LED pin as output:
    pinMode(LED, OUTPUT);
    pinMode(SENSOR1, INPUT_ANALOG);
    pinMode(SENSOR2, INPUT_ANALOG);
    pinMode(SENSOR3, INPUT_ANALOG);
    // Initial values:
    sensor1avg = AnalogSensor(SENSOR1);
    sensor2avg = AnalogSensor(SENSOR2);
    sensor3avg = AnalogSensor(SENSOR3);

    Serial1.begin(115200);  // STM32F103C8 pins PA2/A2(tx) PA3/A3(rx)
}

void loop() {

    if (millis() - timerA >= TIMERA) {
        timerA = millis();
        sensor1avg = (sensor1avg * smoothness + AnalogSensor(SENSOR1)) / (smoothness + 1);
        sensor2avg = (sensor2avg * smoothness + AnalogSensor(SENSOR2)) / (smoothness + 1);      
        sensor3avg = (sensor3avg * smoothness + AnalogSensor(SENSOR3)) / (smoothness + 1);      
    }

    if (millis() - timerB >= TIMERB) {
        timerB = millis();
        digitalWrite(LED,!digitalRead(LED)); // Toggle status LED
        OutputSentence(sensor1avg, sensor2avg, sensor3avg);
    }
}

uint16_t AnalogSensor(int iopin)
{
  uint16_t reading = analogRead(iopin);
  reading = reading * 10000 / 4095;
  // reading = % of vcc * 100
  // TODO(kcbudd): Convert to engineering units for pressure
  return reading;
}

void OutputSentence(uint16_t s1, uint16_t s2, uint16_t s3)
{
  Serial1.print("@");
  Serial1.print(s1);
  Serial1.print(",");
  Serial1.println(s2);
  Serial1.print(",");
  Serial1.println(s3);
}

