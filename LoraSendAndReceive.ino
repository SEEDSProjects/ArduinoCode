
#include <MKRWAN.h>

LoRaModem modem;

#include "arduino_secrets.h"

#include "DHT.h"
#define DHTPIN 7
#define DHTTYPE DHT11   // DHT 11

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

DHT dht(DHTPIN, DHTTYPE);

int moisturePin = A0;    // select the input pin for the potentiometer
int moistureValue = 0;  // variable to store the value coming from the sensor


String appEui = SECRET_APP_EUI;
String appKey = SECRET_APP_KEY;
_lora_class z = CLASS_A;

bool relayStatus = false;

char incomingByte;
int val;

int DisplayStatus = 3;
int WriteStatus = 2;
int firstBit = 1;
int secondBit = 0;

int heaterOn = 5;
int displayOn = 4;
int power = 14;

int heat = 0;
int count = 0;
int triggerCounter = 0;
int triggerDelay = 1 * 1000; //1 Second

int activity = 0;
int activityCounter = 5;
double tempacclerationX = 10.91;
double tempacclerationY = 0.40;
double tempacclerationZ = 1.07;
int msgctr = 0;

int uplinkCounter = 0;
int flashCounter = 1;
int cycleCounter = 0;


String msg = "hi";

void Accelerometer ()
{


  /* Get a new sensor event */
  sensors_event_t event;
  accel.getEvent(&event);



  double acclerationX = -1 * (event.acceleration.x - 0.08);
  double acclerationY = -1 * (event.acceleration.y - 0.2);
  double acclerationZ = -1 * (event.acceleration.z - 2.8);



  if ( abs(tempacclerationX - acclerationX) > 2 || abs(tempacclerationY - acclerationY) > 2 || abs(tempacclerationZ - acclerationZ) > 2 )
  {
    tempacclerationX = acclerationX;
    tempacclerationY = acclerationY;
    tempacclerationZ = acclerationZ;

    activity = 1;

  }

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\033[20;0H\n");
  Serial.print("X: "); Serial.print(tempacclerationX); Serial.print("  ");
  Serial.print("Y: "); Serial.print(tempacclerationY); Serial.print("  ");
  Serial.print("Z: "); Serial.print(tempacclerationZ); Serial.print("  "); Serial.print("m/s^2 ");
  Serial.print("\tActivity = "); Serial.print(activity);
  Serial.print("\tMsg Ctr = "); Serial.print(msgctr);

  //If someone is tampering or if vehicle hit
  if (activity == 1)
  {
    msg = "sos";
    LoRaMessage();
    msgctr++;
    delay (20 * triggerDelay);
    activity = 0;
    msg = "hi";
  }
}


void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayDataRate(void)
{
  Serial.print  ("Data Rate:    ");

  switch (accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      Serial.print  ("3200 ");
      break;
    case ADXL345_DATARATE_1600_HZ:
      Serial.print  ("1600 ");
      break;
    case ADXL345_DATARATE_800_HZ:
      Serial.print  ("800 ");
      break;
    case ADXL345_DATARATE_400_HZ:
      Serial.print  ("400 ");
      break;
    case ADXL345_DATARATE_200_HZ:
      Serial.print  ("200 ");
      break;
    case ADXL345_DATARATE_100_HZ:
      Serial.print  ("100 ");
      break;
    case ADXL345_DATARATE_50_HZ:
      Serial.print  ("50 ");
      break;
    case ADXL345_DATARATE_25_HZ:
      Serial.print  ("25 ");
      break;
    case ADXL345_DATARATE_12_5_HZ:
      Serial.print  ("12.5 ");
      break;
    case ADXL345_DATARATE_6_25HZ:
      Serial.print  ("6.25 ");
      break;
    case ADXL345_DATARATE_3_13_HZ:
      Serial.print  ("3.13 ");
      break;
    case ADXL345_DATARATE_1_56_HZ:
      Serial.print  ("1.56 ");
      break;
    case ADXL345_DATARATE_0_78_HZ:
      Serial.print  ("0.78 ");
      break;
    case ADXL345_DATARATE_0_39_HZ:
      Serial.print  ("0.39 ");
      break;
    case ADXL345_DATARATE_0_20_HZ:
      Serial.print  ("0.20 ");
      break;
    case ADXL345_DATARATE_0_10_HZ:
      Serial.print  ("0.10 ");
      break;
    default:
      Serial.print  ("???? ");
      break;
  }
  Serial.println(" Hz");
}

void displayRange(void)
{
  Serial.print  ("Range:         +/- ");

  switch (accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      Serial.print  ("16 ");
      break;
    case ADXL345_RANGE_8_G:
      Serial.print  ("8 ");
      break;
    case ADXL345_RANGE_4_G:
      Serial.print  ("4 ");
      break;
    case ADXL345_RANGE_2_G:
      Serial.print  ("2 ");
      break;
    default:
      Serial.print  ("?? ");
      break;
  }
  Serial.println(" g");
}

//Temperature Sensor
void TempCheck ()
{
  // read the value from the sensor:
  moistureValue = analogRead(moisturePin);

  Serial.print("\033[25;0H\n");

  Serial.print("moisture value  = " );
  Serial.print(moistureValue);
  Serial.print("    ");

  float h = dht.readHumidity();

  // Read temperature as Celsius
  float t = dht.readTemperature();

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t))
  {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  //Display Values
  Serial.print("Heater: ");
  Serial.print(heat);
  Serial.print("   Count: ");
  Serial.print(count);
  Serial.print("  \t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.write(13);

  //Check for cold weather
  if (t < 3)
  {
    //HeaterOn();
    digitalWrite(heaterOn, HIGH);
    heat = 1;
    count = 480000 / triggerDelay;
    flashCounter = 2;
  }

  if (count < 1)
  {
    // HeaterOff();
    digitalWrite(heaterOn, LOW);
    heat = 0;
    count = 0;
    flashCounter = 1;
  }

  count--;
}

//HeaterOn
void HeaterOn()
{
  digitalWrite(heaterOn, HIGH);
  return;
}

//HeaterOff
void HeaterOff()
{
  digitalWrite(heaterOn, LOW);
  return;
}


//Update The Display with Preloaded Image
void updateDisplay(int image)
{

  digitalWrite(displayOn, HIGH);

  while (Serial1.read() != 'A');
  delay(1000);
  Serial1.write('I');
  Serial1.write(image);
  delay(1000);
  while (Serial1.read() != 'A');
  digitalWrite(displayOn, LOW);
}


//Update The Display with Custom License Plate
void displayPlate(int len, char* plate)
{
  digitalWrite(displayOn, HIGH);

  delay(1000);
  while (Serial1.read() != 'A');
  Serial1.write('P');
  Serial1.write(len);
  delay(1000);
  Serial1.write(plate);
  while (Serial1.read() != 'A');

  digitalWrite(displayOn, LOW);
}

//Ping up to the network (Uplink)
void LoRaMessage()
{

  int err;
  modem.beginPacket();
  modem.print(msg);
  err = modem.endPacket(true);
  delay(1000);
  if (!modem.available())
  {
    Serial.println("No downlink message received at this time.");
  }
  else
  {

    char rcv[64];
    int i = 0;

    while (modem.available())
    {
      rcv[i++] = (char)modem.read();
    }

    int messageLength = strlen(rcv);

    //If message is for a custom plate
    if (messageLength > 3)
    {
      for (unsigned int j = 0; j < flashCounter; j++)
      {
        displayPlate(messageLength, rcv);
        //Serial.print("\t Message = ");  Serial.print(rcv);
        delay(5000);
      }
    }
    
    //Else for a parking restriction
    else
    {
      for (unsigned int j = 0; j < flashCounter; j++)
      {
        updateDisplay(val);
        delay (5000);
      }
    }
  }
}


void setup()
{

  pinMode(power, OUTPUT);
  pinMode(displayOn, OUTPUT);
  pinMode(heaterOn, OUTPUT);

  pinMode(DisplayStatus, INPUT);
  pinMode(WriteStatus, OUTPUT);
  pinMode(firstBit, OUTPUT);
  pinMode(secondBit, OUTPUT);

  digitalWrite(WriteStatus, LOW);
  digitalWrite(firstBit, LOW);
  digitalWrite(secondBit, LOW);

  digitalWrite(power, HIGH);
  digitalWrite(displayOn, LOW);
  digitalWrite(heaterOn, LOW);

  Serial.begin(115200);
  Serial1.begin(9600);
  dht.begin();

  if (!modem.begin(US915))
  {
    Serial.println("Failed to start module");
    while (1) {}
  };

  delay (5000);
  Serial.print("Your module version is: ");
  Serial.println(modem.version());
  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());

  bool cl = modem.configureClass(z);
  if (cl == 1)
  {
    Serial.print("Your device is using class: ");
    Serial.println("CLASS A");
  }

  Serial.println("Your device is trying to connect to network - -  ");
  //display conecting message
  updateDisplay(3);


  //attempt to connect
  int connected = modem.joinOTAA(appEui, appKey);
  int attempts = 0;

  while (!connected)
  {
    if (attempts == 4)
    {
      Serial.println("Cannot connect, power cycle device");
      updateDisplay(3);
      while (1) {};
    }

    else
    {
      Serial.println("Attempt Failed Retry in 30 seconds");
      delay(30000);
      connected = modem.joinOTAA(appEui, appKey);
      ++attempts;
    }
  }

  Serial.println("Connected");
  LoRaMessage();
  updateDisplay(2);

  // Set poll interval to 60 secs.
  modem.minPollInterval(60);
  // NOTE: independently by this setting the modem will
  // not allow to send more than one message every 2 minutes,
  // this is enforced by firmware and can not be changed.

  Serial.println("Test"); Serial.println("");

  /* Initialise the sensor */
  if (!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while (1);
  }

  /* Set the range to whatever is appropriate for your project */
  //accel.setRange(ADXL345_RANGE_16_G);
  // accel.setRange(ADXL345_RANGE_8_G);
  accel.setRange(ADXL345_RANGE_4_G);

}

void loop()
{

  //Code for ping time every 2 minutes
  if (triggerCounter < 1 && activity == 0)
  {

    LoRaMessage();

    if (cycleCounter == 1)
    {
      triggerCounter = 1;
      cycleCounter = 0;
    }

    else
    {
      triggerCounter = 120000 / triggerDelay;
    }

    uplinkCounter++;
  }

  Serial.print("\033[15;0H\n");
  Serial.print("Uplink Counter = "); Serial.print(uplinkCounter);
  Serial.print("\tTimer = "); Serial.print(triggerCounter);

  Accelerometer(); //Accelerometer
  TempCheck(); //Temperature

  delay(triggerDelay);

  triggerCounter --;

}
