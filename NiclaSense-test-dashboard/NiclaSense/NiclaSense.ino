#include <Arduino_BHY2.h>
#include <ArduinoBLE.h>
#include <Nicla_System.h>

#define NICLA_SENSE_UUID(val) ("6fbe1da7-" val "-44de-92c4-bb6e04fb0212")

const int VERSION = 0x00000000;
const float sampling_freq_motion = 100.0f;

BLEService service(NICLA_SENSE_UUID("0000"));
BLEUnsignedIntCharacteristic versionCharacteristic(NICLA_SENSE_UUID("1001"), BLERead);
BLECharacteristic accelerationCharacteristic(NICLA_SENSE_UUID("3001"), BLENotify, 3 * sizeof(float));   // Array of 3 floats, G
BLECharacteristic gyroscopeCharacteristic(NICLA_SENSE_UUID("3002"), BLENotify, 3 * sizeof(float));      // Array of 3 floats, dps
BLECharacteristic magneticFieldCharacteristic(NICLA_SENSE_UUID("3003"), BLENotify, 3 * sizeof(float));  // Array of 3 floats, uT
BLEFloatCharacteristic pressureCharacteristic(NICLA_SENSE_UUID("4001"), BLERead);                       // Float, kPa
BLEFloatCharacteristic temperatureCharacteristic(NICLA_SENSE_UUID("4002"), BLERead);                    // Float, Celcius
BLEFloatCharacteristic humidityCharacteristic(NICLA_SENSE_UUID("4003"), BLERead);                       // Float, Relative humidity
BLECharacteristic orientationCharacteristic(NICLA_SENSE_UUID("5001"), BLENotify, 4 * sizeof(float));    // Array of 4 signed floats, unit Quaternion
BLECharacteristic rgbLedCharacteristic(NICLA_SENSE_UUID("6001"), BLERead | BLEWrite, 3 * sizeof(byte)); // Array of 3 bytes, RGB

// String to calculate the local and device name
String name;

SensorXYZ accel(SENSOR_ID_ACC);
SensorXYZ gyro(SENSOR_ID_GYRO);
SensorXYZ mag(SENSOR_ID_MAG);
SensorQuaternion quat(SENSOR_ID_GAMERV);
Sensor baro(SENSOR_ID_BARO);
Sensor temp(SENSOR_ID_TEMP);
Sensor hum(SENSOR_ID_HUM);


void setup()
{
  Serial.begin(115200);
  nicla::begin();
  nicla::leds.begin();
  nicla::leds.setColor(red);

  Serial.println("Started");

  if (!BHY2.begin())
  {
    Serial.println("Failed to initialize the BHY260!");

    while (1)
      ;
  }

  if (!accel.begin(sampling_freq_motion))
  {
    Serial.println("Failed to initialize Accelerometer!");

    while (1)
      ;
  }

  if (!gyro.begin(sampling_freq_motion))
  {
    Serial.println("Failed to initialize Gyroscope!");

    while (1)
      ;
  }

  if (!mag.begin(sampling_freq_motion))
  {
    Serial.println("Failed to initialize Magnetometer!");

    while (1)
      ;
  }

  if (!quat.begin(sampling_freq_motion))
  {
    Serial.println("Failed to initialize Quaternion virtual sensor!");

    while (1)
      ;
  }

  if (!baro.begin(1.0f))
  {
    Serial.println("Failed to initialize Barometer!");

    while (1)
      ;
  }

  if (!temp.begin(1.0f))
  {
    Serial.println("Failed to initialize Thermometer!");

    while (1)
      ;
  }

  if (!hum.begin(1.0f))
  {
    Serial.println("Failed to initialize Hygrometer!");

    while (1)
      ;
  }

  if (!BLE.begin())
  {
    Serial.println("Failed to initialized BLE!");

    while (1)
      ;
  }

  String address = BLE.address();

  Serial.print("address = ");
  Serial.println(address);

  address.toUpperCase();

  name = "NiclaSense-";
  name += address[address.length() - 5];
  name += address[address.length() - 4];
  name += address[address.length() - 2];
  name += address[address.length() - 1];

  Serial.print("name = ");
  Serial.println(name);

  BLE.setLocalName(name.c_str());
  BLE.setDeviceName(name.c_str());
  BLE.setAdvertisedService(service);

  service.addCharacteristic(versionCharacteristic);
  service.addCharacteristic(accelerationCharacteristic);
  service.addCharacteristic(gyroscopeCharacteristic);
  service.addCharacteristic(magneticFieldCharacteristic);
  service.addCharacteristic(orientationCharacteristic);

  service.addCharacteristic(pressureCharacteristic);
  service.addCharacteristic(temperatureCharacteristic);
  service.addCharacteristic(humidityCharacteristic);
  service.addCharacteristic(rgbLedCharacteristic);

  versionCharacteristic.setValue(VERSION);
  pressureCharacteristic.setEventHandler(BLERead, onPressureCharacteristicRead);
  temperatureCharacteristic.setEventHandler(BLERead, onTemperatureCharacteristicRead);
  humidityCharacteristic.setEventHandler(BLERead, onHumidityCharacteristicRead);
  rgbLedCharacteristic.setEventHandler(BLEWritten, onRgbLedCharacteristicWrite);

  BLE.addService(service);

  BLE.advertise();
}

void loop()
{
  while (BLE.connected())
  {
    BHY2.update();

    if (accelerationCharacteristic.subscribed())
    {
      float acceleration[3] = {
          accel.x() / 4096.0f,
          accel.y() / 4096.0f,
          accel.z() / 4096.0f,
      };

      accelerationCharacteristic.writeValue(acceleration, sizeof(acceleration));
    }

    if (gyroscopeCharacteristic.subscribed())
    {
      float dps[3] = {gyro.x() * 2000.0f / 32768.0f, gyro.y() * 2000.0f / 32768.0f, gyro.z() * 2000.0f / 32768.0f};

      gyroscopeCharacteristic.writeValue(dps, sizeof(dps));
    }

    if (magneticFieldCharacteristic.subscribed())
    {
      float magneticField[3] = {mag.x() * 2500.0f / 32768.0f, mag.y() * 2500.0f / 32768.0f, mag.z() * 2500.0f / 32768.0f};

      magneticFieldCharacteristic.writeValue(magneticField, sizeof(magneticField));
    }

    if (orientationCharacteristic.subscribed())
    {
      float quaternion[4] = {quat.w(), quat.x(), quat.y(), quat.z()};

      orientationCharacteristic.writeValue(quaternion, sizeof(quaternion));
    }
  }
}

void onPressureCharacteristicRead(BLEDevice central, BLECharacteristic characteristic)
{
  float pressure = baro.value();

  pressureCharacteristic.writeValue(pressure);
}

void onTemperatureCharacteristicRead(BLEDevice central, BLECharacteristic characteristic)
{
  float temperature = temp.value();

  temperatureCharacteristic.writeValue(temperature);
}

void onHumidityCharacteristicRead(BLEDevice central, BLECharacteristic characteristic)
{
  float humidity = hum.value();

  humidityCharacteristic.writeValue(humidity);
}

void onRgbLedCharacteristicWrite(BLEDevice central, BLECharacteristic characteristic)
{
  uint8_t r = rgbLedCharacteristic[0];
  uint8_t g = rgbLedCharacteristic[1];
  uint8_t b = rgbLedCharacteristic[2];

  nicla::leds.setColor(r, g, b);
}