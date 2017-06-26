#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_DHT.h>
#include <Adafruit_HMC5883.h>

#include "defs.h"

int loopDelay = defaultLoopDelay;
bool bmpAvailable = true;
bool magAvailable = true;
bool dhtAvailable = true;
bool pumping = false;

// Variables
double bilgeLevel;
double bilgeReading;
double windSpeed;
double windDirection;

double dhtAirTemperature;
double bmpAirTemperature;
double barometricPressure;
double humidity;
double dewpoint;

double waterTemperature;
double waterDepth;
double waterSpeed;

double latitude;
double longitude;
double altitude;
double magHeading;
double trueHeading;

double battery1Voltage;
double battery2Voltage;

double uvIndex;
// end variables
Calibration calibration;

#define DHTPIN D4
#define DHTTYPE DHT21

DHT dht(DHTPIN, DHTTYPE);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
//Adafruit_BMP085 bmp;

void initCalibration() {
  Particle.publish("joliebrise/system/calibration/initialize");
  calibration.version = 1;
  calibration.magOffset = 0;
  calibration.windVoltsMin = WINDVOLTAGEMIN;
  calibration.windVoltsMax = WINDVOLTAGEMAX;
  calibration.bilgeMax = BILGEDEPTHCALIBRATEDMAX;
  calibration.bilgeMin = BILGEDEPTHCALIBRATEDMIN;
  calibration.bilgeSeriesResistor = BILGEDEPTHSERIESRESISTOR;
  writeCalibration();
}

void writeCalibration() {
  EEPROM.put(0, calibration);
}

void printCalibration() {
  char publishString[255];
  sprintf(publishString,
      "{v: %u, mag: %f, wvMin: %f, wvMax: %f, bMax: %u, bMin: %u, bRes: %u}",
      calibration.version,
      calibration.magOffset,
      calibration.windVoltsMin,
      calibration.windVoltsMax,
      calibration.bilgeMax,
      calibration.bilgeMin,
      calibration.bilgeSeriesResistor);
  Particle.publish("joliebrise/system/calibration", publishString);
}

void setup() {
  Particle.publish("joliebrise/system/setup/started");
  EEPROM.get(0,calibration);
  printCalibration();

	pinMode(windSpeedPin, INPUT_PULLDOWN);
	pinMode(windDirectionPin, INPUT_PULLDOWN);
	pinMode(waterPin, INPUT);
	pinMode(pumpPin, OUTPUT);
  digitalWrite(pumpPin, LOW);

	Particle.function("pumpBilge",pumpBilge);
	Particle.function("update", update);
  Particle.function("calibrate", calibrate);

	/*if (!bmp.begin()) {
		bmpAvailable = false;
	}*/
  bmpAvailable = false;

	dht.begin();
  if (!mag.begin()) {
		Particle.publish("joliebrise/sensors/hmc5883l/failed");
    magAvailable = false;
	} else {
    Particle.publish("joliebrise/sensors/hmc5883l/initialized");
    magAvailable =  true;
  }

	Particle.variable("bilgeraw", bilgeReading);
	Particle.variable("bilgelevel", bilgeLevel);
	Particle.variable("windSpeed", windSpeed);
	Particle.variable("windDir", windDirection);
	Particle.variable("airTemp", dhtAirTemperature);
	Particle.variable("baroPress", barometricPressure);
	Particle.variable("humidity", humidity);

	Particle.variable("waterTemp", waterTemperature);
	Particle.variable("waterDepth", waterDepth);
	Particle.variable("waterSpeed", waterSpeed);

	Particle.variable("latitude", latitude);
	Particle.variable("longitude", longitude);
	Particle.variable("altitude", altitude);
	Particle.variable("magHeading", magHeading);
	Particle.variable("trueHeading", trueHeading);
  Particle.publish("joliebrise/system/setup");
}

void loop() {
	update("all");
  processData();
	delay(1000);
}

void processData() {
  if (!pumping && bilgeLevel > 5) {
    Particle.publish("joliebrise/controls/pump/start");
		digitalWrite(pumpPin, HIGH);
    pumping = true;
  }
  if (pumping && bilgeLevel < 2) {
    Particle.publish("joliebrise/controls/pump/stop");
    pumping = false;
  }
}
int update(String cmd) {
	if (cmd == "all" || cmd == "bilgeLevel") {
		getBilgeLevel();
	}
	if (cmd == "all" || cmd == "seaTalk") {
		//getSeaTalk();
	}
	if (cmd == "all" || cmd == "mag") {
		getHeading();
	}
	if (cmd == "all" || cmd == "meteo") {
		getMeteo();
	}
	if (cmd == "all" || cmd == "windSpeed" || cmd == "wind") {
		getWindSpeed();
	}
	if (cmd == "all" || cmd == "windDirection" || cmd == "wind") {
		getWindDirection();
	}
	return 1;
}

int calibrate(String cmd) {
  if (cmd == "init") {
    initCalibration();
  }
  if (cmd == "north") {
    getHeading();
    calibration.magOffset = magHeading;
  }
  if (cmd == "bilgeMin") {
    calibration.bilgeMin = readResistance(waterPin, calibration.bilgeSeriesResistor);
  }
  if (cmd == "bilgeMax") {
    calibration.bilgeMax = readResistance(waterPin, calibration.bilgeSeriesResistor);
  }
  writeCalibration();
  printCalibration();
  return 1;
}

int pumpBilge(String cmd) {
	cmd.trim();
	cmd.toUpperCase();
	if (cmd == "START") {
		Particle.publish("starting pump");
		digitalWrite(pumpPin, HIGH);
	} else if (cmd == "STOP") {
		Particle.publish("stopping pump");
		digitalWrite(pumpPin, LOW);
	}
	return 1;
}

float readResistance(int pin, int seriesResistance) {
  // Get ADC value.
  float resistance = analogRead(pin);
  Particle.publish("resist", String(resistance));
  // Convert ADC reading to resistance.
  resistance = (4095.0 / resistance) - 1.0;
  resistance = seriesResistance / resistance;
  return resistance;
}

float resistanceToVolume(float resistance, float zeroResistance, float calResistance, float calVolume) {
  if (resistance > zeroResistance || (zeroResistance - calResistance) == 0.0) {
    // Stop if the value is above the zero threshold, or no max resistance is set (would be divide by zero).
    return 0.0;
  }
  // Compute scale factor by mapping resistance to 0...1.0+ range relative to maxResistance value.
  float scale = (zeroResistance - resistance) / (zeroResistance - calResistance);
  // Scale maxVolume based on computed scale factor.
  return calVolume * scale;
}

double getBilgeLevel() {
	bilgeReading = readResistance(waterPin, calibration.bilgeSeriesResistor);
	bilgeLevel = resistanceToVolume(bilgeReading, calibration.bilgeMin, calibration.bilgeMax, 12.0);
	return bilgeLevel;
}

void getWindSpeed() {
	double reading = analogRead(windSpeedPin) * analogToVolts;
	windSpeed = (reading - calibration.windVoltsMin)*windSpeedMax/(calibration.windVoltsMax - calibration.windVoltsMin);
}

void getWindDirection() {
	windDirection = analogRead(windDirectionPin);
}

void getHeading() {
	// read data from HMC5883L
	if (magAvailable) {
		sensors_event_t event;
		mag.getEvent(&event);
		double heading = atan2(event.magnetic.y, event.magnetic.x);

    if (heading < 0) {
			heading += 2*PI;
		}

		// Check for wrap due to addition of declination.
		if (heading > 2*PI) {
			heading -= 2*PI;
		}
		magHeading = heading * 180/M_PI - calibration.magOffset;
	}
}

void getMeteo() {
	/*if (bmpAvailable) {
		bmpAirTemperature = bmp.readTemperature();
		barometricPressure = bmp.readPressure();
	}*/
	if (dhtAvailable) {
		humidity = dht.getHumidity();
		dhtAirTemperature = dht.getTempCelcius();
		dewpoint = dht.getDewPoint();
	}
}

void getSeaTalk() {

}

void readGPS() {

}
