#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_DHT.h>
#include <Adafruit_HMC5883.h>
//#include <google-maps-device-locator.h>
#include "TinyGPS.h"
#include "defs.h"

// GPS
int loopsSinceUpdate = 0;
TinyGPS gps;
char szInfo[64];
// Every 15 minutes
int debug = 0;
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
double course;
int satellites;
double speed;

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
  calibration.locationFrequency = LOCATIONFREQUENCY;
  calibration.heartbeatIntervalSeconds = NOTIFYINTERVAL;
  calibration.loopDelaySeconds = LOOPTIME;
  writeCalibration();
}

void writeCalibration() {
  EEPROM.put(0, calibration);
}

void printCalibration() {
  char publishString[255];
  sprintf(publishString,
      "{v: %u, mag: %f, wvMin: %f, wvMax: %f, bMax: %f, bMin: %f, bRes: %u, hb: %u, l: %u}",
      calibration.version,
      calibration.magOffset,
      calibration.windVoltsMin,
      calibration.windVoltsMax,
      calibration.bilgeMax,
      calibration.bilgeMin,
      calibration.bilgeSeriesResistor,
      calibration.heartbeatIntervalSeconds,
      calibration.loopDelaySeconds);
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
  // GPS
  Serial1.begin(19200);

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
	Particle.variable("course", course);
  Particle.variable("satellites", satellites);
  Particle.variable("speed", speed);

  Particle.publish("joliebrise/system/setup");
}

void loop() {
	update("all");
  processGPS();
  processData();
  loopsSinceUpdate++;
  if ((loopsSinceUpdate * calibration.loopDelaySeconds) >= calibration.heartbeatIntervalSeconds ) {
    Particle.publish("joliebrise/notify/alive");
    //Particle.publish("joliebrise/notify/bilgeLevel", String(bilgeLevel));
    loopsSinceUpdate = 0;
  }
	delay(calibration.loopDelaySeconds * 1000);
}

void processGPS() {
  bool isValidGPS = false;

    for (unsigned long start = millis(); millis() - start < 1000;){
        // Check GPS data is available
        while (Serial1.available()){
            char c = Serial1.read();

            // parse GPS data
            if (gps.encode(c))
                isValidGPS = true;
        }
    }

    // If we have a valid GPS location then publish it
    if (isValidGPS){
        float lat, lon;
        unsigned long age;

        gps.f_get_position(&lat, &lon, &age);
        latitude = lat;
        longitude = lon;
        sprintf(szInfo, "%.6f,%.6f", (lat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : lat), (lon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : lon));
    }
    else{
        // Not a vlid GPS location, jsut pass 0.0,0.0
        // This is not correct because 0.0,0.0 is a valid GPS location, we have to pass a invalid GPS location
        // and check it at the client side
        sprintf(szInfo, "not available");
    }

    if (debug) {
      Particle.publish("joliebrise/location", szInfo);
    }
    satellites = gps.satellites();
    speed = gps.f_speed_knots();
    course = gps.f_course();
    altitude = gps.f_altitude();
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
  //locator.publishLocation();
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
  int colonPos = cmd.indexOf(":");
  if (colonPos > -1){
    if (cmd.substring(0, colonPos) == "heartbeat") {
      calibration.heartbeatIntervalSeconds = cmd.substring(colonPos+1, cmd.length()).toInt();
    }
    if (cmd.substring(0, colonPos) == "loopDelay") {
      calibration.loopDelaySeconds = cmd.substring(colonPos+1, cmd.length()).toInt();
    }
    if (cmd.substring(0, colonPos) == "debug") {
      debug = cmd.substring(colonPos+1, cmd.length()).toInt();
    }
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
  // Particle.publish("resist", String(resistance));
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
