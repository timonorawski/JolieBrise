#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_DHT.h>

#include "defs.h"

int loopDelay = defaultLoopDelay;
Adafruit_BMP085 bmp;
bool bmpAvailable = true;
bool magAvailable = false;
bool dhtAvailable = true;

// Variables
double bilgeLevel;
double bilgeReading;
double windSpeed;
double windDirection;

double airTemperature;
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

// end variables
#define DHTPIN D4
#define DHTTYPE DHT21

DHT dht(DHTPIN, DHTTYPE);

void setup() {
    pinMode(windSpeedPin, INPUT_PULLDOWN);
    pinMode(windDirectionPin, INPUT_PULLDOWN);
    pinMode(waterPin, INPUT_PULLDOWN);
    pinMode(pumpPin, OUTPUT);
    Particle.function("pumpBilge",pumpBilge);
    Particle.function("update", update);
    if (!bmp.begin()) {
        bmpAvailable = false;
    }
    dht.begin();

    Particle.variable("bilgeraw", bilgeReading);
    Particle.variable("bilgelevel", bilgeLevel);
    Particle.variable("windSpeed", windSpeed);
    Particle.variable("windDir", windDirection);
    Particle.variable("airTemp", airTemperature);
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
}

void loop() {
    update("all");
    delay(loopDelay);
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

int pumpBilge(String cmd) {
    cmd.trim();
    cmd.toUpperCase();
    if (cmd == "START") {
        Particle.publish("starting pump");
        digitalWrite(pumpPin, HIGH);
    } else if (cmd == "STOP") {
        Particle.publish("stopping y pump");
        digitalWrite(pumpPin, LOW);
    }
    return 1;
}

double getBilgeLevel() {
    double reading = analogRead(waterPin);
    reading = (1023 / reading)  - 1;
    bilgeReading = BILGEDEPTHSERIESRESISTOR / reading;
    bilgeLevel = 12.0 - ((reading - BILGEDEPTHCALIBRATEDMIN)/(BILGEDEPTHCALIBRATEDMAX - BILGEDEPTHCALIBRATEDMIN)  * 12.0);
    return bilgeLevel;
}

void getWindSpeed() {
    double reading = analogRead(windSpeedPin) * analogToVolts;
    windSpeed = (reading - windVoltageMin)*windSpeedMax/(windVoltageMax - windVoltageMin);
}

void getWindDirection() {
    windDirection = analogRead(windDirectionPin);
}

void getHeading() {
    // read data from HMC5883L
    if (magAvailable) {
        //Particle.variable("heading", )
    }
}

void getMeteo() {
    if (bmpAvailable) {
        airTemperature = bmp.readTemperature();
        barometricPressure = bmp.readPressure();
        //Particle.variable("altitude", bmp.readAltitude());
    }
    if (dhtAvailable) {
      humidity = dht.getHumidity();
    	airTemperature = dht.getTempCelcius();
      dewpoint = dht.getDewPoint();
    }
}

void getSeaTalk() {

}

void readGPS() {

}
