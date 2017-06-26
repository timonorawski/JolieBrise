const int windSpeedPin = A0;
const int windDirectionPin = A1;
const int waterPin = A2;
const int pumpPin = D3;
const int defaultLoopDelay = 1000*60*30;

#define USEGPS 1
int gpsFrequency = 900;

// definitions for bilge level sensor
const int BILGEDEPTHSERIESRESISTOR = 560;
const int BILGEDEPTHCALIBRATEDMAX = 2048;
const int BILGEDEPTHCALIBRATEDMIN = 300;

const float analogToVolts = .004882814;
const float windVoltageMin = 0.4;
const float windVoltageMax = 2.0;
const float windSpeedMin = 0; // Wind speed in meters/sec corresponding to minimum voltage
const float windSpeedMax = 32; // Wind speed in meters/sec corresponding to maximum voltage
