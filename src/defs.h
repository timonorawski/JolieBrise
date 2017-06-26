const int windSpeedPin = A0;
const int windDirectionPin = A1;
#define waterPin A2
const int pumpPin = D3;
const int defaultLoopDelay = 1000*60*30;

#define USEGPS 1
int gpsFrequency = 900;

// definitions for bilge level sensor
#define BILGEDEPTHSERIESRESISTOR 560;
#define BILGEDEPTHCALIBRATEDMAX 2048;
#define BILGEDEPTHCALIBRATEDMIN 300;

#define analogToVolts .004882814;
#define WINDVOLTAGEMIN 0.4;
#define WINDVOLTAGEMAX 2.0;
const int windSpeedMin = 0; // Wind speed in meters/sec corresponding to minimum voltage
const int windSpeedMax = 32; // Wind speed in meters/sec corresponding to maximum voltage

typedef struct {
  char version;
  float magOffset;
  float windVoltsMin;
  float windVoltsMax;
  float bilgeMax;
  float bilgeMin;
  int bilgeSeriesResistor;
  char future[36];
} Calibration;

#ifdef GY_87
#define USE_HMC5883L
#define USE_MPU6050
#define USE_BMP180
#endif
