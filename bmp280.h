#ifndef BMP280_H
#define BMP280_H

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <log.h>
#include <hw.h>
#include <utils.h>
#include <interval.h>
#include <simpleEvents.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 8


struct BarometerData {
    float altitude_m = 0;
    float absPressure = 0;
    float alt_ground_m = 0;
    float temperature_m = 0;
};


typedef void (*barometerDataEvent)(BarometerData*);


class BMP280:  public SimpleEvent {
    public:
        /******************************************************************************************
         */
        BMP280(TwoWire& _wire, boolean _noInterval = false): SimpleEvent("bmp280"), wire(_wire), noInterval(_noInterval) {
            initialized_m = bmp.begin();                  // Default initialisation, place the BMP280 into SLEEP_MODE
            bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                            Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                            Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                            Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                            Adafruit_BMP280::STANDBY_MS_63); /* Standby time. */

            if(!initialized_m) {
                logme(kLogError, LINEINFOFORMAT"%s:%d Could not find a valid BMP280 sensor, check wiring!", LINEINFO, POS_LOG_ARG);
            }
            else {
                logme(kLogInfo, LINEINFOFORMAT"%s:%d BMP280 sensor is initialized", LINEINFO, POS_LOG_ARG);
            }
        }


        /******************************************************************************************
         */
        void print() {
            if(!initialized_m) {
                return;
            }

            printme(NEWLINE, NO_TIMESTAMP, "pressure = %fPa, alt ground = %4.1fm, alt = %4.1fm, temp = %fC", sensorData.absPressure, sensorData.alt_ground_m, sensorData.altitude_m, sensorData.temperature_m);
        }

        /******************************************************************************************
         */
        void onBarometerEvent(barometerDataEvent eventFunction) {
            logme(kLogInfo, LINEINFOFORMAT "Register Event kBarometeDataEvent = %d", LINEINFO, kBarometeDataEvent);
            registerEvent(kBarometeDataEvent, (event) eventFunction);
        }

        /******************************************************************************************
         */
        void ResetAltitude() {
            printme(NEWLINE, NO_TIMESTAMP, "Resetting barometer ground altitude", LINEINFO);
            sensorData.alt_ground_m = sensorData.altitude_m;
        }

        /******************************************************************************************
         */
        void run() {
            if(!initialized_m) {
                //  return;
            }

            if(expired_interval(timer, interval) || noInterval) { /* if no interval = true, ignore interval, i.e. taskScheduler is used */
                sensors_event_t temp_event, pressure_event;
                Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
                Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
                bmp_temp->getEvent(&temp_event);
                bmp_pressure->getEvent(&pressure_event);

                // printme(NEWLINE, TIMESTAMP, "Resetting barometer ground altitude", LINEINFO);
                sensorData.absPressure = pressure_event.pressure;
                sensorData.temperature_m = temp_event.temperature;

                // need to set ground alt_ground
                float altitude = bmp.readAltitude(1013.25);

                if(!alGroundReset && millis() > 20000) {
                    sensorData.alt_ground_m = altitude;
                    alGroundReset = true;
                }

                sensorData.altitude_m = altitude - sensorData.alt_ground_m;
                fireEvent(kBarometeDataEvent, &sensorData);
            }
        }


        /******************************************************************************************
         */
        float GetAltitude() {
            return sensorData.altitude_m;
        }
        /******************************************************************************************
         */
        float GetTemperature() {
            float temp;
            temp = sensorData.temperature_m;
            return temp;
        }


        /******************************************************************************************
         */
        float GetPressurePa() {
            if(!initialized_m) {
                return 101300;
            }

            return sensorData.absPressure * 100;
        }

        /******************************************************************************************
         */
        bool isInitialized() {
            return initialized_m;
        }

    private:
        TwoWire& wire;
        boolean &noInterval;
        const static int kBarometeDataEvent = 0;
        size_t reftime = millis();
        size_t timer = millis();
        int interval = 70; // 100 msec in 10mSec increments
        bool initialized_m = false;
        int count = 0;
        BarometerData sensorData;
        Adafruit_BMP280 bmp{&wire};
        bool alGroundReset = false;
};

#endif


