/*
  FrSky GPS sensor class for Teensy LC/3.x/4.x, ESP8266, ATmega2560 (Mega) and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20210108
  Not for commercial use
*/

#ifndef _FRSKY_SPORT_SENSOR_GPS_H_
#define _FRSKY_SPORT_SENSOR_GPS_H_

#include "FrSkySportSensor.h"

#define GPS_DEFAULT_ID ID4
#define GPS_DATA_COUNT 11
#define GPS_LAT_LON_DATA_ID   0x0800
#define GPS_ALT_DATA_ID       0x0820
#define GPS_SPEED_DATA_ID     0x0830
#define GPS_COG_DATA_ID       0x0840
#define GPS_DATE_TIME_DATA_ID 0x0850
#define GPS_HDOP_DATA_ID      0x0860 //add Hdop
#define GPS_SAT_DATA_ID       0x0870 //add sat
#define GPS_STAT_DATA_ID      0x0880 //add status
#define GPS_VMAX_DATA_ID      0x0890 //add vmax

#define GPS_LAT_LON_DATA_PERIOD   1000
#define GPS_ALT_DATA_PERIOD       500
#define GPS_SPEED_DATA_PERIOD     500
#define GPS_COG_DATA_PERIOD       500
#define GPS_DATE_TIME_DATA_PERIOD 10000
#define GPS_HDOP_DATA_PERIOD      1000   
#define GPS_SAT_DATA_PERIOD       1000   
#define GPS_STAT_DATA_PERIOD      1000 
#define GPS_VMAX_DATA_PERIOD      1000  


class FrSkySportSensorGps_Cust : public FrSkySportSensor
{
  public:
    FrSkySportSensorGps_Cust(SensorId id = GPS_DEFAULT_ID);
    void setData(float lat, float lon, float alt, float speed, float cog, uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, float hdop, uint8_t sat, uint8_t stat, float vmax);
    virtual uint16_t send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now);
    virtual uint16_t decodeData(uint8_t id, uint16_t appId, uint32_t data);
    float getLat();
    float getLon();
    float getAltitude();
    float getSpeed();
    float getCog();
    uint8_t getYear();
    uint8_t getMonth();
    uint8_t getDay();
    uint8_t getHour();
    uint8_t getMinute();
    uint8_t getSecond();
    float getHdop();
    uint8_t getSat();
    uint8_t getStat();
    float getVmax();

  private:
    static uint32_t setLatLon(float latLon, bool isLat);
    static uint32_t setDateTime(uint8_t yearOrHour, uint8_t monthOrMinute, uint8_t dayOrSecond, bool isDate);
    uint32_t latData;
    uint32_t lonData;
    int32_t altData;
    uint32_t speedData;
    uint32_t cogData;
    uint32_t dateData;
    uint32_t timeData;
    uint32_t hdopData;
    uint32_t satData;
    uint32_t statData;
    uint32_t vmaxData;
    
    uint32_t latTime;
    uint32_t lonTime;
    uint32_t altTime;
    uint32_t speedTime;
    uint32_t cogTime;
    uint32_t dateTime;
    uint32_t timeTime;
    uint32_t hdopTime; //add hdop
    uint32_t satTime;  //add sat
    uint32_t statTime; //add stat
    uint32_t vmaxTime; //add vmax
    
    float lat;
    float lon;
    float altitude;
    float speed;
    float cog;
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    float hdop;
    uint8_t sat;
    uint8_t stat;
    float vmax;
    
};

#endif // _FRSKY_SPORT_SENSOR_GPS_H_
