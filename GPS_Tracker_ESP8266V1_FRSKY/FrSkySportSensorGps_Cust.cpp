/*
  FrSky GPS sensor class for Teensy LC/3.x/4.x, ESP8266, ATmega2560 (Mega) and ATmega328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20210108
  Not for commercial use
*/

#include "FrSkySportSensorGps_Cust.h" 

FrSkySportSensorGps_Cust::FrSkySportSensorGps_Cust(SensorId id) : FrSkySportSensor(id) { }

void FrSkySportSensorGps_Cust::setData(float lat, float lon, float alt, float speed, float cog, uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, float hdop, uint8_t sat, uint8_t stat, float vmax)
{
  latData = setLatLon(lat, true);
  lonData = setLatLon(lon, false);
  cogData = cog * 100;
  speedData = speed * 1944; // Convert m/s to knots
  altData = alt * 100;
  dateData = setDateTime(year, month, day, true);
  timeData = setDateTime(hour, minute, second, false);
  hdopData = hdop * 100;
  satData = sat;
  statData = stat;
  vmaxData = vmax * 360.0; // Convert m/s en km/h
}

uint32_t FrSkySportSensorGps_Cust::setLatLon(float latLon, bool isLat)
{
  uint32_t data = (uint32_t)((latLon < 0 ? -latLon : latLon) * 60 * 10000) & 0x3FFFFFFF;
  if(isLat == false) data |= 0x80000000;
  if(latLon < 0) data |= 0x40000000;

  return data;
}

uint32_t FrSkySportSensorGps_Cust::setDateTime(uint8_t yearOrHour, uint8_t monthOrMinute, uint8_t dayOrSecond, bool isDate)
{
  uint32_t data = yearOrHour;
  data <<= 8;
  data |= monthOrMinute;
  data <<= 8;
  data |= dayOrSecond;
  data <<= 8;
  if(isDate == true) data |= 0xFF;

  return data;
}

uint16_t FrSkySportSensorGps_Cust::send(FrSkySportSingleWireSerial& serial, uint8_t id, uint32_t now)
{
  uint16_t dataId = SENSOR_NO_DATA_ID;
  if(sensorId == id)
  {
    switch(sensorDataIdx)
    {
      case 0:
        dataId = GPS_LAT_LON_DATA_ID;
        if(now > latTime)
        {
          latTime = now + GPS_LAT_LON_DATA_PERIOD;
          serial.sendData(dataId, latData);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
      case 1:
        dataId = GPS_LAT_LON_DATA_ID;
        if(now > lonTime)
        {
          lonTime = now + GPS_LAT_LON_DATA_PERIOD;
          serial.sendData(dataId, lonData);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
      case 2:
        dataId = GPS_ALT_DATA_ID;
        if(now > altTime)
        {
          altTime = now + GPS_ALT_DATA_PERIOD;
          serial.sendData(dataId, altData);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
      case 3:
        dataId = GPS_SPEED_DATA_ID;
        if(now > speedTime)
        {
          speedTime = now + GPS_SPEED_DATA_PERIOD;
          serial.sendData(dataId, speedData);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
      case 4:
        dataId = GPS_COG_DATA_ID;
        if(now > cogTime)
        {
          cogTime = now + GPS_COG_DATA_PERIOD;
          serial.sendData(dataId, cogData);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
      case 5:
        dataId = GPS_DATE_TIME_DATA_ID;
        if(now > dateTime)
        {
          dateTime = now + GPS_DATE_TIME_DATA_PERIOD;
          serial.sendData(dataId, dateData);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
      case 6:
        dataId = GPS_DATE_TIME_DATA_ID;
        if(now > timeTime)
        {
          timeTime = now + GPS_DATE_TIME_DATA_PERIOD;
          serial.sendData(dataId, timeData);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
      case 7:
        dataId = GPS_HDOP_DATA_ID;
        if(now > hdopTime)
        {
          hdopTime = now + GPS_HDOP_DATA_PERIOD;
          serial.sendData(dataId, hdopData);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
      case 8:
        dataId = GPS_SAT_DATA_ID;
        if(now > satTime)
        {
          satTime = now + GPS_SAT_DATA_PERIOD;
          serial.sendData(dataId,satData);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;
      case 9:
        dataId = GPS_STAT_DATA_ID;
        if(now > statTime)
        {
          statTime = now + GPS_STAT_DATA_PERIOD;
          serial.sendData(dataId,statData);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break; 
      case 10:
        dataId = GPS_VMAX_DATA_ID;
        if(now > vmaxTime)
        {
          vmaxTime = now + GPS_VMAX_DATA_PERIOD;
          serial.sendData(dataId,vmaxData);
        }
        else
        {
          serial.sendEmpty(dataId);
          dataId = SENSOR_EMPTY_DATA_ID;
        }
        break;                                   
    }
    sensorDataIdx++;
    if(sensorDataIdx >= GPS_DATA_COUNT) sensorDataIdx = 0;
  }
  return dataId;
}

uint16_t FrSkySportSensorGps_Cust::decodeData(uint8_t id, uint16_t appId, uint32_t data)
{
  if((sensorId == id) || (sensorId == FrSkySportSensor::ID_IGNORE))
  {
    switch(appId)
    {
      case GPS_LAT_LON_DATA_ID:
        {
          float latLonData = (data & 0x3FFFFFFF) / 10000.0 / 60.0;
          if((data & 0x40000000) > 0) latLonData = -latLonData;                 // is negative?
          if((data & 0x80000000) == 0) lat = latLonData; else lon = latLonData; // is latitude?
        }
        return appId;
      case GPS_ALT_DATA_ID:
        altitude = ((int32_t)data) / 100.0;
        return appId;
      case GPS_SPEED_DATA_ID:
        speed = data / 1944.0; // Convert knots to m/s
        return appId;
      case GPS_COG_DATA_ID:
        cog = data / 100.0;
        return appId;
      case GPS_DATE_TIME_DATA_ID:
        if((data & 0xFF) > 0)  // is date?
        {
          data >>= 8; day = data & 0xFF;
          data >>= 8; month = data & 0xFF;
          data >>= 8; year = data & 0xFF;
        }
        else
        {
          data >>= 8; second = data & 0xFF;
          data >>= 8; minute = data & 0xFF;
          data >>= 8; hour = data & 0xFF;
        }
        return appId;
      case GPS_HDOP_DATA_ID:
        hdop = data / 100.0;
        return appId;
      case GPS_SAT_DATA_ID:
        sat = data;
        return appId;
      case GPS_STAT_DATA_ID:
        stat = data;
        return appId; 
      case GPS_VMAX_DATA_ID:
        vmax = data/360.0; // Convert km/h en m/s
        return appId;                            
    }
  }
  return SENSOR_NO_DATA_ID;
}

float FrSkySportSensorGps_Cust::getLat() { return lat; }
float FrSkySportSensorGps_Cust::getLon() { return lon; }
float FrSkySportSensorGps_Cust::getAltitude() { return altitude; }
float FrSkySportSensorGps_Cust::getSpeed() { return speed; }
float FrSkySportSensorGps_Cust::getCog() { return cog; }
uint8_t FrSkySportSensorGps_Cust::getYear() { return year; }
uint8_t FrSkySportSensorGps_Cust::getMonth() { return month; }
uint8_t FrSkySportSensorGps_Cust::getDay() { return day; }
uint8_t FrSkySportSensorGps_Cust::getHour() { return hour; }
uint8_t FrSkySportSensorGps_Cust::getMinute() { return minute; }
uint8_t FrSkySportSensorGps_Cust::getSecond() { return second; }
float FrSkySportSensorGps_Cust::getHdop() { return hdop; }
uint8_t FrSkySportSensorGps_Cust::getSat() { return sat; }
uint8_t FrSkySportSensorGps_Cust::getStat() { return stat; }
float FrSkySportSensorGps_Cust::getVmax() { return vmax ; }
