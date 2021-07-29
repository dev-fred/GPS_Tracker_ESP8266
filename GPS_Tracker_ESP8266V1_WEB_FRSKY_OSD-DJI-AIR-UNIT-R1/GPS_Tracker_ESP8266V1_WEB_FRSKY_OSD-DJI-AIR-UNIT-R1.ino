/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
  
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
  
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
/*------------------------------------------------------------------------------
  5/03/2021
  Author: Fred.Dev
  Platforms: ESP8266
  Language: C++/Arduino
  Basé sur :
  https://github.com/khancyr/droneID_FR
  https://github.com/f5soh/balise_esp32/blob/master/droneID_FR.h (version 1 https://discuss.ardupilot.org/t/open-source-french-drone-identification/56904/98 )
  https://github.com/f5soh/balise_esp32
  FrSkySportTelemetry https://www.rcgroups.com/forums/showthread.php?2245978-FrSky-S-Port-telemetry-library-easy-to-use-and-configurable
  https://github.com/d3ngit/djihdfpv_mavlink_to_msp_V2
  
  Add MSP to DJI Air unit
  Utilise une carte NodeMCU Amica V2 ESP8266MOD 12-F, se compile avec la carte NodeMCU 1.0 (ESP-12E Module)
------------------------------------------------------------------------------*/


/* DJI HD FPV to MSP
  *  Converts Frsky telemetry data to MSP telemetry data compatible with the DJI HD FPV system.
  *
  *  Arduino nodeMCU TX to DJI Air unit RX(115200)
*/

#include "MSP.h"
#include "MSP_OSD.h"
#include "OSD_positions_config.h"

#define MSP_BAUD_RATE 115200
MSP msp;

#define SPEED_IN_KILOMETERS_PER_HOUR

msp_raw_gps_t raw_gps = {0};
msp_comp_gps_t comp_gps = {0};

double gps_home_lon = 0.0;
double gps_home_lat = 0.0;
int32_t gps_home_alt = 0;
int16_t heading = 0;
float distanceToHome = 0;    // distance to home in meters
int16_t directionToHome = 0; // direction to home in degrees
char craftname[15] = "Ranger2000";
msp_name_t name = {0};
//int32_t relative_alt = 1000;       // in milimeters
msp_altitude_t altitude = {0};
msp_analog_t analog = {0};
msp_battery_state_t battery_state = {0};
uint8_t batteryState = 0;// voltage color 0==white, 1==red

uint32_t flightModeFlags = 0;
msp_status_BF_t status_BF = {0};
//DJI supported flightModeFlags
// 0b00000001 acro/arm
// 0b00000010 stab
// 0b00000100 hor
// 0b00001000 head
// 0b00010000 !fs!
// 0b00100000 resc
// 0b01000000 acro
// 0b10000000 acro

void send_msp_to_airunit(double gps_lat,double gps_lon,int8_t numSat,float groundspeed,int16_t _heading,int32_t relative_alt,uint16_t vbat)
{
    //MSP_RAW_GPS
    raw_gps.lat = (int32_t)(gps_lat*10000000.0f);
    raw_gps.lon = (int32_t)(gps_lon*10000000.0f);
    raw_gps.numSat = numSat;
    raw_gps.alt = (int16_t) relative_alt / 10; //cm
    raw_gps.groundSpeed = (int16_t)(groundspeed * 100);
    msp.send(MSP_RAW_GPS, &raw_gps, sizeof(raw_gps));

    //MSP_ALTITUDE
    altitude.estimatedActualPosition = (int32_t)relative_alt / 10; //cm
    msp.send(MSP_ALTITUDE, &altitude, sizeof(altitude));
  
    //MSP_COMP_GPS
    comp_gps.distanceToHome = (int16_t)(distanceToHome);
    comp_gps.directionToHome = directionToHome - _heading;
    msp.send(MSP_COMP_GPS, &comp_gps, sizeof(comp_gps));

    //MSP_NAME
    memcpy(name.craft_name, craftname, sizeof(craftname));
    msp.send(MSP_NAME, &name, sizeof(name));

    //MSP_ANALOG
    analog.vbat = vbat;
    //analog.rssi = rssi;
    //analog.amperage = amperage;
    //analog.mAhDrawn = mAhDrawn;
    msp.send(MSP_ANALOG, &analog, sizeof(analog));
    
    //MSP_BATTERY_STATE
    battery_state.batteryVoltage = vbat ;
    battery_state.batteryState = batteryState;// voltage color 0==white, 1==red
    msp.send(MSP_BATTERY_STATE, &battery_state, sizeof(battery_state));

    //MSP_STATUS -> Start DJI recording when flightModeFlags = 0b00000001 acro/arm 
    status_BF.flightModeFlags = flightModeFlags;
    msp.send(MSP_STATUS, &status_BF, sizeof(status_BF));
    
}

msp_osd_config_t msp_osd_config = {0};

void send_osd_config()
{
    msp_osd_config.units = 1;//0 is Imperial
    msp_osd_config.osd_item_count = 56;
    msp_osd_config.osd_stat_count = 24;
    msp_osd_config.osd_timer_count = 2;
    msp_osd_config.osd_warning_count = 16;             // 16
    msp_osd_config.osd_profile_count = 1;              // 1
    msp_osd_config.osdprofileindex = 1;                // 1
    msp_osd_config.overlay_radio_mode = 0;             // 0
  
    msp_osd_config.osd_rssi_value_pos = osd_rssi_value_pos;
    msp_osd_config.osd_main_batt_voltage_pos = osd_main_batt_voltage_pos;
    msp_osd_config.osd_crosshairs_pos = osd_crosshairs_pos;
    msp_osd_config.osd_artificial_horizon_pos = osd_artificial_horizon_pos;
    msp_osd_config.osd_horizon_sidebars_pos = osd_horizon_sidebars_pos;
    msp_osd_config.osd_item_timer_1_pos = osd_item_timer_1_pos;
    msp_osd_config.osd_item_timer_2_pos = osd_item_timer_2_pos;
    msp_osd_config.osd_flymode_pos = osd_flymode_pos;
    msp_osd_config.osd_craft_name_pos = osd_craft_name_pos;
    msp_osd_config.osd_throttle_pos_pos = osd_throttle_pos_pos;
    msp_osd_config.osd_vtx_channel_pos = osd_vtx_channel_pos;
    msp_osd_config.osd_current_draw_pos = osd_current_draw_pos;
    msp_osd_config.osd_mah_drawn_pos = osd_mah_drawn_pos;
    msp_osd_config.osd_gps_speed_pos = osd_gps_speed_pos;
    msp_osd_config.osd_gps_sats_pos = osd_gps_sats_pos;
    msp_osd_config.osd_altitude_pos = osd_altitude_pos;
    msp_osd_config.osd_roll_pids_pos = osd_roll_pids_pos;
    msp_osd_config.osd_pitch_pids_pos = osd_pitch_pids_pos;
    msp_osd_config.osd_yaw_pids_pos = osd_yaw_pids_pos;
    msp_osd_config.osd_power_pos = osd_power_pos;
    msp_osd_config.osd_pidrate_profile_pos = osd_pidrate_profile_pos;
    msp_osd_config.osd_warnings_pos = osd_warnings_pos;
    msp_osd_config.osd_avg_cell_voltage_pos = osd_avg_cell_voltage_pos;
    msp_osd_config.osd_gps_lon_pos = osd_gps_lon_pos;
    msp_osd_config.osd_gps_lat_pos = osd_gps_lat_pos;
    msp_osd_config.osd_debug_pos = osd_debug_pos;
    msp_osd_config.osd_pitch_angle_pos = osd_pitch_angle_pos;
    msp_osd_config.osd_roll_angle_pos = osd_roll_angle_pos;
    msp_osd_config.osd_main_batt_usage_pos = osd_main_batt_usage_pos;
    msp_osd_config.osd_disarmed_pos = osd_disarmed_pos;
    msp_osd_config.osd_home_dir_pos = osd_home_dir_pos;
    msp_osd_config.osd_home_dist_pos = osd_home_dist_pos;
    msp_osd_config.osd_numerical_heading_pos = osd_numerical_heading_pos;
    msp_osd_config.osd_numerical_vario_pos = osd_numerical_vario_pos;
    msp_osd_config.osd_compass_bar_pos = osd_compass_bar_pos;
    msp_osd_config.osd_esc_tmp_pos = osd_esc_tmp_pos;
    msp_osd_config.osd_esc_rpm_pos = osd_esc_rpm_pos;
    msp_osd_config.osd_remaining_time_estimate_pos = osd_remaining_time_estimate_pos;
    msp_osd_config.osd_rtc_datetime_pos = osd_rtc_datetime_pos;
    msp_osd_config.osd_adjustment_range_pos = osd_adjustment_range_pos;
    msp_osd_config.osd_core_temperature_pos = osd_core_temperature_pos;
    msp_osd_config.osd_anti_gravity_pos = osd_anti_gravity_pos;
    msp_osd_config.osd_g_force_pos = osd_g_force_pos;
    msp_osd_config.osd_motor_diag_pos = osd_motor_diag_pos;
    msp_osd_config.osd_log_status_pos = osd_log_status_pos;
    msp_osd_config.osd_flip_arrow_pos = osd_flip_arrow_pos;
    msp_osd_config.osd_link_quality_pos = osd_link_quality_pos;
    msp_osd_config.osd_flight_dist_pos = osd_flight_dist_pos;
    msp_osd_config.osd_stick_overlay_left_pos = osd_stick_overlay_left_pos;
    msp_osd_config.osd_stick_overlay_right_pos = osd_stick_overlay_right_pos;
    msp_osd_config.osd_display_name_pos = osd_display_name_pos;
    msp_osd_config.osd_esc_rpm_freq_pos = osd_esc_rpm_freq_pos;
    msp_osd_config.osd_rate_profile_name_pos = osd_rate_profile_name_pos;
    msp_osd_config.osd_pid_profile_name_pos = osd_pid_profile_name_pos;
    msp_osd_config.osd_profile_name_pos = osd_profile_name_pos;
    msp_osd_config.osd_rssi_dbm_value_pos = osd_rssi_dbm_value_pos;
    msp_osd_config.osd_rc_channels_pos = osd_rc_channels_pos;
  
    msp.send(MSP_OSD_CONFIG, &msp_osd_config, sizeof(msp_osd_config));
}

const double kmpdeg = 111.195f;  // km per degree
double coslat; // 1.000 @ 0° ,  0.500 @ 60°

#define TAN_89_99_DEGREES 5729.57795f

void GPS_distance_cm_bearing(double *currentLat1, double *currentLon1, double *destinationLat2, double *destinationLon2, double *dist, int32_t *bearing)
{
  if (coslat == 0.0) {  // no need to calculate every time
    double lat = (*destinationLat2 + *currentLat1) / 2;
    coslat = cos(lat * PI / 180);
  } 
    double dLat = *destinationLat2 - *currentLat1; // difference of latitude in 1/10 000 000 degrees
    double dLon = (*destinationLon2 - *currentLon1) * coslat;
    // Get distance between two points in km
    *dist = sqrtf(dLat*dLat + dLon*dLon) * kmpdeg;
    // Get bearing from pos1 to pos2, returns an 1deg = 100 precision
    *bearing = 9000.0f + atan2f(-dLat, dLon) * TAN_89_99_DEGREES;      // Convert the output radians to 100xdeg
    if (*bearing < 0)
  *bearing += 36000;
}

void GPS_calculateDistanceAndDirectionToHome(double gps_lat,double gps_lon)
{
  if (gps_home_lat != 0 && gps_home_lon != 0) {// If we don't have home set, do not display anything
        double dist;
        int32_t dir;
        GPS_distance_cm_bearing(&gps_lat, &gps_lon, &gps_home_lat, &gps_home_lon, &dist, &dir);
        distanceToHome = dist * 1000; //km -> m
        directionToHome = dir / 100;
    } else {
    distanceToHome = 0;
    directionToHome = 0;
  }
}

//Frsky telemetry
#include <FrSkySportSensor.h>
#include <FrSkySportSingleWireSerial.h>
#include <FrSkySportTelemetry.h>
#include "FrSkySportSensorGps_Cust.h"
#include "FrSkySportSensorFlvss.h"

FrSkySportSensorGps_Cust gpsfrsky;     // Create GPS sensor with default ID
FrSkySportSensorFlvss flvss;          // Create FLVSS sensor with default ID
FrSkySportTelemetry telemetry;
#define SPORT_PIN FrSkySportSingleWireSerial::SOFT_SERIAL_PIN_D8 //frsky sport

#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <ESP8266WebServer.h>
#include "index.h" //Web page file
#include "droneID_FR.h"

const char prefixe_ssid[] = "BALISE"; // Enter SSID here
// déclaration de la variable qui va contenir le ssid complet = prefixe + MAC adresse
char ssid[32];    
const char* password = "XXXXXXXXXXX"; //not used

/**
  * CHANGEZ l'ID du drone par celui que Alphatango vous a fourni (Trigramme + Modèle + numéro série) !
*/
//*********************** "000000000000000000000000000000"  // 30 caractères
const char drone_id[31] = "ILLEGAL_DRONE_APPELEZ_POLICE17"; // si l'id est inférieur à 30 caractères, le compléter avec des "0" au début

droneIDFR drone_idfr;

// beacon frame definition
static constexpr uint16_t MAX_BEACON_SIZE = 40 + 32 + droneIDFR::FRAME_PAYLOAD_LEN_MAX; // default beaconPacket size + max ssid size + max drone id frame size

extern "C" {
  #include "user_interface.h"
  int wifi_send_pkt_freedom(uint8 *buf, int len, bool sys_seq);
}

// beacon frame definition
uint8_t beaconPacket[MAX_BEACON_SIZE] = {
  /*  0 - 3  */ 0x80, 0x00, 0x00, 0x00, // Type/Subtype: managment beacon frame
  /*  4 - 9  */ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // Destination: broadcast
  /* 10 - 15 */ 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, // Source
  /* 16 - 21 */ 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, // Source
  
  // Fixed parameters
  /* 22 - 23 */ 0x00, 0x00, // Fragment & sequence number (will be done by the SDK)
  /* 24 - 31 */ 0x83, 0x51, 0xf7, 0x8f, 0x0f, 0x00, 0x00, 0x00, // Timestamp
  /* 32 - 33 */ 0xe8, 0x03, // Interval: 0x64, 0x00 => every 100ms - 0xe8, 0x03 => every 1s
  /* 34 - 35 */ 0x21, 0x04, // capabilities Tnformation
  
  // Tagged parameters
  
  // SSID parameters
  /* 36 - 38 */ 0x03, 0x01, 0x06, // DS Parameter set, current channel 6 (= 0x06), // TODO: manually set it
  /* 39 - 40 */ 0x00, 0x20,       // 39-40: SSID parameter set, 0x20:maxlength:content
};

// Ensure the AP SSID is max 31 letters
// 31 lettres maxi selon l'api, 17 caractères de l'adresse mac, reste 15 pour ceux de la chaine du début moins le caractère de fin de chaine ça fait 14, 14+17=31
static_assert((sizeof(prefixe_ssid)/sizeof(*prefixe_ssid))<=(14+1), "Prefix of AP SSID should be less than 14 letters");
// Vérification drone_id max 30 
static_assert((sizeof(drone_id)/sizeof(*drone_id))<=31, "Drone ID should be less that 30 letters !");  // 30 lettres + null termination

// ========================================================== //

/* Put IP Address details-> smartphone */
IPAddress local_ip(192, 168, 1, 10);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

ESP8266WebServer server(80);

#define led_pin 2  //internal blue LED
int led ;

void flip_Led() {
  //Flip internal LED
  if (led == HIGH) {
    digitalWrite(led_pin, led);
    led = LOW;} else { 
    digitalWrite(led_pin, led);
  led= HIGH;}
}

#define GPS_9600 9600           // Valeur par défaut
#define GPS_57600 57600         // Autre config possible du GPS
#define GPS_RX_PIN 5            // D1 Brancher le fil Tx du GPS
#define GPS_TX_PIN 4            // D2 Brancher le fil Rx du GPS

SoftwareSerial softSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

#define USE_SERIAL Serial

char buff[14][34];
//0$ID;1$UTC;2$SAT;3$HDOP;4$LNG;5$LAT;6$ALT;7$VMAX;8$STATUS;9$TRAME;10$DEP;11$DLNG;12$DLAT;13$TIME 

void handleReadValues() {   //pour traiter la requette de mise à jour de la page HTML 
  String mes = "";
  for (int i = 0; i <= 14; i++) {
    if (i != 0)  mes += ";";
    mes += (String)i + "$" + buff[i] ;
  }
  server.send(200, "text/plain", mes);
}

void beginServer()
{
  server.begin();
  server.on("/",[](){server.send_P(200, "text/html", webpage);});
  server.on("/readValues", handleReadValues);
  Serial.println ( "HTTP server started" );
}

//testé sur BN220
void SelectChannels()
{
  // CFG-GNSS packet GPS + Galileo + Glonas  
  byte packet[] = {
    0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x00, 0x08, 0x10, 0x00,
    0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x04,
    0x08, 0x00, 0x01, 0x00, 0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01,
    0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x01, 0x00, 0x01, 0x01, 0x2E, 0x75};
  sendPacket(packet, sizeof(packet));    
}

void BaudRate9600()
{
    byte packet[] = {0xB5,0x62, 0x06,0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0xB5};
    sendPacket(packet, sizeof(packet));
}

void BaudRate57600()
{
    byte packet[] = {0xB5,0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0xC9};
    sendPacket(packet, sizeof(packet));
}

void Rate166()
{
    byte packet[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xA6, 0x00, 0x01, 0x00, 0x01, 0x00, 0xBC, 0x9E};
    sendPacket(packet, sizeof(packet));
}

void ubloxInit()
{
    byte packet[] =
    {
    //Preprocessor Pedestrian Dynamic Platform Model Option
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00,           // CFG-NAV5 - Set engine settings
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,           // Collected by resetting a GPS unit to defaults. Changing mode to Pedistrian and
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,           // capturing the data from the U-Center binary console.
    0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xC2,
    
    // Enable UBLOX messages
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47,           // set POSLLH MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49,           // set STATUS MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F,           // set SOL MSG rate
    //0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x01, 0x3C, 0xA3,           // set SVINFO MSG rate (every cycle - high bandwidth)
    //0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x05, 0x40, 0xA7,           // set SVINFO MSG rate (evey 5 cycles - low bandwidth)
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67,           // set VELNED MSG rate
   };
sendPacket(packet, sizeof(packet)); }

// Send the packet specified to the receiver.
void sendPacket(byte *packet, byte len)
{
    for (byte i = 0; i < len; i++)
    {
        softSerial.write(packet[i]);
  }
}

uint8_t set_home = 1;
uint16_t blink_sats_orig_pos = osd_gps_sats_pos;
uint16_t blink_sats_blank_pos = 234;

void invert_pos(uint16_t *pos1, uint16_t *pos2)
{
    uint16_t tmp_pos = *pos1;
    *pos1 = *pos2;
    *pos2 = tmp_pos;
}

void blink_sats()
{
    if(set_home == 1 && blink_sats_orig_pos > 2000){
    invert_pos(&osd_gps_sats_pos, &blink_sats_blank_pos);
  }
    else if(set_home == 0){
    osd_gps_sats_pos = blink_sats_orig_pos;
  }
}

uint8_t alarme_bat = 0;
uint16_t blink_batt_orig_pos = osd_main_batt_voltage_pos;
void blink_batt()
{
    if(alarme_bat == 1) { 
      if (osd_main_batt_voltage_pos > 2000){osd_main_batt_voltage_pos = 234;} 
      else 
      {osd_main_batt_voltage_pos = blink_batt_orig_pos; }}
    if(alarme_bat == 0) {
    osd_main_batt_voltage_pos = blink_batt_orig_pos;
  }
}



void setup()
{
  Serial.begin(115600);
  Serial.println();
  
  for(uint8_t t = 4; t > 0; t--) {
    Serial.printf("[SETUP] BOOT WAIT %d...\r\n", t);
    Serial.flush();
    delay(1000);
  }
  //telemetrie frsky pour le GPS et la batterie, cellule 1
  telemetry.begin(SPORT_PIN, &gpsfrsky,&flvss);
  //telemetry.begin(SPORT_PIN, &gpsfrsky);  
  //connection sur le terrain à un smartphone
    // start WiFi
    WiFi.mode(WIFI_AP);
    //conversion de l'adresse mac:
    String temp = WiFi.macAddress();
    temp.replace(":","_");
    //concat du prefixe et de l'adresse mac
    temp = String(prefixe_ssid) + "_" + temp;
    //transfert dans la variable globale ssid 
    temp.toCharArray(ssid, 32);
    
    // set default AP settings
    WiFi.softAP(ssid, nullptr, 6, false, 4); // ssid, pwd, channel, hidden, max_cnx 
    WiFi.softAPConfig(local_ip, gateway, subnet);
    WiFi.setOutputPower(20.5); // max 20.5dBm
    delay(100);
  
    Serial.println();
    Serial.println("Started \\o/");
    Serial.print("IP page WEB: ");Serial.println(local_ip);
    Serial.print("ID Drone: ");Serial.println(drone_id);
    WiFi.printDiag(Serial);
    
    softap_config current_config;
    wifi_softap_get_config(&current_config);
    current_config.beacon_interval = 1000;
    wifi_softap_set_config(&current_config);
    
    beginServer(); //lancement du server WEB
  
    //--------------------------------------------- 57600 ->BAUDRATE 9600
    softSerial.begin(GPS_57600);
    delay(100); // Little delay before flushing.
    softSerial.flush();
    Serial.println("GPS BAUDRATE 9600");
    BaudRate9600();
    delay(100); // Little delay before flushing.
    softSerial.flush(); 
    softSerial.begin(GPS_9600);
    delay(100); // Little delay before flushing.
    softSerial.flush();
    //--------Init provenant de Betaflight 
    ubloxInit();
    delay(100); // Little delay before flushing.
    softSerial.flush();
    //--------------------------------------------- 9600 ->BAUDRATE 57600
    Serial.println("GPS BAUDRATE 57600");
    BaudRate57600();
    delay(100); // Little delay before flushing.
    softSerial.flush(); 
    softSerial.begin(GPS_57600);
    //-------Configure GPS ublox
    delay(100); // Little delay before flushing.
    softSerial.flush();    
    //-------Config RATE = 166 ms/6Hz
    Serial.println("Configure GPS RATE = 6 Hz/166 ms");
    Rate166();
    delay(100); // Little delay before flushing.
    softSerial.flush();
    //--------Config CHANNELS 
    Serial.println("Configure GPS CHANNELS = GPS + Galileo + Glonas");   
    SelectChannels();
    delay(100); // Little delay before flushing.
    softSerial.flush();
  
    drone_idfr.set_drone_id(drone_id);
    snprintf(buff[0], sizeof(buff[0]), "ID:%s", drone_id);  
    delay(5000);
    
    //built in blue LED -> change d'état à chaque envoi de trame
    pinMode(led_pin, OUTPUT);
    
/*    Serial
The ESP8266 has two hardware UARTS (Serial ports):
UART0 on pins 1 and 3 (TX0 and RX0 resp.), and UART1 on pins 2 and 8 (TX1 and RX1 resp.), 
however, GPIO8 is used to connect the flash chip. This means that UART1 can only transmit data.
To use UART1 (TX = GPIO2), use the Serial1 object.
*/

    //MSP = D4/2 de D1 mini = Serial1 TX-> RX Vista
    Serial1.begin(MSP_BAUD_RATE);
    msp.begin(Serial1);
}

unsigned int TRBcounter = 0;
unsigned int nb_sat = 0;
unsigned int SV = 0;
uint64_t gpsSec = 0;
uint64_t beaconSec = 0;
float VMAX = 0.0;
float altitude_ref = 0.0 ;
float HLng = 0.0;
float HLat = 0.0;
static uint64_t gpsMap = 0;
static uint64_t sendosd = 0;
static uint64_t sendSerial = 0;
unsigned  long _heures=0;
unsigned  long _minutes=0;
unsigned  long _secondes=0;
const unsigned int limite_sat = 5;
const float limite_hdop = 2.0;
float _hdop = 0.0;
unsigned int _sat = 0;
float GPS[]={0.0,0.0,0.0,0.0};
uint8_t Y=0,M=0,D=0,H=0,MN=0,S=0;
unsigned int status = 0;//status télémétrie
uint16_t vbat = 0;
const int analogInPin = A0;  // ESP8266 Analog Pin ADC0 = A0
double voltage=0;

/* Status Widget Balise
    [0]  = "NO STAT",
    [1]  = "NO GPS",
    [2]  = "ATT SAT",
    [3]  = "DEPART"
*/
void loop()
{
  server.handleClient();
  
  //MSP_OSD
  if (millis() - sendosd > 2000) {
    GPS_calculateDistanceAndDirectionToHome(GPS[0],GPS[1]);
    vbat=analogRead(analogInPin);//On ne mesure qu'une cellule de la batterie
    voltage = map(vbat,0,1023, 0, 330);//ADC 10 bits = 1023, A0 = 3.3V max sur le NodeMcu X 100 pour garder la résolution
    vbat=voltage*0.2;//Diviseur résistif par 2 (R1=39K, R2=39K) + division par 10 de vbat, il reste X 10 car affichage fixe X.X-> décalage à gauche
    flvss.setData(voltage*0.02);  // Cell 1 voltage in volts note: prend en compte à partir de 0.5V
    if (vbat<36)//seuil alarme de 3,6 Volts X 10
    {batteryState=1;alarme_bat=1;} else {batteryState=0;alarme_bat=0;}// voltage color 0==white, 1==red
    blink_sats();
    blink_batt();
    send_osd_config();
    send_msp_to_airunit(GPS[0],GPS[1],gps.satellites.value(),GPS[3],(uint16_t)gps.course.deg(),1000*(GPS[2]-altitude_ref),vbat);
    sendosd=millis();
  }
  
  // preparation telemetrie GPS
  gpsfrsky.setData(GPS[0],GPS[1],     // Latitude and longitude in degrees decimal (positive for N/E, negative for S/W)
    GPS[2]-altitude_ref,            // Altitude in m (can be negative)
    GPS[3],                         // Speed in m/s
    gps.course.deg(),               // Course over ground in degrees (0-359, 0 = north)
    Y,M,D,                          // Date (year - 2000, month, day)
    H,MN,S,                         // Time (hour, minute, second) - will be affected by timezone setings in your radio
    gps.hdop.hdop(),                // Hdop  
    gps.satellites.value(),         // nb satellite
    status,                         // Status
  VMAX);                              // VMAX
  //envoi telemetrie
  telemetry.send(); 
  
  // Ici on lit les données qui arrivent du GPS et on les passe à la librairie TinyGPS++ pour les traiter
  while (softSerial.available())  gps.encode(softSerial.read());
  
  // On traite le cas où le GPS a un problème
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    //Serial.println("NO GPS");
    strncpy(buff[8], "NO GPS", sizeof(buff[8]));
    status = 1; //"NO GPS"
    return;
  }
  
  // On traite le cas si la position GPS n'est pas valide
  if (!gps.location.isValid()) {
    if (millis() - gpsMap > 1000) {
      SV=gps.satellites.value();
      //Serial.print("Waiting... SAT=");  Serial.println(SV);
      snprintf(buff[8], sizeof(buff[8]),"ATT SAT %u",SV);
      status = 2; //"ATT SAT"
      //Flip internal LED
      flip_Led();
      gpsMap = millis();
    }
    return;
    } else if (gps.location.age()> 3000) {
    //Serial.println("NO SAT");
    strncpy(buff[8], "NO SAT", sizeof(buff[8]));
    return;
    } else if (gps.location.isUpdated() && gps.altitude.isUpdated() && gps.course.isUpdated() && gps.speed.isUpdated()){
    // On traite le cas où la position GPS est valide.
    // On renseigne le point de démarrage quand la précision est satisfaisante
    _hdop = gps.hdop.hdop();
    if ( gps.satellites.value() > nb_sat ) {//+ de satellites
      nb_sat = gps.satellites.value();
      //Serial.print("NEW SAT="); Serial.println(nb_sat);
      strncpy(buff[8], "+SAT", sizeof(buff[8]));
      snprintf(buff[2], sizeof(buff[2]), "SAT:%u",nb_sat);
      snprintf(buff[3], sizeof(buff[3]), "HDOP:%.2f",_hdop);
    }
    if ( gps.satellites.value() < nb_sat ) {//- de satellites
      nb_sat = gps.satellites.value();
      //Serial.print("LOST SAT="); Serial.println(nb_sat);
      strncpy(buff[8], "-SAT", sizeof(buff[8]));
      snprintf(buff[2], sizeof(buff[2]), "SAT:%u",nb_sat);
      snprintf(buff[3], sizeof(buff[3]), "HDOP:%.2f",_hdop);
    }
    if (!drone_idfr.has_home_set() && gps.satellites.value() > limite_sat && gps.hdop.hdop() < limite_hdop) {    
      //Serial.println("Setting Home Position");
      HLat=gps.location.lat(); HLng=gps.location.lng();
      altitude_ref=gps.altitude.meters();
      drone_idfr.set_home_position(HLat, HLng, altitude_ref);
      //MSP_OSD_HOME
      gps_home_lat = HLat;
      gps_home_lon = HLng;
      gps_home_alt = altitude_ref;
      set_home = 0;
      flightModeFlags = 0b00000001;//Start DJI recording
      
      snprintf(buff[11], sizeof(buff[11]), "DLNG:%.4f", HLng);
      snprintf(buff[12], sizeof(buff[12]), "DLAT:%.4f", HLat);
      strncpy(buff[10], "DEPART", sizeof(buff[10]));
      VMAX=0;
      status = 3; //"PRET"
      D = gps.date.day();
      M = gps.date.month();   
      Y = gps.date.year()-2000;
    }
    
    // On actualise les données GPS de la librairie d'identification drone.
    GPS[0]=gps.location.lat(); 
    GPS[1]=gps.location.lng(); 
    GPS[2]=gps.altitude.meters();
    GPS[3]=gps.speed.mps();
    //drone_idfr.set_ground_speed(gps.speed.mps());     
    drone_idfr.set_ground_speed(GPS[3]);
    drone_idfr.set_heading(gps.course.deg());    
    //drone_idfr.set_current_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters());    
    drone_idfr.set_current_position(GPS[0], GPS[1], (int16_t)GPS[2]);
    
    //Calcul VMAX
    //80m/s max 
    if (GPS[3] < 80) {if (VMAX <  GPS[3]){VMAX =  GPS[3];}} 
    
    //UTC time   
    H = gps.time.hour();
    MN = gps.time.minute();   
    S = gps.time.second();  
  }
  
    /**
    * On regarde s'il est temps d'envoyer la trame d'identification drone : 
    *  - soit toutes les 3s,
    *  - soit si le drone s'est déplacé de 30m,
    *  - uniquement si la position Home est déjà définie,
    *  - et dans le cas où les données GPS sont nouvelles.
  */       
    if (drone_idfr.has_home_set() && drone_idfr.time_to_send()) {
    float time_elapsed = (float(millis() - beaconSec) / 1000); 
    beaconSec = millis();
    
    //On renseigne page WEB
    snprintf(buff[1], sizeof(buff[1]), "UTC:%d:%d:%d",H,MN,S);
    _sat = gps.satellites.value(); if (_sat < limite_sat){snprintf(buff[2], sizeof(buff[2]), "--SAT:%u", _sat);}else{snprintf(buff[2], sizeof(buff[2]), "SAT:%u", _sat);}
    _hdop = gps.hdop.hdop(); if (_hdop > limite_hdop){ snprintf(buff[3], sizeof(buff[3]), "++HDOP:%.2f", _hdop);}else{snprintf(buff[3], sizeof(buff[3]), "HDOP:%.2f", _hdop);}
    snprintf(buff[4], sizeof(buff[4]), "LNG:%.4f", GPS[1]);
    snprintf(buff[5], sizeof(buff[5]), "LAT:%.4f", GPS[0]);
    snprintf(buff[6], sizeof(buff[6]), "ALT:%.2f", GPS[2]-altitude_ref);
    snprintf(buff[7], sizeof(buff[7]), "VMAX(km/h):%.2f", float (VMAX*3.6));
    
    /*
      Serial.print(time_elapsed,1); Serial.print("s Send beacon: "); Serial.print(drone_idfr.has_pass_distance() ? "Distance" : "Time");
      Serial.print(" with ");  Serial.print(drone_idfr.get_distance_from_last_position_sent()); Serial.print("m Speed="); Serial.println(drone_idfr.get_ground_speed_kmh()); 
    */
    /**
      * On commence par renseigner le ssid du wifi dans la trame
    */
    // write new SSID into beacon frame
    const size_t ssid_size = (sizeof(ssid)/sizeof(*ssid)) - 1; // remove trailling null termination
    beaconPacket[40] = ssid_size;  // set size
    memcpy(&beaconPacket[41], ssid, ssid_size); // set ssid
    const uint8_t header_size = 41 + ssid_size;  //TODO: remove 41 for a marker
    /**
      * On génère la trame wifi avec l'identification
    */
    const uint8_t to_send = drone_idfr.generate_beacon_frame(beaconPacket, header_size);  // override the null termination
    
    /**
      * On envoie la trame
    */
    wifi_send_pkt_freedom(beaconPacket, to_send, 0);
    
    //Flip internal LED
    flip_Led();
    
    //incrementation compteur de trame de balise envoyé
    TRBcounter++; 
    snprintf(buff[9], sizeof(buff[9]), "TRAME:%u", TRBcounter); 
    
    _secondes = millis()/1000; //convect millisecondes en secondes
    _minutes=_secondes/60; //convertir secondes en minutes
    _heures=_minutes/60; //convertir minutes en heures
    _secondes=_secondes-(_minutes*60); // soustraire les secondes converties afin d'afficher 59 secondes max
    _minutes=_minutes-(_heures*60);    //soustraire les minutes converties afin d'afficher 59 minutes max
    
    snprintf(buff[13], sizeof(buff[13]), " %dmn:%ds",_minutes, _secondes );
    
    /**
      * On reset la condition d'envoi
    */
    drone_idfr.set_last_send();
  } 
}
