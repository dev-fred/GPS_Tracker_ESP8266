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
	15/02/2021
	Author: Fred.Dev
	Platforms: ESP8266
	Language: C++/Arduino
	Basé sur :
	https://github.com/khancyr/droneID_FR
	https://github.com/f5soh/balise_esp32/blob/master/droneID_FR.h (version 1 https://discuss.ardupilot.org/t/open-source-french-drone-identification/56904/98 )
	https://github.com/f5soh/balise_esp32
	FrSkySportTelemetry https://www.rcgroups.com/forums/showthread.php?2245978-FrSky-S-Port-telemetry-library-easy-to-use-and-configurable
	
	-Add VMAX
	-Add GPS rate 6Hz/166ms Baudrate 57600
	
------------------------------------------------------------------------------*/
//Frsky telemetry
#include <FrSkySportSensor.h>
#include <FrSkySportSingleWireSerial.h>
#include <FrSkySportTelemetry.h>
#include "FrSkySportSensorGps_Cust.h"

FrSkySportSensorGps_Cust gpsfrsky;     // Create GPS sensor with default ID
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

extern "C" {
	#include "user_interface.h"
	int wifi_send_pkt_freedom(uint8 *buf, int len, bool sys_seq);
}

droneIDFR drone_idfr;

// beacon frame definition
static constexpr uint16_t MAX_BEACON_SIZE = 40 + 32 + droneIDFR::FRAME_PAYLOAD_LEN_MAX; // default beaconPacket size + max ssid size + max drone id frame size

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
// 0$ID;1$UTC;2$SAT;3$HDOP;4$LNG;5$LAT;6$ALT;7$VMAX;8$STATUS;9$TRAME;10$DEP;11$DLNG;12$DLAT;13$TIME


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
	byte packet[] = {0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x00, 0x08, 0x10, 0x00,
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

// Send the packet specified to the receiver.
void sendPacket(byte *packet, byte len)
{
    for (byte i = 0; i < len; i++)
    {
        softSerial.write(packet[i]);
	}
}



void setup()
{
	Serial.begin(115200);
	Serial.println();
	
	for(uint8_t t = 4; t > 0; t--) {
		Serial.printf("[SETUP] BOOT WAIT %d...\r\n", t);
		Serial.flush();
		delay(1000);
	}
	//telemetrie frsky pour le GPS
	telemetry.begin(SPORT_PIN, &gpsfrsky);
    
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
    WiFi.softAP(ssid, nullptr, 6, false, 1); // ssid, pwd, channel, hidden, max_cnx 
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
	//--------------------------------------------- 9600 ->BAUDRATE 57600
    softSerial.begin(GPS_9600);
    delay(100); // Little delay before flushing.
    softSerial.flush();
    Serial.println("GPS BAUDRATE 57600");
    BaudRate57600();
    delay(100); // Little delay before flushing.
    softSerial.flush(); 
    softSerial.begin(GPS_57600);
    delay(100); // Little delay before flushing.
    softSerial.flush(); 
    
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
unsigned  long _heures=0;
unsigned  long _minutes=0;
unsigned  long _secondes=0;
const unsigned int limite_sat = 5;
const float limite_hdop = 2.0;
float _hdop = 0.0;
unsigned int _sat = 0;
float GPS[]={0.0,0.0,0.0,0.0,0.0};
uint8_t Y=0,M=0,D=0,H=0,MN=0,S=0;
uint8_t stat = 0;//status télémétrie

/* Status Widget Balise
	*  [0]  = "NO STAT",
    [1]  = "NO GPS",
    [2]  = "ATT SAT",
    [3]  = "DEPART"
*/
void loop()
{
	server.handleClient();
	
	// preparation telemetrie GPS
	gpsfrsky.setData(GPS[0],GPS[1],   // Latitude and longitude in degrees decimal (positive for N/E, negative for S/W)
		GPS[2]-altitude_ref,            // Altitude in m (can be negative)
		GPS[3],                         // Speed in m/s
		GPS[4],                         // Course over ground in degrees (0-359, 0 = north)
		Y,M,D,                          // Date (year - 2000, month, day)
		H,MN,S,                         // Time (hour, minute, second) - will be affected by timezone setings in your radio
		gps.hdop.hdop(),                // Hdop  
		gps.satellites.value(),         // nb satellite
		stat,                           // Status
	VMAX);                            // VMAX
	//envoi telemetrie
	telemetry.send(); 
	
	// Ici on lit les données qui arrivent du GPS et on les passe à la librairie TinyGPS++ pour les traiter
	while (softSerial.available())
    gps.encode(softSerial.read());
	// On traite le cas où le GPS a un problème
	if (millis() > 5000 && gps.charsProcessed() < 10) {
		Serial.println("NO GPS");
		strncpy(buff[8], "NO GPS", sizeof(buff[8]));
		stat = 1; //"NO GPS"
		return;
	}
	// On traite le cas si la position GPS n'est pas valide
	if (!gps.location.isValid()) {
		if (millis() - gpsMap > 1000) {
			SV=gps.satellites.value();
			Serial.print("Waiting... SAT=");  Serial.println(SV);
			snprintf(buff[8], sizeof(buff[8]),"ATT SAT %u",SV);
			stat = 2; //"ATT SAT"
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
		if ( gps.satellites.value() > nb_sat ) {//+ de satellites
			nb_sat = gps.satellites.value();
			Serial.print("NEW SAT="); Serial.println(nb_sat);
			strncpy(buff[8], "+SAT", sizeof(buff[8]));
		}
		if ( gps.satellites.value() < nb_sat ) {//- de satellites
			nb_sat = gps.satellites.value();
			Serial.print("LOST SAT="); Serial.println(nb_sat);
			strncpy(buff[8], "-SAT", sizeof(buff[8]));
		}
		if (!drone_idfr.has_home_set() && gps.satellites.value() > limite_sat && gps.hdop.hdop() < limite_hdop) {    
			
			Serial.println("Setting Home Position");
			HLat=gps.location.lat(); HLng=gps.location.lng();
			drone_idfr.set_home_position(HLat, HLng, gps.altitude.meters());
			altitude_ref=gps.altitude.meters();
			snprintf(buff[11], sizeof(buff[11]), "DLNG:%.4f", HLng);
			snprintf(buff[12], sizeof(buff[12]), "DLAT:%.4f", HLat);
			strncpy(buff[10], "DEPART", sizeof(buff[10]));
			VMAX=0;
			stat = 3; //"PRET"
			D = gps.date.day();
			M = gps.date.month();   
			Y = gps.date.year()-2000;
		}
		
		// On actualise les données GPS de la librairie d'identification drone.
		GPS[0]=gps.location.lat(); 
		GPS[1]=gps.location.lng(); 
		GPS[2]=gps.altitude.meters();
		GPS[3]=gps.speed.mps();
		GPS[4]=gps.course.deg();
		//  drone_idfr.set_ground_speed(gps.speed.mps());     
		drone_idfr.set_ground_speed(GPS[3]);
		//  drone_idfr.set_heading(gps.course.deg());
		drone_idfr.set_heading(GPS[4]);    
		//  drone_idfr.set_current_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters());    
		drone_idfr.set_current_position(GPS[0], GPS[1], (int16_t)GPS[2]);
		
		//Calcul VMAX
		//80m/s max 
		if (GPS[3] < 80) {
			if (VMAX <  GPS[3]){VMAX =  GPS[3]; 
				Serial.print("VMAX=");Serial.print(VMAX*3.60);
			Serial.print(" HDOP=");Serial.println(gps.hdop.hdop());}
		} else { Serial.print(">>>>VMAX=");Serial.println(GPS[3]*3.60);}
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
