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
/*
 * Basé sur :
 * https://github.com/khancyr/droneID_FR
 * https://github.com/khancyr/TTGO_T_BEAM
 * 
 * Adapté pour fonctionner avec un ESP8266 Wemos D1 mini
 */
// =========== Includes ======================= //
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>   
#include "droneID_FR.h"
//installer manuellement http://arduiniana.org/libraries/tinygpsplus/
#include <TinyGPS++.h> 
 
const char prefixe_ssid[] = "BALISE"; // Enter SSID here
// déclaration de la variable qui va contenir le ssid complet = prefixe + MAC adresse
char ssid[32]; 
/**
  * CHANGEZ l'ID du drone par celui que Alphatango vous a fourni (Trigramme + Modèle + numéro série) !
  */
//                        "000000000000000000000000000000"  // 30 caractères
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


// Vérification ssid max 30 
static_assert((sizeof(ssid)/sizeof(*ssid))<=32, "AP SSID should be less than 32 letters");
// Vérification drone_id max 30 
static_assert((sizeof(drone_id)/sizeof(*drone_id))<=31, "Drone ID should be less that 30 letters !");  // 30 lettres + null termination

// ========================================================== //

#define GPS_9600 9600           // Valeur par défaut
#define GPS_57600 57600         // Autre config possible du GPS
#define GPS_RX_PIN 5            // D1 Brancher le fil Tx du GPS
#define GPS_TX_PIN 4            // D2 Brancher le fil Rx du GPS

#define ENABLE_BUZZER
#define BUZZER_PIN_G         14 //D5
#define BUZZER_PIN_P         13 //D7

// utilisation
//buzz(4, 2500, 1000); // buzz sur pin 4 à 2500Hz
void buzz(int targetPin, long frequency, long length) {
  #ifdef ENABLE_BUZZER
    long delayValue = 1000000/frequency/2;
    long numCycles = frequency * length/ 1000;
    for (long i=0; i < numCycles; i++)
    {
      digitalWrite(targetPin,HIGH);
      delayMicroseconds(delayValue);
      digitalWrite(targetPin,LOW);
      delayMicroseconds(delayValue);
    }
  #endif
}

#define led_pin 2  //internal blue LED
int led;

void flip_Led() {
      //Flip internal LED
      if (led == HIGH) {
      digitalWrite(led_pin, led);
      led = LOW;} else { 
      digitalWrite(led_pin, led);
      led= HIGH;}
}

SoftwareSerial softSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;
uint8_t program = 0; // 1: test beacon without GPS
char buff[5][256];
uint64_t gpsSec = 0;
uint64_t beaconSec = 0;
int nb_sat = 0;

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

void Rate500()
{     
  byte packet[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xF4, 0x01, 0x01, 0x00, 0x01, 0x00, 0x0B, 0x77};
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

void setup() {
  // start serial
  Serial.begin(115200);
  Serial.println();

#ifdef ENABLE_BUZZER
  pinMode(BUZZER_PIN_G, OUTPUT); // set a pin for buzzer output
  digitalWrite(BUZZER_PIN_G,LOW);
  pinMode(BUZZER_PIN_P, OUTPUT); // set a pin for buzzer output
  
#endif
  buzz(BUZZER_PIN_P, 2500, 100);
  
  //built in blue LED
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);
  
  // start WiFi
  WiFi.mode(WIFI_OFF);
  //conversion de l'adresse mac:
  String temp = WiFi.macAddress();
  temp.replace(":","");
  //concat du prefixe et de l'adresse mac
  temp = String(prefixe_ssid) + "" + temp;
  //transfert dans la variable globale ssid
  temp.toCharArray(ssid, 32);
  // set default AP settings
  WiFi.softAP(ssid, nullptr, 6, false, 0); // ssid, pwd, channel, hidden, max_cnx, 
  WiFi.setOutputPower(20.5); // max 20.5dBm
  
  softap_config current_config;
  wifi_softap_get_config(&current_config);

  current_config.beacon_interval = 1000;
  wifi_softap_set_config(&current_config);
  
  Serial.println();
  Serial.println("Started \\o/");
  Serial.println();

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
//-------Config RATE = 500 ms
    Serial.println("Configure GPS RATE = 500 ms");
    Rate500();
    delay(100); // Little delay before flushing.
    softSerial.flush();
    //--------Config CHANNELS 
    Serial.println("Configure GPS CHANNELS = GPS + Galileo + Glonas");   
    SelectChannels();
    delay(100); // Little delay before flushing.
    softSerial.flush();
     								
  drone_idfr.set_drone_id(drone_id); 

  delay(5000);
}

/**
 * Début du code principal. C'est une boucle infinie.
 */
void loop()
{
    static uint64_t gpsMap = 0;
        
    switch (program) {
    case 0:
        // Ici on lit les données qui arrivent du GPS et on les passe à la librairie TinyGPS++ pour les traiter
        while (softSerial.available())
            gps.encode(softSerial.read());
        // On traite le cas où le GPS a un problème
        if (millis() > 5000 && gps.charsProcessed() < 10) {
            Serial.println("No GPS detected");
            return;
        }
        // On traite le cas si la position GPS n'est pas valide
        if (!gps.location.isValid()) {
            if (millis() - gpsMap > 1000) {
                Serial.print("Waiting... SAT=");  Serial.println(gps.satellites.value());
                buzz(BUZZER_PIN_P, 2500, 10);//tick
                //Flip internal LED
                flip_Led();
                gpsMap = millis();
            }
            return;
        } else if (gps.location.isUpdated() && gps.altitude.isUpdated() && gps.course.isUpdated() && gps.speed.isUpdated()){
            // On traite le cas où la position GPS est valide.
            // On renseigne le point de démarrage quand la précision est satisfaisante
            if ( gps.satellites.value() > nb_sat ) {//+ de satellites
            buzz(BUZZER_PIN_P, 6000, 100);
            delay (100);
            nb_sat = gps.satellites.value();
            Serial.print("Waiting... SAT=");  Serial.println(gps.satellites.value());
            }
            if (!drone_idfr.has_home_set() && gps.satellites.value() > 5 && gps.hdop.hdop() < 2.0) {
                Serial.println("Setting Home Position");
                drone_idfr.set_home_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
                buzz(BUZZER_PIN_P, 7000, 200);
                delay (50);
                buzz(BUZZER_PIN_P, 7000, 200);
                delay (50);
                buzz(BUZZER_PIN_P, 7000, 200);
            }

            // On actualise les données GPS de la librairie d'identification drone.
            drone_idfr.set_current_position(gps.location.lat(), gps.location.lng(), gps.altitude.meters());
            drone_idfr.set_heading(gps.course.deg());
            drone_idfr.set_ground_speed(gps.speed.mps());

            /*
            float gps_time_elapsed = (float(millis() - gpsMap) / 1000.0f); 
            gpsMap = millis();
        
            Serial.println(gps_time_elapsed,3);
            */
            
            // Ici on affiche les données GPS pour sur le port Serie, toutes les secondes.
            // Décommenter ce block pour voir les informations détaillées sur le port série
            /*if (millis() - gpsMap > 1000) {         
                Serial.print("LAT=");  Serial.print(gps.location.lat(), 6); Serial.print(" LONG="); Serial.print(gps.location.lng(), 6);
                Serial.print(" ALT=");  Serial.print(gps.altitude.meters());  Serial.print(" SAT=");  Serial.print(gps.satellites.value());
                Serial.print(" DOP=");  Serial.print(gps.hdop.hdop());  Serial.print(" DISTfromLAST_POS_Sent=");  Serial.println(drone_idfr.get_distance_from_last_position_sent());
                gpsMap = millis();
            }*/
        }
        break;
    case 1:
        // Envoi données GPS de test
        drone_idfr.set_current_position(43.1234, -0.1234, 223);
        drone_idfr.set_heading(234);
        drone_idfr.set_ground_speed(45);

        drone_idfr.set_home_position(43.1234, -0.1234, 100);
        
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
        
        Serial.print(time_elapsed,1); Serial.print("s Send beacon: "); Serial.print(drone_idfr.has_pass_distance() ? "Distance" : "Time");
        Serial.print(" with ");  Serial.print(drone_idfr.get_distance_from_last_position_sent()); Serial.print("m Speed="); Serial.println(drone_idfr.get_ground_speed_kmh()); 
        
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
        // Décommenter ce block pour voir la trame entière sur le port série
        /* Serial.println("beaconPacket : ");
        for (auto i=0; i<sizeof(beaconPacket);i++) {
            Serial.print(beaconPacket[i], HEX);
            Serial.print(" ");
        }
        Serial.println(" ");
        */
        
        /**
         * On envoie la trame
         */
         wifi_send_pkt_freedom(beaconPacket, to_send, 0);
         //Flip internal LED
         flip_Led();
       
        /**
         * On reset la condition d'envoi
         */
        drone_idfr.set_last_send();
    }
}
