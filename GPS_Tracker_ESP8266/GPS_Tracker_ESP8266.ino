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
 * Adapté pour fonctionner avec un ESP01 (512k, 1M) et https://github.com/esp8266/Arduino
 */

 
// ======= Paramètres balise esp32 ** à modifier ** ============= //
/**
  * Le nom du point d'acces wifi CHANGEZ LE par ce que vous voulez !!!
  */
const char ssid[] = "ILLEGAL_DRONE_AP";
/**
  * CHANGEZ l'ID du drone par celui que Alphatango vous a fourni (Trigramme + Modèle + numéro série) !
  */
                       // "000000000000000000000000000000"  // 30 caractères
const char drone_id[31] = "ILLEGAL_DRONE_APPELEZ_POLICE17"; // si l'id est inférieur à 30 caractères, le compléter avec des "0"

// =========== Includes ======================= //
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>   
#include "droneID_FR.h"

#include <TinyGPS++.h> 

#define GPS_BAUD_RATE 9600 // ou 19200 avec refresh rate à 5Hz (le GPS doit stocker la configuration dans ce cas)
#define GPS_RX_PIN 5       // Brancher le fil Tx du GPS
#define GPS_TX_PIN 4       // Brancher le fil Rx du GPS

#define ENABLE_BUZZER
#define BUZZER_PIN         0


extern "C" {
#include "user_interface.h"
  int wifi_send_pkt_freedom(uint8 *buf, int len, bool sys_seq);
}

// ========================================================== //

SoftwareSerial softSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

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
  /* 34 - 35 */ 0x21, 0x04, // capabilities Information

  // Tagged parameters

  // SSID parameters
  /* 36 - 38 */ 0x03, 0x01, 0x06, // DS Parameter set, current channel 6 (= 0x06), // TODO: manually set it
  /* 39 - 40 */ 0x00, 0x20,       // 39-40: SSID parameter set, 0x20:maxlength:content
};


// Vérification ssid max 30 
static_assert((sizeof(ssid)/sizeof(*ssid))<=32, "AP SSID should be less than 32 letters");
// Vérification drone_id max 30 
static_assert((sizeof(drone_id)/sizeof(*drone_id))<=31, "Drone ID should be less that 30 letters !");  // 30 lettres + null termination

bool has_set_home = false;
double home_alt = 0.0;

int nb_sat = 0;

uint8_t program = 0;
char buff[5][256];
uint64_t gpsSec = 0;

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

void setup() {
  // start serial
  Serial.begin(115200);
  delay(1000);

#ifdef ENABLE_BUZZER
  pinMode(BUZZER_PIN, OUTPUT); // set a pin for buzzer output
#endif

  buzz(BUZZER_PIN, 2500, 100);
   
   // start WiFi
  WiFi.mode(WIFI_OFF);
  
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

  softSerial.begin(GPS_BAUD_RATE);
 
  drone_idfr.set_drone_id(drone_id); 

  delay(3000);
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

                buzz(BUZZER_PIN, 2500, 10);//tick

                gpsMap = millis();
            }

            return;
            
        } else {
            // On traite le cas où la position GPS est valide.
            // On renseigne le point de démarrage quand la précision est satisfaisante
            //Serial.println("GPS detected");
            if ( gps.satellites.value() > nb_sat ) {//+ de satellites
              buzz(BUZZER_PIN, 6000, 100);
              delay (100);
              nb_sat = gps.satellites.value();
             }
            if (!has_set_home && gps.satellites.value() > 6 && gps.hdop.hdop() < 2.0) {
                Serial.println("Setting Home Position");
                Serial.println("Setting Home Position");
                buzz(BUZZER_PIN, 7000, 200);
                delay (50);
                buzz(BUZZER_PIN, 7000, 200);
                delay (50);
                buzz(BUZZER_PIN, 7000, 200);
                drone_idfr.set_home_lat_lon(gps.location.lat(), gps.location.lng());
                has_set_home = true;
                home_alt = gps.altitude.meters();
            }
            // On envoie les données à la librairie d'identification drone pour le formatage.
            drone_idfr.set_lat_lon(gps.location.lat(), gps.location.lng());
            drone_idfr.set_altitude(gps.altitude.meters());
            drone_idfr.set_heading(gps.course.deg());
            drone_idfr.set_ground_speed(gps.speed.mps());
            drone_idfr.set_heigth(gps.altitude.meters() - home_alt);
            // Ici on ecrit sur le port Serie des données GPS pour visualisation seulement.
            if (millis() - gpsMap > 1000) {         
                Serial.print("LAT=");  Serial.print(gps.location.lat(), 6); Serial.print(" LONG="); Serial.print(gps.location.lng(), 6);
                Serial.print(" ALT=");  Serial.print(gps.altitude.meters());  Serial.print(" SAT=");  Serial.println(gps.satellites.value());
                
                gpsMap = millis();
            }
        }
        break;
    }


    /**
     * On regarde s'il est temps d'envoyer la trame d'identification drone : soit toutes les 3s soit si le drone s'est déplacé de 30m en moins de 3s.
     */            
     if (drone_idfr.time_to_send()) {
        Serial.println("Send beacon");
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
        // Décommenter ce block pour voir la trame entière sur le port usb
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
        
        /**
         * On reset la condition d'envoi
         */
        drone_idfr.set_last_send();
    }

}
