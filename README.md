#### Préambule

Durant la mise au point des balises, voici un décodeur/afficheur de trames beacons bien pratique basé sur une carte TTGO T-Display ESP32 décrite ici http://www.lilygo.cn/prod_view.aspx?TypeId=50033&Id=1126&FId=t3:50033:3

<img width = "400" src="img/TTGOCapture1.png" />

https://github.com/dev-fred/Decode_balise_ESP32/tree/master/Decode_Balise_ESP32_TFT


# GPS_Tracker_ESP8266V1  

### [Code](GPS_Tracker_ESP8266V1)

<img src="GPS_Tracker_ESP8266V1/img/Balise.jpg" width = "300"> 


Balise basé sur https://github.com/f5soh/balise_esp32 et adapté pour un ESP8266 D1 mini.

Fonctionne avec un buzzer actif optionnel et un GPS BN-180 ou un BN-220, pèse 11g.

# GPS_Tracker_ESP8266V1_FRSKY

### [Code](GPS_Tracker_ESP8266V1_FRSKY)

Battit au-dessus de GPS_Tracker_ESP8266V1, cette version ajoute une sortie FRSKY S.port sur le connecteur JR d'alimentation afin d'envoyer les données du GPS de la balise à un récepteur FRSKY via la télémétrie que l'on pourra afficher avec un script LUA.
Le gros avantage c’est la réception des infos GPS à travers la télémétrie qui passe par le couple émetteur/récepteur qui a une bien meilleure portée que le couple Balise/Smartphone ce qui permet à la balise de rendre un service de localisation assez fiable en cas de perte de l’appareil.

<img src="GPS_Tracker_ESP8266V1_FRSKY/img/Capture Balise0_censored.jpg" width = "600">


# GPS_Tracker_ESP8266V1_WEB

### [Code](GPS_Tracker_ESP8266V1_WEB)

Ajoute un serveur WEB dans la balise qui permet de recevoir en même temps que la trame est émise, les données GPS sur son smartphone via un navigateur. Il faudra au préalabre se connecter sur l'adresse IP de ce serveur.

<img src="GPS_Tracker_ESP8266V1_WEB/img/smart.PNG" width = "300">

# GPS_Tracker_ESP8266V1_WEB_FRSKY

### [Code](GPS_Tracker_ESP8266V1_WEB_FRSKY)

<img src="GPS_Tracker_ESP8266V1_WEB_FRSKY/img/Balise-S6R.PNG" width = "300">

Battit au-dessus de GPS_Tracker_ESP8266V1_WEB, cette version ajoute une sortie FRSKY S.port sur le connecteur JR d'alimentation afin d'envoyer les données du GPS de la balise à un récepteur FRSKY via la télémétrie que l'on pourra afficher avec un script LUA.
Le gros avantage c’est la réception des infos GPS à travers la télémétrie qui passe par le couple émetteur/récepteur qui a une bien meilleure portée que le couple Balise/Smartphone ce qui permet à la balise de rendre un service de localisation assez fiable en cas de perte de l’appareil.
Il est possible de cascader ce dispositif avec un module de télémétrie Frsky/Sport comme un Vario par exemple; dans ce cas il faut insérer une résistance de 1,5K entre la sortie D8 du signal sport et le connecteur JST pour protéger la sortie de l'esp8266.

<img src="GPS_Tracker_ESP8266V1_WEB_FRSKY/img/Capture Balise0_censored.jpg" width = "600">

# GPS_Tracker_ESP8266V1_WEB_FRSKY_OTA

### [Code](GPS_Tracker_ESP8266V1_WEB_FRSKY_OTA)

<img src="GPS_Tracker_ESP8266V1_WEB_FRSKY_OTA/img/C1.PNG" width = "300">

Battit au-dessus de GPS_Tracker_ESP8266V1_WEB_FRSKY, cette version ajoute une fonction **OTA** (Over The Air) de mise à jour du logiciel à travers la liaison WiFi, très utile pour une implémentation du projet sur un ESP01.

# GPS_Tracker_ESP8266V1_WEB_FRSKY_OSD-DJI-AIR-UNIT

### [Code](GPS_Tracker_ESP8266V1_WEB_FRSKY_OSD-DJI-AIR-UNIT)

# GPS_Tracker_ESP8266V1_WEB_FRSKY_OSD-DJI-OTA-ESP-01S
### [Code](GPS_Tracker_ESP8266V1_WEB_FRSKY_OSD-DJI-OTA-ESP-01S)

<img src="GPS_Tracker_ESP8266V1_WEB_FRSKY_OSD-DJI-AIR-UNIT/img/GOOGLES.PNG" width = "600">

Battit au-dessus de GPS_Tracker_ESP8266V1_WEB_FRSKY, cette version ajoute une sortie MSP vers un 
DJI FPV Air Unit ou Caddx Vista, ce qui permet d'afficher dans l'OSD les infos provenant du GPS de la balise :
* La latitude, la longitude
* L'altitude relative
* Le nombre de satellites (clignote tant que la position de départ n'est pas définie)
* La vitesse sol
* La direction du point de départ
* La distance au point de départ
* La tension d'une cellule de la batterie avec une alarme en rouge si < 3,6V

La partie FRSKY, également connectée, envoie les données de télémesure à la radio.


# GPS_Tracker_ESP8266V1_MAP

### [Code](GPS_Tracker_ESP8266V1_MAP)

Projet déconseillé, il ne tient pas ses promesses; la portée de la balise et la fréquence d'échantillonage des positions GPS sont trés insuffisantes.
