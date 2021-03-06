/* 
 *  
 * https://github.com/loginov-rocks/UbxGps/blob/master/extras/Configuration/Auto-configuration-Mega/Auto-configuration-Mega.ino
 * 
 *  
 *  Pour trouver la séquence d'octets liée à une fonction, on utilise les écrans se trouvant sous "Message View" -> UBX -> CFG 
 *  En cliquant le bouton Show Hex Toggle ( avant dernier en bas à gauche ), on ouvre une fenêtre contenant les octets correspondant à une configuration
 *  ce bouton reste grisé si l'on accède aux écrans depuis "Config View". 
 *  Exemple de séquence:
 *  RATES 500: B5 62 06 08 06 00 F4 01 01 00 01 00 0B 77 
 *  RATES 100: B5 62 06 08 06 00 64 00 01 00 01 00 7A 12 
 *
 *  32 UBX Protocol p168 ->  https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29.pdf
 *
 * Notes : 
 * La mise à jour d'un écran de paramètres affichés par u-center grâce à la touche Poll : 
 * le titre du message choisi, dans la colonne de gauche, change brièvement de couleur quand il est mis à jour. 
 * 
 * La configuration n'est pas sauvegardée dans la flash, le but est de mettre au point une séquence de configuration
 *       
 * 
*/

#include <Arduino.h>
#include <SoftwareSerial.h> // Include the SoftwareSerial library
#define GPS_RX_PIN 5       //D1 Brancher le fil Tx du GPS
#define GPS_TX_PIN 4       //D2 Brancher le fil Rx du GPS
SoftwareSerial GPS_SERIAL(GPS_RX_PIN, GPS_TX_PIN);
#define GPS_BAUDRATE 9600

#define PC_SERIAL Serial
#define PC_BAUDRATE 115200


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

void Rate500()
{     
  byte packet[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xF4, 0x01, 0x01, 0x00, 0x01, 0x00, 0x0B, 0x77};
    sendPacket(packet, sizeof(packet));
}

void Rate200()
{     
  byte packet[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
    sendPacket(packet, sizeof(packet));
}

void Rate100()
{
    byte packet[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00,0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
    sendPacket(packet, sizeof(packet));
}

// Send the packet specified to the receiver.
void sendPacket(byte *packet, byte len)
{
    for (byte i = 0; i < len; i++)
    {
        GPS_SERIAL.write(packet[i]);
    }

    printPacket(packet, len);
}

// Print the packet specified to the PC serial in a hexadecimal form.
void printPacket(byte *packet, byte len)
{
    char temp[3];

    for (byte i = 0; i < len; i++)
    {
        sprintf(temp, "%.2X", packet[i]);
        PC_SERIAL.print(temp);

        if (i != len - 1)
        {
            PC_SERIAL.print(' ');
        }
    }
    
    PC_SERIAL.println();
}

void setup()
{
    GPS_SERIAL.begin(GPS_BAUDRATE);
    delay(100); // Little delay before flushing.
    GPS_SERIAL.flush();

    PC_SERIAL.begin(PC_BAUDRATE);
    delay(100); // Little delay before flushing.
    PC_SERIAL.flush();
    PC_SERIAL.println();
    PC_SERIAL.println();
    PC_SERIAL.println("Phase de configuration...");

//--------------------------------------------- RATE
    PC_SERIAL.println("RATE 500");
    //Rate100();
    //Rate200();
    Rate500();
    delay(100); // Little delay before flushing.
    GPS_SERIAL.flush();
    
//--------------------------------------------- CHANNELS    
    PC_SERIAL.println("CHANNELS GPS + Galileo + Glonas");
    SelectChannels();
    delay(100); // Little delay before flushing.
    GPS_SERIAL.flush();
    
//--------------------------------------------- FIN  
    PC_SERIAL.println("Configuration terminée!");
    PC_SERIAL.println("Phase BOUCLE PASSTHROUGH fermer la console et connecter u-center sur le même port");
    delay(5000);
}

void loop()
{
    if (GPS_SERIAL.available())
    {
        PC_SERIAL.write(GPS_SERIAL.read());
    }

    if (PC_SERIAL.available())
    {
        GPS_SERIAL.write(PC_SERIAL.read());
    }
}
