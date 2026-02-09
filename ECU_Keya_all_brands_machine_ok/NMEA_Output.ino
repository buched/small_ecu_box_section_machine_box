//FlorianT NMEA OUT - Version avec Menu de Configuration
// Fichier pour générer une nouvelle trame NMEA 0183 corrigée en dévers parametrable

#include <math.h>

/************************* NMEA 0183 User Settings *************************/
// Hauteur de l'antenne pour la correction en dévers (Antenna Phase Center Height)
float ANTENNA_HEIGHT_M = 3.0; 

// Configuration du port série de sortie NMEA 0183
Stream* NmeaOutputSerial = &Serial8; // Serial : PORT USB ou exemple : Serial8
long baudNMEA = 38400;       

// Messages NMEA à envoyer (paramétrable)
bool SEND_GPGGA = true;
bool SEND_GPVTG = true;
bool SEND_GPRMC = true;
bool SEND_GPZDA = true;

// Fréquences d'envoi indépendantes pour chaque message (en Hz)
float GPGGA_FREQUENCY_HZ = 10.0;  // 10 Hz
float GPVTG_FREQUENCY_HZ = 10.0;  // 10 Hz
float GPRMC_FREQUENCY_HZ = 10.0;  // 10 Hz
float GPZDA_FREQUENCY_HZ = 1.0;   // 1 Hz

// Variables pour les intervalles (seront recalculées)
uint32_t GPGGA_INTERVAL_MS = 100;
uint32_t GPVTG_INTERVAL_MS = 100;
uint32_t GPRMC_INTERVAL_MS = 100;
uint32_t GPZDA_INTERVAL_MS = 1000;

/************************* Menu Configuration *************************/
bool menuActive = false;
String inputBuffer = "";

void recalculateIntervals() {
    GPGGA_INTERVAL_MS = (uint32_t)(1000.0 / GPGGA_FREQUENCY_HZ);
    GPVTG_INTERVAL_MS = (uint32_t)(1000.0 / GPVTG_FREQUENCY_HZ);
    GPRMC_INTERVAL_MS = (uint32_t)(1000.0 / GPRMC_FREQUENCY_HZ);
    GPZDA_INTERVAL_MS = (uint32_t)(1000.0 / GPZDA_FREQUENCY_HZ);
}

void displayMenu() {
    Serial.println(F("\n╔════════════════════════════════════════════════╗"));
    Serial.println(F("║      MENU CONFIGURATION NMEA OUTPUT            ║"));
    Serial.println(F("╠════════════════════════════════════════════════╣"));
    Serial.println(F("║  1. Activer/Désactiver messages                ║"));
    Serial.println(F("║  2. Régler fréquences (Hz)                     ║"));
    Serial.println(F("║  3. Régler Baudrate                            ║"));
    Serial.println(F("║  4. Régler hauteur antenne                     ║"));
    Serial.println(F("║  5. Afficher configuration actuelle            ║"));
    Serial.println(F("║  6. Sauvegarder config (EEPROM)                ║"));
    Serial.println(F("║  7. Charger config (EEPROM)                    ║"));
    Serial.println(F("║  0. Quitter le menu                            ║"));
    Serial.println(F("╚════════════════════════════════════════════════╝"));
    Serial.print(F("Choix: "));
}

void displayCurrentConfig() {
    Serial.println(F("\n╔════════════════════════════════════════════════╗"));
    Serial.println(F("║      CONFIGURATION ACTUELLE                    ║"));
    Serial.println(F("╠════════════════════════════════════════════════╣"));
    Serial.print(F("║ Baudrate: ")); Serial.print(baudNMEA); Serial.println(F(" bps"));
    Serial.print(F("║ Hauteur antenne: ")); Serial.print(ANTENNA_HEIGHT_M, 2); Serial.println(F(" m"));
    Serial.println(F("║                                                ║"));
    Serial.println(F("║ Messages actifs:                               ║"));
    Serial.print(F("║   GPGGA: ")); Serial.print(SEND_GPGGA ? "ON " : "OFF"); 
    Serial.print(F(" @ ")); Serial.print(GPGGA_FREQUENCY_HZ, 1); Serial.println(F(" Hz"));
    Serial.print(F("║   GPVTG: ")); Serial.print(SEND_GPVTG ? "ON " : "OFF"); 
    Serial.print(F(" @ ")); Serial.print(GPVTG_FREQUENCY_HZ, 1); Serial.println(F(" Hz"));
    Serial.print(F("║   GPRMC: ")); Serial.print(SEND_GPRMC ? "ON " : "OFF"); 
    Serial.print(F(" @ ")); Serial.print(GPRMC_FREQUENCY_HZ, 1); Serial.println(F(" Hz"));
    Serial.print(F("║   GPZDA: ")); Serial.print(SEND_GPZDA ? "ON " : "OFF"); 
    Serial.print(F(" @ ")); Serial.print(GPZDA_FREQUENCY_HZ, 1); Serial.println(F(" Hz"));
    Serial.println(F("╚════════════════════════════════════════════════╝\n"));
}

void submenuMessages() {
    Serial.println(F("\n=== ACTIVATION/DÉSACTIVATION MESSAGES ==="));
    Serial.println(F("1. GPGGA (Position) - Actuellement: ") + String(SEND_GPGGA ? "ON" : "OFF"));
    Serial.println(F("2. GPVTG (Vitesse)  - Actuellement: ") + String(SEND_GPVTG ? "ON" : "OFF"));
    Serial.println(F("3. GPRMC (Position+Vitesse) - Actuellement: ") + String(SEND_GPRMC ? "ON" : "OFF"));
    Serial.println(F("4. GPZDA (Date/Heure) - Actuellement: ") + String(SEND_GPZDA ? "ON" : "OFF"));
    Serial.println(F("0. Retour"));
    Serial.print(F("Choix: "));
    
    while (!Serial.available()) { delay(10); }
    int choice = Serial.parseInt();
    while (Serial.available()) Serial.read();
    
    switch(choice) {
        case 1: SEND_GPGGA = !SEND_GPGGA; Serial.println(SEND_GPGGA ? "GPGGA ON" : "GPGGA OFF"); break;
        case 2: SEND_GPVTG = !SEND_GPVTG; Serial.println(SEND_GPVTG ? "GPVTG ON" : "GPVTG OFF"); break;
        case 3: SEND_GPRMC = !SEND_GPRMC; Serial.println(SEND_GPRMC ? "GPRMC ON" : "GPRMC OFF"); break;
        case 4: SEND_GPZDA = !SEND_GPZDA; Serial.println(SEND_GPZDA ? "GPZDA ON" : "GPZDA OFF"); break;
    }
}

void submenuFrequencies() {
    Serial.println(F("\n=== RÉGLAGE FRÉQUENCES (Hz) ==="));
    Serial.println(F("1. GPGGA - Actuellement: ") + String(GPGGA_FREQUENCY_HZ, 1) + F(" Hz"));
    Serial.println(F("2. GPVTG - Actuellement: ") + String(GPVTG_FREQUENCY_HZ, 1) + F(" Hz"));
    Serial.println(F("3. GPRMC - Actuellement: ") + String(GPRMC_FREQUENCY_HZ, 1) + F(" Hz"));
    Serial.println(F("4. GPZDA - Actuellement: ") + String(GPZDA_FREQUENCY_HZ, 1) + F(" Hz"));
    Serial.println(F("0. Retour"));
    Serial.print(F("Choix: "));
    
    while (!Serial.available()) { delay(10); }
    int choice = Serial.parseInt();
    while (Serial.available()) Serial.read();
    
    if (choice >= 1 && choice <= 4) {
        Serial.print(F("Nouvelle fréquence (0.1 à 50 Hz): "));
        while (!Serial.available()) { delay(10); }
        float newFreq = Serial.parseFloat();
        while (Serial.available()) Serial.read();
        
        if (newFreq >= 0.1 && newFreq <= 50.0) {
            switch(choice) {
                case 1: GPGGA_FREQUENCY_HZ = newFreq; break;
                case 2: GPVTG_FREQUENCY_HZ = newFreq; break;
                case 3: GPRMC_FREQUENCY_HZ = newFreq; break;
                case 4: GPZDA_FREQUENCY_HZ = newFreq; break;
            }
            recalculateIntervals();
            Serial.println(F("Fréquence mise à jour!"));
        } else {
            Serial.println(F("Valeur invalide!"));
        }
    }
}

void submenuBaudrate() {
    Serial.println(F("\n=== RÉGLAGE BAUDRATE ==="));
    Serial.println(F("Baudrate actuel: ") + String(baudNMEA));
    Serial.println(F("\nBaudrates standards:"));
    Serial.println(F("1. 4800"));
    Serial.println(F("2. 9600"));
    Serial.println(F("3. 19200"));
    Serial.println(F("4. 38400"));
    Serial.println(F("5. 57600"));
    Serial.println(F("6. 115200"));
    Serial.println(F("7. Personnalisé"));
    Serial.println(F("0. Retour"));
    Serial.print(F("Choix: "));
    
    while (!Serial.available()) { delay(10); }
    int choice = Serial.parseInt();
    while (Serial.available()) Serial.read();
    
    long newBaud = 0;
    switch(choice) {
        case 1: newBaud = 4800; break;
        case 2: newBaud = 9600; break;
        case 3: newBaud = 19200; break;
        case 4: newBaud = 38400; break;
        case 5: newBaud = 57600; break;
        case 6: newBaud = 115200; break;
        case 7:
            Serial.print(F("Entrer baudrate personnalisé: "));
            while (!Serial.available()) { delay(10); }
            newBaud = Serial.parseInt();
            while (Serial.available()) Serial.read();
            break;
    }
    
    if (newBaud > 0) {
        baudNMEA = newBaud;
        if (NmeaOutputSerial != NULL && NmeaOutputSerial != &Serial) {
            ((HardwareSerial*)NmeaOutputSerial)->end();
            ((HardwareSerial*)NmeaOutputSerial)->begin(baudNMEA);
        }
        Serial.println(F("Baudrate mis à jour!"));
    }
}

void submenuAntennaHeight() {
    Serial.println(F("\n=== HAUTEUR ANTENNE ==="));
    Serial.print(F("Hauteur actuelle: ")); Serial.print(ANTENNA_HEIGHT_M, 2); Serial.println(F(" m"));
    Serial.print(F("Nouvelle hauteur (0.1 à 10.0 m): "));
    
    while (!Serial.available()) { delay(10); }
    float newHeight = Serial.parseFloat();
    while (Serial.available()) Serial.read();
    
    if (newHeight >= 0.1 && newHeight <= 10.0) {
        ANTENNA_HEIGHT_M = newHeight;
        Serial.println(F("Hauteur mise à jour!"));
    } else {
        Serial.println(F("Valeur invalide!"));
    }
}

#define EEPROM_NMEA_ADDR 500  // Adresse de départ dans l'EEPROM

struct NmeaConfig {
    uint16_t signature = 0xABCD;  // Signature pour vérifier si config valide
    float antennaHeight;
    long baudrate;
    bool sendGGA;
    bool sendVTG;
    bool sendRMC;
    bool sendZDA;
    float freqGGA;
    float freqVTG;
    float freqRMC;
    float freqZDA;
};

void saveConfigToEEPROM() {
    NmeaConfig config;
    config.antennaHeight = ANTENNA_HEIGHT_M;
    config.baudrate = baudNMEA;
    config.sendGGA = SEND_GPGGA;
    config.sendVTG = SEND_GPVTG;
    config.sendRMC = SEND_GPRMC;
    config.sendZDA = SEND_GPZDA;
    config.freqGGA = GPGGA_FREQUENCY_HZ;
    config.freqVTG = GPVTG_FREQUENCY_HZ;
    config.freqRMC = GPRMC_FREQUENCY_HZ;
    config.freqZDA = GPZDA_FREQUENCY_HZ;
    
    EEPROM.put(EEPROM_NMEA_ADDR, config);
    Serial.println(F("✓ Configuration sauvegardée dans l'EEPROM!"));
}

void loadConfigFromEEPROM() {
    NmeaConfig config;
    EEPROM.get(EEPROM_NMEA_ADDR, config);
    
    if (config.signature == 0xABCD) {
        ANTENNA_HEIGHT_M = config.antennaHeight;
        baudNMEA = config.baudrate;
        SEND_GPGGA = config.sendGGA;
        SEND_GPVTG = config.sendVTG;
        SEND_GPRMC = config.sendRMC;
        SEND_GPZDA = config.sendZDA;
        GPGGA_FREQUENCY_HZ = config.freqGGA;
        GPVTG_FREQUENCY_HZ = config.freqVTG;
        GPRMC_FREQUENCY_HZ = config.freqRMC;
        GPZDA_FREQUENCY_HZ = config.freqZDA;
        
        recalculateIntervals();
        
        if (NmeaOutputSerial != NULL && NmeaOutputSerial != &Serial) {
            ((HardwareSerial*)NmeaOutputSerial)->end();
            ((HardwareSerial*)NmeaOutputSerial)->begin(baudNMEA);
        }
        
        Serial.println(F("✓ Configuration chargée depuis l'EEPROM!"));
    } else {
        Serial.println(F("✗ Aucune configuration valide trouvée dans l'EEPROM"));
    }
}

void processMenu() {
    if (!menuActive) return;
    
    if (Serial.available()) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (inputBuffer.length() > 0) {
                int choice = inputBuffer.toInt();
                inputBuffer = "";
                
                Serial.println();
                
                switch(choice) {
                    case 0:
                        menuActive = false;
                        Serial.println(F("Menu fermé. Appuyez sur 'm' pour réouvrir."));
                        break;
                    case 1:
                        submenuMessages();
                        displayMenu();
                        break;
                    case 2:
                        submenuFrequencies();
                        displayMenu();
                        break;
                    case 3:
                        submenuBaudrate();
                        displayMenu();
                        break;
                    case 4:
                        submenuAntennaHeight();
                        displayMenu();
                        break;
                    case 5:
                        displayCurrentConfig();
                        displayMenu();
                        break;
                    case 6:
                        saveConfigToEEPROM();
                        displayMenu();
                        break;
                    case 7:
                        loadConfigFromEEPROM();
                        displayMenu();
                        break;
                    default:
                        Serial.println(F("Choix invalide!"));
                        displayMenu();
                }
            }
        } else if (c >= '0' && c <= '9') {
            inputBuffer += c;
            Serial.print(c);
        }
    }
}

void checkMenuActivation() {
    if (Serial.available()) {
        char c = Serial.peek();
        if (c == 'm' || c == 'M') {
            Serial.read();
            menuActive = true;
            displayMenu();
        }
    }
}

/************************* Variables Globales et Déclarations *************************/
// [Le reste du code reste identique...]
extern const char* asciiHex;
extern char fixTime[12];
extern char latitude[15];
extern char latNS[3];
extern char longitude[15];
extern char lonEW[3];
extern char fixQuality[2];
extern char numSats[4];
extern char HDOP[5];
extern char altitude[12];
extern char ageDGPS[10];
extern char rmcDate[7];
extern char rmcMagVar[6];
extern char rmcMagEW[3];
extern char vtgHeading[12]; 
extern char speedKnots[10];
extern char imuRoll[6];     
extern char imuPitch[6];    
extern bool GGA_Available;

elapsedMillis nmeaGgaTimer = 0;
elapsedMillis nmeaVtgTimer = 0;
elapsedMillis nmeaRmcTimer = 0;
elapsedMillis nmeaZdaTimer = 0;

char nmea0183Buffer[100]; 
const float EARTH_RADIUS_M = 6371000.0;

/************************* Fonctions d'Utilitaires *************************/

// Calcule et ajoute le checksum NMEA 0183 à la chaîne 'nmea'
void BuildChecksum0183(char *nmea)
{
  uint8_t sum = 0;
  char *p = nmea;
  
  if (*p == '$') p++; 

  while (*p != '\0')
  {
    sum ^= *p;
    p++;
  }
  
  *p++ = '*'; 

  uint8_t chk = (sum >> 4); 
  *p++ = asciiHex[chk];

  chk = (sum & 0x0F);
  *p++ = asciiHex[chk];

  *p++ = '\r';
  *p++ = '\n';
  *p = '\0'; 
}

// Convertit la position NMEA (ddmm.mmmm) en degrés décimaux (dd.ddddd)
double nmeaToDecimal(char* nmea_str, char ns_ew_char)
{
    char* endptr;
    double ddmm = strtod(nmea_str, &endptr);
    if (ddmm == 0.0) return 0.0;
    
    double degrees = floor(ddmm / 100.0);
    double minutes = ddmm - (degrees * 100.0);
    double decimal_degrees = degrees + (minutes / 60.0);
    
    if (ns_ew_char == 'S' || ns_ew_char == 'W') {
        return -decimal_degrees;
    }
    return decimal_degrees;
}

// Convertit la position décimale (dd.ddddd) en format NMEA (ddmm.mmmm) et définit le marqueur N/S ou E/W
void decimalToNMEA(double decimal_degrees, char* nmea_buf, char* ns_ew_buf, int min_prec, PositionType pos_type)
{
    // Si c'est l'altitude, on utilise le format simplifié habituel
    if (min_prec == 1) { 
        dtostrf(decimal_degrees, 6, min_prec, nmea_buf);
        return;
    }
    
    // --- 1. Détermination de la Direction (N/S ou E/W) ---
    if (decimal_degrees < 0) {
        *ns_ew_buf = (pos_type == LATITUDE) ? 'S' : 'W'; 
        decimal_degrees = -decimal_degrees;
    } else {
        *ns_ew_buf = (pos_type == LATITUDE) ? 'N' : 'E'; 
    }
    *(ns_ew_buf + 1) = '\0';

    // --- 2. Séparation des parties (Degrés, Minutes entières, Minutes fractionnaires) ---
    int deg = (int)floor(decimal_degrees); // Partie entière des degrés (ex: 1 pour 1.86°)
    double minutes_float = (decimal_degrees - deg) * 60.0; // Minutes avec décimales (ex: 51.6')
    
    int min_int = (int)floor(minutes_float); // Partie entière des minutes (ex: 51)
    
    // Calcul de la partie fractionnaire des minutes (4 décimales pour NMEA)
    // min_prec est 4 pour Lat/Lon
    int min_frac = (int)round((minutes_float - min_int) * pow(10.0, min_prec)); 
    
    // --- 3. Formatage avec sprintf pour garantir les zéros de début ---
    // Cette méthode est la seule à garantir les zéros de début (%0xd)

    if (pos_type == LATITUDE) 
    {
        // LAT: ddmm.mmmm
        // %02d : 2 chiffres de degrés avec zéro si nécessaire (ex: 43)
        // %02d : 2 chiffres de minutes avec zéro si nécessaire (ex: 46)
        // %04d : 4 décimales de minutes avec zéros si nécessaire
        sprintf(nmea_buf, "%02d%02d.%04d", deg, min_int, min_frac);
    } 
    else // LONGITUDE
    { 
        // LON: dddmm.mmmm
        // %03d : 3 chiffres de degrés avec zéros de début (ex: 001) <--- CORRECTION CLÉ
        // %02d : 2 chiffres de minutes
        // %04d : 4 décimales
        sprintf(nmea_buf, "%03d%02d.%07d", deg, min_int, min_frac);
    }
}

/************************* Correction Dévers (Roll/Pitch) *************************/

// Applique la correction de position (Lat/Lon) et d'altitude due au roulis (Roll) et au tangage (Pitch).
double applyRollPitchCorrection(double& lat_deg, double& lon_deg, float heading_deg)
{
    // Roulage (Roll) et Tangage (Pitch) en radians
    char* endptr;

    // --- Conversion avec division par 10 (NÉCESSAIRE car imuRoll/Pitch est angle * 10) ---
    float roll_deg_x10 = strtod(imuRoll, &endptr);
    float roll_deg = roll_deg_x10 / 10.0; // Correction: Angle en degrés réels
    float roll_rad = roll_deg * (PI / 180.0);
    
    float pitch_deg_x10 = strtod(imuPitch, &endptr);
    float pitch_deg = pitch_deg_x10 / 10.0; // Correction: Angle en degrés réels
    float pitch_rad = pitch_deg * (PI / 180.0);

    
    // Direction (Heading) en radians
    float heading_deg_clamped = strtod(vtgHeading, &endptr);
    float heading_rad = heading_deg_clamped * (PI / 180.0);

    // 1. Correction d'altitude (projection verticale)
    double alt_correction_m = ANTENNA_HEIGHT_M * (1.0 - (cos(roll_rad) * cos(pitch_rad)));
    
    // 2. Décalage horizontal (Latéral et Longitudinal)
    double dY_m = ANTENNA_HEIGHT_M * sin(roll_rad); // Décalage latéral (Cross-track)
    double dX_m = ANTENNA_HEIGHT_M * sin(pitch_rad); // Décalage longitudinal (Along-track)

    // Composantes Nord (dN_m) et Est (dE_m) du décalage
    double dN_m = (dY_m * sin(heading_rad)) + (dX_m * cos(heading_rad));
    double dE_m = (-dY_m * cos(heading_rad)) + (dX_m * sin(heading_rad));

    // 3. Conversion de mètres en degrés de Lat/Lon
    double meters_per_deg_lat = EARTH_RADIUS_M * (PI / 180.0); 
    double meters_per_deg_lon = meters_per_deg_lat * cos(lat_deg * (PI / 180.0));

    double dLat_deg = dN_m / meters_per_deg_lat;
    double dLon_deg = dE_m / meters_per_deg_lon;

    // 4. Application de la correction : on SOUSTRAIT le décalage 
    lat_deg -= dLat_deg; 
    lon_deg -= dLon_deg; 
    
    return alt_correction_m;
}

/************************* Constructeurs de Trames NMEA *************************/

// Construit et envoie la trame GPGGA (corrigée)
void BuildGGA0183()
{
    if (!GGA_Available || !SEND_GPGGA) return;
    
    char* endptr;
    
    // 1. Préparation des données
    double lat_deg = nmeaToDecimal(latitude, latNS[0]);  // Get first character
    double lon_deg = nmeaToDecimal(longitude, lonEW[0]); // Get first character
    double alt_m = strtod(altitude, &endptr);
    float heading_deg = strtod(vtgHeading, &endptr);
    
    // 2. Application de la Correction
    double alt_correction_m = applyRollPitchCorrection(lat_deg, lon_deg, heading_deg);
    double corrected_alt_m = alt_m - alt_correction_m;

    // 3. Conversion des données corrigées en format NMEA
    char corrected_lat_nmea[15];
    char corrected_lon_nmea[15];
    char corrected_latNS[3];
    char corrected_lonEW[3];
    char corrected_alt_str[12];
    
    // Utilisation de LATITUDE et LONGITUDE pour corriger la direction N/S/E/W + Choix du nombre de décimales
    decimalToNMEA(lat_deg, corrected_lat_nmea, corrected_latNS, 7, LATITUDE); 
    decimalToNMEA(lon_deg, corrected_lon_nmea, corrected_lonEW, 7, LONGITUDE); 
    decimalToNMEA(corrected_alt_m, corrected_alt_str, NULL, 1, LATITUDE); 

    // 4. Construction de la trame
    strcpy(nmea0183Buffer, "$GPGGA,");
    strcat(nmea0183Buffer, fixTime);         strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, corrected_lat_nmea); strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, corrected_latNS);  strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, corrected_lon_nmea); strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, corrected_lonEW);  strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, fixQuality);       strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, numSats);          strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, HDOP);             strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, corrected_alt_str); strcat(nmea0183Buffer, ",M,");
    strcat(nmea0183Buffer, "46.9");           strcat(nmea0183Buffer, ",M,"); 
    strcat(nmea0183Buffer, ageDGPS);          strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, "");             
    
    // 5. Checksum et Envoi
    BuildChecksum0183(nmea0183Buffer);
    NmeaOutputSerial->print(nmea0183Buffer);
}

// Construit et envoie la trame GPVTG
void BuildVTG0183()
{
    if (!GGA_Available || !SEND_GPVTG) return;
    
    char* endptr;
    
    // Conversion Nœuds (speedKnots) en Km/h (speedKnots * 1.852)
    char speedKph[10];
    float kph = strtod(speedKnots, &endptr) * 1.852;
    dtostrf(kph, 4, 1, speedKph);
    
    // 1. Construction de la trame
    strcpy(nmea0183Buffer, "$GPVTG,");
    strcat(nmea0183Buffer, vtgHeading);       strcat(nmea0183Buffer, ",T,"); 
    strcat(nmea0183Buffer, "");               strcat(nmea0183Buffer, ",M,"); 
    strcat(nmea0183Buffer, speedKnots);       strcat(nmea0183Buffer, ",N,"); 
    strcat(nmea0183Buffer, speedKph);         strcat(nmea0183Buffer, ",K,"); 
    strcat(nmea0183Buffer, "A");              strcat(nmea0183Buffer, ","); 
    
    // 2. Checksum et Envoi
    BuildChecksum0183(nmea0183Buffer);
    NmeaOutputSerial->print(nmea0183Buffer);
}



void getGpsDate(char* date_buf)
{
    // Utiliser la date réelle extraite par RMC_Handler
    if (rmcDate[0] != '\0') { 
        strcpy(date_buf, rmcDate);
    } else {
        // Retour à la date statique si aucune RMC n'a été reçue (fallback)
        strcpy(date_buf, "010123"); 
    }
}

// Construit et envoie la trame GPRMC (corrigée)
void BuildRMC0183()
{
    if (!GGA_Available || !SEND_GPRMC) return;

    char* endptr;
    
    // 1. Préparation des données (mêmes que pour GGA)
    double lat_deg = nmeaToDecimal(latitude, latNS[0]);  // Get first character
    double lon_deg = nmeaToDecimal(longitude, lonEW[0]); // Get first character
    float heading_deg = strtod(vtgHeading, &endptr);
    
    // 2. Application de la Correction (seule la position est corrigée pour RMC)
    // L'altitude est ignorée, on passe NULL pour l'altitude correction
    applyRollPitchCorrection(lat_deg, lon_deg, heading_deg); // Correction de lat/lon in-place

    // 3. Conversion des données corrigées en format NMEA
    char corrected_lat_nmea[18];
    char corrected_lon_nmea[18];
    char corrected_latNS[3];
    char corrected_lonEW[3];
    char dateToSend[7]; // Utiliser un tampon local pour la date
    
    decimalToNMEA(lat_deg, corrected_lat_nmea, corrected_latNS, 7, LATITUDE);
    decimalToNMEA(lon_deg, corrected_lon_nmea, corrected_lonEW, 7, LONGITUDE); 
    getGpsDate(dateToSend); // Appel de la fonction mise à jour

    // 4. Construction de la trame
    strcpy(nmea0183Buffer, "$GPRMC,"); // Utilisation de GP comme Talker ID de sortie
    strcat(nmea0183Buffer, fixTime);        strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, "A");            strcat(nmea0183Buffer, ","); 
    strcat(nmea0183Buffer, corrected_lat_nmea); strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, corrected_latNS);  strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, corrected_lon_nmea); strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, corrected_lonEW);  strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, speedKnots);     strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, vtgHeading);     strcat(nmea0183Buffer, ","); 
    strcat(nmea0183Buffer, dateToSend);     strcat(nmea0183Buffer, ",");
    
    // Variation magnétique extraite (Champ 10 & 11)
    strcat(nmea0183Buffer, rmcMagVar);      strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, rmcMagEW);       
    
    // Mode Indicator (Champ 12, NMEA 4.10) - Simule D, R, A, ou N
    strcat(nmea0183Buffer, ",");
    if (fixQuality[0] == '4' || fixQuality[0] == '5') {
        strcat(nmea0183Buffer, "R"); // R = RTK Fixed/Float
    } else if (fixQuality[0] == '2') {
        strcat(nmea0183Buffer, "D"); // D = Differential (DGPS)
    } else if (fixQuality[0] == '1') {
        strcat(nmea0183Buffer, "A"); // A = Autonomous
    } else {
        strcat(nmea0183Buffer, "N"); // N = Data not valid
    }
    // Fin de trame

    // 5. Checksum et Envoi
    BuildChecksum0183(nmea0183Buffer);
    NmeaOutputSerial->print(nmea0183Buffer);
}

// Construit et envoie la trame GPZDA
void BuildZDA0183()
{
    if (!GGA_Available || !SEND_GPZDA) return;

    char dateToSend[7];
    getGpsDate(dateToSend); // Date réelle (DDMMYY)
    
    // rmcDate est au format ddMMyy:
    char day[3] = {dateToSend[0], dateToSend[1], '\0'};
    char month[3] = {dateToSend[2], dateToSend[3], '\0'};
    char year[5] = {'2', '0', dateToSend[4], dateToSend[5], '\0'}; 
    
    // Cela correspond à UTC+2 (Heure d'été en France).
    const char* LOCAL_HOUR_OFFSET = "02"; // Régler ici le décalage (00 pour UTC)
    const char* LOCAL_MINUTE_OFFSET = "00"; 

    // 1. Construction de la trame
    strcpy(nmea0183Buffer, "$GPZDA,");
    strcat(nmea0183Buffer, fixTime);    strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, day);        strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, month);      strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, year);       strcat(nmea0183Buffer, ",");
    strcat(nmea0183Buffer, LOCAL_HOUR_OFFSET); strcat(nmea0183Buffer, ","); 
    strcat(nmea0183Buffer, LOCAL_MINUTE_OFFSET);                                   

    // 2. Checksum et Envoi
    BuildChecksum0183(nmea0183Buffer);
    NmeaOutputSerial->print(nmea0183Buffer);
}


/************************* Fonction Principale d'Envoi *************************/

// Fonction à appeler dans la loop() pour envoyer les trames NMEA
void setupNmeaOutput()
{
    // Charger la config depuis l'EEPROM au démarrage
    loadConfigFromEEPROM();
    
    if (NmeaOutputSerial != NULL && NmeaOutputSerial != &Serial) {
        ((HardwareSerial*)NmeaOutputSerial)->begin(baudNMEA);
    }
    
    Serial.println(F("\n*** NMEA Output configuré ***"));
    Serial.println(F("Appuyez sur 'm' pour ouvrir le menu de configuration"));
    displayCurrentConfig();
}

// Fonction à appeler dans la loop() principale
void SendNmea0183()
{
    // Vérifier l'activation du menu
    if (!menuActive) {
        checkMenuActivation();
    } else {
        processMenu();
    }
    
    // [Le reste du code SendNmea0183 reste identique...]
    bool isOutputReady = true;
    if (NmeaOutputSerial == &Serial) {
        isOutputReady = Serial.dtr(); 
    }
    
    if (GGA_Available && isOutputReady)
    {
        if (SEND_GPGGA && nmeaGgaTimer >= GPGGA_INTERVAL_MS) {
            BuildGGA0183();
            nmeaGgaTimer = 0;
        }
        
        if (SEND_GPVTG && nmeaVtgTimer >= GPVTG_INTERVAL_MS) {
            BuildVTG0183();
            nmeaVtgTimer = 0;
        }
        
        if (SEND_GPRMC && nmeaRmcTimer >= GPRMC_INTERVAL_MS) {
            BuildRMC0183();
            nmeaRmcTimer = 0;
        }
        
        if (SEND_GPZDA && nmeaZdaTimer >= GPZDA_INTERVAL_MS) {
            BuildZDA0183();
            nmeaZdaTimer = 0;
        }
    }
}
