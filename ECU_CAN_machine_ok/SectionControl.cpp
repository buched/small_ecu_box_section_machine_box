// SectionControl.cpp
// Implémentation du contrôle de sections pour AutoSteer Teensy 4.1
// Configuration adaptée au PCB Triple CAN avec Mode Manuel/Auto
//Inspiration code Daniel Desmartins
/* 
 * PINS RÉSERVÉS (NE PAS UTILISER) :
 * ----------------------------------
 * 0,1,3,4,9,5,24,6,7,8,14,15,16,17,18,19,22,23,30,31,A0,A1,A2 
 * 
 * PINS DISPONIBLES POUR SECTION CONTROL :
 * ----------------------------------------
 * 2, 10, 11, 12, 13, 20, 21, 25, 26, 27, 28, 29, 32, 33
 * 34, 35, 36, 37, 38, 39, 40, 41 (A16-A23)
 * 
 * PINS MODE MANUEL (À CONFIGURER SELON VOS BESOINS) :
 * ----------------------------------------------------
 * PIN_MODE_AUTO_MANUAL : Pin interrupteur Mode Auto/Manuel
 * manualSectionPins[16] : Pins interrupteurs pour sections 1-16 en mode manuel
*/

#include "SectionControl.h"
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

// Déclarations forward des fonctions
void sendHelloToAgIO();
void sendSectionStatus();
void sendManualAutoStatus();
void processHydraulicLift();
void processAutoMode();
void processManualMode();
void switchRelaysOff();
void SetRelays();
void updatePinMapping(uint8_t* udpData);
void handlePGN239(uint8_t* udpData);

extern IPAddress ip;
extern uint8_t ipDestination[];
extern unsigned int AOGPort;
extern EthernetUDP Udp;

extern uint8_t relay;
extern uint8_t relayHi;
extern uint8_t tram; 
extern uint8_t uTurn;
extern uint8_t hydLift;
extern float gpsSpeed;

// Configuration machine
extern struct Config {
    uint8_t raiseTime;
    uint8_t lowerTime;
    uint8_t enableToolLift;
    uint8_t isRelayActiveHigh;
} aogConfig;

// Tableaux pour le mappage des broches et états des relais
uint8_t pin[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t relayState[23] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
bool isRaise = false, isLower = false;
uint8_t raiseTimer = 0, lowerTimer = 0, lastTrigger = 0;

// Pins physiques du Teensy disponibles pour positions configurables
uint8_t teensy_pins[] = {39,27,28,32,36,37,26,11};
uint8_t NUM_CONFIGURABLE_PINS = sizeof(teensy_pins) / sizeof(teensy_pins[0]);


// CONFIGURATION MODE MANUEL - MODIFIER CES PINS SELON VOS BESOINS
// Pin pour l'interrupteur Mode Auto/Manuel
const uint8_t PIN_MODE_AUTO_MANUAL = 41;  // Mettre à GND pour mode MANUEL, HIGH pour AUTO

// Pins pour les 16 interrupteurs manuels des sections (INPUT_PULLUP)
// Mettre à GND pour ACTIVER la section en mode manuel
const uint8_t manualSectionPins[16] = {2,3,4,5,8,9,10,12};

// PGN 237 - Section Control Status (retour vers AgOpenGPS)
uint8_t PGN_237[] = { 0x80, 0x81, 0x7f, 237, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };
int8_t PGN_237_Size = sizeof(PGN_237) - 1;

// PGN 234 (0xEA) - Status manuel/auto vers AgOpenGPS
uint8_t PGN_234[] = { 0x80, 0x81, 0x7B, 234, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };
int8_t PGN_234_Size = sizeof(PGN_234) - 1;

// Hello from Machine - PGN 123 pour la détection par AgIO
uint8_t helloFromMachine[] = { 128, 129, 123, 123, 5, 0, 0, 0, 0, 0, 71 };

// Timer pour le contrôle hydraulique
uint32_t lastHydraulicTime = 0;
const uint16_t HYDRAULIC_LOOP_TIME = 200; // 200ms = 5Hz

// ============================================================================
// VARIABLES MODE MANUEL
// ============================================================================

// Variables pour le mode manuel/auto (accessibles depuis le .ino principal)
bool autoModeIsOn = true;          // Mode actuel (true = Auto, false = Manuel)

// Variables locales pour la communication avec AgOpenGPS
uint8_t onLo = 0, offLo = 0;       // Sections 1-8 ON/OFF pour AgOpenGPS
uint8_t onHi = 0, offHi = 0;       // Sections 9-16 ON/OFF pour AgOpenGPS
uint8_t mainByte = 1;              // 1 = Auto, 2 = Manuel

// Timer pour le hello machine
uint32_t lastHelloTime = 0;
const uint16_t HELLO_LOOP_TIME = 2000; // 2 secondes

// ============================================================================
// SETUP
// ============================================================================

void sectionControlSetup()
{
    EEPROM.get(120, pin); 
    
    Serial.println("\r\n=== Section Control Configuration ===");
    Serial.println("AgOpenGPS Pin Mapping:");
    Serial.println("--------------------------------------");
    
    for (uint8_t i = 0; i < NUM_CONFIGURABLE_PINS; i++)
    {
        Serial.print("  Pin ");
        if (i < 9) Serial.print(" ");
        Serial.print(i + 1);
        Serial.print("  |   ");
        
        if (pin[i] == 0)
        {
            Serial.print("None (-)");
        }
        else if (pin[i] <= 16)
        {
            Serial.print("Section ");
            if (pin[i] < 10) Serial.print(" ");
            Serial.print(pin[i]);
        }
        else if (pin[i] == 17)
        {
            Serial.print("Hyd Up");
        }
        else if (pin[i] == 18)
        {
            Serial.print("Hyd Down");
        }
        else if (pin[i] == 19)
        {
            Serial.print("Tramline");
        }
        else if (pin[i] == 20)
        {
            Serial.print("GeoStop");
        }
        
        Serial.print("          |    Teensy Pin ");
        if (teensy_pins[i] < 10) Serial.print(" ");
        Serial.println(teensy_pins[i]);
    }

    // Initialiser les pins de sortie (relais)
    Serial.println("\nInitializing Physical Output Pins:");
    for (uint8_t i = 0; i < NUM_CONFIGURABLE_PINS; i++)
    {
        pinMode(teensy_pins[i], OUTPUT);
        digitalWrite(teensy_pins[i], aogConfig.isRelayActiveHigh ? LOW : HIGH);
        Serial.print("  ✓ Teensy Pin ");
        if (teensy_pins[i] < 10) Serial.print(" ");
        Serial.print(teensy_pins[i]);
        Serial.println(" ready");
    }

    // Initialiser le pin Mode Auto/Manuel avec pullup interne
    pinMode(PIN_MODE_AUTO_MANUAL, INPUT_PULLUP);
    Serial.println("\nManual Mode Configuration:");
    Serial.print("  Mode Switch Pin: ");
    Serial.println(PIN_MODE_AUTO_MANUAL);
    
    // Initialiser les pins interrupteurs manuels avec pullup interne
    Serial.println("  Manual Section Switch Pins:");
    for (uint8_t i = 0; i < 16; i++)
    {
        pinMode(manualSectionPins[i], INPUT_PULLUP);
        Serial.print("    Section ");
        if (i < 9) Serial.print(" ");
        Serial.print(i + 1);
        Serial.print(" → Pin ");
        Serial.println(manualSectionPins[i]);
    }

    // Initialiser les états
    for (uint8_t i = 0; i < 23; i++)
    {
        relayState[i] = 0;
    }

    raiseTimer = 0;
    lowerTimer = 0;
    lastTrigger = 0;
    isRaise = false;
    isLower = false;
    
    onLo = onHi = 0;
    offLo = offHi = 0xFF;

    Serial.println("\n=== Section Control Ready ===");
    Serial.print("Relay Active High: ");
    Serial.println(aogConfig.isRelayActiveHigh ? "YES" : "NO");
    Serial.print("Total configurable positions: ");
    Serial.println(NUM_CONFIGURABLE_PINS);
    Serial.println("Mode: Waiting for switch detection...");
    Serial.println("Waiting for AgIO connection...\r\n");
}

// ============================================================================
// LOOP PRINCIPAL
// ============================================================================

void sectionControlLoop()
{
    uint32_t currentTime = millis();

    // Lire l'état du switch Mode Auto/Manuel
    bool newAutoMode = digitalRead(PIN_MODE_AUTO_MANUAL); // HIGH = Auto, LOW = Manuel
    
    if (newAutoMode != autoModeIsOn)
    {
        autoModeIsOn = newAutoMode;
    
        if (!autoModeIsOn)
        {
            // Passage en mode Manuel - éteindre toutes les sections
            relay = 0;
            relayHi = 0;
            onLo = onHi = 0;
            offLo = offHi = 0xFF;
        }
    }

    // Mettre à jour mainByte pour AgOpenGPS
    mainByte = autoModeIsOn ? 1 : 2;

    // Envoyer le Hello régulièrement (toutes les 2 secondes)
    if (currentTime - lastHelloTime >= HELLO_LOOP_TIME)
    {
        lastHelloTime = currentTime;
        sendHelloToAgIO();
    }

    // Boucle de contrôle (5Hz)
    if (currentTime - lastHydraulicTime >= HYDRAULIC_LOOP_TIME)
    {
        lastHydraulicTime = currentTime;
        
        processHydraulicLift();
        
        // Dans sectionControlLoop()
// ...
if (autoModeIsOn)
{
    // Mode AUTO - Contrôle par AgOpenGPS
    processAutoMode();
        }
        else
        {
            // Mode MANUEL - Contrôle par interrupteurs
            processManualMode();
}

        
        SetRelays();
        sendSectionStatus();
        sendManualAutoStatus();
    }
}

// ============================================================================
// MODE AUTO - Contrôle par AgOpenGPS
// ============================================================================

void processAutoMode()
{
    onLo = onHi = 0;
    offLo = offHi = 0;
}

// ============================================================================
// MODE MANUEL - Contrôle par interrupteurs
// ============================================================================

void processManualMode()
{
    // Réinitialiser
    relay = 0;
    relayHi = 0;
    onLo = 0;
    onHi = 0;
    offLo = 0;
    offHi = 0;
    
    // Lire l'état des 16 interrupteurs manuels
    for (uint8_t i = 0; i < 16; i++)
    {
        bool switchClosed = !digitalRead(manualSectionPins[i]); // LOW = interrupteur fermé = section ON
        
        if (i < 8) // Sections 1-8
        {
            if (switchClosed)
            {
                bitSet(relay, i);      // Activer la section
                bitSet(onLo, i);       // Informer AgOpenGPS
            }
            else
            {
                bitSet(offLo, i);      // Informer AgOpenGPS que section est OFF
            }
        }
        else // Sections 9-16
        {
            uint8_t bit = i - 8;
            if (switchClosed)
            {
                bitSet(relayHi, bit);
                bitSet(onHi, bit);
            }
            else
            {
                bitSet(offHi, bit);
            }
        }
    }
    
    
}

// ============================================================================
// ÉTEINDRE TOUS LES RELAIS
// ============================================================================

void switchRelaysOff()
{
    relay = 0;
    relayHi = 0;
    onLo = onHi = 0;
    offLo = offHi = 0xFF;
}

// ============================================================================
// HELLO TO AgIO
// ============================================================================

void sendHelloToAgIO()
{
    helloFromMachine[5] = relay;
    helloFromMachine[6] = relayHi;
    
    int16_t CK_A = 0;
    for (uint8_t i = 2; i < sizeof(helloFromMachine) - 1; i++)
    {
        CK_A = (CK_A + helloFromMachine[i]);
    }
    helloFromMachine[sizeof(helloFromMachine) - 1] = CK_A;
    
    Udp.beginPacket(ipDestination, AOGPort);
    Udp.write(helloFromMachine, sizeof(helloFromMachine));
    Udp.endPacket();
}

// ============================================================================
// HYDRAULIQUE
// ============================================================================

void processHydraulicLift()
{
    if (hydLift != lastTrigger && (hydLift == 1 || hydLift == 2))
    {
        lastTrigger = hydLift;
        lowerTimer = 0;
        raiseTimer = 0;

        switch (hydLift)
        {
            case 1: // Abaisser
                lowerTimer = aogConfig.lowerTime * 5;
                break;

            case 2: // Lever
                raiseTimer = aogConfig.raiseTime * 5;
                break;
        }
    }

    if (raiseTimer)
    {
        raiseTimer--;
        lowerTimer = 0;
    }
    if (lowerTimer) lowerTimer--;

    if ((hydLift != 1 && hydLift != 2))
    {
        lowerTimer = 0;
        raiseTimer = 0;
        lastTrigger = 0;
    }

    if (aogConfig.isRelayActiveHigh)
    {
        isLower = isRaise = false;
        if (lowerTimer) isLower = true;
        if (raiseTimer) isRaise = true;
    }
    else
    {
        isLower = isRaise = true;
        if (lowerTimer) isLower = false;
        if (raiseTimer) isRaise = false;
    }
}

// ============================================================================
// SET RELAYS - Appliquer les états aux sorties physiques
// ============================================================================

void SetRelays()
{
    // Charger l'état actuel des sections 1-16
    for (uint8_t i = 0; i < 8; i++)
    {
        relayState[i] = bitRead(relay, i);
        relayState[i + 8] = bitRead(relayHi, i);
    }

    // Hydraulique
    relayState[16] = isLower;
    relayState[17] = isRaise;
    // Tram
    relayState[18] = bitRead(tram, 0);
    relayState[19] = bitRead(tram, 1);
    // GeoStop
    relayState[20] = 0;
    
    // Éteindre tous les pins configurables
    for (uint8_t i = 0; i < NUM_CONFIGURABLE_PINS; i++)
    {
        digitalWrite(teensy_pins[i], aogConfig.isRelayActiveHigh ? LOW : HIGH);
    }
    
    // Appliquer les états selon le mapping
    for (uint8_t i = 0; i < NUM_CONFIGURABLE_PINS; i++)
    {
        uint8_t functionAssigned = pin[i];
        
        if (functionAssigned > 0 && functionAssigned <= 21)
        {
            uint8_t physicalPin = teensy_pins[i];
            bool logicalState = false;
            
            // Sections 1-16
            if (functionAssigned >= 1 && functionAssigned <= 16)
            {
                logicalState = relayState[functionAssigned - 1];
            }
            // Hyd Up (fonction 17)
            else if (functionAssigned == 17)
            {
                logicalState = relayState[16];
            }
            // Hyd Down (fonction 18)
            else if (functionAssigned == 18)
            {
                logicalState = relayState[17];
            }
            // Tram Right (fonction 19)
            else if (functionAssigned == 19)
            {
                logicalState = relayState[18];
            }
            // Tram Left (fonction 20)
            else if (functionAssigned == 20)
            {
                logicalState = relayState[19];
            }
            // GeoStop (fonction 21)
            else if (functionAssigned == 21)
            {
                logicalState = relayState[20];
            }
            
            // Convertir en état physique
            bool physicalState = aogConfig.isRelayActiveHigh ? logicalState : !logicalState;
            digitalWrite(physicalPin, physicalState ? HIGH : LOW);
        }
    }
}

// ============================================================================
// ENVOYER STATUS À AgOpenGPS (PGN 237)
// ============================================================================

void sendSectionStatus()
{
    PGN_237[5] = relay;
    PGN_237[6] = relayHi;
    PGN_237[7] = tram;
    PGN_237[8] = isRaise ? 2 : (isLower ? 1 : 0);

    int16_t CK_A = 0;
    for (uint8_t i = 2; i < PGN_237_Size; i++)
    {
        CK_A = (CK_A + PGN_237[i]);
    }
    PGN_237[PGN_237_Size] = CK_A;

    Udp.beginPacket(ipDestination, AOGPort);
    Udp.write(PGN_237, sizeof(PGN_237));
    Udp.endPacket();
}

// ============================================================================
// ENVOYER STATUS MANUEL/AUTO À AgOpenGPS (PGN 234 / 0xEA)
// ============================================================================

void sendManualAutoStatus()
{
    // PGN 234 (0xEA) - Informe AgOpenGPS du mode et des sections activées manuellement
    PGN_234[5] = mainByte;      // 1 = Auto, 2 = Manuel
    PGN_234[6] = 0;             // Réservé
    PGN_234[7] = 0;             // Réservé
    PGN_234[8] = 0;             // Réservé
    PGN_234[9] = onLo;          // Sections 1-8 activées manuellement
    PGN_234[10] = offLo;        // Sections 1-8 désactivées manuellement
    PGN_234[11] = onHi;         // Sections 9-16 activées manuellement
    PGN_234[12] = offHi;        // Sections 9-16 désactivées manuellement

    // Calculer le checksum
    int16_t CK_A = 0;
    for (uint8_t i = 2; i < PGN_234_Size; i++)
    {
        CK_A = (CK_A + PGN_234[i]);
    }
    PGN_234[PGN_234_Size] = CK_A;

    // Envoyer à AgOpenGPS
    Udp.beginPacket(ipDestination, AOGPort);
    Udp.write(PGN_234, sizeof(PGN_234));
    Udp.endPacket();
}

// ============================================================================
// MISE À JOUR DU MAPPING (depuis AgOpenGPS)
// ============================================================================

void updatePinMapping(uint8_t* udpData)
{
    // Lit les 24 octets de la nouvelle configuration de pin (commençant à l'index 5)
    // et les stocke dans le tableau 'pin' du module.
    for (uint8_t i = 0; i < 24; i++)
    {
        pin[i] = udpData[i + 5];
    }
    
    // Sauvegarde le tableau de configuration de pins dans l'EEPROM
    // pour que la configuration soit conservée après redémarrage.
    EEPROM.put(120, pin);
}

// ============================================================================
// RÉCEPTION PGN 239 (Machine Data from AgOpenGPS)
// ============================================================================

void handlePGN239(uint8_t* udpData)
{
    // Extraction des données de contrôle PGN 239
    uTurn = udpData[5];
    gpsSpeed = (float)udpData[6];
    hydLift = udpData[7];
    tram = udpData[8];  
    
    // Commandes des sections 1-8 et 9-16
    uint8_t receivedRelay = udpData[11];  
    uint8_t receivedRelayHi = udpData[12]; 
    
    // --- Logique d'Inversion et de Débogage RAW ---
    
    // DEBUG - Afficher ce qui est reçu AVANT inversion
    static uint32_t lastDebugTime = 0;
    if (millis() - lastDebugTime > 1000)
    
    
//    // Inversion des commandes de tram et de section si les relais sont actifs HIGH
//    if (aogConfig.isRelayActiveHigh)
//    {
//        tram = 255 - tram;
//        receivedRelay = 255 - receivedRelay;
//        receivedRelayHi = 255 - receivedRelayHi;
//    }
    
    // --- Application des Commandes (Mode AUTO) ---
    
    // Mettre à jour relay et relayHi UNIQUEMENT si le mode AUTO est actif.
    if (autoModeIsOn) 
    {
        // Appliquer les commandes de section reçues
        relay = receivedRelay;
        relayHi = receivedRelayHi;
                        
        // DEBUG - Afficher les commandes appliquées
        static uint8_t lastRelay = 0xFF;
        static uint8_t lastRelayHi = 0xFF;
        if (relay != lastRelay || relayHi != lastRelayHi)
        {
            lastRelay = relay;
            lastRelayHi = relayHi;
        }
    }
    else // !autoModeIsOn (Mode MANUEL)
    {
        // En mode MANUEL, ignorer les commandes de section AgOpenGPS
        static uint32_t lastIgnoreMsg = 0;
        if (millis() - lastIgnoreMsg > 3000)
        {
            lastIgnoreMsg = millis();
        }
    }
}
