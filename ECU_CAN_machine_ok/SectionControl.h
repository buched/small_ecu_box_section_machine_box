// SectionControl.h
// Header pour le contrôle de sections AutoSteer Teensy 4.1

#ifndef SECTIONCONTROL_H
#define SECTIONCONTROL_H

#include <Arduino.h>
#include <EEPROM.h>

// Déclarations des fonctions principales
void sectionControlSetup();
void sectionControlLoop();

// Déclarations des fonctions de communication
void sendHelloToAgIO();
void sendSectionStatus();
void sendManualAutoStatus();

// Déclarations des fonctions de contrôle
void processHydraulicLift();
void processAutoMode();
void processManualMode();
void switchRelaysOff();
void SetRelays();

// Déclarations des fonctions de configuration
void updatePinMapping(uint8_t* udpData);
void handlePGN200(uint8_t* udpData);

// Nouvelle fonction pour gérer le PGN 239 (Machine Data)
void handlePGN239(uint8_t* udpData);

#endif // SECTIONCONTROL_H
