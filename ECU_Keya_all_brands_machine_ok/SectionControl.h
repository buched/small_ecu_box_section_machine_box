// SectionControl.h
// Header pour le contrôle de sections AutoSteer Teensy 4.1
//Ajout fonction mode SC via I2C 

#ifndef SECTIONCONTROL_H
#define SECTIONCONTROL_H

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>

// ============================================================================
// CONFIGURATION I2C
// ============================================================================
// Activer le mode I2C (true) ou GPIO Teensy (false)
#define SC_I2C true

// Adresse I2C du module XL9535 (0x20 = 0100 000 en binaire avec A2=A1=A0=0)
#define XL9535_ADDRESS 0x20

// Déclarations des fonctions principales
void sectionControlSetup();
void sectionControlLoop();

// Déclarations des fonctions de communication
void sendSectionStatus();
void sendManualAutoStatus();

// Déclarations des fonctions de contrôle
void processHydraulicLift();
void processAutoMode();
void processManualMode();
void switchRelaysOff();
void SetRelays();

// Déclarations des fonctions de configuration
void updatePinMapping(uint8_t* autoSteerUdpData);
void handlePGN200(uint8_t* autoSteerUdpData);
void handlePGN239(uint8_t* udpData);

// Déclarations des fonctions I2C
void initXL9535();
void setI2CRelay(uint8_t relayNumber, bool state);
void updateAllI2CRelays();

#endif // SECTIONCONTROL_H