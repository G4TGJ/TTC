/*
 * io.h
 *
 * Created: 11/09/2019
 * Author : Richard Tomlinson G4TGJ
 */ 
 
#ifndef IO_H
#define IO_H

#include <inttypes.h>

// Initialise all IO ports
void ioInit();

// Read the morse dot and dash paddles
bool ioReadDotPaddle();
bool ioReadDashPaddle();

// Read the rotary control and switch
void ioReadRotary( int rotaryNum, bool *pbA, bool *pbB, bool *pbSw );

// Read the pushbuttons
bool ioReadButton( uint8_t button );

// Set the morse output high or low
void ioWriteMorseOutputHigh();
void ioWriteMorseOutputLow();

// Set the RX enable low or high
void ioWriteRXEnableLow();
void ioWriteRXEnableHigh();

// Switch the sidetone output on or off
void ioWriteSidetoneOn();
void ioWriteSidetoneOff();

// Switch the preamp enable output on or off
void ioWritePreampOn();
void ioWritePreampOff();

// Switch a band relay output on or off
void ioWriteBandRelay( uint8_t relay, bool bOn );

void ioSetSidetoneVolume( uint8_t vol );

// Turn LEDs on or off
void ioWriteRightLED( bool bOn );
void ioWriteCentreLED( bool bOn );
void ioWriteLeftLED( bool bOn );

uint8_t ioGetSidetoneVolume( void );

void ioSetVolume( uint8_t volume );
uint8_t ioGetVolume( void );

uint8_t ioGetFilter( void );
const char *ioGetFilterText( void );
void ioSetFilter( uint8_t filter );
uint8_t ioGetNumFilters( void );

uint8_t ioGetHilbertFilter( void );
const char *ioGetHilbertFilterText( void );
void ioSetHilbertFilter( uint8_t filter );
uint8_t ioGetNumHilbertFilters( void );

// Set the IF frequency
void ioSetIF( void );

// Select normal, binaural or peaked output
enum eOutput
{
    NORMAL_OUTPUT = 0,
    BINAURAL_OUTPUT,
    PEAKED_OUTPUT,
    NUM_OUTPUTS
};

#endif //IO_H
