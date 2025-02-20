/*
 * main.c
 *
 * Created: 30/11/2018 21:26:21
 * Author : Richard Tomlinson G4TGJ
 */ 

//#define DISABLE_LCD

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "config.h"
#include "main.h"
#include "io.h"
#include "osc.h"
#include "millis.h"
#include "nvram.h"
#include "lcd.h"
#include "morse.h"
#include "rotary.h"
#include "pushbutton.h"

#ifdef PSDR
#include "sdr.h"
#include "WM8960.h"
#endif

#ifdef LCD_DISPLAY
#include "display.h"
#endif

#ifdef OLED_DISPLAY
#include "ssd1306.h"
#include "font.h"
#endif

#ifdef CAT_CONTROL
#include "cat.h"
#endif

// Bit map for the various rotary and pushbutton inputs
#define ROTARY_CW               0x0001
#define ROTARY_CCW              0x0002
#define ROTARY_SHORT_PRESS      0x0004
#define ROTARY_LONG_PRESS       0x0008
#define BUTTON_SHORT_PRESS      0x0010
#define BUTTON_LONG_PRESS       0x0020

// There will be several buttons (as well as the rotary button) so macros for
// these
#define SHORT_PRESS(button)     (BUTTON_SHORT_PRESS<<((button)*2))
#define LONG_PRESS(button)      (BUTTON_LONG_PRESS <<((button)*2))

// Previously these were parameters to the various functions that handle the input
// so replace with definitions that access the bitmap to save changing all the code
#define bCW                 (inputState & ROTARY_CW)
#define bCCW                (inputState & ROTARY_CCW)
#define bShortPress         (inputState & ROTARY_SHORT_PRESS)
#define bLongPress          (inputState & ROTARY_LONG_PRESS)
#define bShortPressLeft     (inputState & SHORT_PRESS(LEFT_BUTTON))
#define bLongPressLeft      (inputState & LONG_PRESS(LEFT_BUTTON))
#define bShortPressRight    (inputState & SHORT_PRESS(RIGHT_BUTTON))
#define bLongPressRight     (inputState & LONG_PRESS(RIGHT_BUTTON))

#ifndef SOTA2
// Menu functions
static bool menuVFOBand( uint16_t inputState );
static bool menuVFOMode( uint16_t inputState );
static bool menuBreakIn( uint16_t inputState );
static bool menuTestRXMute( uint16_t inputState );
static bool menuSidetone( uint16_t inputState );
static bool menuRXClock( uint16_t inputState );
static bool menuUnmuteDelay( uint16_t inputState );
static bool menuMuteDelay( uint16_t inputState );
static bool menuTXDelay( uint16_t inputState );
static bool menuTXClock( uint16_t inputState );
static bool menuTXOut( uint16_t inputState );
static bool menuXtalFreq( uint16_t inputState );
static bool menuIntermediateFreq( uint16_t inputState );
static bool menuKeyerMode( uint16_t inputState );

#ifdef PSDR
static bool menuSDR( uint16_t inputState );
static bool menuIGain( uint16_t inputState );
static bool menuQGain( uint16_t inputState );
static bool menuIQGain( uint16_t inputState );
static bool menuAdjustPhase( uint16_t inputState );
static bool menuApplyGains( uint16_t inputState );
static bool menuRoofing( uint16_t inputState );
static bool menuHilbertFilter( uint16_t inputState );
static bool menuMuteFactor( uint16_t inputState );
static bool menuLeftADCVolume( uint16_t inputState );
static bool menuRightADCVolume( uint16_t inputState );
static bool menuIQPhasing( uint16_t inputState );
#ifdef LCD_DISPLAY
static bool menuFilter( uint16_t inputState );
#endif
#endif

#ifdef LCD_DISPLAY
static bool menuBacklight( uint16_t inputState );
#endif

#ifdef VARIABLE_SIDETONE_VOLUME
static bool menuSidetoneVolume( uint16_t inputState );
#endif

// Menu structure arrays

struct sMenuItem
{
    char *text;
    bool (*func)(uint16_t);
};

#define NUM_VFO_MENUS 3
static const struct sMenuItem vfoMenu[NUM_VFO_MENUS] =
{
    { "",               NULL },
    { "VFO Band",       menuVFOBand },
    { "VFO Mode",       menuVFOMode },
};

#ifdef PSDR
#ifdef LCD_DISPLAY
#define NUM_SDR_MENUS 14
#else
#define NUM_SDR_MENUS 13
#endif
static const struct sMenuItem sdrMenu[NUM_SDR_MENUS] =
{
    { "",               NULL },
    { "SDR Mode",       menuSDR },
#ifdef LCD_DISPLAY
    { "Filter",         menuFilter },
#endif
    { "IQ Phasing",     menuIQPhasing },
    { "Left ADC Vol",   menuLeftADCVolume },
    { "Right ADC Vol",  menuRightADCVolume },
    { "Roofing filter", menuRoofing },
    { "Apply gains",    menuApplyGains },
    { "Adjust phase",   menuAdjustPhase },
    { "I Gain",         menuIGain },
    { "Q Gain",         menuQGain },
    { "IQ Gain",        menuIQGain },
    { "Mute Factor",    menuMuteFactor },
    { "Hilbert filter", menuHilbertFilter },
};

#define SDR_FILTER_MENU_ITEM 2
#endif

#define NUM_TEST_MENUS 10
static const struct sMenuItem testMenu[NUM_TEST_MENUS] =
{
    { "",               NULL },
    { "Break in",       menuBreakIn },
    { "Test RX Mute",   menuTestRXMute },
    { "Sidetone",       menuSidetone },
    { "RX Clock",       menuRXClock },
    { "Unmute delay",   menuUnmuteDelay },
    { "Mute delay",     menuMuteDelay },
    { "TX delay",       menuTXDelay },
    { "TX Clock",       menuTXClock },
    { "TX Out",         menuTXOut },
};

// Work out how many items are in the config menu
// Depends on the display type and whether variable sidetone
#ifdef LCD_DISPLAY
#define NUM_LCD_MENUS 1
#else
#define NUM_LCD_MENUS 0
#endif

#ifdef VARIABLE_SIDETONE_VOLUME
#define NUM_SIDETONE_MENUS 1
#else
#define NUM_SIDETONE_MENUS 0
#endif

#define NUM_CONFIG_MENUS (4 + NUM_LCD_MENUS + NUM_SIDETONE_MENUS)

static const struct sMenuItem configMenu[NUM_CONFIG_MENUS] =
{
    { "",               NULL },
    { "Xtal Frequency", menuXtalFreq },
    { "IF Frequency",   menuIntermediateFreq },
    { "Keyer Mode",     menuKeyerMode },
#ifdef LCD_DISPLAY
    { "Backlight",      menuBacklight },
#endif    
#ifdef VARIABLE_SIDETONE_VOLUME
    { "Sidetone vol",   menuSidetoneVolume },
#endif
};

enum eMenuTopLevel
{
#ifdef PSDR
    SDR_MENU,
#endif
    VFO_MENU,
    TEST_MENU,
    CONFIG_MENU,
    NUM_MENUS
};

// Top-level menu structure
static const struct
{
    char                    *text;
    const struct sMenuItem  *subMenu;
    uint8_t                 numItems;
}
menu[NUM_MENUS] =
{
#ifdef PSDR
    { "SDR",    sdrMenu,    NUM_SDR_MENUS },
#endif
    { "VFO",    vfoMenu,    NUM_VFO_MENUS },
    { "Test",   testMenu,   NUM_TEST_MENUS },
    { "Config", configMenu, NUM_CONFIG_MENUS },
};

static uint8_t currentMenu;
static uint8_t currentSubMenu;

// Set to true when we are in a menu item rather than in the top level
static bool bInMenuItem;

// Set to true if we enter a menu item
// Allows us to only enter wpm state if we have not been in a menu
static bool bEnteredMenuItem;

// Set to true if entered the VFO menu quickly so that we
// can leave quickly
static bool bInQuickMenuItem;

// Text for the quick menu line
// In split mode cannot enter RIT or XIT so show them in lower case
// Also show we are in split mode
#define QUICK_MENU_TEXT         "A/B A=B R X SPLT"
#define QUICK_MENU_SPLIT_TEXT   "A/B A=B r x splt"

// RIT item as we will start here as this is most likely to use in a rush
#define QUICK_MENU_RIT 2

// Quick menu array
struct sQuickMenuItem
{
    uint8_t pos;    // Position on the line - needs to match the above text
    uint8_t sx;     // Position on OLED line - will be calculated
    uint8_t sWidth; // Width of the OLED item
    void (*func)(); // Function to call when item is selected
};

static void quickMenuSwap();
static void quickMenuEqual();
static void quickMenuRIT();
static void quickMenuXIT();
static void quickMenuSplit();

#define NUM_QUICK_MENUS 5
static struct sQuickMenuItem quickMenu[NUM_QUICK_MENUS] =
{
    { 0,  0, 0, quickMenuSwap },
    { 4,  0, 0, quickMenuEqual },
    { 8,  0, 0, quickMenuRIT },
    { 10, 0, 0, quickMenuXIT },
    { 12, 0, 0, quickMenuSplit },
};

// Current quick menu item
// Start on RIT but we will remember thereafter (until reboot)
static uint8_t quickMenuItem = QUICK_MENU_RIT;

// Current user interface mode we are in
static enum eCurrentMode
{
    modeVFO, // Default tuning mode
    modeMenu,
    modeWpm,
    modeQuickMenu,
} currentMode = modeVFO;

#ifdef LCD_DISPLAY
// Backlight mode
static enum eBacklightMode currentBacklightMode;

// The last time the backlight went on in auto mode
static uint32_t lastBacklightTime;

#ifdef PSDR
// The last time we changed the volume
static uint32_t lastVolumeTime;
#endif
#endif
#endif

// Band frequencies
#ifdef SOTA2
#define NUM_BANDS 2
#else
#define NUM_BANDS 13
#endif

static const struct
{
    char     *bandName;     // Text for the menu
    uint32_t  minFreq;      // Min band frequency
    uint32_t  maxFreq;      // Max band frequency
    uint32_t  defaultFreq;  // Where to start on this band e.g. QRP calling
#ifdef SOTA2
    uint32_t  leftFreq;     // Below this frequency light the left LED
    uint32_t  rightFreq;    // Above this frequency light the right LED
#endif
    bool      bTXEnabled;   // True if TX enabled on this band
    uint8_t   relayState;   // What state to put the relays in on this band
#ifndef SOTA2
    bool      bQuickVFOMenu;// True if this band appears in the quick VFO menu
#endif
}
band[NUM_BANDS] =
{
#ifdef SOTA2
    { "40m",      7000000,  7199999,  7030000,   7020000,  7040000, TX_ENABLED_40M,  RELAY_STATE_40M },
    { "20m",     14000000, 14349999, 14060000,  14050000, 14070000, TX_ENABLED_20M,  RELAY_STATE_20M },
#else
    { "160m",     1810000,  1999999,  1836000, TX_ENABLED_160M, RELAY_STATE_160M, QUICK_VFO_160M },
    { "80m",      3500000,  3799999,  3560000, TX_ENABLED_80M,  RELAY_STATE_80M,  QUICK_VFO_80M },
    { "RWM 4996", 4996000,  4996000,  4996000, false,           RELAY_STATE_60M,  false },
    { "60m UK",   5258500,  5263999,  5262000, TX_ENABLED_60M,  RELAY_STATE_60M,  QUICK_VFO_60M },
    { "60m EU",   5354000,  5357999,  5355000, TX_ENABLED_60M,  RELAY_STATE_60M,  QUICK_VFO_60M },
    { "40m",      7000000,  7199999,  7030000, TX_ENABLED_40M,  RELAY_STATE_40M,  QUICK_VFO_40M },
    { "RWM 9996", 9996000,  9996000,  9996000, false,           RELAY_STATE_30M,  false },
    { "30m",     10100000, 10150000, 10116000, TX_ENABLED_30M,  RELAY_STATE_30M,  QUICK_VFO_30M },
    { "20m",     14000000, 14349999, 14060000, TX_ENABLED_20M,  RELAY_STATE_20M,  QUICK_VFO_20M },
    { "17m",     18068000, 18167999, 18086000, TX_ENABLED_17M,  RELAY_STATE_17M,  QUICK_VFO_17M },
    { "15m",     21000000, 21449999, 21060000, TX_ENABLED_15M,  RELAY_STATE_15M,  QUICK_VFO_15M },
    { "12m",     24890000, 24989999, 24906000, TX_ENABLED_12M,  RELAY_STATE_12M,  QUICK_VFO_12M },
    { "10m",     28000000, 29699999, 28060000, TX_ENABLED_10M,  RELAY_STATE_10M,  QUICK_VFO_10M },
#endif
};

// Current band - initialised from NVRAM
static uint8_t currentBand;

// Current relay state - always set from the frequency
static uint8_t currentRelay;

#ifndef SOTA2
// Is the VFO on the first or second frequency line?
static bool bVFOFirstLine = true;

// For the setting frequency (e.g. xtal or BFO) the current digit position that is changing
static uint8_t settingFreqPos;

// True if asking whether to save the frequency setting
static bool bAskToSaveSettingFreq = false;

#endif

// Set to true if break in is enabled
static bool bBreakIn = true;

// Set to true if the oscillator is successfully initialised over I2C
static bool bOscInit;

// VFO modes
enum eVFOMode
{
    vfoSimplex,
    vfoRIT,
    vfoXIT,
    vfoNumModes // Num of VFO modes. Must be the last entry.
};

#ifndef SOTA2

// Display mode
enum eDisplayMode
{
    displayStrength,
    displayWpm,
    displayVolume
} displayMode;

#define DEFAULT_DISPLAY_MODE displayWpm

// The last display mode - used to correctly revert after volume displayed
enum eDisplayMode lastDisplayMode;

// The cursor position along with its corresponding frequency change
// Also the screen pixel position for an OLED - these will be calculated
struct sCursorPos
{
    uint8_t x, y;   // Position on LCD
    uint8_t sx;     // x on OLED screen
    uint8_t sy;     // y on OLED screen - full height digit
    uint8_t shy;    // y on OLED screen - half height digit
    uint8_t sWidth; // y on OLED screen - cursor width
    uint32_t freqChange;
};

// Mark the end of the cursor transitions
#define CURSOR_TRANSITION_END 0xFF

// The cursor transitions for the VFO
#define NUM_CURSOR_TRANSITIONS 6
static struct sCursorPos vfoCursorTransition[NUM_CURSOR_TRANSITIONS] =
{
    { 5, 1, 0, 0, 0, 0, 10000 },
    { 6, 1, 0, 0, 0, 0, 1000 },
    { 7, 1, 0, 0, 0, 0, 250 },
    { 8, 1, 0, 0, 0, 0, 100 },
    { 9, 1, 0, 0, 0, 0, 10 },
    { CURSOR_TRANSITION_END, CURSOR_TRANSITION_END, CURSOR_TRANSITION_END, CURSOR_TRANSITION_END, CURSOR_TRANSITION_END, CURSOR_TRANSITION_END, CURSOR_TRANSITION_END }
};

// Start the cursor on the dot (250Hz) position
#define CURSOR_DOT 2
#define DEFAULT_CURSOR_INDEX CURSOR_DOT
#define FINEST_CURSOR_INDEX 4

// Index for the current cursor position
// and the previous index so when we leave RIT
// goes back where it was
static uint8_t cursorIndex = DEFAULT_CURSOR_INDEX;
static uint8_t prevCursorIndex = DEFAULT_CURSOR_INDEX;

#ifdef OLED_DISPLAY
// Cursor position for WPM - initialised in screenInit()
static int wpmCursorX, wpmCursorY, wpmCursorWidth;

// Positions of screen objects
static int wpmX, wpmY;
static int menuX, menuY, menuWidth, menuCursorX, menuCursorY;
static int volX, volY;
static int preampX, preampY;
static int filterX, filterY, filterWidth;
static int modeX, modeY, modeWidth;
#endif

// In fast mode, if the dial is spun the rate speeds up
#define VFO_SPEED_UP_DIFF  150  // If dial clicks are no more than this ms apart then speed up
#define VFO_SPEED_UP_FACTOR 10  // Multiply the rate by this
#endif

static void rotaryVFO( uint16_t inputState );

// Maintain transmit and receive frequencies for two VFOs
// The current VFO
static uint8_t currentVFO;

// Macro to give the other VFO
#define OTHER_VFO ((currentVFO)^1)

// Macro to give the letter for the VFO i.e. A or B
#define VFO_LETTER(vfo) (((vfo)==VFO_A)?'A':'B')

// State of each vfo
static struct sVFOState 
{
    uint32_t        freq;       // Frequency
    int32_t         offset;     // Offset when in RIT or XIT
    enum eVFOMode   mode;       // Simplex, RIT or XIT
} vfoState[NUM_VFOS];

// True if in split mode (RX on current VFO, TX on other VFO)
static bool bVFOSplit;

// Set to true when testing RX mute function
static bool bTestRXMute;

// Set to true when RX clock enabled
static bool bRXClockEnabled = true;

// Set to true when TX clock enabled
static bool bTXClockEnabled = true;

// Set to true when morse output enabled
static bool bTXOutEnabled = true;

// Set to true when sidetone enabled
static bool bSidetone = true;

#ifdef PSDR
// Set to true when the preamp is on
static bool bPreampOn = false;

// Select normal, binaural or peaked output
extern enum eOutput outputMode;
#endif

// Delay before muting and unmuting
static uint8_t muteDelay = 5;
static uint16_t unmuteDelay = 5;
static uint8_t txDelay = 10;

// Set to true when transmitting
static bool bTransmitting = false;

// Intermediate frequency - 0 for direct conversion
static uint32_t intermediateFrequency;

// Whether IF is above or below
extern bool ifBelow;

#ifdef PSDR
// Keep track of ADC and output overload
extern bool adcOverload;
static bool prevAdcOverload;
extern int outOverload;
static int prevOutOverload;

#ifdef DISPLAY_AGC
// Amplitude and gain as used by the AGC
extern uint32_t agcAmplitude;
static uint32_t prevAgcAmplitude;
extern int agcGain;
static int prevAgcGain;
#endif

// Maximum mute factor
extern int maxMuteFactor;

#ifdef DISPLAY_MIN_MAX
extern int maxIn, maxOut, minIn, minOut;
static int prevMaxIn, prevMaxOut, prevMinIn, prevMinOut;
#endif

// Volume knob controls volume when true else front end
// sample shift.
static bool bVolumeMode = true;

// The front end input shift
//extern int inputShift;
#endif

/**
 * @brief Works out the current RX frequency from the VFO settings.
 *
 * This function returns the current RX (receive) frequency.
 *
 * @return The RX frequency as a 32-bit unsigned integer.
 */
static uint32_t getRXFreq()
{
    uint32_t freq;

    // Start with the current VFO's base frequency
    freq = vfoState[currentVFO].freq;

    // If in RIT mode need to add the offset
    if( vfoState[currentVFO].mode == vfoRIT )
    {
        freq += vfoState[currentVFO].offset;
    }

    return freq;
}

/**
 * @brief Works out the current TX frequency from the VFO settings.
 *
 * This function returns the current TX (transmit) frequency.
 *
 * @return The TX frequency as a 32-bit unsigned integer.
 */
static uint32_t getTXFreq()
{
    uint32_t freq;
    uint8_t vfo;

    // If in split mode then the TX frequency comes from the other VFO,
    // else it's the current VFO
    if( bVFOSplit )
    {
        vfo = OTHER_VFO;
    }
    else
    {
        vfo = currentVFO;
    }

    // Start with the VFO's base frequency
    freq = vfoState[vfo].freq;

    // If in XIT mode need to add the offset
    if( vfoState[vfo].mode == vfoXIT )
    {
        freq += vfoState[vfo].offset;
    }

    return freq;
}

/**
 * @brief Checks if transmission is enabled at the current transmit frequency.
 *
 * This function determines whether the transmission is currently enabled.
 *
 * @return true if transmission is enabled, false otherwise.
 */
static bool txEnabled()
{
    // Assume TX is not enabled
    bool bEnabled = false;

    // If the TX frequency is in the current band then get its TX enabled status
    if( (getTXFreq() >= band[currentBand].minFreq) && (getTXFreq() <= band[currentBand].maxFreq) )
    {
        bEnabled = band[currentBand].bTXEnabled;
    }

    return bEnabled;
}

/**
 * @brief Sets the band based on the given frequency.
 *
 * This function determines the band corresponding to the provided frequency
 * and updates the current band and relay state accordingly. If the frequency
 * is outside the current band limits, it searches for the appropriate band.
 *
 * @param freq The frequency to determine the band for.
 */
void setBandFromFrequency( uint32_t freq )
{
    // Is the frequency outside the current band limits?
    if( (freq < band[currentBand].minFreq) || (freq > band[currentBand].maxFreq) )
    {
        // Yes it is
        // See what band it is in
        for( uint8_t b = 0 ; b < NUM_BANDS ; b++ )
        {
            if( freq <= band[b].maxFreq )
            {
                // Whether or not we are in band this must be the relay state we need
                // for the correct LPF
                currentRelay = band[b].relayState;

                currentBand = b;

                // Store the band in the NVRAM
                nvramWriteBand( b );

                // We can stop looking whether out of band or not
                break;
            }
        }
    }
}

/**
 * @brief Sets the relay for the current frequency.
 *
 * This function configures the relay based on the current frequency and relay state.
 */
static void setRelay()
{
#if NUM_RELAYS == 1
    // If there is only one relay then it is either on or off rather
    // than a relay per band
    ioWriteBandRelay( 0, currentRelay );
#else
    // Set each relay to off except for the current relay
    for( int i = 0 ; i < NUM_RELAYS ; i++ )
    {
        ioWriteBandRelay( i, currentRelay == i );
    }
#endif
}

/**
 * @brief Enables or disables the RX clock outputs.
 *
 * @param bEnable True to enable the RX clock outputs, false to disable them.
 */
static void enableRXClock( bool bEnable )
{
    oscClockEnable( RX_CLOCK_A, bEnable );
    oscClockEnable( RX_CLOCK_B, bEnable );
}

/**
 * @brief Enables or disables the TX clock output.
 *
 * @param bEnable True to enable the TX clock output, false to disable it.
 */
static void enableTXClock( bool bEnable )
{
    oscClockEnable( TX_CLOCK, bEnable );
}

/**
 * @brief Controls the transmit state.
 *
 * This function turns on or off the TX clock and PA based on the transmit state.
 * If break-in is disabled, it will not change the transmit state.
 *
 * @param bTX True to enable transmit, false to disable transmit.
 */
static void Transmit( bool bTX )
{
    if( bBreakIn )
    {
        if( bTX )
        {
            // Turn on the TX clock
            if( bTXClockEnabled )
            {
                enableTXClock( true );
                delay(txDelay);
            }

            // Set the morse output high
            if( bTXOutEnabled )
            {
                ioWriteMorseOutputHigh();
            }
        }
        else
        {
            // Delay the same as before transmit so that the dot
            // or dash is not truncated
            delay(txDelay);

            // Set the morse output low
            ioWriteMorseOutputLow();
            delay(txDelay);

            // Turn off the TX clock
            enableTXClock( false );
        }
        bTransmitting = bTX;
    }
}

/**
 * @brief Mutes or unmutes the receiver.
 *
 * This function controls the RX enable line to mute or unmute the receiver.
 *
 * @param bMute True to mute the receiver, false to unmute.
 */
static void muteRX( bool bMute )
{
    if( bMute )
    {
        ioWriteRXEnableLow();
    }
    else
    {
        ioWriteRXEnableHigh();
    }
}

/**
 * @brief Turns the sidetone on or off.
 *
 * This function enables or disables the sidetone based on the provided parameter.
 *
 * @param bOn True to turn on the sidetone, false to turn it off.
 */
static void sidetoneOn( bool bOn )
{
    if( bSidetone )
    {
        if( bOn )
        {
            ioWriteSidetoneOn();
        }
        else
        {
            ioWriteSidetoneOff();
        }
    }
}

/**
 * @brief Handles key down and key up events.
 *
 * This function manages the state changes when the key is pressed down or released.
 * It handles muting the receiver, enabling/disabling the transmitter, and controlling
 * the sidetone.
 *
 * @param bDown True if the key is pressed down, false if it is released.
 */
void keyDown( bool bDown )
{
    if( bDown )
    {
        // Key down only if TX is enabled on the current
        // TX frequency
        if( txEnabled() )
        {
            if( !bTestRXMute )
            {
                // Mute RX first
                muteRX( true );
                delay(muteDelay);
            }

            if( bRXClockEnabled )
            {
                // Turn off the RX clock
                enableRXClock( false );
            }

            // Turn on the TX oscillator and PA
            Transmit( true );

            // Turn on sidetone
            sidetoneOn( true );
        }
    }
    else
    {
        // Delay the same as before key down so that dot or dash
        // is not truncated
        delay(muteDelay);

        // Turn off the PA and TX oscillator
        Transmit( false );

        // Turn off sidetone
        sidetoneOn( false );

        if( bRXClockEnabled )
        {
            // Turn on the RX clock
            enableRXClock( true );
        }

        if( !bTestRXMute )
        {
            // Unmute the RX last
            delay(unmuteDelay);
            muteRX( false );
        }
    }
}

/**
 * @brief Displays the given Morse code text on the screen.
 *
 * This function displays the provided Morse code text on the screen
 * if the current mode is not the menu mode.
 *
 * @param text The Morse code text to display.
 */
void displayMorse( char *text )
{
/*
    if( currentMode != modeMenu )
    {
        displayText( MORSE_LINE, text, false );
    }
*/
}

#ifdef OLED_DISPLAY
/**
 * @brief Draws the cursor on the OLED screen.
 *
 * This function draws the cursor at the specified position and width on the OLED screen.
 * It also clears the previous cursor position before drawing the new one.
 *
 * @param x The x-coordinate of the cursor.
 * @param y The y-coordinate of the cursor.
 * @param width The width of the cursor.
 */
static void drawCursor( int x, int y, int width )
{
    static int prevX, prevY, prevWidth;

    // Clear previous cursor
    if( prevWidth > 0 )
    {
        oledDrawLine( prevX, prevY, prevX + prevWidth, prevY, false, true );
    }

    // Draw the new cursor
    if( width > 0 )
    {
        oledDrawLine( x, y, x + width, y, true, true );
    }

    prevX = x;
    prevY = y;
    prevWidth = width;
}
#endif

#ifdef SOTA2
/**
 * @brief Updates LEDs for the current frequency.
 *
 * For the SOTA transceiver we have 3 LEDs instead of an LCD.
 * The default frequency is the centre frequency (e.g. 14060)
 * and we have left and right frequencies (e.g. 14050 and 14070)
 * Centre LED lit between the left and right frequencies
 * Left LED lit below the centre
 * Right LED lit above the centre
 * All LEDs light out of band
 */
static void displayFrequencies( void )
{
    uint32_t freq = getRXFreq();

    // Check we are in band
    if( (freq >= band[currentBand].minFreq) &&
    (freq <= band[currentBand].maxFreq) )
    {
        // Centre LED is lit if between the left and right frequencies
        ioWriteCentreLED( (freq >= band[currentBand].leftFreq) &&
        (freq <= band[currentBand].rightFreq) );

        // Left LED is lit if below the centre frequency
        ioWriteLeftLED( freq < band[currentBand].defaultFreq);

        // Right LED is lit if above the centre frequency
        ioWriteRightLED( freq > band[currentBand].defaultFreq );
    }
    else
    {
        // Out of band so light all the LEDs
        ioWriteLeftLED( true );
        ioWriteCentreLED( true );
        ioWriteRightLED( true );
    }
}

#else
/**
 * @brief Updates the cursor position based on the current mode and VFO state.
 *
 * This function updates the cursor position on the display based on the current
 * mode (VFO, WPM, etc.) and VFO state (simplex, split, RIT, XIT).
 */
static void updateCursor()
{
    uint8_t line = FREQ_LINE;
#ifdef OLED_DISPLAY
    uint8_t y = vfoCursorTransition[cursorIndex].sy;
#endif

    // Only update the cursor if in VFO mode
    if( currentMode == modeVFO )
    {
        // If split, RIT or XIT mode may need to put the cursor on the other line
        if( bVFOSplit || vfoState[currentVFO].mode != vfoSimplex )
        {
#ifdef OLED_DISPLAY
            y = vfoCursorTransition[cursorIndex].shy;
#endif
            // Display the cursor on the correct line
            if( !bVFOFirstLine )
            {
                line = FREQ_LINE + 1;
#ifdef OLED_DISPLAY
                y += getFontHeight(HALF_FREQUENCY_FONT);
#endif                
            }
        }

#ifdef LCD_DISPLAY
        displayCursor( vfoCursorTransition[cursorIndex].x, line, cursorUnderline );
#endif

#ifdef OLED_DISPLAY
        drawCursor( vfoCursorTransition[cursorIndex].sx, y, vfoCursorTransition[cursorIndex].sWidth );
#endif        
    }
    else if( currentMode == modeWpm )
    {
#ifdef LCD_DISPLAY
        // Make the cursor blink on the wpm
        displayCursor( WPM_COL, WPM_LINE, cursorBlink );
#endif

#ifdef OLED_DISPLAY
        // Draw it on the OLED, but it won't blink
        drawCursor( wpmCursorX, wpmCursorY, wpmCursorWidth );
#endif
    }
}

/**
 * @brief Displays the given menu text on the screen.
 *
 * This function displays the provided menu text on the screen.
 *
 * @param text The menu text to display.
 */
void displayMenu( const char *text )
{
#ifdef LCD_DISPLAY
    displayText( MENU_LINE, text, true );
#endif

#ifdef OLED_DISPLAY
    // Clear previous menu line before writing new line
    oledDrawRectangle( menuX, menuY, menuWidth, getFontHeight( MENU_FONT ), false, false );
    oledWriteString( menuX, menuY, text, MENU_FONT, true );
#endif
}

#ifdef OLED_DISPLAY
/**
 * @brief Displays the frequency on the OLED screen.
 *
 * This function displays the given frequency on the specified line of the OLED screen.
 * It uses different fonts and positions for the frequency components.
 *
 * @param line The line number on the OLED screen where the frequency will be displayed.
 * @param cA The first character to display (e.g., VFO letter).
 * @param cB The second character to display (e.g., mode indicator).
 * @param freq The frequency to display.
 * @param bigFont True to use the big font, false to use the half-height font.
 */
static void displayFrequencyOLED( int line, char cA, char cB, uint32_t freq, bool bigFont )
{
    char freqText[TEXT_BUF_LEN*2];
    int font = bigFont ? FREQUENCY_FONT : HALF_FREQUENCY_FONT;
    int x = 0;
    int y = 0;
    char *format = "%5lu";
    int firstDigitPosition = x + getFontWidth(SMALL_FONT);
    int dotX = firstDigitPosition + 5*getFontWidth(font);
    int dotY =  y + getFontHeight( SMALL_FONT );

    if( line == 1 )
    {
        y = getFontHeight( HALF_FREQUENCY_FONT );
        dotY = y + getFontHeight( SMALL_FONT );

        // Clear line first
        oledDrawRectangle( x, y, OLED_WIDTH, getFontHeight( HALF_FREQUENCY_FONT ), false, false );
    }
    else if( bigFont )
    {
        y = 0;
        dotY = y + 3*getFontHeight(SMALL_FONT);

        // Clear line first
        oledDrawRectangle( x, y, OLED_WIDTH, getFontHeight( FREQUENCY_FONT ), false, false );
    }

    sprintf( freqText, "%c", cA );
    oledWriteString(x, y, freqText, SMALL_FONT, false);

    sprintf( freqText, "%c", cB );
    oledWriteString(x, y + getFontHeight(SMALL_FONT), freqText, SMALL_FONT, false);

    sprintf( freqText, format, freq/1000);

    oledWriteString(firstDigitPosition, y, freqText, font, false);

    oledWriteString (dotX, dotY, ".", SMALL_FONT, false );

    int decimalDigitPosition = dotX + getFontWidth(SMALL_FONT);
    sprintf( freqText, "%02lu", (freq%1000)/10);
    oledWriteString(decimalDigitPosition, y, freqText, font, true);
}
#endif

// Keep the wpm, volume and frequency texts so we can display one along with
// the other when updating
static char wpmText[TEXT_BUF_LEN];
static char freqText[TEXT_BUF_LEN];
static char volText[TEXT_BUF_LEN];

#ifdef LCD_DISPLAY
/**
 * @brief Displays the frequency on the LCD screen.
 *
 * This function updates the LCD screen with the current frequency and other relevant information.
 */
static void displayFrequencyLCD( void )
{
    char buf[TEXT_BUF_LEN];

    if( displayMode == displayVolume )
    {
        sprintf( buf, "%s %s", freqText, volText );
    }
    else
    {
        sprintf( buf, "%s %s", freqText, wpmText );
    }
    displayText( FREQ_LINE, buf, true );
}
#endif

/**
 * @brief Checks if the current mode is CW reverse or LSB.
 *
 * This function determines whether the current mode is either CW reverse or LSB.
 *
 * @return true if the current mode is CW reverse or LSB, false otherwise.
 */
static bool getCWReverse( void )
{
    switch( nvramReadTRXMode() )
    {
        case cwRevMode:
        case lsbMode:
            return true;
            
        default:
            return false;
    }
}

/**
 * @brief Displays the current frequencies on the screen.
 *
 * This function updates the display with the current RX and TX frequencies.
 * It handles different modes such as split, RIT, and XIT, and updates the
 * display accordingly.
 * Normally displays one frequency on the top line but in
 * split or RIT/XIT modes uses two lines
 */
static void displayFrequencies( void )
{
    // Frequency for the first and second lines
    uint32_t freq1 = 0;
    uint32_t freq2 = 0;
    
    // Character at the start of each line e.g. A, B, X, R, +, -
    char cLine1A = ' ';
    char cLine1B = ' ';
    char cLine2A = ' ';
    char cLine2B = ' ';

    // The dot to separate kHz from hundreds
    // Changes to 'x' if TX not enabled for the current
    // TX frequency
    char cDot = txEnabled() ? '.' : 'x';

    // First line begins with a letter to tell us which VFO (A or B) or
    // if the oscillator is not OK (N)
    if( bOscInit )
    {
        cLine1A = VFO_LETTER(currentVFO);
    }
    else
    {
        cLine1A = 'N';
    }

    if( bVFOSplit )
    {
        // In split mode the second line has the transmit frequency
        freq2 = getTXFreq();
        cLine2A = VFO_LETTER(OTHER_VFO);
    }
        
    // If in RIT or XIT display the appropriate letter
    if( vfoState[currentVFO].mode == vfoRIT || vfoState[currentVFO].mode == vfoXIT )
    {
        if( vfoState[currentVFO].mode == vfoRIT )
        {
            cLine2A = 'R';
        }
        else
        {
            cLine2A = 'X';
        }

        // The second line has the offset
        // If the offset is negative then display it with a minus sign
        if( vfoState[currentVFO].offset < 0 )
        {
            freq2 = -vfoState[currentVFO].offset;
            cLine2B = '-';
        }
        else
        {
            // Positive offset
            freq2 = vfoState[currentVFO].offset;
            cLine2B = '+';
        }
    }

#ifdef LCD_DISPLAY
    // Display if we are in CW reverse mode
    if( getCWReverse() )
    {
        cLine1B = '^';
    }
#endif

    // Line 1 has the RX frequency
    freq1 = getRXFreq();

#ifdef OLED_DISPLAY
    bool bigFont = true;
#endif

    // All modes other than simplex have a second line
    if( bVFOSplit || (vfoState[currentVFO].mode != vfoSimplex) )
    {
        sprintf( freqText, "%c%c%5lu%c%02lu", cLine2A, cLine2B, freq2/1000, cDot, (freq2%1000)/10);

#ifdef LCD_DISPLAY
        displaySplitLine( 0, FREQ_LINE + 1 );
        displayText( FREQ_LINE + 1, freqText, true );
#endif

#ifdef OLED_DISPLAY
        bigFont = false;

        // Display on the OLED
        displayFrequencyOLED( FREQ_LINE + 1, cLine2A, cLine2B, freq2, bigFont );
#endif

#ifdef LCD_DISPLAY
        // Second line is split so frequency not overwritten if we are displaying morse
        displaySplitLine( 10, FREQ_LINE + 1 );
#endif
    }
    else
    {
        // Second line is not split and needs to be cleared if not in the menu or quick menu
        if( (currentMode != modeQuickMenu) && (currentMode != modeMenu) )
        {
#ifdef LCD_DISPLAY
            displaySplitLine( 0, FREQ_LINE + 1 );
#endif
            displayMenu( "" );
        }
    }

    // Line 1 has the frequency as determined above
    sprintf( freqText, "%c%c%5lu%c%02lu", cLine1A, cLine1B, freq1/1000, cDot, (freq1%1000)/10);

#ifdef LCD_DISPLAY
    // Display on the LCD
    displayFrequencyLCD();
#endif

#ifdef OLED_DISPLAY
    // Display on the OLED
    displayFrequencyOLED( FREQ_LINE, cLine1A, cLine1B, freq1, bigFont );
#endif
}

/**
 * @brief Displays the current Morse code speed (WPM) on the screen.
 *
 * This function updates the display with the current Morse code speed (words per minute).
 * If the speed is 0, it indicates straight key mode or tune mode.
 */
static void displayMorseWpm( void )
{
    int wpm = morseGetWpm();

    // Create the text for the morse speed
    if( wpm > 0 )
    {
        sprintf( wpmText, "%2dwpm", wpm);
    }
    else
    {
        // A morse wpm of 0 means straight key mode
        // May be in tune mode (continuous transmit until dot pressed)
        if( morseInTuneMode() )
        {
            sprintf( wpmText, "Tune " );
        }
        else
        {
            sprintf( wpmText, "SKey " );
        }
    }

#ifdef LCD_DISPLAY
    // Display on the LCD
    displayFrequencyLCD();
#endif

#ifdef OLED_DISPLAY
    // Display on the OLED
    oledWriteString( wpmX, wpmY, wpmText, WPM_FONT, true);
#endif
}

#ifdef PSDR
/**
 * @brief Displays the current volume and input boost gain on the screen.
 *
 * This function updates the display with the current volume and input boost gain.
 * It handles both OLED and LCD displays.
 */
static void displayVol( void )
{
//    sprintf( volText, "%c:%d%c%02d", ifBelow ? '-' : '+', inputShift, bVolumeMode ? '>' : '<', ioGetVolume());
    sprintf( volText, "%c:%d%c%02d", ifBelow ? '-' : '+', WM8960GetInputBoostGain(), bVolumeMode ? '>' : '<', ioGetVolume());

#ifdef OLED_DISPLAY
    // Display on the OLED
    oledWriteString( volX, volY, volText, VOLUME_FONT, true);
#endif

#ifdef LCD_DISPLAY
    // Display on the LCD
    displayFrequencyLCD();
#endif
}

#ifdef DISPLAY_AGC
/**
 * @brief Displays the Automatic Gain Control (AGC) information.
 *
 * This function is responsible for displaying the AGC information.
 * It retrieves and processes the necessary data to provide a visual
 * representation of the AGC status.
 */
static void displayAgc( void )
{
#ifdef OLED_DISPLAY
    char buf[TEXT_BUF_LEN];
    sprintf( buf, "%6u %6d", agcAmplitude / AGC_BUFFER_LEN / AGC_BUFFER_LEN, agcGain );

    // Display on the OLED
    oledWriteString( menuX, menuY, buf, MENU_FONT, true);
#endif
}
#endif

/**
 * @brief Displays the preamp status on the screen.
 *
 * This function updates the display with the current status of the preamp,
 * ADC overload, and output overload. It handles both OLED and LCD displays.
 */
static void displayPreamp( void )
{
    char buf[TEXT_BUF_LEN];
    char preampChar, adcChar, outChar;

    if( bPreampOn )
    {
        preampChar = 'P';
    }
    else
    {
        preampChar = 'p';
    }

    if( adcOverload )
    {
        adcChar = 'A';
    }
    else
    {
        adcChar = 'a';
    }

    if( outOverload == 1)
    {
        outChar = '+';
    }
    else if( outOverload == -1)
    {
        outChar = '-';
    }
    else
    {
        outChar = '0';
    }

    sprintf( buf, "%c%c%c", preampChar, adcChar, outChar );

#ifdef OLED_DISPLAY
    // Display on the OLED
    oledWriteString( preampX, preampY, buf, PREAMP_FONT, true);

#ifdef DISPLAY_MIN_MAX
    sprintf(buf, "%04x %04x %04x", minIn, maxIn, maxOut );
    oledWriteString( menuX, menuY, buf, WPM_FONT, true);
#endif
#endif
}
#endif

#ifdef OLED_DISPLAY

#if 0
static char *getModeText( void )
{
    switch( nvramReadTRXMode() )
    {
        case cwMode:
            return "CW";

        case cwRevMode:
            return "CWR";

        case lsbMode:
            return "LSB";

        case usbMode:
            return "USB";

        default:
            return "???";
    }
}

static void displayTRXMode( void )
{
    // Clear previous mode text before writing new text
    oledDrawRectangle( modeX, modeY, modeWidth, getFontHeight( MODE_FONT ), false, false );
    oledWriteString (modeX, modeY, getModeText(), MODE_FONT, true );
}
#else
/**
 * @brief Displays the current output mode on the OLED screen.
 *
 * This function updates the display with the current output mode,
 * which can be normal, binaural, or peaked.
 */
static void displayOutputMode( void )
{
    char *output;

    // Clear previous mode text before writing new text
    oledDrawRectangle( modeX, modeY, modeWidth, getFontHeight( MODE_FONT ), false, false );

    switch( outputMode )
    {
        case NORMAL_OUTPUT:
            output = "CW ";
            break;

        case BINAURAL_OUTPUT:
            output = "CWB";
            break;

        case PEAKED_OUTPUT:
            output = "CWP";
            break;

        default:
            output = "CW?";
            break;
    }
    oledWriteString (modeX, modeY, output, MODE_FONT, true );
}
#endif

/**
 * @brief Displays the current filter setting on the OLED screen.
 *
 * This function updates the display with the current filter setting.
 */
static void displayFilter( void )
{
    // Clear previous filter text before writing new text
    oledDrawRectangle( filterX, filterY, filterWidth, getFontHeight( FILTER_FONT ), false, false );
    oledWriteString( filterX, filterY, ioGetFilterText(), FILTER_FONT, true );
}
#endif
#endif

/**
 * @brief Sets the RX frequency.
 *
 * This function configures the RX frequency based on the current mode (CW, CW reverse, LSB, USB)
 * and the intermediate frequency. It also sets the appropriate quadrature phase shift and relay state.
 *
 * @param freq The frequency to set for RX.
 */
static void setRXFrequency( uint32_t freq )
{
    // For CW the RX oscillator has to be offset for the CW tone to be audible
    // but not for SSB. Also need to set the correct quadrature for either
    // sideband.
    int offset = 0;
    int q = 0;

#ifdef SOTA2
    offset = -RX_OFFSET;
    q = -1;
#else
    switch( nvramReadTRXMode() )
    {
        default:
        case cwMode:
            // Normal mode is USB so need the LO below the RX frequency
            offset = -RX_OFFSET;
            q = -1;
            break;

        case cwRevMode:
            // Reverse mode is LSB so need the LO above the RX frequency
            offset = RX_OFFSET;
            q = 1;
            break;

        case lsbMode:
            q = 1;
            break;

        case usbMode:
            q = -1;
            break;
    }
#endif

#ifdef PSDR
    if( ifBelow )
    {
        freq -= INTERMEDIATE_FREQUENCY;
    }
    else
    {
        freq += INTERMEDIATE_FREQUENCY;
    }

    oscSetFrequency( RX_CLOCK_A, freq, 0 );
    oscSetFrequency( RX_CLOCK_B, freq, q );
#else

    // Set the oscillator frequency.
    if( intermediateFrequency == 0 )
    {
        // Direct conversion so set the correct quadrature phase shift.
        oscSetFrequency( RX_CLOCK_A, freq + offset, 0 );
        oscSetFrequency( RX_CLOCK_B, freq + offset, q );
    }
    else
    {
        // Superhet so set the LFO and BFO
        oscSetFrequency( RX_CLOCK_A, freq + intermediateFrequency, 0 );
        oscSetFrequency( RX_CLOCK_B, intermediateFrequency - RX_OFFSET, 0 );
    }
#endif

    // Set the relay
    setRelay();
}

/**
 * @brief Sets the RX and TX frequencies based on the current VFO settings.
 *
 * This function updates the RX and TX frequencies according to the current VFO settings.
 * It also updates the display and cursor to reflect the new frequencies.
 */
static void setFrequencies()
{
    // See if this is a new band and if so set the new band
    setBandFromFrequency( getRXFreq() );

    // Set RX and TX frequencies
    setRXFrequency( getRXFreq() );
    oscSetFrequency( TX_CLOCK, getTXFreq(), 0 );

    // Ensure the display and cursor reflect this
    displayFrequencies();
#ifndef SOTA2
    updateCursor();
#endif
}

#ifndef SOTA2
// Turn off the cursor
/**
 * @brief Turns off the cursor on the display.
 *
 * This function disables the cursor on both LCD and OLED displays.
 */
static void turnCursorOff( void )
{
#ifdef LCD_DISPLAY
    displayCursor( 0, 0, cursorOff );
#endif

#ifdef OLED_DISPLAY
    drawCursor( 0, 0, 0 );
#endif
}
#endif

/**
 * @brief Sets the band to the specified new band.
 *
 * This function updates the VFO states, split mode, and frequencies
 * to match the specified new band. It also stores the new band in NVRAM.
 *
 * @param newBand The new band to set.
 */
static void setBand( int newBand )
{
    // Set the default frequency for both VFOs
    // Go into simplex mode
    vfoState[VFO_A].freq = band[newBand].defaultFreq;
    vfoState[VFO_A].offset = 0;
    vfoState[VFO_A].mode = vfoSimplex;
    vfoState[VFO_B].freq = band[newBand].defaultFreq;
    vfoState[VFO_B].offset = 0;
    vfoState[VFO_B].mode = vfoSimplex;

    // Turn off split mode
    bVFOSplit = false;

    // Set the new TX and RX frequencies
    setFrequencies();
}

#ifndef SOTA2
/**
 * @brief Displays the current menu or sub-menu text on the screen.
 *
 * This function updates the display with the current menu or sub-menu text.
 * It also turns off the cursor after displaying the text.
 */
static void menuDisplayText()
{
    char text[TEXT_BUF_LEN];

#ifdef LCD_DISPLAY
    // Turn off the split line for the menu
    displaySplitLine( 0, MENU_LINE );
#endif

    // Display either the menu or the sub-menu text
    if( currentSubMenu == 0 )
    {
        // Not in a sub-menu
        sprintf( text, "%c %s", 'A' + currentMenu, menu[currentMenu].text );
    }
    else
    {
        // In a sub-menu
        sprintf( text, "%c.%d %s", 'A' + currentMenu, currentSubMenu, menu[currentMenu].subMenu[currentSubMenu].text );
    }
    
    displayMenu( text );

    turnCursorOff();    
}

/**
 * @brief Enter the WPM (Words Per Minute) setting mode.
 *
 * This function switches the current mode to WPM setting mode,
 * updates the display mode to show WPM, and displays the current
 * Morse code speed (WPM) on the screen. It also updates the cursor
 * position to reflect the new mode.
 */
static void enterWpm()
{
    currentMode = modeWpm;
    displayMode = displayWpm;

    displayMorseWpm();
    updateCursor();
}

/**
 * @brief Enter the menu mode.
 *
 * This function switches the current mode to menu mode,
 * initialises the menu state, and displays the top-level menu.
 */
static void enterMenu()
{
    // In the menu but not yet in a menu item
    currentMenu = currentSubMenu = 0;
    currentMode = modeMenu;
    bInMenuItem = false;
    displayMode = DEFAULT_DISPLAY_MODE;

    // Display the current menu text
    menuDisplayText();
}

/**
 * @brief Quick way to enter the VFO band menu.
 *
 * This function sets the current menu to the VFO menu and the current sub-menu
 * to the VFO band menu item. It also sets the mode to menu mode and indicates
 * that we are in a quick menu item. The VFO band menu is then displayed.
 */
static void enterVFOBandMenu()
{
    currentMenu = VFO_MENU;
    currentSubMenu = 1;
    currentMode = modeMenu;
    bInMenuItem = true;
    displayMode = DEFAULT_DISPLAY_MODE;

    // We are entering the menu quickly so we can get out quickly
    bInQuickMenuItem = true;

    // Display the current menu text
    menuVFOBand(0);

    turnCursorOff();    
}

#ifdef PSDR
#ifdef LCD_DISPLAY
/**
 * @brief Quick way to the filter menu.
 *
 * This function sets the current menu to the SDR menu and the current sub-menu
 * to the filter menu item. It also sets the mode to menu mode and indicates
 * that we are in a quick menu item. The filter menu is then displayed.
 */
static void enterSDRFilterMenu()
{
    currentMenu = SDR_MENU;
    currentSubMenu = SDR_FILTER_MENU_ITEM;
    currentMode = modeMenu;
    bInMenuItem = true;
    displayMode = DEFAULT_DISPLAY_MODE;

    // We are entering the menu quickly so we can get out quickly
    bInQuickMenuItem = true;

    // Display the current menu text
    menuFilter(0);

    turnCursorOff();    
}
#endif
#endif

/**
 * @brief Enter the VFO (Variable Frequency Oscillator) mode.
 *
 * This function switches the current mode to VFO mode, updates the display mode,
 * and ensures the cursor and display reflect the current VFO state.
 */
static void enterVFOMode()
{
    // Go to VFO mode
    currentMode = modeVFO;
    displayMode = DEFAULT_DISPLAY_MODE;

    // Next time we enter the menu start not in a menu item
    bEnteredMenuItem = false;

    // Display the cursor in the right place
    updateCursor();

    // Clear the second line as no longer in the menu
#ifdef LCD_DISPLAY
    displaySplitLine( 0, FREQ_LINE + 1 );
#endif    
    displayMenu( "" );

    // Update the display with the correct split for the current VFO mode
    displayFrequencies();
}

/**
 * @brief Swaps the VFOs.
 *
 * This function swaps the current VFO with the other VFO and updates
 * the frequencies and display accordingly.
 * Called either from the quick menu or CAT control.
 */
void vfoSwap()
{
    // Swap the VFOs
    currentVFO = OTHER_VFO;

    // Update the frequencies and display
    setFrequencies();
}

/**
 * @brief Quick menu item Swap VFOs.
 *
 * This function swaps the VFOs and updates the frequencies and display accordingly.
 */
static void quickMenuSwap()
{
    vfoSwap();
}

/**
 * @brief Sets the other VFO to the current VFO.
 *
 * This function copies the state of the current VFO to the other VFO.
 * If in split mode, it ensures the transmit frequency is updated.
 * Called either from the quick menu or CAT control.
*/
void vfoEqual()
{
    vfoState[OTHER_VFO] = vfoState[currentVFO];

    // If in split mode need to ensure the transmit frequency is updated
    if( bVFOSplit )
    {
        setFrequencies();
    }
}

/**
 * @brief Quick menu item Equal VFOs.
 *
 * This function sets the other VFO to the current VFO and updates the frequencies and display accordingly.
 */
static void quickMenuEqual()
{
    vfoEqual();
}

/**
 * @brief Sets the RIT (Receiver Incremental Tuning) mode for the current VFO.
 *
 * This function enables or disables the RIT mode for the current VFO.
 * When RIT is enabled, the offset is set to zero and the cursor is moved
 * to the finest frequency control position. When RIT is disabled, the
 * VFO mode is set to simplex and the cursor is restored to its previous position.
 * Called either from the quick menu or CAT control.
 *
 * @param bRIT True to enable RIT mode, false to disable it.
 */
void setCurrentVFORIT( bool bRIT )
{
    // Ignore if in split mode
    if( !bVFOSplit )
    {
        if( bRIT )
        {
            vfoState[currentVFO].mode = vfoRIT;

            // Always on the second line in RIT mode
            bVFOFirstLine = false;

            // Start with no offset
            setCurrentVFOOffset( 0 );

            // Remember the current cursor index so when we come out of RIT
            // it goes back where it was
            prevCursorIndex = cursorIndex;

            // In RIT mode want the finest frequency control by default
            cursorIndex = FINEST_CURSOR_INDEX;
        }
        else
        {
            vfoState[currentVFO].mode = vfoSimplex;

            // Put the cursor back where it was
            cursorIndex = prevCursorIndex;
        }

        // Update the frequencies and display
        setFrequencies();
        updateCursor();
    }
}

/**
 * @brief Quick menu item RIT selected.
 *
 * This function toggles the RIT mode for the current VFO.
 * If the current VFO is in RIT mode, it switches back to simplex mode.
 * If the current VFO is in simplex or XIT mode, it switches to RIT mode.
 */
static void quickMenuRIT()
{
    switch( vfoState[currentVFO].mode )
    {
        // If in RIT then back to simplex
        case vfoRIT:
            setCurrentVFORIT( false );
            break;
        // If in simplex or XIT then go to RIT
        case vfoSimplex:
        case vfoXIT:
        default:
            setCurrentVFORIT( true );
            break;
    }

    enterVFOMode();
}

/**
 * @brief Sets the XIT (Transmit Incremental Tuning) mode for the current VFO.
 *
 * This function enables or disables the XIT mode for the current VFO.
 * When XIT is enabled, the offset is set to zero and the cursor is moved
 * to the finest frequency control position. When XIT is disabled, the
 * VFO mode is set to simplex and the cursor is restored to its previous position.
 * Called either from the quick menu or CAT control.
 *
 * @param bXIT True to enable XIT mode, false to disable it.
 */
void setCurrentVFOXIT( bool bXIT )
{
    // Ignore if in split mode
    if( !bVFOSplit )
    {
        if( bXIT )
        {
            vfoState[currentVFO].mode = vfoXIT;

            // Always on the second line in XIT mode
            bVFOFirstLine = false;

            // Start with no offset
            setCurrentVFOOffset( 0 );

            // Remember the current cursor index so when we come out of RIT
            // it goes back where it was
            prevCursorIndex = cursorIndex;

            // In RIT mode want the finest frequency control by default
            cursorIndex = FINEST_CURSOR_INDEX;
        }
        else
        {
            vfoState[currentVFO].mode = vfoSimplex;

            // Put the cursor back where it was
            cursorIndex = prevCursorIndex;
        }

        // Update the frequencies and display
        setFrequencies();
        updateCursor();
    }
}

/**
 * @brief Quick menu item XIT selected.
 *
 * This function toggles the XIT mode for the current VFO.
 * If the current VFO is in XIT mode, it switches back to simplex mode.
 * If the current VFO is in simplex or RIT mode, it switches to XIT mode.
 */
static void quickMenuXIT()
{
    switch( vfoState[currentVFO].mode )
    {
        // If in XIT then back to simplex
        case vfoXIT:
            setCurrentVFOXIT( false );
            break;

        // If in simplex or RIT then go to XIT
        case vfoSimplex:
        case vfoRIT:
        default:
            setCurrentVFOXIT( true );
            break;
    }

    enterVFOMode();
}

/**
 * @brief Sets the VFO split state.
 *
 * This function enables or disables the VFO split mode. When split mode is enabled,
 * the transmit and receive frequencies are set to different VFOs. When split mode
 * is disabled, the VFOs are set to simplex mode.
 *
 * @param bSplit True to enable split mode, false to disable it.
 */
void setVFOSplit( bool bSplit )
{
    // Ignore if no change
    if( bSplit != bVFOSplit )
    {
        // Moving into split
        if( bSplit )
        {
            // Before moving into split mode, work out the current
            // TX and RX frequencies.
            uint32_t txFreq = getTXFreq();
            uint32_t rxFreq = getRXFreq();

            // Now ensure RIT and XIT are off with zero offsets
            vfoState[currentVFO].mode = vfoSimplex;
            vfoState[currentVFO].offset = 0;
            vfoState[OTHER_VFO].mode = vfoSimplex;
            vfoState[OTHER_VFO].offset = 0;

            // Set the current VFO to the RX frequency and the other
            // VFO to the TX frequency
            vfoState[currentVFO].freq = rxFreq;
            vfoState[OTHER_VFO].freq = txFreq;
        }

        bVFOSplit = bSplit;

        // Update the frequencies and display
        setFrequencies();
    }
}

/**
 * @brief Quick menu item Split selected.
 *
 * This function toggles the split mode for the VFOs.
 * If the VFOs are in split mode, it switches back to simplex mode.
 * If the VFOs are in simplex mode, it switches to split mode.
 */
static void quickMenuSplit()
{
    // Toggle split state
    setVFOSplit( !bVFOSplit );
    enterVFOMode();
}

/**
 * @brief Displays the quick menu text on the screen.
 *
 * This function updates the display with the current quick menu text.
 * It also positions the cursor on the current quick menu item.
 */
static void quickMenuDisplayText()
{
#ifdef LCD_DISPLAY
    // Turn off the split line for the menu
    displaySplitLine( 0, MENU_LINE );
#endif

    // Display the quick menu
    // Slightly different text in split mode
    displayMenu( (bVFOSplit ? QUICK_MENU_SPLIT_TEXT : QUICK_MENU_TEXT) );
    
#ifdef LCD_DISPLAY
    // Make the cursor blink on the current item
    displayCursor( quickMenu[quickMenuItem].pos, MENU_LINE, cursorBlink );
#endif

#ifdef OLED_DISPLAY
    // Display the cursor on the OLED item
    drawCursor( quickMenu[quickMenuItem].sx, menuCursorY, quickMenu[quickMenuItem].sWidth );
#endif
}

/**
 * @brief Enters the quick menu.
 *
 * This function is responsible for transitioning the application
 * into the quick menu mode. It performs necessary setup and 
 * initialisation required for the quick menu to function properly.
 */
static void enterQuickMenu()
{
    currentMode = modeQuickMenu;
    displayMode = DEFAULT_DISPLAY_MODE;

    // Display the current menu text
    quickMenuDisplayText();
}

/**
 * @brief Handles the rotary control while in the menu mode.
 *
 * This function processes the rotary control and button presses
 * when the system is in menu mode. It navigates through the menu
 * items and sub-menu items, and handles entering and exiting menu items.
 *
 * @param inputState The current state of the rotary control and buttons.
 */
static void rotaryMenu( uint16_t inputState )
{
    // If in a menu item then pass control to its function
    if( bInMenuItem )
    {
        // We'll let the menu function have first go at dealing with
        // any presses etc. Only if it hasn't used it will we do
        // anything
        if( !menu[currentMenu].subMenu[currentSubMenu].func( inputState ) )
        {
            // A long press takes us out of the menu item
            if( bLongPress )
            {
                // Now out of the menu item
                bInMenuItem = false;

                // Display the current menu text
                menuDisplayText();
            }
        }
    }
    else
    {
        // May be in the top level menu or a sub menu
        if( currentSubMenu == 0 )
        {
            // In the top level menu
            if( bShortPressRight )
            {
                if( currentMenu == (NUM_MENUS-1))
                {
                    currentMenu = 0;
                }
                else
                {
                    currentMenu++;
                }

                // Display the new menu item
                menuDisplayText();
            }
            else if( bShortPressLeft )
            {
                if( currentMenu == 0)
                {
                    currentMenu = (NUM_MENUS-1);
                }
                else
                {
                    currentMenu--;
                }

                // Display the new menu item
                menuDisplayText();
            }
            else if( bShortPress )
            {
                // A short press takes us into the sub menu
                currentSubMenu = 1;

                // Display the new menu item
                menuDisplayText();
            }
            else if( bLongPress )
            {
                enterVFOMode();
            }
        }
        else
        {
            // In a sub menu
            if( bShortPressRight )
            {
                if( currentSubMenu == (menu[currentMenu].numItems-1))
                {
                    currentSubMenu = 1;
                }
                else
                {
                    currentSubMenu++;
                }

                // Display the new menu item
                menuDisplayText();
            }
            else if( bShortPressLeft )
            {
                if( currentSubMenu == 1)
                {
                    currentSubMenu = (menu[currentMenu].numItems-1);
                }
                else
                {
                    currentSubMenu--;
                }

                // Display the new menu item
                menuDisplayText();
            }
            else if( bShortPress )
            {
                // Now in the menu item
                bInMenuItem = true;
                bEnteredMenuItem = true;

                // A short press on a menu item calls its function
                menu[currentMenu].subMenu[currentSubMenu].func( 0 );
            }
            else if( bLongPress )
            {
                // A long press take us out of the sub menu
                currentSubMenu = 0;

                // Display the new menu item
                menuDisplayText();
            }
        }
    }
}

/**
 * @brief Enter simplex VFO mode.
 *
 * This function sets the current VFO mode to simplex, disables RIT and XIT modes,
 * and ensures the VFO is not in split mode. It then enters VFO mode.
 */
static void enterSimplex()
{
    setCurrentVFORIT( false );
    setCurrentVFOXIT( false );
    setVFOSplit( false );
    enterVFOMode();
}

/**
 * @brief Handles the rotary control while in the quick menu mode.
 *
 * This function processes the rotary control and button presses
 * when the system is in quick menu mode. It navigates through the
 * quick menu items and handles entering and exiting the quick menu.
 *
 * @param inputState The current state of the rotary control and buttons.
 */
static void rotaryQuickMenu( uint16_t inputState )
{
    if( bShortPressRight )
    {
        if( quickMenuItem == (NUM_QUICK_MENUS-1))
        {
            quickMenuItem = 0;
        }
        else
        {
            quickMenuItem++;
        }

        // Display the new menu item
        quickMenuDisplayText();
    }
    else if( bShortPressLeft )
    {
        if( quickMenuItem == 0)
        {
            quickMenuItem = (NUM_QUICK_MENUS-1);
        }
        else
        {
            quickMenuItem--;
        }

        // Display the new menu item
        quickMenuDisplayText();
    }
    else if( bShortPress )
    {
        // A short press calls the function for this item and takes us out the quick menu
        quickMenu[quickMenuItem].func();
        enterVFOMode();
    }
    else if( bLongPress )
    {
        // A long press take us out of the menu
        enterVFOMode();
    }
    else if( bLongPressLeft )
    {
        // A long left press puts us back to simplex VFO mode
        enterSimplex();
    }
    else if( bLongPressRight )
    {
        // A long right press puts us into WPM mode
        enterWpm();
    }
}

/**
 * @brief Returns the next band in the specified direction.
 *
 * This function determines the next band based on the current band,
 * the direction of change, and whether all bands are allowed or only
 * those marked for quick VFO menu.
 *
 * @param oldBand The current band.
 * @param direction The direction to change the band (+1 for next, -1 for previous).
 * @param bAllBands True to include all bands, false to include only quick VFO menu bands.
 * @return The next band.
 */
static uint8_t nextBand( uint8_t oldBand, int8_t direction, bool bAllBands )
{
    // The new band starts with the old band - signed as may try to go back from 0
    int8_t newBand = oldBand;

    // We keep looping until we find a band we want to appear
    for(;;)
    {
        newBand += direction;

        if( newBand >= NUM_BANDS )
        {
            newBand = 0;
        }
        else if( newBand < 0 )
        {
            newBand = NUM_BANDS - 1;
        }

        // If all bands are valid then can stop looping
        // otherwise only stop if the band is meant to appear
        if( bAllBands || band[newBand].bQuickVFOMenu )
        {
            break;
        }
    }

    return newBand;
}

/**
 * @brief Handles the menu for changing the VFO band.
 *
 * This function processes the rotary control and button presses
 * to navigate and select the VFO band. It updates the display
 * with the current band and allows the user to set a new band.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuVFOBand( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    // The new band - need to maintain this between calls
    static int newBand;
    
    // If just entered the menu note the current band
    if( !(bCW || bCCW || bShortPress || bLongPress || bShortPressLeft || bLongPressLeft || bShortPressRight || bLongPressRight) )
    {
        newBand = currentBand;
    }

    // Right and left buttons change band
    if( bShortPressRight )
    {
        newBand = nextBand( newBand, +1, !bInQuickMenuItem );
        bUsed = true;
    }
    else if( bShortPressLeft )
    {
        newBand = nextBand( newBand, -1, !bInQuickMenuItem );
        bUsed = true;
    }

    // Display the current band
    char buf[TEXT_BUF_LEN];
    sprintf( buf, "Band: %s", band[newBand].bandName);
    displayMenu( buf );

    // Short press sets the new band
    if( bShortPress )
    {
        // Nothing to do unless the band has changed
        if( newBand != currentBand )
        {
            // Set the new band
            setBand( newBand );

            // Store the band in the NVRAM
            nvramWriteBand( newBand );

            // Leave the menu and go back to VFO mode
            enterVFOMode();

            // Left the menu
            bInQuickMenuItem = false;
        }
        
        bUsed = true;
    }

    // If we entered the menu quickly then a long press takes us out
    // and back into VFO mode
    if( bLongPress && bInQuickMenuItem )
    {
        bInQuickMenuItem = false;
        bInMenuItem = false;
        enterVFOMode();
        bUsed = true;
    }
    
    return bUsed;
}

/**
 * @brief Sets the TX/RX mode.
 *
 * This function updates the TX/RX mode and stores the new mode in NVRAM.
 * It also updates the frequencies to reflect the new mode.
 *
 * @param mode The new TX/RX mode to set.
 */
void setTRXMode( enum eTRXMode mode )
{
    // Store in the NVRAM
    nvramWriteTRXMode( mode );

    //displayTRXMode();

    // Action the change in sideband
    setFrequencies();
}

/**
 * @brief Sets the CW reverse state.
 *
 * This function updates the CW reverse state and stores the new state in NVRAM.
 *
 * @param bCWReverse True to enable CW reverse mode, false to disable it.
 */
void setCWReverse( bool bCWReverse )
{    // Store in the NVRAM
    setTRXMode( bCWReverse ? cwRevMode : cwMode );
}

/**
 * @brief Handles the menu for changing the VFO mode.
 *
 * This function processes the rotary control and button presses
 * to navigate and select the VFO mode. It updates the display
 * with the current mode and allows the user to set a new mode.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuVFOMode( uint16_t inputState )
{
    // Get the mode from NVRAM
    enum eCurrentMode mode = nvramReadTRXMode();

    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    // Left or right changes
    if( bShortPressRight )
    {
        mode = (mode+1) % NUM_TRX_MODES;

        // Set the new mode
        setTRXMode( mode );

        bUsed = true;
    }
    else if( bShortPressLeft )
    {
        if( mode == 0 )
        {
            mode = NUM_TRX_MODES - 1;
        }
        else
        {
            mode--;
        }

        // Set the new mode
        setTRXMode( mode );

        bUsed = true;
    }

    switch( mode )
    {
        case cwMode:
            displayMenu( "VFO CW Normal" );
            break;

        case cwRevMode:
            displayMenu( "VFO CW Reverse" );
            break;

        case lsbMode:
            displayMenu( "VFO LSB" );
            break;

        case usbMode:
            displayMenu( "VFO USB" );
            break;
    }
    
    return bUsed;
}

#ifdef PSDR
#ifdef LCD_DISPLAY
/**
 * @brief Handles the menu for changing the filter.
 *
 * This function processes the rotary control and button presses
 * to navigate and select the filter. It updates the display
 * with the current filter and allows the user to set a new filter.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuFilter( uint16_t inputState )
{
    uint8_t currentFilter = ioGetFilter();

    bool bDisplay = !(bCW || bCCW || bShortPress || bLongPress || bShortPressLeft || bLongPressLeft || bShortPressRight || bLongPressRight);

    // Set to true if we have used the presses etc
    bool bUsed = false;

    if( bShortPressRight )
    {
        currentFilter = (currentFilter+1)%ioGetNumFilters();
        bDisplay = true;
        bUsed = true;
    }
    else if( bShortPressLeft )
    {
        if( currentFilter == 0 )
        {
            currentFilter = ioGetNumFilters() - 1;
        }
        else
        {
            currentFilter--;
        }
        bDisplay = true;
        bUsed = true;
    }
    // If we entered the menu quickly then a short or long press takes us out
    // and back into VFO mode
    else if( (bShortPress || bLongPress) && bInQuickMenuItem )
    {
        bInQuickMenuItem = false;
        bInMenuItem = false;
        enterVFOMode();
        bUsed = true;
        bDisplay = false;
    }

    if( bDisplay )
    {
        ioSetFilter( currentFilter );
        displayMenu( ioGetFilterText() );
    }

    return bUsed;
}
#endif

static bool menuHilbertFilter( uint16_t inputState )
{
    uint8_t currentFilter = ioGetHilbertFilter();

    bool bDisplay = !(bCW || bCCW || bShortPress || bLongPress || bShortPressLeft || bLongPressLeft || bShortPressRight || bLongPressRight);

    // Set to true if we have used the presses etc
    bool bUsed = false;

    if( bShortPressRight )
    {
        currentFilter = (currentFilter+1)%ioGetNumHilbertFilters();
        bDisplay = true;
        bUsed = true;
    }
    else if( bShortPressLeft )
    {
        if( currentFilter == 0 )
        {
            currentFilter = ioGetNumHilbertFilters() - 1;
        }
        else
        {
            currentFilter--;
        }
        bDisplay = true;
        bUsed = true;
    }
    // If we entered the menu quickly then a short or long press takes us out
    // and back into VFO mode
    else if( (bShortPress || bLongPress) && bInQuickMenuItem )
    {
        bInQuickMenuItem = false;
        bInMenuItem = false;
        enterVFOMode();
        bUsed = true;
        bDisplay = false;
    }

    if( bDisplay )
    {
        ioSetHilbertFilter( currentFilter );
        displayMenu( ioGetHilbertFilterText() );
    }

    return bUsed;
}

/**
 * @brief Handles the menu for changing the SDR output source.
 *
 * This function processes the rotary control and button presses
 * to navigate and select the SDR output source. It updates the display
 * with the current output source and allows the user to set a new source.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuSDR( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    if( bShortPressRight )
    {
        outputSource = (outputSource+1)%sdrNum;
        bUsed = true;
    }
    else if( bShortPressLeft )
    {
        if( outputSource == 0 )
        {
            outputSource = sdrNum - 1;
        }
        else
        {
            outputSource--;
        }
        bUsed = true;
    }

    switch(outputSource)
    {
        case sdrIMQ:
            displayMenu( "I-Q" );
            break;
        case sdrIPQ:
            displayMenu( "I+Q" );
            break;
        case sdrI:
            displayMenu( "I" );
            break;
        case sdrQ:
            displayMenu( "Q" );
            break;
        case sdrIHilbert:
            displayMenu( "I Hilbert" );
            break;
        case sdrQHilbert:
            displayMenu( "Q Hilbert" );
            break;
        case sdrIDelayed:
            displayMenu( "I Delayed" );
            break;
        case sdrQDelayed:
            displayMenu( "Q Delayed" );
            break;
        case sdrSineWave:
            displayMenu( "Sinewave" );
            break;
        case sdrSilence:
            displayMenu( "Silence" );
            break;
        default:
            displayMenu( "Invalid I/Q" );
            break;
    }

    return bUsed;
}
#endif

/**
 * @brief Handles the menu for enabling or disabling break-in.
 *
 * This function processes the rotary control and button presses
 * to toggle the break-in mode. It updates the display with the
 * current break-in status.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuBreakIn( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    // Left or right toggles
    if( bShortPressLeft || bShortPressRight )
    {
        bBreakIn = !bBreakIn;
        bUsed = true;
    }

    if( bBreakIn )
    {
        displayMenu( "Break in: On" );
    }
    else
    {
        displayMenu( "Break in: Off" );
    }
    
    return bUsed;
}

/**
 * @brief Handles the menu for enabling or disabling the sidetone.
 *
 * This function processes the rotary control and button presses
 * to toggle the sidetone. It updates the display with the
 * current sidetone status.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuSidetone( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    // Left or right toggles
    if( bShortPressLeft || bShortPressRight )
    {
        bSidetone = !bSidetone;
        bUsed = true;
    }

    if( bSidetone )
    {
        displayMenu( "Sidetone: Enabled" );
    }
    else
    {
        displayMenu( "Sidetone: Disabled" );
    }
    
    return bUsed;
}

/**
 * @brief Handles the menu for testing RX mute.
 *
 * This function processes the rotary control and button presses
 * to toggle the RX mute test mode. It updates the display with the
 * current RX mute test status.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuTestRXMute( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    // Left or right toggles
    if( bShortPressLeft || bShortPressRight )
    {
        bTestRXMute = !bTestRXMute;
        bUsed = true;
    }

    if( bTestRXMute )
    {
        muteRX( true );
        displayMenu( "Test RX Mute: On" );
    }
    else
    {
        muteRX( false );
        displayMenu( "Test RX Mute: Off" );
    }
    
    return bUsed;
}


/**
 * @brief Handles the menu for enabling or disabling the RX clock.
 *
 * This function processes the rotary control and button presses
 * to toggle the RX clock. It updates the display with the
 * current RX clock status.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuRXClock( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    // Left or right toggles
    if( bShortPressLeft || bShortPressRight )
    {
        bRXClockEnabled = !bRXClockEnabled;
        bUsed = true;
    }

    if( bRXClockEnabled )
    {
        enableRXClock( true );
        displayMenu( "RX Clock: Enabled" );
    }
    else
    {
        enableRXClock( false );
        displayMenu( "RX Clock: Disabled" );
    }
    
    return bUsed;
}

#ifdef PSDR
/**
 * @brief Handles the menu for enabling or disabling the roofing filter.
 *
 * This function processes the rotary control and button presses
 * to toggle the roofing filter. It updates the display with the
 * current roofing filter status.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuRoofing( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    // Left or right toggles
    if( bShortPressLeft || bShortPressRight || bCW || bCCW )
    {
        applyRoofingFilter = !applyRoofingFilter;
        bUsed = true;
    }

    if( applyRoofingFilter )
    {
        displayMenu( "Roofing:Enabled" );
    }
    else
    {
        displayMenu( "Roofing:Disabled" );
    }
    
    return bUsed;
}

/**
 * @brief Handles the menu for applying gains.
 *
 * This function processes the rotary control and button presses
 * to toggle the apply gains mode. It updates the display with the
 * current apply gains status.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuApplyGains( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    // Left or right toggles
    if( bShortPressLeft || bShortPressRight )
    {
        applyGains = !applyGains;
        bUsed = true;
    }

    if( applyGains )
    {
        displayMenu( "Gains: Enabled" );
    }
    else
    {
        displayMenu( "Gains: Disabled" );
    }
    
    return bUsed;
}

/**
 * @brief Handles the menu for adjusting the phase.
 *
 * This function processes the rotary control and button presses
 * to toggle the phase adjustment mode. It updates the display with the
 * current phase adjustment status.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuAdjustPhase( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    // Left or right toggles
    if( bShortPressLeft || bShortPressRight )
    {
        adjustPhase = !adjustPhase;
        bUsed = true;
    }

    if( adjustPhase )
    {
        displayMenu( "Phase: Enabled" );
    }
    else
    {
        displayMenu( "Phase: Disabled" );
    }
    
    return bUsed;
}
#endif

/**
 * @brief Handles the menu for setting the TX delay.
 *
 * This function processes the rotary control and button presses
 * to adjust the TX delay. It updates the display with the
 * current TX delay value.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuTXDelay( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    if( bCW )
    {
        if( txDelay < 50 )
        {
            txDelay++;
        }
        else if( txDelay < 500 )
        {
            txDelay += 50;
        }
        bUsed = true;
    }
    else if( bCCW )
    {
        if( txDelay > 50 )
        {
            txDelay -= 50;
        }
        else if( txDelay > 0 )
        {
            txDelay--;
        }
        bUsed = true;
    }
    else if( bShortPress )
    {
        txDelay = 10;
        bUsed = true;
    }
    char buf[TEXT_BUF_LEN];
    sprintf( buf, "TX delay: %d", txDelay);
    displayMenu( buf );
    
    return bUsed;
}

/**
 * @brief Handles the menu for enabling or disabling the TX clock.
 *
 * This function processes the rotary control and button presses
 * to toggle the TX clock. It updates the display with the
 * current TX clock status.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuTXClock( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    // Left or right toggles
    if( bShortPressLeft || bShortPressRight )
    {
        bTXClockEnabled = !bTXClockEnabled;
        bUsed = true;
    }

    if( bTXClockEnabled )
    {
        displayMenu( "TX Clock: Enabled" );
    }
    else
    {
        displayMenu( "TX Clock: Disabled" );
    }
    
    return bUsed;
}

/**
 * @brief Handles the menu for enabling or disabling the TX output.
 *
 * This function processes the rotary control and button presses
 * to toggle the TX output. It updates the display with the
 * current TX output status.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuTXOut( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    // Left or right toggles
    if( bShortPressLeft || bShortPressRight )
    {
        bTXOutEnabled = !bTXOutEnabled;
        bUsed = true;
    }

    if( bTXOutEnabled )
    {
        displayMenu( "TX Out: Enabled" );
    }
    else
    {
        displayMenu( "TX Out: Disabled" );
    }
    
    return bUsed;
}

/**
 * @brief Handles the menu for setting the unmute delay.
 *
 * This function processes the rotary control and button presses
 * to adjust the unmute delay. It updates the display with the
 * current unmute delay value.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuUnmuteDelay( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    if( bCW )
    {
        if( unmuteDelay < 50 )
        {
            unmuteDelay++;
        }
        else if( unmuteDelay < 500 )
        {
            unmuteDelay += 50;
        }
        bUsed = true;
    }
    else if( bCCW )
    {
        if( unmuteDelay > 50 )
        {
            unmuteDelay -= 50;
        }
        else if( unmuteDelay > 0 )
        {
            unmuteDelay--;
        }
        bUsed = true;
    }
    else if( bShortPress )
    {
        unmuteDelay = 5;
        bUsed = true;
    }
    char buf[TEXT_BUF_LEN];
    sprintf( buf, "Unmute dly: %d", unmuteDelay);
    displayMenu( buf );
    
    return bUsed;
}

/**
 * @brief Handles the menu for setting the mute delay.
 *
 * This function processes the rotary control and button presses
 * to adjust the mute delay. It updates the display with the
 * current mute delay value.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuMuteDelay( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    if( bCW )
    {
        if( muteDelay < 50 )
        {
            muteDelay++;
        }
        else if( muteDelay < 500 )
        {
            muteDelay += 50;
        }
        bUsed = true;
    }
    else if( bCCW )
    {
        if( muteDelay > 50 )
        {
            muteDelay -= 50;
        }
        else if( muteDelay > 0 )
        {
            muteDelay--;
        }
        bUsed = true;
    }
    else if( bShortPress )
    {
        muteDelay = 5;
        bUsed = true;
    }
    char buf[TEXT_BUF_LEN];
    sprintf( buf, "Mute dly: %d", muteDelay);
    displayMenu( buf );
    
    return bUsed;
}

/**
 * @brief Sets the cursor position on the menu line.
 *
 * This function positions the cursor at the specified position on the menu line.
 *
 * @param pos The position on the menu line to set the cursor.
 */
static void cursorMenu( uint8_t pos )
{
#ifdef LCD_DISPLAY
    displayCursor( pos, MENU_LINE, cursorUnderline );
#endif    

#ifdef OLED_DISPLAY
    // Draw it on the OLED
    drawCursor( menuCursorX+pos*getFontWidth(MENU_FONT), menuCursorY, getFontWidth(MENU_FONT) );
#endif
}

/**
 * @brief Handles the menu for changing the crystal frequency.
 *
 * This function processes the rotary control and button presses
 * to navigate and select the crystal frequency. It updates the display
 * with the current frequency and allows the user to set a new frequency.
 * Each digit can be changed individually.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuXtalFreq( uint16_t inputState )
{
    // When the menu is entered we are changing the first changeable digit of the
    // crystal frequency i.e. MHz. This is digit 7.
    #define INITIAL_FREQ_CHANGE 1000000
    #define INITIAL_FREQ_POS    7

    // The final changeable digit position
    #define FINAL_FREQ_POS      13

    // Set to true as expecting to use the presses etc
    bool bUsed = true;
    
    // The current change in frequency
    static uint32_t freqChange;
    
    // The new frequency starts with the current value
    // Will only want to update it if they have changed
    static uint32_t oldFreq, newFreq;

    // If just entered the menu...
    if( !bCW && !bCCW && !bShortPress &&!bLongPress && !bShortPressLeft && !bShortPressRight )
    {
        // Start with the current xtal frequency from NVRAM
        newFreq = oldFreq = nvramReadXtalFreq();
        
        // Start with changing the first digit
        // i.e. the tens of MHz
        freqChange = INITIAL_FREQ_CHANGE;
        settingFreqPos = INITIAL_FREQ_POS;
        
        // Set the cursor on the digit to be changed
        cursorMenu( settingFreqPos );

        // We aren't asking to save the new frequency to NVRAM just yet
        bAskToSaveSettingFreq = false;
    }

    // We are asking whether or not to save the new value to NVRAM
    if( bAskToSaveSettingFreq )
    {
        // A long press means we are quitting without saving
        if( bLongPress )
        {
            // Set back the original frequency
            oscSetXtalFrequency( nvramReadXtalFreq() );

            // Retune to use the xtal frequency
            setFrequencies();

            bUsed = false;
        }
        // A short press means we are writing the new frequency
        // and quitting
        else if( bShortPress )
        {
            // Write the new crystal frequency to NVRAM
            nvramWriteXtalFreq( newFreq );
            
            // Take us out of the menu item
            enterVFOMode();

            bUsed = false;
        }
    }
    else
    {
        // If a long press then we are ending without
        // writing new value to NVRAM
        if( bLongPress )
        {
            // Don't want the cursor any more
            turnCursorOff();    

            // Set back the original frequency
            oscSetXtalFrequency( nvramReadXtalFreq() );

            // Retune to use the xtal frequency
            setFrequencies();

            // This ensures the long press is processed to quit
            bUsed = false;
        }
        // If a short press then we are ending - need to see if we should
        // write new value to NVRAM
        else if( bShortPress )
        {
            // Has the frequency changed?
            if( newFreq != oldFreq )
            {
                // Yes, so see if user wants to save
                bAskToSaveSettingFreq = true;
                
                // Don't want the cursor any more
                turnCursorOff();    

                displayMenu( "Short press to save" );
            }
        }
        else
        {
            // Rotation is a change up or down in frequency
            if( bCW )
            {
                // Clock wise so increasing in frequency provided we aren't
                // going to go over the maximum
                if( newFreq <= (MAX_XTAL_FREQUENCY - freqChange) )
                {
                    newFreq += freqChange;
                }
            }
            else if( bCCW )
            {
                // Counter clockwise so decreasing in frequency provided we
                // aren't going to go under the minimum
                if( newFreq >= (MIN_XTAL_FREQUENCY + freqChange) )
                {
                    newFreq -= freqChange;
                }
            }

            // A short left or right press changes the rate we tune at
            if( bShortPressRight )
            {
                if( freqChange == 1 )
                {
                    // Currently at the lowest position so start at the top
                    // again
                    freqChange = INITIAL_FREQ_CHANGE;
                    settingFreqPos = INITIAL_FREQ_POS;
                }
                else
                {
                    // Now changing the next digit
                    freqChange /= 10;
                    settingFreqPos++;
                }

                // Set the cursor to the correct position for the current amount of change
                cursorMenu( settingFreqPos );
            }
            else if( bShortPressLeft )
            {
                if( freqChange == INITIAL_FREQ_CHANGE )
                {
                    // Currently at the highest position so start at the bottom
                    // again
                    freqChange = 1;
                    settingFreqPos = FINAL_FREQ_POS;
                }
                else
                {
                    // Now changing the next digit
                    freqChange *= 10;
                    settingFreqPos--;
                }

                // Set the cursor to the correct position for the current amount of change
                cursorMenu( settingFreqPos );
            }

            // Display the current frequency
            char buf[TEXT_BUF_LEN];
            sprintf( buf, "Xtal: %0lu", newFreq);
            displayMenu( buf );

            // If the frequency is to change...
            if( newFreq != oldFreq )
            {
                // Set it in the oscillator driver
                oscSetXtalFrequency( newFreq );
                
                // Set the RX and TX frequencies again - this will pick up the
                // new crystal frequency
                setFrequencies();
            }
        }
    }
    return bUsed;
}

/**
 * @brief Handles the menu for changing the intermediate frequency (0 means direct conversion receiver).
 *
 * This function processes the rotary control and button presses
 * to navigate and select the intermediate frequency. It updates the display
 * with the current frequency and allows the user to set a new frequency.
 * Each digit can be changed individually.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuIntermediateFreq( uint16_t inputState )
{
    // When the menu is entered we are changing the first changeable digit of the
    // intermediate frequency i.e. 10MHz. This is character 5.
    #define INITIAL_IF_CHANGE 10000000
    #define INITIAL_IF_POS    5

    // The final changeable digit position
    #define FINAL_IF_POS      12

    // Set to true as expecting to use the presses etc
    bool bUsed = true;
    
    // The current change in frequency
    static uint32_t freqChange;
    
    // The new frequency starts with the current value
    // Will only want to update it if they have changed
    static uint32_t oldFreq, newFreq;

    // If just entered the menu...
    if( !bCW && !bCCW && !bShortPress &&!bLongPress && !bShortPressLeft &&!bShortPressRight )
    {
        // Start with the current intermediate frequency
        newFreq = oldFreq = intermediateFrequency;
        
        // Start with changing the first digit
        // i.e. the tens of MHz
        freqChange = INITIAL_IF_CHANGE;
        settingFreqPos = INITIAL_IF_POS;
        
        // Set the cursor on the digit to be changed
        cursorMenu( settingFreqPos );

        // We aren't asking to save the new frequency to NVRAM just yet
        bAskToSaveSettingFreq = false;
    }

    // We are asking whether or not to save the new value to NVRAM
    if( bAskToSaveSettingFreq )
    {
        // A long press means we are quitting without saving
        if( bLongPress )
        {
            // Set back the original frequency
            intermediateFrequency = nvramReadIntermediateFreq();

            // Retune to use the intermediate frequency
            setFrequencies();

            bUsed = false;
        }
        // A short press means we are writing the new frequency
        // and quitting
        else if( bShortPress )
        {
            // Write the new intermediate frequency to NVRAM
            nvramWriteIntermediateFreq( newFreq );
            
            // Take us out of the menu item
            enterVFOMode();

            bUsed = false;
        }
    }
    else
    {
        // If a long press then we are ending without
        // writing new value to NVRAM
        if( bLongPress )
        {
            // Don't want the cursor any more
            turnCursorOff();    

            // Set back the original frequency
            intermediateFrequency = nvramReadIntermediateFreq();

            // Retune to use the intermediate frequency
            setFrequencies();

            // This ensures the long press is processed to quit
            bUsed = false;
        }
        // If a short press then we are ending - need to see if we should
        // write new value to NVRAM
        else if( bShortPress )
        {
            // Has the frequency changed?
            if( newFreq != oldFreq )
            {
                // Yes, so see if user wants to save
                bAskToSaveSettingFreq = true;
                
                // Don't want the cursor any more
                turnCursorOff();    

                displayMenu( "Short press to save" );
            }
        }
        else
        {
            // Rotation is a change up or down in frequency
            if( bCW )
            {
                // Clock wise so increasing in frequency provided we aren't
                // going to go over the maximum
                if( newFreq <= (MAX_INTERMEDIATE_FREQUENCY - freqChange) )
                {
                    newFreq += freqChange;
                }
            }
            else if( bCCW )
            {
                // Counter clockwise so decreasing in frequency provided we
                // aren't going to go under the minimum
                if( newFreq >= (MIN_INTERMEDIATE_FREQUENCY + freqChange) )
                {
                    newFreq -= freqChange;
                }
            }

            // A short left or right press changes the rate we tune at
            if( bShortPressRight )
            {
                if( freqChange == 1 )
                {
                    // Currently at the lowest position so start at the top
                    // again
                    freqChange = INITIAL_IF_CHANGE;
                    settingFreqPos = INITIAL_IF_POS;
                }
                else
                {
                    // Now changing the next digit
                    freqChange /= 10;
                    settingFreqPos++;
                }

                // Set the cursor to the correct position for the current amount of change
                cursorMenu( settingFreqPos );
            }
            else if( bShortPressLeft )
            {
                if( freqChange == INITIAL_IF_CHANGE )
                {
                    // Currently at the highest position so start at the bottom
                    // again
                    freqChange = 1;
                    settingFreqPos = FINAL_IF_POS;
                }
                else
                {
                    // Now changing the next digit
                    freqChange *= 10;
                    settingFreqPos--;
                }

                // Set the cursor to the correct position for the current amount of change
                cursorMenu( settingFreqPos );
            }

            // Display the current frequency
            char buf[TEXT_BUF_LEN];
            sprintf( buf, "IF: %08lu", newFreq);
            displayMenu( buf );

            // If the frequency is to change...
            if( newFreq != intermediateFrequency )
            {
                // Set it in the oscillator driver
                intermediateFrequency = newFreq;
                
                // Set the RX and TX frequencies again - this will pick up the
                // new intermediate frequency
                setFrequencies();
            }
        }
    }
    return bUsed;
}

/**
 * @brief Handles the menu for changing the keyer mode.
 *
 * This function processes the rotary control and button presses
 * to navigate and select the keyer mode. It updates the display
 * with the current mode and allows the user to set a new mode.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuKeyerMode( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    // Current keyer mode
    static enum eMorseKeyerMode keyerMode;
    
    // If just entered the menu get the current mode
    if( !bCW && !bCCW && !bShortPress &&!bLongPress && !bShortPressLeft && !bShortPressRight )
    {
        keyerMode = morseGetKeyerMode();
    }

    // Left or right button presses
    if( bShortPressRight )
    {
        keyerMode++;
        if( keyerMode == MORSE_NUM_KEYER_MODES)
        {
            keyerMode = 0;
        }
    }
    else if( bShortPressLeft )
    {
        if( keyerMode == 0 )
        {
            keyerMode = MORSE_NUM_KEYER_MODES - 1;
        }
        else
        {
            keyerMode--;
        }
    }

    switch( keyerMode )
    {
        case morseKeyerIambicA:
            displayMenu( "Keyer: Iambic A" );
            break;

        case morseKeyerIambicB:
            displayMenu( "Keyer: Iambic B" );
            break;

        case morseKeyerUltimatic:
        default:
            displayMenu( "Keyer: Ultimatic" );
            break;
    }
    
    if( bShortPress )
    {
        // Short press writes the new mode
        morseSetKeyerMode( keyerMode );

        // Also write to NVRAM
        nvramWriteMorseKeyerMode( keyerMode );
        
        // Leave the menu and go back to VFO mode
        enterVFOMode();
    }

    return bUsed;
}

#ifdef LCD_DISPLAY
/**
 * @brief Handles the menu for changing the backlight mode.
 *
 * This function processes the rotary control and button presses
 * to navigate and select the backlight mode. It updates the display
 * with the current mode and allows the user to set a new mode.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuBacklight( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    // Current backlight mode
    static enum eBacklightMode backlightMode;
    
    // If just entered the menu get the current mode
    if( !bCW && !bCCW && !bShortPress &&!bLongPress && !bShortPressLeft && !bShortPressRight )
    {
        backlightMode = currentBacklightMode;
    }

    // Left or right button presses
    if( bShortPressRight )
    {
        backlightMode++;
        if( backlightMode == NUM_BACKLIGHT_MODES)
        {
            backlightMode = 0;
        }
    }
    else if( bShortPressLeft )
    {
        if( backlightMode == 0 )
        {
            backlightMode = NUM_BACKLIGHT_MODES - 1;
        }
        else
        {
            backlightMode--;
        }
    }

    switch( backlightMode )
    {
        case backlightOff:
            displayMenu( "Backlight: Off" );
            break;

        case backlightOn:
            displayMenu( "Backlight: On" );
            break;

        case backlightAuto:
        default:
            displayMenu( "Backlight: Auto" );
            break;
    }

    if( bShortPress )
    {
        // Short press writes the new mode
        currentBacklightMode = backlightMode;

        // Action the new mode
        switch( backlightMode )
        {
            case backlightOff:
                lcdBacklight(false);
                break;

            case backlightOn:
                lcdBacklight(true);
                break;

            case backlightAuto:
            default:
                lcdBacklight(true);
                lastBacklightTime = millis();
                break;
        }

        // Also write to NVRAM
        nvramWriteBacklightMode( currentBacklightMode );
        
        // Leave the menu and go back to VFO mode
        enterVFOMode();
    }

    return bUsed;
}
#endif

#ifdef VARIABLE_SIDETONE_VOLUME
/**
 * @brief Handles the menu for changing the sidetone volume.
 *
 * This function processes the rotary control and button presses
 * to adjust the sidetone volume. It updates the display with the
 * current sidetone volume value.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuSidetoneVolume( uint16_t inputState )
{
    uint8_t sidetoneVolume = ioGetSidetoneVolume();

    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    if( bCW )
    {
        if( sidetoneVolume < MAX_SIDETONE_VOLUME )
        {
            sidetoneVolume += SIDETONE_VOLUME_INC;
        }
        bUsed = true;
    }
    else if( bCCW )
    {
        if( sidetoneVolume > MIN_SIDETONE_VOLUME )
        {
            sidetoneVolume -= SIDETONE_VOLUME_INC;
        }
        bUsed = true;
    }
    else if( bShortPress )
    {
        sidetoneVolume = DEFAULT_SIDETONE_VOLUME;
        bUsed = true;
    }
    char buf[TEXT_BUF_LEN];
    sprintf( buf, "Sidetone vol: %d", sidetoneVolume);
    displayMenu( buf );
    
    if( bUsed )
    {
        ioSetSidetoneVolume( sidetoneVolume );
        nvramWriteSidetoneVolume( sidetoneVolume );
    }
    
    return bUsed;
}
#endif

#ifdef PSDR
/**
 * @brief Handles the menu for adjusting the I Gain.
 *
 * This function processes the rotary control and button presses
 * to adjust the I Gain. It updates the display with the
 * current I Gain value.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuIGain( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    if( bCW )
    {
        if( iGain < MAX_I_GAIN )
        {
            iGain+=100;
        }
        bUsed = true;
    }
    else if( bCCW )
    {
        if( iGain > MIN_I_GAIN )
        {
            iGain-=100;
        }
        bUsed = true;
    }
    else if( bShortPressRight )
    {
        if( iGain < MAX_I_GAIN )
        {
            iGain+=1000;
        }
        bUsed = true;
    }
    else if( bShortPressLeft )
    {
        if( iGain > MIN_I_GAIN )
        {
            iGain-=1000;
        }
        bUsed = true;
    }
    else if( bShortPress )
    {
        iGain = DEFAULT_I_GAIN;
        bUsed = true;
    }
    char buf[TEXT_BUF_LEN];
    sprintf( buf, "I Gain: %d", iGain);
    displayMenu( buf );
    
    return bUsed;
}

/**
 * @brief Handles the menu for adjusting the Q Gain.
 *
 * This function processes the rotary control and button presses
 * to adjust the Q Gain. It updates the display with the
 * current Q Gain value.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuQGain( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    if( bCW )
    {
        if( qGain < MAX_Q_GAIN )
        {
            qGain+=100;
        }
        bUsed = true;
    }
    else if( bCCW )
    {
        if( qGain > MIN_Q_GAIN )
        {
            qGain-=100;
        }
        bUsed = true;
    }
    else if( bShortPressRight )
    {
        if( qGain < MAX_Q_GAIN )
        {
            qGain+=1000;
        }
        bUsed = true;
    }
    else if( bShortPressLeft )
    {
        if( qGain > MIN_Q_GAIN )
        {
            qGain-=1000;
        }
        bUsed = true;
    }
    else if( bShortPress )
    {
        qGain = DEFAULT_Q_GAIN;
        bUsed = true;
    }
    char buf[TEXT_BUF_LEN];
    sprintf( buf, "Q Gain: %d", qGain);
    displayMenu( buf );
    
    return bUsed;
}

/**
 * @brief Handles the menu for adjusting the IQ Gain.
 *
 * This function processes the rotary control and button presses
 * to adjust the IQ Gain. It updates the display with the
 * current IQ Gain value.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuIQGain( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    if( bCW )
    {
        if( iqGain < MAX_IQ_GAIN )
        {
            iqGain+=100;
        }
        bUsed = true;
    }
    else if( bCCW )
    {
        if( iqGain > MIN_IQ_GAIN )
        {
            iqGain-=100;
        }
        bUsed = true;
    }
    else if( bShortPressRight )
    {
        if( iqGain < MAX_IQ_GAIN )
        {
            iqGain+=1000;
        }
        bUsed = true;
    }
    else if( bShortPressLeft )
    {
        if( iqGain > MIN_IQ_GAIN )
        {
            iqGain-=1000;
        }
        bUsed = true;
    }
    else if( bShortPress )
    {
        iqGain = DEFAULT_IQ_GAIN;
        bUsed = true;
    }
    char buf[TEXT_BUF_LEN];
    sprintf( buf, "IQ Gain: %d", iqGain);
    displayMenu( buf );
    
    return bUsed;
}

/**
 * @brief Handles the menu for adjusting the mute factor.
 *
 * This function processes the rotary control and button presses
 * to adjust the mute factor. It updates the display with the
 * current mute factor value.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuMuteFactor( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;
    
    if( bCW )
    {
        if( maxMuteFactor < 10 )
        {
            maxMuteFactor++;
        }
        else if( maxMuteFactor <= (MAX_MUTE_FACTOR - 10) )
        {
            maxMuteFactor+=10;
        }
        bUsed = true;
    }
    else if( bCCW )
    {
        if( maxMuteFactor >= 20 )
        {
            maxMuteFactor-=10;
        }
        else if( maxMuteFactor > MIN_MUTE_FACTOR )
        {
            maxMuteFactor--;
        }
        bUsed = true;
    }
    else if( bShortPressRight )
    {
        if( maxMuteFactor < (MAX_MUTE_FACTOR - 100) )
        {
            maxMuteFactor+=100;
        }
        bUsed = true;
    }
    else if( bShortPressLeft )
    {
        if( maxMuteFactor >= (MIN_MUTE_FACTOR + 100) )
        {
            maxMuteFactor-=100;
        }
        bUsed = true;
    }
    else if( bShortPress )
    {
        if( maxMuteFactor == DEFAULT_MAX_MUTE_FACTOR )
        {
            maxMuteFactor = 10 * DEFAULT_MAX_MUTE_FACTOR;
        }
        else
        {
            maxMuteFactor = DEFAULT_MAX_MUTE_FACTOR;
        }
        bUsed = true;
    }
    char buf[TEXT_BUF_LEN];
    sprintf( buf, "Mute f: %d", maxMuteFactor);
    displayMenu( buf );
    
    return bUsed;
}

/**
 * @brief Handles the menu for adjusting the left ADC volume.
 *
 * This function processes the rotary control and button presses
 * to adjust the left ADC volume. It updates the display with the
 * current left ADC volume value.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuLeftADCVolume( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;

    uint8_t vol, currentVol;
    vol = currentVol = WM8960GetLeftADCVolume();
    
    if( bCW )
    {
        if( vol < MAX_ADC_VOLUME )
        {
            vol++;
        }
        bUsed = true;
    }
    else if( bCCW )
    {
        if( vol > MIN_ADC_VOLUME )
        {
            vol--;
        }
        bUsed = true;
    }
    else if( bShortPress )
    {
        vol = DEFAULT_LEFT_ADC_VOLUME;
        bUsed = true;
    }

    if( vol != currentVol )
    {
        WM8960SetLeftADCVolume( vol );
    }

    char buf[TEXT_BUF_LEN];
    sprintf( buf, "Left vol: %d", vol);
    displayMenu( buf );
    
    return bUsed;
}

/**
 * @brief Handles the menu for adjusting the right ADC volume.
 *
 * This function processes the rotary control and button presses
 * to adjust the right ADC volume. It updates the display with the
 * current right ADC volume value.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuRightADCVolume( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;

    uint8_t vol, currentVol;
    vol = currentVol = WM8960GetRightADCVolume();
    
    if( bCW )
    {
        if( vol < MAX_ADC_VOLUME )
        {
            vol++;
        }
        bUsed = true;
    }
    else if( bCCW )
    {
        if( vol > MIN_ADC_VOLUME )
        {
            vol--;
        }
        bUsed = true;
    }
    else if( bShortPress )
    {
        vol = DEFAULT_LEFT_ADC_VOLUME;
        bUsed = true;
    }

    if( vol != currentVol )
    {
        WM8960SetRightADCVolume( vol );
    }

    char buf[TEXT_BUF_LEN];
    sprintf( buf, "Right vol: %d", vol);
    displayMenu( buf );
    
    return bUsed;
}

/**
 * @brief Handles the menu for adjusting the IQ phasing.
 *
 * This function processes the rotary control and button presses
 * to adjust the IQ phasing. It updates the display with the
 * current IQ phasing value.
 *
 * @param inputState The current state of the rotary control and buttons.
 * @return true if the input was used, false otherwise.
 */
static bool menuIQPhasing( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;

    extern volatile int changeIQPhasing, iqPhasing;

    if( bCW )
    {
        changeIQPhasing = 1;
        bUsed = true;
    }
    else if( bCCW )
    {
        changeIQPhasing = -1;
        bUsed = true;
    }

    sleep_ms(25);
    char buf[TEXT_BUF_LEN];
    sprintf( buf, "IQ Phasing: %d", iqPhasing );
    displayMenu( buf );
    
    return bUsed;
}
#endif

/**
 * @brief Gets the frequency of the specified VFO.
 *
 * This function returns the frequency of the specified VFO.
 * Usually called from CAT control.
 *
 * @param vfo The VFO index (0 or 1).
 * @return The frequency of the specified VFO.
 */
uint32_t getVFOFreq( uint8_t vfo )
{
    uint32_t freq = 0;
    if( vfo < NUM_VFOS )
    {
        freq = vfoState[vfo].freq;
    }
    return freq;
}

/**
 * @brief Get the current VFO (Variable Frequency Oscillator) frequency.
 * 
 * @return uint32_t The current VFO frequency in Hertz.
 */
uint32_t getCurrentVFOFreq()
{
    return getVFOFreq( currentVFO );
}

/**
 * @brief Get the frequency of the other VFO.
 *
 * This function returns the frequency of the other VFO.
 *
 * @return The frequency of the other VFO.
 */
uint32_t getOtherVFOFreq()
{
    return getVFOFreq( OTHER_VFO );
}

/**
 * @brief Get the current VFO (Variable Frequency Oscillator) offset.
 *
 * This function retrieves the current offset value for the VFO.
 *
 * @return The current VFO offset as a 16-bit signed integer.
 */
int16_t getCurrentVFOOffset()
{
    return vfoState[currentVFO].offset;
}

/**
 * @brief Returns true if the current VFO is in RIT mode.
 *
 * This function checks if the current VFO is in Receiver Incremental Tuning (RIT) mode.
 *
 * @return true if the current VFO is in RIT mode, false otherwise.
 */
bool getCurrentVFORIT()
{
    return (vfoState[currentVFO].mode == vfoRIT);
}

/**
 * @brief Returns true if the current VFO is in XIT mode.
 *
 * This function checks if the current VFO is in Transmit Incremental Tuning (XIT) mode.
 *
 * @return true if the current VFO is in XIT mode, false otherwise.
 */
bool getCurrentVFOXIT()
{
    return (vfoState[currentVFO].mode == vfoXIT);
}

/**
 * @brief Get the offset value for the other VFO (Variable Frequency Oscillator).
 *
 * This function retrieves the offset value for the other VFO, which is used
 * to adjust the frequency of the oscillator.
 *
 * @return int16_t The offset value for the other VFO.
 */
int16_t getOtherVFOOffset()
{
    return vfoState[currentVFO].offset;
}

/**
 * @brief Returns true if the other VFO is in RIT mode.
 *
 * This function checks if the other VFO is in Receiver Incremental Tuning (RIT) mode.
 *
 * @return true if the other VFO is in RIT mode, false otherwise.
 */
bool getOtherVFORIT()
{
    return (vfoState[currentVFO].mode == vfoRIT);
}

/**
 * @brief Returns true if the other VFO is in XIT mode.
 *
 * This function checks if the other VFO is in Transmit Incremental Tuning (XIT) mode.
 *
 * @return true if the other VFO is in XIT mode, false otherwise.
 */
bool getOtherVFOXIT()
{
    return (vfoState[currentVFO].mode == vfoXIT);
}

/**
 * @brief Sets the frequency of the specified VFO (Variable Frequency Oscillator).
 *
 * This function configures the frequency of the given VFO to the specified value.
 *
 * @param vfo The VFO to set the frequency for. This is an 8-bit unsigned integer.
 * @param freq The frequency to set for the VFO. This is a 32-bit unsigned integer.
 */
void setVFOFrequency( uint8_t vfo, uint32_t freq )
{
    if( vfo < NUM_VFOS )
    {
        vfoState[vfo].freq = freq;
    }
    setFrequencies();
}

/**
 * @brief Get the current VFO (Variable Frequency Oscillator).
 *
 * This function returns the index of the current VFO.
 *
 * @return The index of the current VFO.
 */
uint8_t getCurrentVFO()
{
    return currentVFO;
}

/**
 * @brief Sets the current VFO (Variable Frequency Oscillator).
 *
 * This function updates the current VFO to the specified VFO index
 * and updates the frequencies and display accordingly.
 *
 * @param vfo The VFO index to set as the current VFO.
 */
void setCurrentVFO( uint8_t vfo )
{
    if( vfo < NUM_VFOS )
    {
        currentVFO = vfo;

        // Update the frequencies and display
        setFrequencies();
    }
}

/**
 * @brief Retrieves the state of the VFO split.
 *
 * This function checks and returns the current state of the VFO (Variable Frequency Oscillator) split.
 *
 * @return true if the VFO split is enabled, false otherwise.
 */
bool getVFOSplit()
{
    return bVFOSplit;
}

/**
 * @brief Get the current transmitting state.
 *
 * This function returns the current state of transmission.
 *
 * @return true if currently transmitting, false otherwise.
 */
bool getTransmitting()
{
    return bTransmitting;
}
#endif

/**
 * @brief Adjusts the VFO frequency and offset.
 *
 * This function adjusts the frequency and offset of the specified VFO
 * by the given changes. It then updates the TX and RX frequencies.
 * Can change both frequency and offset but not a normal usage 
 * (simplex changes the frequency, RIT and XIT change the offset ).
 *
 * @param vfo The VFO index to adjust.
 * @param freqChange The change in frequency to apply.
 * @param offsetChange The change in offset to apply.
 */
static void adjustVFO( uint8_t vfo, int32_t freqChange, int32_t offsetChange )
{
    // Record the new frequency and offset
    vfoState[vfo].freq = vfoState[vfo].freq + freqChange;
    vfoState[vfo].offset = vfoState[vfo].offset + offsetChange;

    // Set the new TX and RX frequencies
    setFrequencies();
}

#ifdef SOTA2
static void rotaryVFO( uint16_t inputState )
{
    // How much to change frequency by
    int16_t change = SOTA2_FREQ_CHANGE;
    
    // Rotation is a change up or down in frequency
    if( bCW )
    {
        // Leave change as is (+ve)
    }
    else if( bCCW )
    {
        // Counter clockwise means change is -ve
        change = -change;
    }
    else
    {
        // Control not moved so no change
        change = 0;
    }

    if( bShortPress )
    {
        // A short press takes us back to the home frequency for the current band
        setBand( currentBand );
    }
    else if( bLongPress )
    {
        // A long press changes band
        setBand( (currentBand+1)%NUM_BANDS );
    }
    else
    {
        adjustVFO( currentVFO, change, 0);
    }
}
#else
/**
 * @brief Handles the rotary control while in the VFO mode.
 *
 * This function processes the rotary control and button presses
 * when the system is in VFO mode. It adjusts the VFO frequency,
 * changes the cursor position, and handles entering and exiting
 * different modes such as menu and quick menu.
 *
 * @param inputState The current state of the rotary control and buttons.
 */
static void rotaryVFO( uint16_t inputState )
{
    // How much to change frequency by
    int16_t change;
    
    // Set the amount each click changes VFO by
    change = vfoCursorTransition[cursorIndex].freqChange;

    // Rotation is a change up or down in frequency
    if( bCW )
    {
        // Leave change as is (+ve)
    }
    else if( bCCW )
    {
        // Counter clockwise means change is -ve
        change = -change;
    }
    else
    {
        // Control not moved so no change
        change = 0;
    }

    if( bShortPress )
    {
        // In split mode change between lines
        if( bVFOSplit )
        {
            // Change to the other line and display the cursor on this line
            if( bVFOFirstLine )
            {
                bVFOFirstLine = false;
            }
            else
            {
                bVFOFirstLine = true;
            }
        }
        else if( vfoState[currentVFO].mode == vfoSimplex )
        {
            // In simplex mode a short press enters the band setting menu
            enterVFOBandMenu();
        }
        else
        {
            // In RIT and XIT modes a short press zeroes the offset
            setCurrentVFOOffset( 0 );
        }

        // Display the cursor in the correct place
        updateCursor();
    }
    else if( bLongPress )
    {
        // A long press takes us to the menu
        enterMenu();
    }
    else if( bShortPressRight )
    {
        cursorIndex++;
        if( vfoCursorTransition[cursorIndex].x == CURSOR_TRANSITION_END )
        {
            cursorIndex = 0;
        }
        updateCursor();
    }
    else if( bLongPressLeft )
    {
        // A long left press takes us to the quick menu
        enterQuickMenu();
    }
    else if( bShortPressLeft )
    {
        if( cursorIndex == 0 )
        {
            cursorIndex = NUM_CURSOR_TRANSITIONS - 2;
        }
        else
        {
            cursorIndex--;
        }
        updateCursor();
    }
    else if( bLongPressRight )
    {
        // A long right press takes us to WPM
        enterWpm();
    }
    else
    {
        // Adjust the VFO
        if( bVFOSplit )
        {
            // In split mode so adjust the VFO according to which line
            // we are on
            if( bVFOFirstLine )
            {
                adjustVFO( currentVFO, change, 0);
            }
            else
            {
                adjustVFO( OTHER_VFO, change, 0);
            }
        }
        // Not in split mode
        else if( vfoState[currentVFO].mode == vfoSimplex )
        {
            // Simplex so change the frequency
            adjustVFO( currentVFO, change, 0);
        }
        else
        {
            // RIT or XIT so adjust the offset
            adjustVFO( currentVFO, 0, change );
        }
    }
}

/**
 * @brief Sets the current VFO (Variable Frequency Oscillator) offset.
 *
 * This function sets the current VFO offset to the specified RIT (Receiver Incremental Tuning) value.
 *
 * @param rit The RIT value to set the VFO offset to. This is a signed 16-bit integer.
 */
void setCurrentVFOOffset( int16_t rit )
{
    // Set the RIT by changing the offset.
    adjustVFO( currentVFO, 0, (int32_t) rit-vfoState[currentVFO].offset);
}

/**
 * @brief Handles the rotary control while in the WPM setting mode.
 *
 * This function processes the rotary control and button presses
 * when the system is in WPM setting mode. It adjusts the WPM value,
 * changes between straight key mode and WPM mode, and handles entering
 * and exiting different modes such as menu and quick menu.
 *
 * @param inputState The current state of the rotary control and buttons.
 */
static void rotaryWpm( uint16_t inputState )
{
    // When switching into straight key mode want to remember current wpm
    // so that we switch back to that instead of the default
    static uint8_t previousWpm = DEFAULT_MORSE_WPM;

    // The new wpm starts with the current value
    // Will only want to update them if they have changed
    uint8_t newWpm;
    uint8_t morseWpm;
    newWpm = morseWpm = morseGetWpm();

    // Only change it if not in straight key mode
    if( newWpm )
    {
        if( bCW )
        {
            newWpm++;
        }
        else if( bCCW )
        {
            newWpm--;
        }
    }

    // A short press takes us to straight key mode
    if( bShortPress )
    {
        if( newWpm )
        {
            // Remember the current wpm setting
            previousWpm = morseWpm;

            // Wpm of 0 means straight key mode
            newWpm = 0;
        }
        else
        {
            // Already in straight key mode
            // If in tune mode, stop transmitting
            if( morseInTuneMode() )
            {
                morseSetTuneMode( false );
            }
            else
            {
                // Otherwise out of straight key mode
                newWpm = previousWpm;
            }
        }
    }
    // A long press enters the menu
    else if( bLongPress )
    {
        enterMenu();
    }
    // A right press (short or long) puts us back to VFO mode
    else if( bShortPressRight || bLongPressRight )
    {
        enterVFOMode();
    }
    // A left press puts us in the quick menu
    else if( bShortPressLeft )
    {
        enterQuickMenu();
    }

    // If the wpm is to change then check it is in range before
    // setting it
    if( newWpm != morseWpm )
    {
        // wpm of 0 is straight key mode so no need to check
        if( newWpm != 0 )
        {
            if( newWpm > MAX_MORSE_WPM )
            {
                newWpm = MAX_MORSE_WPM;
            }
            else if( newWpm < MIN_MORSE_WPM )
            {
                newWpm = MIN_MORSE_WPM;
            }
        }
        morseWpm = newWpm;
        morseSetWpm( newWpm );
        
        // Write the new morse speed to NVRAM
        nvramWriteWpm( newWpm );

        // Display the new speed
        displayMorseWpm();
    }
}

#ifdef PSDR
/**
 * @brief Handles the volume control.
 *
 * This function processes the rotary control and button presses
 * to adjust the volume or input boost gain. It updates the display
 * with the current volume or gain value.
 */
static void handleVolume()
{
    bool bVolumeShortPress;
    bool bVolumeLongPress;
    bool bVolumeCW;
    bool bVolumeCCW;

    // Whether or not to display the volume
    // Even if we can't go up or down because at max or min
    // still want to display it briefly (on LCD)
    bool bDisplay = false;

    // Get the currently set volume
    uint8_t currentVolume = ioGetVolume();
    uint8_t volume = currentVolume;

    // Read the rotary state
    readRotary(VOLUME_ROTARY, &bVolumeCW, &bVolumeCCW, &bVolumeShortPress, &bVolumeLongPress);

    // The rotary control is either adjusting audio volume or front end gain (actually a shift in the input samples)
    if( bVolumeMode )
    {
        if( bVolumeCW )
        {
            if( volume < MAX_VOLUME )
            {
                volume++;
            }
            bDisplay = true;
        }
        else if( bVolumeCCW )
        {
            if( volume > MIN_VOLUME )
            {
                volume--;
            }
            bDisplay = true;
        }
        else if( bVolumeShortPress )
        {
            // Switch between IF below or above RX frequency
            ifBelow = !ifBelow;
            ioSetIF();
            bDisplay = true;
            setFrequencies();
        }
        else if( bVolumeLongPress )
        {
            // Switch to front end shift control
            bVolumeMode = false;
            bDisplay = true;
        }
    }
    else
    {
#if 1
        uint8_t gain = WM8960GetInputBoostGain();
        uint8_t prevGain = gain;

        if( bVolumeCW )
        {
            if( gain < MAX_INPUT_BOOST_GAIN )
            {
                gain++;
            }
            bDisplay = true;
        }
        else if( bVolumeCCW )
        {
            if( gain > MIN_INPUT_BOOST_GAIN )
            {
                gain--;
            }
            bDisplay = true;
        }
        else if( bVolumeShortPress )
        {
            gain = DEFAULT_INPUT_BOOST_GAIN;
            bDisplay = true;
        }
        else if( bVolumeLongPress )
        {
            bVolumeMode = true;
            bDisplay = true;
        }

        if( gain != prevGain )
        {
            WM8960SetInputBoostGain( gain );
        }
#else
        if( bVolumeCW )
        {
            if( inputShift < MAX_INPUT_SHIFT )
            {
                inputShift++;
            }
            bDisplay = true;
        }
        else if( bVolumeCCW )
        {
            if( inputShift > MIN_INPUT_SHIFT )
            {
                inputShift--;
            }
            bDisplay = true;
        }
        else if( bVolumeShortPress )
        {
            inputShift = DEFAULT_INPUT_SHIFT;
            bDisplay = true;
        }
        else if( bVolumeLongPress )
        {
            bVolumeMode = true;
            bDisplay = true;
        }
#endif
    }

    if( bDisplay )
    {
#ifdef LCD_DISPLAY        
        // Remember which mode we were in before going into volume mode
        if( displayMode != displayVolume )
        {
            lastDisplayMode = displayMode;
        }
        displayMode = displayVolume;
        lastVolumeTime = millis();
#endif
        // Need to set the volume before we display it
        ioSetVolume( volume );

#ifdef LCD_DISPLAY
        // Turn off the cursor
        turnCursorOff();
        displayFrequencyLCD();
#endif        

        displayVol();
    }
}

/**
 * @brief Toggles the state of the preamplifier.
 *
 * This function switches the preamplifier between its on and off states.
 */
static void togglePreamp( void )
{
    if( bPreampOn = !bPreampOn )
    {
        ioWritePreampOn();
    }
    else
    {
        ioWritePreampOff();
    }
    displayPreamp();
}
#endif
#endif

/**
 * @brief Handles the rotary control and pushbuttons.
 *
 * This function processes the rotary control and button presses
 * to navigate and select various options in the user interface.
 * It updates the display and handles entering and exiting different modes.
 */
static void handleRotary()
{
    int button;

    bool bRotaryShortPress;
    bool bRotaryLongPress;
    bool bRotaryCW;
    bool bRotaryCCW;
    bool bShortPressButton[NUM_PUSHBUTTONS];
    bool bLongPressButton[NUM_PUSHBUTTONS];

    // Button debounce states
    static struct sDebounceState debounceStateButton[NUM_PUSHBUTTONS];

    // Debounce the pushbuttons
    for( button = 0 ; button < NUM_PUSHBUTTONS ; button++ )
    {
        debouncePushbutton( ioReadButton(button),  &bShortPressButton[button],  &bLongPressButton[button], DEBOUNCE_TIME, LONG_PRESS_TIME, &debounceStateButton[button]);
    }

    // Read the rotary state
    readRotary(MAIN_ROTARY, &bRotaryCW, &bRotaryCCW, &bRotaryShortPress, &bRotaryLongPress);

    // Create the bitmask with all the clicks etc
    uint16_t inputState = (bRotaryCW*ROTARY_CW) |  (bRotaryCCW*ROTARY_CCW) | (bRotaryShortPress*ROTARY_SHORT_PRESS) | (bRotaryLongPress*ROTARY_LONG_PRESS);
    for( button = 0 ; button < NUM_PUSHBUTTONS ; button++ )
    {
        inputState |= bShortPressButton[button]*SHORT_PRESS(button);
        inputState |= bLongPressButton[button] *LONG_PRESS(button);
    }
    // Call the handler if anything has happened
    if( inputState )
    {
#ifdef SOTA2
        rotaryVFO( inputState );
#else
#ifdef LCD_DISPLAY
        // If we have an auto backlight then turn it on and note the time
        if( currentBacklightMode == backlightAuto )
        {
            lcdBacklight( true );
            lastBacklightTime = millis();
        }
#endif
#ifdef PSDR
#ifdef OLED_DISPLAY
        if( inputState & SHORT_PRESS(BUTTON_A) )
        {
            ioSetFilter( (ioGetFilter() + 1) % ioGetNumFilters() );
            displayFilter();
        }
        else if( inputState & LONG_PRESS(BUTTON_A) )
        {
            uint8_t filter = ioGetFilter();
            if( filter == 0 )
            {
                filter = ioGetNumFilters() - 1;
            }
            else
            {
                filter--;
            }
            ioSetFilter( filter );
            displayFilter();
        }
        else 
#endif
        if( inputState & SHORT_PRESS(BUTTON_B) )
        {
            togglePreamp();
        }
        else if( inputState & LONG_PRESS(BUTTON_B) )
        {
            outputMode++;
            if( outputMode >= NUM_OUTPUTS )
            {
                outputMode = 0;
            }
            displayOutputMode();
        }
        else
#endif
        switch( currentMode )
        {
            case modeMenu:
                rotaryMenu(inputState);
                break;

            case modeQuickMenu:
                rotaryQuickMenu(inputState);
                break;

            case modeWpm:
                rotaryWpm(inputState);
                break;

            default:
            case modeVFO:
                rotaryVFO(inputState);
                break;
        }
#endif
    }
}


/**
 * @brief Main loop function.
 *
 * This function is called repeatedly and handles various tasks such as
 * checking the morse paddles, handling the rotary control and pushbuttons,
 * managing the backlight, and performing CAT control.
 */
static void loop()
{
#ifndef SOTA2
#ifdef LCD_DISPLAY
    // If the backlight mode is auto then see if it is time
    // to turn off the backlight
    if( currentBacklightMode == backlightAuto )
    {
        if( lastBacklightTime &&
            (millis() - lastBacklightTime) > BACKLIGHT_AUTO_DELAY )
        {
            lcdBacklight( false );

            // Setting this to zero stops us checking any more
            lastBacklightTime = 0;
        }
    }

#ifdef PSDR
    if( displayMode == displayVolume )
    {
        if( lastVolumeTime &&
            (millis() - lastVolumeTime) > VOLUME_DISPLAY_DELAY )
        {
            displayMode = lastDisplayMode;
            displayFrequencyLCD();
            updateCursor();

            // Setting this to zero stops us checking any more
            lastVolumeTime = 0;
        }
    }
#endif
#endif
#endif

    // See if the morse paddles or straight key have been pressed
    // If not active then deal with other things too
	if( !morseScanPaddles() )
    {
        // Deal with the rotary control/pushbutton
        handleRotary();

#ifdef PSDR
        // Deal with the volume control
        handleVolume();
#endif

#ifdef CAT_CONTROL
        // Do CAT control
        catControl();
#endif
#ifdef PSDR
        static uint32_t lastInputTime;
        static uint8_t lastScale;

        uint32_t now = millis();
        if( (now - lastInputTime) > 500 )
        {
            lastInputTime = now;

            // Display change in ADC or output overload state
            if( (adcOverload != prevAdcOverload) ||
                (outOverload != prevOutOverload)
#ifdef DISPLAY_MIN_MAX
                 ||
                (maxOut != prevMaxOut) ||
                (maxIn != prevMaxIn) ||
                (minOut != prevMinOut) ||
                (minIn != prevMinIn)
#endif
              )
            {
                prevAdcOverload = adcOverload;
                prevOutOverload = outOverload;
#ifdef DISPLAY_MIN_MAX
                prevMaxOut = maxOut;
                prevMaxIn = maxIn;
                prevMinOut = minOut;
                prevMinIn = minIn;
#endif
                displayPreamp();
            }

#ifdef DISPLAY_AGC
            if( (agcAmplitude != prevAgcAmplitude) || (agcGain != prevAgcGain) )
            {
                prevAgcAmplitude = agcAmplitude;
                prevAgcGain = agcGain;
                displayAgc();
            }
#endif
        }
#endif
    }
}

/**
 * @brief Initialises the screen.
 *
 * This function sets up the screen for displaying information.
 * It initialises the display, calculates cursor positions, and
 * sets up various screen objects and their positions.
 */
void screenInit( void )
{
#ifndef SOTA2
#ifdef LCD_DISPLAY
    displayInit();
#endif

#ifdef OLED_DISPLAY
    int i;

    oledInit();

    // Calculate the OLED cursor positions
    // This allows us to change the font
    for( i = 0 ; vfoCursorTransition[i].x != CURSOR_TRANSITION_END ; i++ )
    {
        // The cursor is below the digits which may be full or half height
        vfoCursorTransition[i].sy = getFontHeight( FREQUENCY_FONT );
        vfoCursorTransition[i].shy = getFontHeight( HALF_FREQUENCY_FONT );

        // The VFO letter and dot are a different font to the digits
        if( i <= CURSOR_DOT )
        {
            vfoCursorTransition[i].sx = getFontWidth( VFO_LETTER_FONT ) + (i+CURSOR_DOT+1)*getFontWidth( FREQUENCY_FONT );
        }
        else
        {
            vfoCursorTransition[i].sx = getFontWidth( VFO_LETTER_FONT ) + getFontWidth(FREQUENCY_DOT_FONT) + (i+CURSOR_DOT)*getFontWidth( FREQUENCY_FONT );
        }

        // Cursor width is different for the dot
        if( i == CURSOR_DOT )
        {
            vfoCursorTransition[i].sWidth = getFontWidth( FREQUENCY_DOT_FONT );
        }
        else
        {
            vfoCursorTransition[i].sWidth = getFontWidth( FREQUENCY_FONT );
        }
    }

    // Calculate the OLED quick menu positions and widths
    // This needs to match the definition of QUICK_MENU_TEXT
    for( i = 0 ; i < NUM_QUICK_MENUS ; i++ )
    {
        quickMenu[i].sx = quickMenu[i].pos * getFontWidth( MENU_FONT );

        int width;
        if( i <= 1 )
        {
            width = 3;
        }
        else if( i <= 3 )
        {
            width = 1;
        }
        else
        {
            width = 4;
        }
        quickMenu[i].sWidth = getFontWidth( MENU_FONT ) * width;
    }

    // Calculate the positions for screen objects and cursors
    wpmX = 0;
    wpmY = getFontHeight(FREQUENCY_FONT) + 2;
    wpmCursorX = 0;
    wpmCursorY = wpmY + getFontHeight( WPM_FONT );
    wpmCursorWidth = getFontWidth( WPM_FONT ) * 5;

    volX = wpmCursorX + wpmCursorWidth + getFontWidth( WPM_FONT );
    volY = wpmY;

    preampX = volX + getFontWidth( VOLUME_FONT ) * 7;
    preampY = volY;

    menuX = 0;
    menuY = wpmY + getFontHeight( WPM_FONT ) + 2;
    menuCursorX = menuX;
    menuCursorY = menuY + getFontHeight( MENU_FONT );
    menuWidth = OLED_WIDTH;

    modeX = 0;
    modeY = menuY + getFontHeight( MENU_FONT ) + 2;
    modeWidth = getFontWidth( MODE_FONT ) * 4;

    filterX = modeX + modeWidth + 2;
    filterY = modeY;
    filterWidth = OLED_WIDTH - modeWidth - 2;

    //displayTRXMode();
    displayOutputMode();
    displayFilter();
#endif

    displayMorseWpm();

#ifdef PSDR
    displayVol();
    displayPreamp();
#endif
#endif
}

/**
 * @brief Main entry point of the application.
 *
 * This function initialises various modules and enters the main loop.
 *
 * @return int Exit status of the application.
 */
int main(void)
{
    // Start the millisecond timer - it enables timer interrupts
    millisInit();

    // Initialise the input/output module
    ioInit();

    // Initialise the NVRAM/EEPROM before other modules as it contains values needed for other setup
    nvramInit();

#ifdef CAT_CONTROL
    // Initialise CAT control
    catInit();
#endif
    // Set up morse and set the speed and keyer mode as read from NVRAM
    morseInit();
    morseSetWpm( nvramReadWpm() );
    morseSetKeyerMode( nvramReadMorseKeyerMode() );

    screenInit();

#ifdef LCD_DISPLAY
    // Set the backlight
    currentBacklightMode = nvramReadBacklighMode();
    lcdBacklight( currentBacklightMode != backlightOff );
    if( currentBacklightMode == backlightAuto )
    {
        lastBacklightTime = millis();
    }
#endif
    
#ifdef VARIABLE_SIDETONE_VOLUME
    // Set the sidetone volume
    uint8_t sidetoneVolume = nvramReadSidetoneVolume();
    ioSetSidetoneVolume( sidetoneVolume );
#endif
   
    // Initialise the oscillator chip
	bOscInit = oscInit();

    // Load the crystal frequency from NVRAM
    oscSetXtalFrequency( nvramReadXtalFreq() );

    // Set the band from the NVRAM
    // This also updates the display with frequency and wpm.
#ifdef SOTA2
    // SOTA2 is direct conversion so intermediate frequency is always 0
    intermediateFrequency = 0;
    setBand( DEFAULT_BAND );
#else
    // Get the intermediate frequency from NVRAM
    intermediateFrequency = nvramReadIntermediateFreq();

    setBand( nvramReadBand() );
#endif

    // Enable the RX clock outputs
    // We enable the TX output only when transmitting
    enableRXClock( true );

    // Unmute the receiver
    muteRX( false );

    while (1) 
    {
        loop();
    }
}
