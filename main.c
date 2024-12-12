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
#include "sdr.h"

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

// Menu functions
static bool menuVFOBand( uint16_t inputState );
static bool menuSDR( uint16_t inputState );
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
static bool menuBFOFreq( uint16_t inputState );
static bool menuKeyerMode( uint16_t inputState );
static bool menuIGain( uint16_t inputState );
static bool menuQGain( uint16_t inputState );
static bool menuIQGain( uint16_t inputState );
static bool menuAdjustPhase( uint16_t inputState );
static bool menuApplyGains( uint16_t inputState );
static bool menuRoofing( uint16_t inputState );
static bool menuPWMDivider( uint16_t inputState );
static bool menuHilbertFilter( uint16_t inputState );
static bool menuMuteFactor( uint16_t inputState );

#ifdef LCD_DISPLAY
static bool menuFilter( uint16_t inputState );
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

#ifdef LCD_DISPLAY
#define NUM_SDR_MENUS 12
#else
#define NUM_SDR_MENUS 11
#endif
static const struct sMenuItem sdrMenu[NUM_SDR_MENUS] =
{
    { "",               NULL },
    { "SDR Mode",       menuSDR },
    { "Hilbert filter", menuHilbertFilter },
#ifdef LCD_DISPLAY
    { "Filter",         menuFilter },
#endif
    { "Roofing filter", menuRoofing },
    { "Apply gains",    menuApplyGains },
    { "Adjust phase",   menuAdjustPhase },
    { "I Gain",         menuIGain },
    { "Q Gain",         menuQGain },
    { "IQ Gain",        menuIQGain },
    { "PWM Divider",    menuPWMDivider },
    { "Mute Factor",    menuMuteFactor },
};

#define SDR_FILTER_MENU_ITEM 2

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
    { "BFO Frequency",  menuBFOFreq },
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
    SDR_MENU,
    VFO_MENU,
    TEST_MENU,
    CONFIG_MENU,
    NUM_MENUS
};

static const struct
{
    char                    *text;
    const struct sMenuItem  *subMenu;
    uint8_t                 numItems;
}
menu[NUM_MENUS] =
{
    { "SDR",    sdrMenu,    NUM_SDR_MENUS },
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

// The last time we changed the volume
static uint32_t lastVolumeTime;

#endif

// Band frequencies
#define NUM_BANDS 13

static const struct  
{
    char     *bandName;     // Text for the menu
    uint32_t  minFreq;      // Min band frequency
    uint32_t  maxFreq;      // Max band frequency
    uint32_t  defaultFreq;  // Where to start on this band e.g. QRP calling
    bool      bTXEnabled;   // True if TX enabled on this band
    uint8_t   relayState;   // What state to put the relays in on this band
    bool      bQuickVFOMenu;// True if this band appears in the quick VFO menu
}
band[NUM_BANDS] =
{
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
};

// Current band - initialised from NVRAM
static uint8_t currentBand;

// Current relay state - always set from the frequency
static uint8_t currentRelay;

// Is the VFO on the first or second frequency line?
static bool bVFOFirstLine = true;

// For the setting frequency (e.g. xtal or BFO) the current digit position that is changing
static uint8_t settingFreqPos;

// True if asking whether to save the frequency setting
static bool bAskToSaveSettingFreq = false;

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

// Set to true when the preamp is on
static bool bPreampOn = false;

// Select normal, binaural or peaked output
extern enum eOutput outputMode;

// Delay before muting and unmuting
static uint8_t muteDelay = 5;
static uint16_t unmuteDelay = 5;
static uint8_t txDelay = 10;

// Set to true when transmitting
static bool bTransmitting = false;

// BFO frequency - 0 for direct conversion
static uint32_t BFOFrequency;

// Whether IF is above or below
extern bool ifBelow;

// Keep track of ADC and output overload
extern bool adcOverload;
static bool prevAdcOverload;
extern int outOverload;
static int prevOutOverload;

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
extern int inputShift;

// Works out the current RX frequency from the VFO settings
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

// Works out the current TX frequency from the VFO settings
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

// Returns true if the TX is enabled at the current transmit frequency
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

// See if this is a new band and if so set the new band
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

// Sets the relay for the current frequency
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

// Enable/disable the RX clock outputs
static void enableRXClock( bool bEnable )
{
    oscClockEnable( RX_CLOCK_A, bEnable );
    oscClockEnable( RX_CLOCK_B, bEnable );
}

// Enable/disable the TX clock output
static void enableTXClock( bool bEnable )
{
    oscClockEnable( TX_CLOCK, bEnable );
}

// Turns on/off the TX clock and PA
// If break-in off disables this
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

// Handle key up and down - mute RX, transmit, sidetone etc.
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

// Display the morse character if not in the menu
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
// Draws the cursor on the OLED screen
// Remembers the previous cursor so it can clear it first
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

// Set the correct cursor for the VFO mode
static void update_cursor()
{
    uint8_t line = FREQ_LINE;
    uint8_t y = vfoCursorTransition[cursorIndex].sy;

    // Only update the cursor if in VFO mode
    if( currentMode == modeVFO )
    {
        // If split, RIT or XIT mode may need to put the cursor on the other line
        if( bVFOSplit || vfoState[currentVFO].mode != vfoSimplex )
        {
            y = vfoCursorTransition[cursorIndex].shy;

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

// Display menu text on either display
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

// Displays the frequencies on the screen
// Normally displays one frequency on the top line but in
// split or RIT/XIT modes uses two lines
static void displayFrequencies( void )
{
    char buf[TEXT_BUF_LEN];

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

    bool bigFont = true;

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

static void displayMorseWpm( void )
{
    char buf[TEXT_BUF_LEN];

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

static void displayVol( void )
{
    sprintf( volText, "%c:%d%c%02d", ifBelow ? '-' : '+', inputShift, bVolumeMode ? '>' : '<', ioGetVolume());

#ifdef OLED_DISPLAY
    // Display on the OLED
    oledWriteString( volX, volY, volText, VOLUME_FONT, true);
#endif

#ifdef LCD_DISPLAY
    // Display on the LCD
    displayFrequencyLCD();
#endif
}

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

static void displayFilter( void )
{
    // Clear previous filter text before writing new text
    oledDrawRectangle( filterX, filterY, filterWidth, getFontHeight( FILTER_FONT ), false, false );
    oledWriteString( filterX, filterY, ioGetFilterText(), FILTER_FONT, true );
}
#endif

// Set the RX frequency
static void setRXFrequency( uint32_t freq )
{
    // For CW the RX oscillator has to be offset for the CW tone to be audible
    // but not for SSB. Also need to set the correct quadrature for either
    // sideband.
    int offset = 0;
    int q = 0;

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

    //offset = 0;

    if( ifBelow )
    {
        freq -= INTERMEDIATE_FREQUENCY;
    }
    else
    {
        freq += INTERMEDIATE_FREQUENCY;
    }

    // Set the oscillator frequency.
    if( BFOFrequency == 0 )
    {
        // Direct conversion so set the correct quadrature phase shift.
#if 1
        oscSetFrequency( RX_CLOCK_A, freq, 0 );
        oscSetFrequency( RX_CLOCK_B, freq, q );
#else
        oscSetFrequency( RX_CLOCK_A, freq + offset, 0 );
        oscSetFrequency( RX_CLOCK_B, freq + offset, q );
#endif
    }
    else
    {
        // Superhet so set the LFO and BFO
        // TODO: Setting LSB/USB
        oscSetFrequency( RX_CLOCK_A, freq + BFOFrequency, 0 );
        oscSetFrequency( RX_CLOCK_B, BFOFrequency + RX_OFFSET, 0 );
    }

    // Set the relay
    setRelay();
}

// Set the TX and RX frequencies
static void setFrequencies()
{
    // See if this is a new band and if so set the new band
    setBandFromFrequency( getRXFreq() );

    // Set RX and TX frequencies
    setRXFrequency( getRXFreq() );
    oscSetFrequency( TX_CLOCK, getTXFreq(), 0 );

    // Ensure the display and cursor reflect this
    displayFrequencies();
    update_cursor();
}

// Turn off the cursor
static void turnCursorOff( void )
{
#ifdef LCD_DISPLAY
    displayCursor( 0, 0, cursorOff );
#endif

#ifdef OLED_DISPLAY
    drawCursor( 0, 0, 0 );
#endif
}

// Set the band - sets the frequencies and memories to the new band's default
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

// Display the menu text for the current menu or sub menu
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

// Enter the wpm setting mode
static void enterWpm()
{
    currentMode = modeWpm;
    displayMode = displayWpm;

    displayMorseWpm();
    update_cursor();
}

// Enter the menu
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

// Quick way to the VFO menu
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

#ifdef LCD_DISPLAY
// Quick way to the filter menu
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

// Go back to VFO mode
static void enterVFOMode()
{
    // Go to VFO mode
    currentMode = modeVFO;
    displayMode = DEFAULT_DISPLAY_MODE;

    // Next time we enter the menu start not in a menu item
    bEnteredMenuItem = false;

    // Display the cursor in the right place
    update_cursor();

    // Clear the second line as no longer in the menu
#ifdef LCD_DISPLAY
    displaySplitLine( 0, FREQ_LINE + 1 );
#endif    
    displayMenu( "" );

    // Update the display with the correct split for the current VFO mode
    displayFrequencies();
}

// Swap the VFOs - called either from the quick menu or CAT control
void vfoSwap()
{
    // Swap the VFOs
    currentVFO = OTHER_VFO;

    // Update the frequencies and display
    setFrequencies();
}

static void quickMenuSwap()
{
    vfoSwap();
}

// Set the other VFO to the current VFO - called either from the quick menu or CAT control
void vfoEqual()
{
    vfoState[OTHER_VFO] = vfoState[currentVFO];

    // If in split mode need to ensure the transmit frequency is updated
    if( bVFOSplit )
    {
        setFrequencies();
    }
}

// Set the other VFO to the current VFO
static void quickMenuEqual()
{
    vfoEqual();
}

// Set RIT on or off - called from quick menu or CAT control
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
        update_cursor();
    }
}

// Quick menu item RIT selected
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

// Set XIT on or off - called from quick menu or CAT control
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
        update_cursor();
    }
}

// Quick menu item XIT selected
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

// Set the VFO split state - for CAT control or the quick menu
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

// Quick menu item Split selected
static void quickMenuSplit()
{
    // Toggle split state
    setVFOSplit( !bVFOSplit );
    enterVFOMode();
}

// Display the quick menu text
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

// Enter the quick menu
static void enterQuickMenu()
{
    currentMode = modeQuickMenu;
    displayMode = DEFAULT_DISPLAY_MODE;

    // Display the current menu text
    quickMenuDisplayText();
}

// Handle the rotary control while in the menu
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

// Return to simplex VFO mode
static void enterSimplex()
{
    setCurrentVFORIT( false );
    setCurrentVFOXIT( false );
    setVFOSplit( false );
    enterVFOMode();
}

// Handle the rotary control while in the quick menu
static void rotaryQuickMenu( uint16_t inputState )
{
    // Rotary movement continues to operate the VFO in simplex mode
    if( bCW || bCCW )
    {
        if( !bVFOSplit && vfoState[currentVFO].mode == vfoSimplex )
        {
            rotaryVFO(inputState);
        }
    }
    else if( bShortPressRight )
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

// Returns the next band. Allows for skipping bands that do not appear in
// the quick VFO menu.
//
// oldBand      The previous band
// direction    +1 to go up a band or -1 to go down a band
// bAllBands    true if all bands in the table are allowed
//              false means we will skip bands that are not to appear in the
//              quick VFO menu
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

// Handle the menu for the VFO band
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

// Set the TX/RX mode
void setTRXMode( enum eTRXMode mode )
{
    // Store in the NVRAM
    nvramWriteTRXMode( mode );

    //displayTRXMode();

    // Action the change in sideband
    setFrequencies();
}

// Set the CW reverse state
void setCWReverse( bool bCWReverse )
{    // Store in the NVRAM
    setTRXMode( bCWReverse ? cwRevMode : cwMode );
}

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

#ifdef LCD_DISPLAY
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

// Put the cursor on the required position on the menu line
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

// Menu for changing the crystal frequency
// Each digit can be changed individually
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

// Menu for changing the BFO frequency (0 means direct conversion receiver)
// Each digit can be changed individually
static bool menuBFOFreq( uint16_t inputState )
{
    // When the menu is entered we are changing the first changeable digit of the
    // BFO frequency i.e. 10MHz. This is character 5.
    #define INITIAL_BFO_FREQ_CHANGE 10000000
    #define INITIAL_BFO_FREQ_POS    5

    // The final changeable digit position
    #define FINAL_BFO_FREQ_POS      12

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
        // Start with the current BFO frequency
        newFreq = oldFreq = BFOFrequency;
        
        // Start with changing the first digit
        // i.e. the tens of MHz
        freqChange = INITIAL_BFO_FREQ_CHANGE;
        settingFreqPos = INITIAL_BFO_FREQ_POS;
        
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
            BFOFrequency = nvramReadBFOFreq();

            // Retune to use the BFO frequency
            setFrequencies();

            bUsed = false;
        }
        // A short press means we are writing the new frequency
        // and quitting
        else if( bShortPress )
        {
            // Write the new BFO frequency to NVRAM
            nvramWriteBFOFreq( newFreq );
            
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
            BFOFrequency = nvramReadBFOFreq();

            // Retune to use the BFO frequency
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
                if( newFreq <= (MAX_BFO_FREQUENCY - freqChange) )
                {
                    newFreq += freqChange;
                }
            }
            else if( bCCW )
            {
                // Counter clockwise so decreasing in frequency provided we
                // aren't going to go under the minimum
                if( newFreq >= (MIN_BFO_FREQUENCY + freqChange) )
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
                    freqChange = INITIAL_BFO_FREQ_CHANGE;
                    settingFreqPos = INITIAL_BFO_FREQ_POS;
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
                if( freqChange == INITIAL_BFO_FREQ_CHANGE )
                {
                    // Currently at the highest position so start at the bottom
                    // again
                    freqChange = 1;
                    settingFreqPos = FINAL_BFO_FREQ_POS;
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
            sprintf( buf, "BFO: %08lu", newFreq);
            displayMenu( buf );

            // If the frequency is to change...
            if( newFreq != BFOFrequency )
            {
                // Set it in the oscillator driver
                BFOFrequency = newFreq;
                
                // Set the RX and TX frequencies again - this will pick up the
                // new BFO frequency
                setFrequencies();
            }
        }
    }
    return bUsed;
}

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

static bool menuPWMDivider( uint16_t inputState )
{
    // Set to true if we have used the presses etc
    bool bUsed = false;

    uint8_t div = ioGetPWMDiv();
    
    if( bCW )
    {
        if( div < 8 )
        {
            div++;
            ioSetPWMDiv( div );
        }
        bUsed = true;
    }
    else if( bCCW )
    {
        if( div > 1 )
        {
            div--;
            ioSetPWMDiv( div );
        }
        bUsed = true;
    }
    else if( bShortPress )
    {
        div = AUDIO_DIVIDE;
        ioSetPWMDiv( div );
        bUsed = true;
    }
    char buf[TEXT_BUF_LEN];
    sprintf( buf, "PWM Divider: %d", div);
    displayMenu( buf );
    
    return bUsed;
}

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

// Gets a VFO frequency - usually called from CAT control
uint32_t getVFOFreq( uint8_t vfo )
{
    uint32_t freq = 0;
    if( vfo < NUM_VFOS )
    {
        freq = vfoState[vfo].freq;
    }
    return freq;
}

// Get the current VFO frequency
uint32_t getCurrentVFOFreq()
{
    return getVFOFreq( currentVFO );
}

// Get the other VFO frequency
uint32_t getOtherVFOFreq()
{
    return getVFOFreq( OTHER_VFO );
}

// Get current VFO RIT/XIT offset
int16_t getCurrentVFOOffset()
{
    return vfoState[currentVFO].offset;
}

// Returns true if the current VFO is in RIT mode
bool getCurrentVFORIT()
{
    return (vfoState[currentVFO].mode == vfoRIT);
}

// Returns true if the current VFO is in XIT mode
bool getCurrentVFOXIT()
{
    return (vfoState[currentVFO].mode == vfoXIT);
}

// Get other VFO RIT/XIT offset
int16_t getOtherVFOOffset()
{
    return vfoState[currentVFO].offset;
}

// Returns true if the other VFO is in RIT mode
bool getOtherVFORIT()
{
    return (vfoState[currentVFO].mode == vfoRIT);
}

// Returns true if the other VFO is in XIT mode
bool getOtherVFOXIT()
{
    return (vfoState[currentVFO].mode == vfoXIT);
}

// Sets a VFO to a frequency - usually called from CAT control
void setVFOFrequency( uint8_t vfo, uint32_t freq )
{
    if( vfo < NUM_VFOS )
    {
        vfoState[vfo].freq = freq;
    }
    setFrequencies();
}

// Return the current VFO - for CAT control
uint8_t getCurrentVFO()
{
    return currentVFO;
}

// Set the current VFO - from CAT control
void setCurrentVFO( uint8_t vfo )
{
    if( vfo < NUM_VFOS )
    {
        currentVFO = vfo;

        // Update the frequencies and display
        setFrequencies();
    }
}

// Get the split state - for CAT control
bool getVFOSplit()
{
    return bVFOSplit;
}

// Get the transmitting state - for CAT control
bool getTransmitting()
{
    return bTransmitting;
}

// Adjust a VFO. Changes the frequency or the offset by the supplied change.
// Could change both but not a normal usage (simplex changes the frequency, 
// RIT and XIT change the offset 
static void adjustVFO( uint8_t vfo, int32_t freqChange, int32_t offsetChange )
{
    // Record the new frequency and offset
    vfoState[vfo].freq = vfoState[vfo].freq + freqChange;
    vfoState[vfo].offset = vfoState[vfo].offset + offsetChange;

    // Set the new TX and RX frequencies
    setFrequencies();
}

// Handle the rotary control while in the VFO mode
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
        update_cursor();
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
        update_cursor();
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
        update_cursor();
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

// Set the RIT - called from CAT control
void setCurrentVFOOffset( int16_t rit )
{
    // Set the RIT by changing the offset.
    adjustVFO( currentVFO, 0, (int32_t) rit-vfoState[currentVFO].offset);
}

// Handle the rotary control while in the wpm setting mode
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

// See if the main rotary control has been touched and handle its movement
// This will update either the VFO or the wpm or the menu
// Also handles the left and right buttons
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
#ifdef LCD_DISPLAY
        // If we have an auto backlight then turn it on and note the time
        if( currentBacklightMode == backlightAuto )
        {
            lcdBacklight( true );
            lastBacklightTime = millis();
        }
#endif
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
        else switch( currentMode )
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
    }
}


// Main loop is called repeatedly
static void loop()
{
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

    if( displayMode == displayVolume )
    {
        if( lastVolumeTime &&
            (millis() - lastVolumeTime) > VOLUME_DISPLAY_DELAY )
        {
            displayMode = lastDisplayMode;
            displayFrequencyLCD();
            update_cursor();

            // Setting this to zero stops us checking any more
            lastVolumeTime = 0;
        }
    }

#endif

    // See if the morse paddles or straight key have been pressed
    // If not active then deal with other things too
	if( !morseScanPaddles() )
    {
        // Deal with the rotary control/pushbutton
        handleRotary();

        // Deal with the volume control
        handleVolume();

#ifdef CAT_CONTROL
        // Do CAT control
        catControl();
#endif
#if 1
        static u_int32_t lastInputTime;
        static uint8_t lastScale;

        uint32_t now = millis();
        if( (now - lastInputTime) > 500 )
        {
            lastInputTime = now;
        
            uint8_t scale = ioGetScale();
            if( scale != lastScale )
            {
                lastScale = scale;
                //update_display();
                //ioClearScale();
            }

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
        }
#endif
    }
}

void screenInit( void )
{
    int i;

#ifdef LCD_DISPLAY
    displayInit();
#endif

#ifdef OLED_DISPLAY
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

    displayVol();
    displayMorseWpm();
    displayPreamp();
}

#if 0
// ##################

static inline void firIn( int sample, uint16_t *current, int *buffer, int bufLen )
{
    printf("firIn sample %d current %d bufLen %d\n", sample, *current, bufLen);

    // Move to the next sample position, wrapping as needed
    *current = (*current+1) & (bufLen-1);
}

// bufLen must be a power of 2
static inline int firOut( uint16_t current, int *buffer, int bufLen, const int *taps, int numTaps, int precision )
{
    printf("firOut current %d bufLen %d numTaps %d\n", current, bufLen, numTaps);
    int tap;
    uint16_t index;
    int32_t result = 0;

    // Calculate the result from the FIR filter
    index = current;
    for( tap = 0 ; tap < numTaps ; tap++ )
    {
        index = (index-1) & (bufLen-1);
        printf("index = %d ", index);
        result += ((int32_t) buffer[index]) * taps[tap];
    }
    printf("\n");
    // Extract the significant bits
    return result >> 16; // precision;
}

static inline void decimate( int factor, int count, int *inBuf, int inBufLen, uint16_t *inPos, int *outBuf, int outBufLen, uint16_t *outPos, const int *taps, int numTaps, int precision )
{
    printf("decimate factor = %d count = %d inBufLen = %d *inPos = %d\n", factor, count, inBufLen, *inPos);

    // Process each decimation
    for( int i = 0 ; i < count/factor ; i++ )
    {
        int out = firOut( *inPos, inBuf, inBufLen, taps, numTaps, precision );
        firIn( out, outPos, outBuf, outBufLen );

        // Move the input pointer past the decimation factor
        *inPos = (*inPos+factor) & (inBufLen-1);
        printf("New *inPos %d\n", *inPos);
    }
}

// ##################
#endif

int main(void)
{
    stdio_init_all();
    printf("Hello\n");

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
    // Get the BFO frequency from NVRAM
    BFOFrequency = nvramReadBFOFreq();

    setBand( nvramReadBand() );

    // Enable the RX clock outputs
    // We enable the TX output only when transmitting
    enableRXClock( true );

    // Unmute the receiver
    muteRX( false );

#if 0
// #####
#define DECIMATE_BUFFER_LEN 128
#define ADC_BUFFER_SIZE 32
static int         iInputBuffer[DECIMATE_BUFFER_LEN];
static uint16_t iIn12864, qIn12864;
static uint16_t iOut12864, qOut12864;
static int  iDecimate6432Buffer[DECIMATE_BUFFER_LEN];
#define DECIMATE_128_64_FILTER_TAP_NUM 7
#define DECIMATE_128_64_FILTER_PRECISION 32768

static int decimate12864FilterTaps[DECIMATE_128_64_FILTER_TAP_NUM] =
{
  -1205,
  -142,
  9414,
  16565,
  9414,
  -142,
  -1205
};

    for( int s = 0 ; s < 100 ; s++ )
    {
        decimate( 2, 16, iInputBuffer, DECIMATE_BUFFER_LEN, &iIn12864, iDecimate6432Buffer, DECIMATE_BUFFER_LEN, &iOut12864, decimate12864FilterTaps, DECIMATE_128_64_FILTER_TAP_NUM, DECIMATE_128_64_FILTER_PRECISION );
    }
// #####
#endif

    while (1) 
    {
        loop();
    }
}
