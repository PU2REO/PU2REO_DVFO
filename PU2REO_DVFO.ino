/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Programmed by PU2REO, Copyright 2023
//
// History:
//  v1.0
//    - Progamable Frequency Generator via Arduino/Si5351
//    - Digital TX/RX controller
//    - Digital RogerBeep
//    - VoiceLock actuating directly on frequency generation via Analogic Input
//    - Adjustables IF Frequency, LSB & USB offset, Voice Lock Range, RogerBeep
//    - Channels and VFO Modes
//    - S-Meter on display, via Analogic Input
// 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                             Adrduino Nano Pinout
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                    +-----+
//                       +------------| USB |------------+
//                       |            +-----+            |
//                       | [ ]D13/SCK        MISO/D12[ ] |
//                       | [ ]3.3V           MOSI/D11[ ] | 
//                       | [ ]V.ref     ___    SS/D10[ ] | DO_RG_BEEP
//                       | [ ]A0       / N \       D9[ ] | DO_PTT_TX
//           DI_ENC_KEY  | [ ]A1      /  A  \      D8[ ] | 
//           DI_ENC_PIN2 | [ ]A2      \  N  /      D7[ ] | DI_PTT_KEY
//           DI_ENC_PIN1 | [ ]A3       \_0_/       D6[ ] | DI_MODE_USB
//               SCI_SDA | [ ]A4/SDA               D5[ ] | DI_MODE_AM
//               SCI_SCL | [ ]A5/SCL               D4[ ] | 
//             AI_SMETER | [ ]A6 **           INT1/D3[ ] | 
//         AI_VOICE_LOCK | [ ]A7 **           INT0/D2[ ] |
//                       | [ ]5V                  GND[ ] |     
//                       | [ ]RST                 RST[ ] |
//                       | [ ]GND   5V MOSI GND   TX1[ ] |
//                       | [ ]Vin   [ ] [ ] [ ]   RX1[ ] |
//                       |          [ ] [ ] [ ]          |
//                       |          MISO SCK RST         |
//                       |            NANO-V3            |
//                       +-------------------------------+
//
//      Note 1 -  A6 and A7 pins can ONLY be used as analogic inputs.
//      Note 2 -  A7 pin open (Voice Lock) will calse multiple set_freq calls, resulting in audio noise.
//      Note 3 -  Voice Lock Potentiometer needs a RC Filter on its terminals. Without the filter, arduinos resets itself from time to time
//		  Note 4 -  Bootloader set all(?) pins to high state, making the TX relay active for bootloader time.
//      Note 5 -  Due to Note 4, a cheap programmer is recommended (removes bootloader when used)
//                       
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      To remove the waste of program memory (About 2k) with the splash screen:
//        1 - Find and open "Adafruit_SSD1306.cpp" inside "Adafruit_SSD1306" library
//        2 - Define: #define SSD1306_NO_SPLASH at the beggining of the file.
//      Repeat these steps after every update for "Adafruit_SSD1306" library
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// How to calculate proper interrupt timings
// Formula: Cvalue = ((FArdClk / (Prescaler * Fint)) - 1
//
// Where:   Cvalue    = Value to be put on counter (must be < 256 for T0 and T2 = 8bits, and < 65536 for T1= 16bits)
//          FardClk   = Arduino Clock Frequency (16 Mhz)
//          Prescaler = Ardunino Prescaler Value
//          Fint      = Interrupt Frequency (10ms = 100Hz)
//
// See:     http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Includes
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_SSD1306.h>
#include <RotaryEncoder.h>                                      // https://github.com/mathertel/RotaryEncoder 
#include <si5351_lite.h>                             						// Based on v2.1.4, modified for just 3 clocks
                                                                // Needs at least 1300 bytes of free dynamic RAM memory to run.
#include <ButtonV2.h>                                           // https://github.com/AndrewMascolo/ButtonV2/tree/master/ButtonV2

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Definitions
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define DVFO_VERSION                              "DVFO v1.0"           // Current DDS Version
#define IF_MODE                                                         // enables IF mode (Adds ACT.EE_Values[EE_INDEX_INTFREQ] to the output - to use with a real radio)
#define DIGITAL_ROGER_BEEP                                              // Enable Digital Roger Beep (tone function in arduino)
// #define PROTEUS                                                      // enable initial setup of EEPROM values for Proteus

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// Yellow Display (L,C): ( 0,0) to (15x127)
// Blue   Display (L,C): (16,0) to (63x127)
#define SCREEN_WIDTH                              128                   // OLED display width, in pixels
#define SCREEN_HEIGHT                             64                    // OLED display height, in pixels
#define DISPLAY_ADDR                              0x3C                  // OLED display Address
#define OLED_RESET                                -1                    // Reset pin # (or -1 if sharing Arduino reset pin)

// FontSize 1: 6 pixels/char

// Encoder Definitions
#define ENC_KEY_PRESS_TIME                        500                   // minimum time to set button "Holding" state in [ms]

// Si5351 Definitions
#define SI5351_VFO_CLOCK                          SI5351_CLK0           // Main VFO Clock
#define SI5351_HFR_CLOCK                          SI5351_CLK2           // HF Receiver Clock
#define SI5351_CAL                                75200                 // in 0.01 [Hz] - Oscilloscope Tektronix TBS1102B + 148GTL
#define SI5351_MIN_FREQ                           24000                 // in 1 [kHz] - 12m band
#define SI5351_INIT_FREQ                          27455                 // in 1 [kHz] - 11m band
#define SI5351_MAX_FREQ                           30000                 // in 1 [kHz] - 10m band
#define SI5351_MAX_COEF                           3                     // max CoefTable index
#define SI5351_FREQ_MULTIPLIER                    100000ULL        		  // display frequency multiplier
#define SI5351_VOICELOCK_MULTIPLIER               10000ULL        		  // voice lock input multiplier (will act on tenths of Hz)

// clock enable/disable control
#define SI5351_CLK_DISABLE                        0                     // disable clock
#define SI5351_CLK_ENABLE                         1                     // enable clock

// Timer definitions
#define TIMERDOWN_BEEP_VALUE                      30                    // x 10[ms]
#define TIMERDOWN_CHANNEL_SAVE_VALUE              500                   // x 10[ms]
#define TIMERDOWN_DELAY_AFTER_BEEP_VALUE          5                     // x 10[ms]
// #define TIMERDOWN_RUNNING_LED_VALUE               50                    // x 10[ms]

// timers
#define MAX_NR_TIMER_DOWN                         3                     // Maximum number of TimerDown Timers
#define TIMERDOWN_BEEP                            0                     // Timer Down for RogerBeep Control
#define TIMERDOWN_CHANNEL_SAVE                    1                     // Timer Down for Channel Save
#define TIMERDOWN_DELAY_AFTER_BEEP                2                     // Timer Down for RogerBeep Control
// #define TIMERDOWN_RUNNING_LED                     3                     // Timer Down for Led Control

// Digital Roger Beep
#define DIGITAL_RB_FREQUENCY                      2000                  // Roger Beep frequency in [Hz]

// Radio States
#define RADIO_STATE_RX                            0                     // Radio State - Radio is Receiving
#define RADIO_STATE_TX                            1                     // Radio State - Radio is Transmitting
#define RADIO_STATE_BEEP                          2                     // Radio State - Radio is Transmitting Beep
#define RADIO_STATE_DELAY_RX                      3                     // Radio State - Radio is waiting to activate reception

// Analog Inputs
#define AI_SMETER                                 A6                    // Analogic Input - S-Meter
#define AI_VOICE_LOCK                             A7                    // Analogic Input - Voice Lock

// Digital Inputs
#define DI_ENC_KEY                                A1                    // Digital Input - Encoder Key - Active Low
#define DI_ENC_PIN2                               A2                    // Digital Input - Encoder Pin 2
#define DI_ENC_PIN1                               A3                    // Digital Input - Encoder Pin 1
#define DI_MODE_AM                                5                     // Digital Input - LSB Key - Active Low
#define DI_MODE_USB                               6                     // Digital Input - USB Key - Active Low
#define DI_PTT_KEY                                7                     // Digital Input - PTT Key - Active Low

// Digital Outputs
#define DO_PTT_TX                                 9                     // Digital Output - Enable Transmition
#define DO_RG_BEEP                                10                    // Digital Output - Enable RogerBeep

// Channel Definitions
#define MAX_CHANNEL_INDEX                         309                   // Max Number of Channels - Refer to ChannelsFreq[] and Channels[] matrices

// menu type definitions
#define MENU_TYPE_VFO                             0                     // Menu Type - Displaying VFO Style 
#define MENU_TYPE_CHANNEL                         1                     // Menu Type - Displaying Channels Style

// roger beep definitions
#define MENU_ROGERBEEP_OFF                        0                     // Roger Beep Type - Roger Beep is Off 
#define MENU_ROGERBEEP_ON                         1                     // Roger Beep Type - Roger Beep is On  

// contrast definitions
#define CONTRAST_COMMAND                          0x81                  // Initial contrast command
#define MENU_CONTRAST_MIN                         0x01                  // Minimum contrast value
#define MENU_CONTRAST_MAX                         0xFF                  // Maximum contrast Value
#define MENU_SMETER_MIN                           0x00                  // Minimum SMeter Value
#define MENU_SMETER_MAX                           0x3FF                 // Maximum SMeter Value
#define DEFAULT_CONTRAST_VALUE                    0x7F                  // Default contrast value

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///  EEPROM data location map 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// #define EE_SAVED_MENUTYP                          00                    // Menu Type - 4 bytes long
// #define EE_SAVED_RB_STATUS                        04                    // Roger Beep Status - 4 bytes long
// #define EE_SAVED_CONTRAST                         08                    // Contrast Value - Range 0-255
// #define EE_SAVED_CALIBRT                          12                    // Si5351 Calibration Value - 4 bytes long
// #define EE_SAVED_INTFREQ                          16                    // Radio's IF (7800 kHz for 148 GTL (XTal X4) - x 1 [kHz] - 4 bytes long
// #define EE_SAVED_LSBOFFS                          20                    // LSB Frequency Offset - x 100 [Hz] - 4 bytes long
// #define EE_SAVED_USBOFFS                          24                    // USB Frequency Offset - x 100 [Hz] - 4 bytes long
// #define EE_SAVED_VL_RANGE                         28                    // Voice lock range in [Hz]
// #define EE_SAVED_SMETER_CALIBRT                   32                    // SMeter Calibration
// #define EE_SAVED_INDEX_CHANNEL_IDX                36                    // Channel Index - 4 bytes long
// #define EE_SAVED_INDEX_FREQUENCY                  40                    // Displayed Frequency - 4 bytes long
// #define EE_SAVED_INITIALIZED                      42                    // EEPROM Initializaed (Initial Saved Data)  

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///  Enumerations
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
enum eeprom_data 
{
      EE_INDEX_MENUTYP,                                                 // Menu Type: 0 - VFO / 1 - channel - 4 bytes long
      EE_INDEX_RB_STATUS,                                               // Roger Beep Status: 0 - Off / 1 - ON - 4 bytes long
      EE_INDEX_CONTRAST,                                                // Contrast Value - Range 0-255 - 4 bytes long
      EE_INDEX_CALIBRT,                                                 // Si5351 Calibration Value - 4 bytes long
      EE_INDEX_INTFREQ,                                                 // Radio's IF (7800 kHz for 148 GTL (XTal X4) - x 1 [kHz] - 4 bytes long
      EE_INDEX_LSBOFFS,                                                 // LSB Frequency Offset - x 1 [Hz] - 4 bytes long
      EE_INDEX_USBOFFS,                                                 // USB Frequency Offset - x 1 [Hz] - 4 bytes long
      EE_INDEX_VL_RANGE,                                                // Voice Lock Range - x 1 [Hz] - 4 bytes long
      EE_INDEX_SMETER_CALIBRT,                                          // SMeter Calibration Range  - EXPRERIMENTAL
	    EE_INDEX_SAVED_CHANNEL_IDX,                                       // Channel Displayed Index - 4 bytes long
	    EE_INDEX_SAVED_FREQUENCY,                                         // Displayed Frequency - 4 bytes long
      EE_INDEX_EEPROM_STATUS,                                           // EEPROM Status (Initial Saved Data)      
      EE_INDEX_MAX
};

// Service Menu max index definition
#define EE_SERV_MENU_INDEX_MAX								    EE_INDEX_SMETER_CALIBRT+1   // set this definition to the maximum item to be shown in service menu

// EEPROM Status definitions
#define EEPROM_STATUS_UNKNOWN                     -1
#define EEPROM_STATUS_INITIALIZED                  1

enum mode_data
{
      MODULATION_MODE_LSB,                                              // Modulation Mode - Raidio Operating in LSB
      MODULATION_MODE_AM,                                               // Modulation Mode - Raidio Operating in AM
      MODULATION_MODE_USB                                               // Modulation Mode - Raidio Operating in USB
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Structs Definitions
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct
{
    int32_t      DispFreq;                                              // Frequency to be displayed
    uint16_t     ChannelIndex;                                          // Channel Index
    int8_t       CoefIndex;                                             // CoefTable Index
    bool         ServMenuRequest;                                       // Request for Service Menu 
    bool         ServMenuInUse;                                         // Service Menu in Use     
} VFO_typ;

typedef struct
{
    int32_t      EE_Values[EE_INDEX_MAX];                               // vector of values to be read / saved in EEPROM
    uint8_t      EE_Values_Index;                                       // index for menus/EE_Values EEPROM vector
    int32_t      OldCalibrationAdjust;                                  // stores old calibration value to avoid unecessary calculations
    uint16_t     OldChannelIndex;                                       // channel Index for saving purposes
    int32_t      OldDispFreq;                                           // displayed frequency for saving purposes
} ACT_typ;

typedef struct
{
    uint16_t     Counter;                                               // timer down counter   
} TimerDown_typ;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Constants Definitions
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const int16_t  CoefTable[] PROGMEM = {      1,  10, 100, 1000};                                                     // kilo Hertz or Channels
const int16_t  Channels[]  PROGMEM = {   -150,-149,-148,-147,-146,-145,-144,-143,-142,-141,                         // Display Channels - 7XX Channels are reserved for Radio Control Radio Service (RCRS)
                                         -140,-139,-138,-137,-136,-135,-134,-133,-132,-131,
                                         -130,-129,-128,-127,-126,-125,-124,-123,-122,-121,
                                         -120,-119,-118,-117,-116,-115,-114,-113,-112,-111,
                                         -110,-109,-108,-107,-106,-105,-104,-103,-102,-101,
                                         -100, -99, -98, -97, -96, -95, -94, -93, -92, -91,
                                          -90, -89, -88, -87, -86, -85, -84, -83, -82, -81,                               
                                          -80, -79, -78, -77, -76, -75, -74, -73, -72, -71,
                                          -70, -69, -68, -67, -66, -65, -64, -63, -62, -61,
                                          -80, -59, -58, -57, -56, -55, -54, -53, -52, -51,
                                          -50, -49, -48, -47, -46, -45, -44, -43, -42, -41,
                                          -40, -39, -38, -37, -36, -35, -34, -33, -32, -31,
                                          -30, -29, -28, -27, -26, -25, -24, -23, -22, -21,
                                          -20, -19, -18, -17, -16, -15, -14, -13, -12, -11,
                                          -10,  -9,  -8,  -7,  -6,  -5,  -4,  -3,  -2,  -1,
                                            1,   2,   3, 701,   4,   5,   6,   7, 702,  8,   9,  10,
                                           11, 703,  12,  13,  14,  15, 704,  16,  17,  18,  19, 705, 20,
                                           21,  22,  23,  24,  25,  26,  27,  28,  29,  30,
                                           31,  32,  33,  34,  35,  36,  37,  38,  39,  40,
                                           41,  42,  43,  44,  45,  46,  47, 706,  48,  49,  50,
                                           51, 707,  52,  53,  54,  55, 708,  56,  57,  58,  59, 709, 60,
                                           61,  62,  63,  64,  65,  66,  67,  68,  69,  70,
                                           71,  72,  73,  74,  75,  76,  77,  78,  79,  80,
                                           81,  82,  83,  84,  85,  86,  87,  88,  89,  90,
                                           91,  92,  93,  94,  95,  96,  97,  98,  99, 100,
                                          101, 102, 103, 104, 105, 106, 107, 108, 109, 110,
                                          111, 112, 113, 114, 115, 116, 117, 118, 119, 120,
                                          121, 122, 123, 124, 125, 126, 127, 128, 129, 130,
                                          131, 132, 133, 134, 135, 136, 137, 138, 139, 140,
                                          141, 142, 143, 144, 145, 146, 147, 148, 149, 150};

const int16_t  ChannelsFreq[] PROGMEM = {   // Display Channels Frequencies
                                            25465, 25475, 25485, 25495, 25505, 25515, 25525, 25535, 25545, 25555,                       // -150 to -141
                                            25565, 25575, 25585, 25595, 25605, 25615, 25625, 25635, 25645, 25655,                       // -140 to -131
                                            25665, 25675, 25685, 25695, 25705, 25715, 25725, 25735, 25745, 25755,                       // -130 to -121
                                            25765, 25775, 25785, 25795, 25805, 25815, 25825, 25835, 25845, 25855,                       // -120 to -111
                                            25865, 25875, 25885, 25895, 25905, 25915, 25925, 25935, 25945, 25955,                       // -110 to -101
                                            25965, 25975, 25985, 25995, 26005, 26015, 26025, 26035, 26045, 26055,                       // -100 to  -91
                                            26065, 26075, 26085, 26095, 26105, 26115, 26125, 26135, 26145, 26155,                       //  -90 to  -81
                                            26165, 26175, 26185, 26195, 26205, 26215, 26225, 26235, 26245, 26255,                       //  -80 to  -71 
                                            26265, 26275, 26285, 26295, 26305, 26315, 26325, 26335, 26345, 26355,                       //  -70 to  -61
                                            26365, 26375, 26385, 26395, 26405, 26415, 26425, 26435, 26445, 26455,                       //  -60 to  -51
                                            26465, 26475, 26485, 26495, 26505, 26515, 26525, 26535, 26545, 26555,                       //  -50 to  -41
                                            26565, 26575, 26585, 26595, 26605, 26615, 26625, 26635, 26645, 26655,                       //  -40 to  -31
                                            26665, 26675, 26685, 26695, 26705, 26715, 26725, 26735, 26745, 26755,                       //  -30 to  -21
                                            26765, 26775, 26785, 26795, 26805, 26815, 26825, 26835, 26845, 26855,                       //  -20 to  -11
                                            26865, 26875, 26885, 26895, 26905, 26915, 26925, 26935, 26945, 26955,                       //  -10 to   -1
                                            26965, 26975, 26985, 26995, 27005, 27015, 27025, 27035, 27045, 27055, 27065, 27075,         //    1 to   10, plus 2 RCRS
                                            27085, 27095, 27105, 27115, 27125, 27135, 27145, 27155, 27165, 27175, 27185, 27195, 27205,  //   11 to   20, plus 3 RCRS
                                            27215, 27225, 27255, 27235, 27245, 27265, 27275, 27285, 27295, 27305,                       //   21 to   30
                                            27315, 27325, 27335, 27345, 27355, 27365, 27375, 27385, 27395, 27405,                       //   31 to   40  
                                            27415, 27425, 27435, 27455, 27465, 27475, 27485, 27495, 27505, 27515, 27525,                //   41 to   50, plus 1 RCRS
                                            27535, 27545, 27555, 27565, 27575, 27585, 27595, 27605, 27615, 27625, 27635, 27645, 27655,  //   51 to   60, plus 3 RCRS
                                            27665, 27675, 27705, 27685, 27695, 27715, 27725, 27735, 27745, 27755,                       //   61 to   70
                                            27765, 27775, 27785, 27795, 27805, 27815, 27825, 27835, 27845, 27855,                       //   71 to   80
                                            27865, 27875, 27885, 27895, 27905, 27915, 27925, 27935, 27945, 27955,                       //   81 to   90
                                            27965, 27975, 27985, 27995, 28005, 28015, 28025, 28035, 28045, 28055,                       //   91 to  100
                                            28065, 28075, 28085, 28095, 28105, 28115, 28125, 28135, 28145, 28155,                       //  101 to  110
                                            28165, 28175, 28185, 28195, 28205, 28215, 28225, 28235, 28245, 28255,                       //  111 to  120
                                            28265, 28275, 28285, 28295, 28305, 28315, 28325, 28335, 28345, 28355,                       //  121 to  130
                                            28365, 28375, 28385, 28395, 28405, 28415, 28425, 28435, 28445, 28455,                       //  131 to  140
                                            28465, 28475, 28485, 28495, 28505, 28515, 28525, 28535, 28545, 28555};                      //  141 to  150

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Global Variables
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
Adafruit_SSD1306              display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);				      // define Diplay control variable
RotaryEncoder                 encoder(DI_ENC_PIN1, DI_ENC_PIN2, RotaryEncoder::LatchMode::FOUR3);   // define Encoder control variable - Library v1.5.0
Si5351                        si5351;                                                               // define Si5351 control variable    
ButtonV2                      EncKey;                                                               // define Improved Button control variable
VFO_typ                       VFO;                                                                  // define VFO control variable
ACT_typ                       ACT;                                                                  // actual values
int8_t                        tmpMode;                                                              // stores current modulation mode
int8_t                        tmpOldMode      = -1;                                                 // stores Old mode - no mode at start
uint8_t                       tmpHFRecStatus  =  5;                                                 // 5 always forces an update, since possible valaues are HIGH and LOW (1 and 0)
uint8_t                       RadioState      =  RADIO_STATE_RX;                                    // stores radio state (Controls PTT button actions)
uint8_t                       MenuType;                                                             // stores menu type
int16_t                       tmpOldVoiceLock =  0;                                                 // stores Old VoiceLock value
int64_t                       tmpFreq = 0ULL;                                                       // frequency calculation variable.
volatile TimerDown_typ        TimerDown[MAX_NR_TIMER_DOWN];                                         // timer vector

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Function Prototypes
///////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
int32_t   CheckLimits32(int32_t Variable, int32_t Minimum, int32_t Maximum);
int64_t   CheckLimits64(int64_t Variable, int64_t Minimum, int64_t Maximum);
void      SaveValuesToEEPROM(void);
void      ReadValuesFromEEPROM(void);
void      SetContrast(uint8_t);
void      ShowLogo(void);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Initialization Routine
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() 
{
	  // stop interrupts
    cli();
	
	  // set entire TCCR1A and TCCR1B registers to 0
    TCCR1A = 0;
    TCCR1B = 0;

	  // initialize Timer 1 counter value to 0	
    TCNT1  = 0; 
	
    // set compare match register for 100 Hz increments
    OCR1A = 19999; // = 16000000 / (8 * 100) - 1 (must be <65536) ///////////////////// 10ms
	
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
	
    // Set CS12, CS11 and CS10 bits for 8 prescaler
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
	
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
	
	  // allow interrupts
    sei(); 

    // Interrupts for encoder pins A2 and A3
    PCICR  |= (1 << PCIE1);                               // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
    PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);            // This enables the interrupt for pin 2 and 3 of Port C.
  
    // initialize digital outputs
    pinMode(DO_PTT_TX,   OUTPUT);
    pinMode(DO_RG_BEEP,  OUTPUT);

    // set digital outputs initial state
    digitalWrite(DO_PTT_TX,  LOW);
    digitalWrite(DO_RG_BEEP, LOW);

    // initialize digital inputs
    pinMode(DI_ENC_KEY,  INPUT_PULLUP);                   // Active Low
    pinMode(DI_MODE_AM,  INPUT_PULLUP);                   // Active Low
    pinMode(DI_MODE_USB, INPUT_PULLUP);                   // Active Low
    pinMode(DI_PTT_KEY,  INPUT_PULLUP);                   // Active Low
  
    // Read EEProm values
    ReadValuesFromEEPROM();

    // check for initialized EEPROM
    if (ACT.EE_Values[EE_INDEX_EEPROM_STATUS] == EEPROM_STATUS_UNKNOWN)
    {
        // adjust initial values and save to EEPROM
        ACT.EE_Values[EE_INDEX_MENUTYP]           =  1;
        ACT.EE_Values[EE_INDEX_RB_STATUS]         =  1;
        ACT.EE_Values[EE_INDEX_CONTRAST]          =  DEFAULT_CONTRAST_VALUE;
        ACT.EE_Values[EE_INDEX_CALIBRT]           =  75200;
        ACT.EE_Values[EE_INDEX_INTFREQ]           =  7800;
        ACT.EE_Values[EE_INDEX_LSBOFFS]           = -1500;
        ACT.EE_Values[EE_INDEX_USBOFFS]           =  1500;
        ACT.EE_Values[EE_INDEX_VL_RANGE]          =  10200;
        ACT.EE_Values[EE_INDEX_SMETER_CALIBRT]    =  230;
        ACT.EE_Values[EE_INDEX_SAVED_CHANNEL_IDX] =  150;
        ACT.EE_Values[EE_INDEX_SAVED_FREQUENCY]   =  SI5351_INIT_FREQ;
        ACT.EE_Values[EE_INDEX_EEPROM_STATUS]     =  EEPROM_STATUS_INITIALIZED;
        SaveValuesToEEPROM();
    }
   
    // restore Si5351 Calibration value
    ACT.OldCalibrationAdjust = ACT.EE_Values[EE_INDEX_CALIBRT];

    // initialize Si5351
    if (!si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0))
    {
       #ifndef PROTEUS
         for(;;); // Don't proceed, loop forever
       #endif
    }
  
    // calibrates Si5351 to the previous stored value
    si5351.set_correction((int32_t)ACT.EE_Values[EE_INDEX_CALIBRT], SI5351_PLL_INPUT_XO);
  
    // Set PLL fixed to SI5351_PLL_FIXED
    si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
    
    // initialize VFO Control variable
    VFO.DispFreq        = SI5351_INIT_FREQ;
    VFO.ServMenuInUse   = 0;
    VFO.ServMenuRequest = 0;
    VFO.ChannelIndex    = (int16_t)ACT.EE_Values[EE_INDEX_SAVED_CHANNEL_IDX];
    ACT.OldChannelIndex = VFO.ChannelIndex;
    VFO.DispFreq        = ACT.EE_Values[EE_INDEX_SAVED_FREQUENCY] != 0 ? ACT.EE_Values[EE_INDEX_SAVED_FREQUENCY]: SI5351_INIT_FREQ;
    ACT.OldDispFreq     = VFO.DispFreq;

    // check initial CoefIndex Value
    VFO.CoefIndex = ACT.EE_Values[EE_INDEX_MENUTYP] == MENU_TYPE_CHANNEL ? 0 : 1;
  
    // set SI5351 power-on frequency
    #ifndef IF_MODE
      si5351.set_freq(((int64_t)VFO.DispFreq * SI5351_FREQ_MULTIPLIER), SI5351_VFO_CLOCK);
    #else
      si5351.set_freq(((int64_t)(VFO.DispFreq + ACT.EE_Values[EE_INDEX_INTFREQ]) * SI5351_FREQ_MULTIPLIER), SI5351_VFO_CLOCK);
    #endif

    // set driver current output
    si5351.drive_strength(SI5351_VFO_CLOCK, SI5351_DRIVE_4MA);
  
    // disable unused clocks. Plans for them will come later.
    si5351.output_enable(SI5351_CLK1, SI5351_CLK_DISABLE);
    si5351.set_clock_disable(SI5351_CLK1, SI5351_CLK_DISABLE_HI_Z);
    si5351.output_enable(SI5351_CLK2, SI5351_CLK_DISABLE);
    si5351.set_clock_disable(SI5351_CLK2, SI5351_CLK_DISABLE_HI_Z);
  
    // initialize encoder buton status
    EncKey.SetStateAndTime(LOW, ENC_KEY_PRESS_TIME);
  
    // initialize display
    if(!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDR)) 
    {
       #ifndef PROTEUS
         for(;;); // Don't proceed, loop forever
       #endif
    }
    display.clearDisplay();
  
    // display contrast
    if (ACT.EE_Values[EE_INDEX_CONTRAST] == 0)
    { 
        // set default value
        ACT.EE_Values[EE_INDEX_CONTRAST] = DEFAULT_CONTRAST_VALUE;
    }
    SetContrast((uint8_t)ACT.EE_Values[EE_INDEX_CONTRAST]);

    // channel/frequency save interval
    TimerDown[TIMERDOWN_CHANNEL_SAVE].Counter = TIMERDOWN_CHANNEL_SAVE_VALUE;

    // Radio state
    RadioState = RADIO_STATE_RX;

    // show logo
    ShowLogo();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Timer 1 Interrupt - 10 [ms] - Manage Timers Vector
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER1_COMPA_vect)
{
    // Timers down
    for(uint8_t i=0; i<MAX_NR_TIMER_DOWN; i++)
    {
        // decrements only if greater than zero
        if(TimerDown[i].Counter != 0)
        {
            TimerDown[i].Counter--;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3: exactly where we need to check.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT1_vect) 
{
    // check ncoder state when inputs have changed
    encoder.tick();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Ciclic Routine
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() 
{
    // variables declaration
    char       LCDstr[11];
    bool       btmpFreqHasChanged      = false; 
    bool       btmpVoiceLockHasChanged = false; 

    /////////////////////////////
    // read inputs
    /////////////////////////////

    // read SMeter input
    #ifdef PROTEUS                                                                                                      // check if Proteus is enabled
      int16_t tmpSMeterBar = 1000;                                                                                      // force ramdomic values
    #else
      int16_t tmpSMeterBar = map(analogRead(AI_SMETER), 0, ACT.EE_Values[EE_INDEX_SMETER_CALIBRT], 0, 1023);                                                                     // Analogic input para o s-meter
    #endif

    // read mode key input
    tmpMode = (!digitalRead(DI_MODE_AM)) | ((!digitalRead(DI_MODE_USB)) << 1);                                          // Digital inputs for band mode    

    // voice lock management
    int16_t tmpVoiceLock = ((analogRead(AI_VOICE_LOCK)-512) * (ACT.EE_Values[EE_INDEX_VL_RANGE]/100)) / 1023.0;       // Analogic input for VoiceLock
    btmpVoiceLockHasChanged = (tmpOldVoiceLock != tmpVoiceLock);                                                      // Check for changes in Voice Lock

    // menu type
    MenuType = (uint8_t)ACT.EE_Values[EE_INDEX_MENUTYP];
    /////////////////////////////
    // end read inputs
    /////////////////////////////

    /////////////////////////////
    // manage encoder
    /////////////////////////////
    // manage button
    byte type = EncKey.CheckButton(DI_ENC_KEY); // current time and length of time to press the button as many times as you can ie. 1.5 seconds
    switch (type)
    {
        case WAITING:
          // check for service menu request
          if (VFO.ServMenuRequest)
          {
              // check if service menu is already on
              if (VFO.ServMenuInUse)
              { 
                  // save all parameters and exit service menu
                  SaveValuesToEEPROM();
        
                  // force update frequency
                  btmpFreqHasChanged = true;
      
                  // check if CoefIndex is properly set in case of MENU_TYPE_CHANNEL  
                  if ((MenuType == MENU_TYPE_CHANNEL) && (VFO.CoefIndex > (SI5351_MAX_COEF-2)))
                  {
                      // Set to x 1
                      VFO.CoefIndex = 0;
                  }

                  // terminate service menu
                  VFO.ServMenuInUse = false;
              }
              else
              {
                  // turn on service menu
                  VFO.ServMenuInUse = true;
              }
    
              // clear request
              VFO.ServMenuRequest = false;
          }
        
          // on-the-fly Calibration 
          // check for active service menu
          if (VFO.ServMenuInUse)
          { 
              // check for new calibration value
              if (ACT.EE_Values[EE_INDEX_CALIBRT] != ACT.OldCalibrationAdjust)
              {
                  // calibrate
                  si5351.set_correction((int32_t)ACT.EE_Values[EE_INDEX_CALIBRT], SI5351_PLL_INPUT_XO);
                
                  // Set PLL fixed to 800MHz (to force correction to be applied)
                  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
    
                  // save new calibration value
                  ACT.OldCalibrationAdjust = ACT.EE_Values[EE_INDEX_CALIBRT];
                  
                  // force update frequency
                  btmpFreqHasChanged = true;  
              }
          }
          break;
      
        // button pressed once
        case PRESSED:
          // controls clock coef index - increase and check limits
          VFO.CoefIndex++;
          
          // check menu type
          if ((MenuType == MENU_TYPE_VFO) || VFO.ServMenuInUse)
          {
              // VFO type or service menu
              if (VFO.CoefIndex > SI5351_MAX_COEF)     VFO.CoefIndex = 0;
          }
          else
          {
              // Channel type
              if (VFO.CoefIndex > (SI5351_MAX_COEF-2)) VFO.CoefIndex = 0;
          }
          break;

        // button pressed twice
        case DOUBLE_PRESSED:
          // check for active service menu
          if(VFO.ServMenuInUse)
          {
              // control service menu index
              ACT.EE_Values_Index++;
              if (ACT.EE_Values_Index >= EE_SERV_MENU_INDEX_MAX) ACT.EE_Values_Index = 0;
          }
          break;
  
        // button pressed three times
        case MULTI_PRESSED:
          // ***************
          // ** NOT USED! **
          // ***************
          break;
        
        // button held pressed
        case HELD:
          // request service menu activation/deactivation
          VFO.ServMenuRequest = true;
          break;
    }  
  
    // manage encoder position
    int newPosition = encoder.getPosition();                // variable does not retain its value between function calls
    if (newPosition != 0) 
    {      
        // check for active service menu
        if (!VFO.ServMenuInUse)
        {  
            switch(MenuType)
            {
                // VFO type
                case MENU_TYPE_VFO:
                  // calculate new frequency
                  VFO.DispFreq = (int32_t)CheckLimits64((int64_t)VFO.DispFreq + (int64_t)newPosition * (int64_t)pgm_read_word(&CoefTable[VFO.CoefIndex]), 
                                                        (int64_t)SI5351_MIN_FREQ, 
                                                        (int64_t)SI5351_MAX_FREQ);
                  break;

                // Channel channel type
                case MENU_TYPE_CHANNEL:
                  // with IF Frequency
                  VFO.ChannelIndex += (uint16_t)newPosition * (uint16_t)pgm_read_word(&CoefTable[VFO.CoefIndex]);
                  if ((VFO.ChannelIndex > (MAX_CHANNEL_INDEX-1)) && (newPosition > 0)) VFO.ChannelIndex = MAX_CHANNEL_INDEX-1;
                  if ((VFO.ChannelIndex > (MAX_CHANNEL_INDEX-1)) && (newPosition < 0)) VFO.ChannelIndex = 0;
                  break;
            }
            
            // new freq has been set
            btmpFreqHasChanged = true;
        }
        else
        {
            // calculate new adjust value
            ACT.EE_Values[ACT.EE_Values_Index] += (uint32_t)newPosition * (uint32_t)pgm_read_word(&CoefTable[VFO.CoefIndex]);

            // check for menu type 0 or 1 Allowed
            if (ACT.EE_Values_Index == EE_INDEX_MENUTYP)
            {
                ACT.EE_Values[ACT.EE_Values_Index] = CheckLimits32(ACT.EE_Values[ACT.EE_Values_Index], MENU_TYPE_VFO, MENU_TYPE_CHANNEL);
            }

            // check for roger beep 0 or 1 Allowed
            if (ACT.EE_Values_Index == EE_INDEX_RB_STATUS)
            {
                ACT.EE_Values[ACT.EE_Values_Index] = CheckLimits32(ACT.EE_Values[ACT.EE_Values_Index], MENU_ROGERBEEP_OFF, MENU_ROGERBEEP_ON);
            }

            // check for contrast 0 or 1 Allowed
            if (ACT.EE_Values_Index == EE_INDEX_CONTRAST)
            {
                ACT.EE_Values[ACT.EE_Values_Index] = CheckLimits32(ACT.EE_Values[ACT.EE_Values_Index], MENU_CONTRAST_MIN, MENU_CONTRAST_MAX);
                SetContrast((uint8_t)ACT.EE_Values[ACT.EE_Values_Index]);
            }     

            // check smeter adjust
            if (ACT.EE_Values_Index == EE_INDEX_SMETER_CALIBRT)
            {
                ACT.EE_Values[ACT.EE_Values_Index] = CheckLimits32(ACT.EE_Values[ACT.EE_Values_Index], MENU_SMETER_MIN, MENU_SMETER_MAX);
            }     
        }

      // reset encoder position
      encoder.setPosition(0);
    } // if (newPosition != 0)
    /////////////////////////////
    // end manage encoder
    /////////////////////////////
    
    /////////////////////////////
    // channel/frequency save
    /////////////////////////////
    // check if it's time to save
    if (TimerDown[TIMERDOWN_CHANNEL_SAVE].Counter == 0)
    {
        // Check menu type
        switch(MenuType)
        {
          // Channel type
          case MENU_TYPE_CHANNEL:
              // check if channel has changed
              if (ACT.OldChannelIndex != VFO.ChannelIndex)
              {
                  // store index to the vector and save
                  ACT.OldChannelIndex = VFO.ChannelIndex;  
                  ACT.EE_Values[EE_INDEX_SAVED_CHANNEL_IDX] = VFO.ChannelIndex;
              }
              break;

          // VFO type
          case MENU_TYPE_VFO:
              // check if channel has changed
              if (ACT.OldDispFreq != VFO.DispFreq)
              {
                  // store index to the vector and save
                  ACT.OldDispFreq = VFO.DispFreq;  
                  ACT.EE_Values[EE_INDEX_SAVED_FREQUENCY] = VFO.DispFreq;
              }
              break;
        }

        // save values
        SaveValuesToEEPROM();
        // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

        // reset timer anyway
        TimerDown[TIMERDOWN_CHANNEL_SAVE].Counter = TIMERDOWN_CHANNEL_SAVE_VALUE;
    }
    else
    {
        // check if channel has changed
        if ((ACT.OldChannelIndex == VFO.ChannelIndex) && (ACT.OldDispFreq == VFO.DispFreq))
        {
            // reset timer anyway
            TimerDown[TIMERDOWN_CHANNEL_SAVE].Counter = TIMERDOWN_CHANNEL_SAVE_VALUE;
        }
    }
    /////////////////////////////
    // end channel save
    /////////////////////////////

    //////////////////////////////////
    // manage frequency/mode changes
    //////////////////////////////////    
    if(btmpFreqHasChanged || btmpVoiceLockHasChanged || (tmpMode != tmpOldMode))
    {
        // set old mode
        tmpOldMode      = tmpMode;
        tmpOldVoiceLock = tmpVoiceLock;
       
        // choose IF mode
        #ifdef IF_MODE
          switch(MenuType)
          {
              // VFO type
              case MENU_TYPE_VFO:
                // with IF Frequency
                tmpFreq = (int64_t)((VFO.DispFreq + ACT.EE_Values[EE_INDEX_INTFREQ]) * SI5351_FREQ_MULTIPLIER + (tmpVoiceLock * SI5351_VOICELOCK_MULTIPLIER));
                break;
  
              // VFO channel type
              case MENU_TYPE_CHANNEL:
                // with IF Frequency
                tmpFreq = (int64_t)((pgm_read_word(&ChannelsFreq[VFO.ChannelIndex]) + ACT.EE_Values[EE_INDEX_INTFREQ]) * SI5351_FREQ_MULTIPLIER  + (tmpVoiceLock * SI5351_VOICELOCK_MULTIPLIER));
                break;
          }
        #else
          switch(MenuType)
          {
              // VFO type
              case MENU_TYPE_VFO:
                // without IF Frequency
                tmpFreq = (int64_t)((VFO.DispFreq) * SI5351_FREQ_MULTIPLIER) + (tmpVoiceLock * SI5351_VOICELOCK_MULTIPLIER);
                break;

              // Channel type
              case MENU_TYPE_CHANNEL:
                // without IF Frequency
                tmpFreq = (int64_t)((pgm_read_word(&ChannelsFreq[VFO.ChannelIndex]) * SI5351_FREQ_MULTIPLIER) + (tmpVoiceLock * SI5351_VOICELOCK_MULTIPLIER));
				        break;
          }
        #endif

        // check mode 
        switch(tmpMode)
        {
            // modulating in AM
            case MODULATION_MODE_AM:
              // set base freq + IF offset
              si5351.set_freq(tmpFreq, SI5351_VFO_CLOCK);
            break;  
            
            // modulating in LSB
            case MODULATION_MODE_LSB:
              // set base freq + IF offset + LSB offset
              tmpFreq += ACT.EE_Values[EE_INDEX_LSBOFFS] * SI5351_FREQ_MULT;
              si5351.set_freq(tmpFreq, SI5351_VFO_CLOCK);
            break;  

            // modulating in USB
            case MODULATION_MODE_USB:
              // set base freq + IF offset + USB offset
              tmpFreq += ACT.EE_Values[EE_INDEX_USBOFFS] * SI5351_FREQ_MULT;
              si5351.set_freq(tmpFreq, SI5351_VFO_CLOCK);
            break;  
        }

        // reset freq changes control
        btmpFreqHasChanged = false;
    }
    ///////////////////////////////////////
    // end manage frequency/mode changes
    ///////////////////////////////////////
    
    /////////////////////////////////
    // display management
    /////////////////////////////////
    // clear display
    display.clearDisplay();
  
    // check for service menu
    if (!VFO.ServMenuInUse)
    {
        /////////////////////////////
        // display signal scale line
        /////////////////////////////
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        // display.println(F("SIG.1.3.5.7.9..+30dB"));
        display.println(F("Sig 1 3 5 7 9  +30dB"));
      
        // Display S-Meter
        display.writeFillRect(16, 9, (tmpSMeterBar * 100.0/1023.0), 3, SSD1306_WHITE);      // x, y, width, heigh, color
      
        // segment the solid bar
        for (int i = 1; i < 16; i++) 
        {
            // draws gaps on the solid bar
            display.fillRect(18+(i * 5), 9, 1, 3, SSD1306_BLACK);
        } // for (i = 1; i < 16; i++)
        //////////////////////////////////
        // end display signal scale line
        //////////////////////////////////
      
        ///////////////////////////////////////////
        // display lines, frequency, unit and mode
        ///////////////////////////////////////////
        // draw a line
        display.writeFastHLine(0,16, 128, SSD1306_WHITE);

        // display frequency
        switch(MenuType)
        {
            // VFO type
            case MENU_TYPE_VFO:
              // set font size
              display.setTextSize(3); 
              display.setTextColor(SSD1306_WHITE);
              display.setCursor(0, 22); 

              // adjust displayed frequency
              #ifndef IF_MODE
                dtostrf(((float)tmpFreq/SI5351_FREQ_MULTIPLIER), 7, 1, LCDstr);
              #else
                dtostrf(((float)tmpFreq/SI5351_FREQ_MULTIPLIER - ACT.EE_Values[EE_INDEX_INTFREQ]), 7, 1, LCDstr);
              #endif
              display.print(LCDstr);
            
              // display mode
              display.setTextSize(1); 
              display.setCursor(92, 57);
              
              // check mode
              switch(tmpMode)
              {
                  // display modulation mode
                  case MODULATION_MODE_AM:  display.print(F("AM ")); break;
                  case MODULATION_MODE_LSB: display.print(F("LSB")); break;
                  case MODULATION_MODE_USB: display.print(F("USB")); break;
                  default:                  display.print(F("ERR")); break;
              } 

              // roger beep status
              if (ACT.EE_Values[EE_INDEX_RB_STATUS]) display.print(F(" RB"));              
              break;

            // Channel type
            case MENU_TYPE_CHANNEL:
              {   
                  // In order to declare variables within a case, brackets are needed
                  // set font size
                  display.setTextSize(3); 
                  display.setTextColor(SSD1306_WHITE);
                  display.setCursor(0, 22);
                  
                  // Display Channel
                  int16_t tmpCH = pgm_read_word(&Channels[VFO.ChannelIndex]);
                  if (tmpCH < 700)
                  {
                      // print channel number
                      dtostrf((float)tmpCH, 4, 0, LCDstr);
                  }
                  else 
                  {
                      // Radio Control Radio Service (RCRS) channel number ("Tele-comando")
                      display.print(F("  TC"));
                      display.setCursor(0, 22);
                      dtostrf((float)(tmpCH-700), 1, 0, LCDstr);
                      display.drawCircle(22, 25, 3, SSD1306_WHITE);    // -> These lines are here to draw a degree symbol after the TC number
                      display.writeFastHLine(19,31, 7, SSD1306_WHITE); // -> These lines are here to draw a degree symbol after the TC number 
                  }
                  display.print(LCDstr);
    
                  // display cof value
                  display.setTextSize(1); 
                  display.setCursor(92, 21);
                  display.print(F("x "));
                  dtostrf((float)pgm_read_word(&CoefTable[VFO.CoefIndex]), 1, 0, LCDstr);
                  display.print(LCDstr);
              }

              // display mode
              display.setTextSize(1); 
              display.setCursor(92, 37);
                  
              // check mode
              switch(tmpMode)
              {
                  // display modulation mode
                  case MODULATION_MODE_AM:  display.print(F("AM ")); break;
                  case MODULATION_MODE_LSB: display.print(F("LSB")); break;
                  case MODULATION_MODE_USB: display.print(F("USB")); break;
                  default:                  display.print(F("ERR")); break;
              } 

              // roger beep status
              if (ACT.EE_Values[EE_INDEX_RB_STATUS]) display.print(F(" RB"));
              break;
        }
      
        // draw a line
        display.writeFastHLine(0, 47, 128, SSD1306_WHITE);
        ///////////////////////////////////////////////
        // end display lines, frequency, unit and mode
        ///////////////////////////////////////////////
    }
    else // if (!ServMenuInUse) -  service menu is in use
    {
        // initialize text settings
        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
    
        // menu pages
        switch(ACT.EE_Values_Index)
        {
            // Menu type
            case EE_INDEX_MENUTYP:
              display.println(F("Disp Type"));
            break;

            // RB status
            case EE_INDEX_RB_STATUS:
              display.println(F("RogerBeep"));
            break;

            // Contrast
            case EE_INDEX_CONTRAST:
              display.println(F("Contrast"));
            break;

            // SI5351 Calibration value
            case EE_INDEX_CALIBRT:
              display.println(F("Adj Si5351"));
            break;
      
            // IF Frequency
            case EE_INDEX_INTFREQ:
              display.println(F(" IF [kHz] "));
            break;
      
            // Lower Side Band Offset
            case EE_INDEX_LSBOFFS:
              display.println(F("Of.LSB[Hz]"));
            break;
            
            // Upper Side Band Offset
            case EE_INDEX_USBOFFS:
              display.println(F("Of.USB[Hz]"));
            break;

            // Voice Lock Range
            case EE_INDEX_VL_RANGE:
              display.println(F("VL.Rng[Hz]"));
            break;

            // SMeter Calibration
            case EE_INDEX_SMETER_CALIBRT:
              display.println(F("SMeter Cal"));
            break;
        }
  
        // display value
        display.setCursor(0, 25);
        dtostrf((float)ACT.EE_Values[ACT.EE_Values_Index], 10, 0, LCDstr);
        display.print(LCDstr);
    }
  
    ///////////////////////////////////////////////
    // display footnotes
    ///////////////////////////////////////////////
    // check for service menu
    if (!VFO.ServMenuInUse)
    {
        // choose correct unit
        switch(MenuType)
        {
            // VFO type
            case MENU_TYPE_VFO:
              display.setTextSize(1); 
              display.setCursor(0, 57);
              // display multiply faactor
              display.print(F("x "));
              dtostrf((float)pgm_read_word(&CoefTable[VFO.CoefIndex]), 1, 0, LCDstr);
              display.print(LCDstr);
              // display unit
              display.print(F(" [kHz]"));
              break;
    
            // Channel type
            case MENU_TYPE_CHANNEL:
              // display frequency and unit
              display.setTextSize(2); 
              display.setCursor(0, 50);
              // show frequency based on voice lock modifications
              #ifndef IF_MODE
                // show frequency set
                dtostrf(((float)tmpFreq/SI5351_FREQ_MULTIPLIER), 7, 1, LCDstr);
              #else
                // subtracts IF frequecy before showing
                dtostrf(((float)tmpFreq/SI5351_FREQ_MULTIPLIER - ACT.EE_Values[EE_INDEX_INTFREQ]), 7, 1, LCDstr); 
              #endif
              display.print(LCDstr);
              display.setTextSize(1); 
              display.setCursor(92, 57);
              display.print(F(" [kHz]"));
              break;
        }
    }
    else
    {
        // display multiply faactor
        display.setTextSize(1); 
        display.setCursor(0, 57);
        display.print(F("x "));
        dtostrf((float)pgm_read_word(&CoefTable[VFO.CoefIndex]), 1, 0, LCDstr);
        display.print(LCDstr);
        // display selected clock info
        display.setCursor(96, 57);
        dtostrf((float)(ACT.EE_Values_Index+1), 1, 0, LCDstr);
        display.print(LCDstr);
        display.print(F("/"));
        dtostrf((float)EE_SERV_MENU_INDEX_MAX, 1, 0, LCDstr);
        display.print(LCDstr);
    }        
    ///////////////////////////////////////////////
    // end display footnotes
    ///////////////////////////////////////////////
  
    // update display
    display.display();      
    /////////////////////////////////
    // end display management
    /////////////////////////////////

    ///////////////////////////////////////////////
    // start PTT and Roger Beep management
    ///////////////////////////////////////////////
    // check radio state (starts with PTT button)
    switch(RadioState)
    {
        case RADIO_STATE_RX:     
          // check for PTT - active low
          if (digitalRead(DI_PTT_KEY) == LOW)
          {   
              // enable transmit
              digitalWrite(DO_PTT_TX, HIGH);
              
              // advance radio state
              RadioState = RADIO_STATE_TX;
          }
          break;
          
        case RADIO_STATE_TX:
          // check for PTT - Inactive High
          if (digitalRead(DI_PTT_KEY) == HIGH)
          {
              // check beep enabled state - ACTIVE HIGH
              if (ACT.EE_Values[EE_INDEX_RB_STATUS])
              {
                  // check for analogic/digital Roger Beep
                  #ifdef DIGITAL_ROGER_BEEP
                    // starts digital beep
                    tone(DO_RG_BEEP, DIGITAL_RB_FREQUENCY);
                  #else
                    // enable analogic beep
                    digitalWrite(DO_RG_BEEP, HIGH);
                  #endif
                
                  // advance radio state
                  RadioState = RADIO_STATE_BEEP;
              }
              else
              {
                  // disable transmit
                  digitalWrite(DO_PTT_TX, LOW);
                  
                  // advance radio state
                  RadioState = RADIO_STATE_RX;
              }
          }

          // resets timer
          TimerDown[TIMERDOWN_BEEP].Counter = TIMERDOWN_BEEP_VALUE;
          break;
      
        case RADIO_STATE_BEEP:
          // wait for timer to finish
          if (TimerDown[TIMERDOWN_BEEP].Counter == 0)
          {
              // check for analogic/digital Roger Beep
              #ifdef DIGITAL_ROGER_BEEP
                // ends roger beep
                noTone(DO_RG_BEEP);
              #else
                // disable analogic beep
                digitalWrite(DO_RG_BEEP, LOW);        
              #endif

              // reset radio state back to reception
              RadioState = RADIO_STATE_DELAY_RX;

              // set timer
              TimerDown[TIMERDOWN_DELAY_AFTER_BEEP].Counter =  TIMERDOWN_DELAY_AFTER_BEEP_VALUE;             
          }
          break;

          case RADIO_STATE_DELAY_RX:
          // wait for timer to finish
          if (TimerDown[TIMERDOWN_DELAY_AFTER_BEEP].Counter == 0)
          {
              // disable transmit
              digitalWrite(DO_PTT_TX, LOW);
              
              // reset radio state back to reception
              RadioState = RADIO_STATE_RX;
          }
          break;
    }
    
    ///////////////////////////////////////////////
    // end PTT and Roger Beep management
    ///////////////////////////////////////////////

    ////////////////////////////////////////////////////
    // monitoring inputs and outputs with BuiltIn Led
    ////////////////////////////////////////////////////
    // digitalWrite(LED_BUILTIN, !digitalRead(DI_PTT_KEY));
    // digitalWrite(LED_BUILTIN, digitalRead(DO_PTT_TX));
    // digitalWrite(LED_BUILTIN, digitalRead(DO_RG_BEEP));

    // 1 [second] beacon
    // if (TimerDown[TIMERDOWN_RUNNING_LED].Counter == 0)
    // {
    //     TimerDown[TIMERDOWN_RUNNING_LED].Counter = TIMERDOWN_RUNNING_LED_VALUE;
    //     digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Read values from EEPROM and store them in a RAM vector
// This routine will only be called when initializing uC
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ReadValuesFromEEPROM(void)
{
    // read values from EPPROM
    for(uint16_t i=0; i<EE_INDEX_MAX; i++)
    {
        // read "sizeof(uint32_t)" bytes per time
        ACT.EE_Values[i] = eeprom_read_dword((const uint32_t *)(i * sizeof(uint32_t)));
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// save values to EEPROM from the RAM vector
// This routine will only be called when exiting Service Menu
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SaveValuesToEEPROM(void)
{
    // read values from EPPROM
    for(uint16_t i=0; i<EE_INDEX_MAX; i++)
    {
        // write "sizeof(uint32_t)" bytes per time
        eeprom_write_dword ((uint32_t *)(i * sizeof(uint32_t)), (uint32_t)ACT.EE_Values[i]);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Check boundaries of a 32bits variable
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int32_t CheckLimits32(int32_t Variable, int32_t Minimum, int32_t Maximum)
{
    if (Variable < Minimum) return Minimum;
    if (Variable > Maximum) return Maximum;
    return Variable;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Check boundaries of a 64bits variable
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int64_t CheckLimits64(int64_t Variable, int64_t Minimum, int64_t Maximum)
{
    if (Variable < Minimum) return Minimum;
    if (Variable > Maximum) return Maximum;
    return Variable;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Display Contrast
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SetContrast(uint8_t value)
{
    // Initiate Contrast Command
    display.ssd1306_command(CONTRAST_COMMAND);
    // Set Contrast Value
    display.ssd1306_command(value);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Start Logo
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ShowLogo(void)
{
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(24, 0);
    display.println(F("PU2REO"));
    display.setCursor(12, 27);
    display.println(F(DVFO_VERSION));
    display.setTextSize(1);
    display.setCursor(8, 54);
    display.println(F("Built Jan 1st, 2024"));
    display.display();      
    delay(1000);
    display.clearDisplay();
}
