#ifdef __cplusplus
extern "C" {
#endif

// I2C address
#define I2C_IO       0x20
#define I2C_RELAY    0x21

// Pico pins used 
#define GP_IRQ     0
#define GP_TOUCH   1
#define GP_ENC_A   2
#define GP_ENC_B   3

// I2C addresses
#define I2C_MCP    0x20

// registers of MCP23X17
#define IODIRA     0
#define IPIOLA     2
#define GPINTENA   4
#define DEFVALA    6
#define INTCONA    8
#define IOCONA    10
#define GPPUA     12
#define INTFA     14
#define INTCAPA   16
#define GPIOA     18
#define OLATA     20

/* constants */
#define RF_GAIN_SHIFT    8                       //rf gain scaled by 256
#define FFT_GAIN_SHIFT   8
#define AGC_GAIN_SHIFT   6                        //agc gain 1..64

/* State definitions, LSB indicates screen touchdown */
#define S_TUNE		     0
#define S_SQL          2
#define S_SET          11
#define S_CAL          111
#define S_BAND         4
#define S_MODE         6
#define S_REFLVL       8
#define S_TIME         10
#define S_IQCAL        12
#define S_FUNC         14
#define S_RFLVL        16

/* Sub state definitions */
#define SS_INIT		      0
#define SS_SHOW         1
#define SS_WAIT         2
#define SS_TOUCHDOWN    3

/* Event definitions */
#define EV_NOEVENT		  0
#define EV_INCREMENT		1
#define EV_DECREMENT		2
#define EV_IO           3
#define EV_TOUCHDOWN    4
#define EV_TOUCHUP      5
#define EV_CAT          6

// modes
#define MODE_USB  0
#define MODE_LSB  1
#define MODE_AM   2
#define MODE_CW   3
#define MODE_FM   4
#define MODE_RTTY 5

//AGC
#define OFF   0
#define SLOW  1
#define FAST  2

//display_mode
#define FFT   0
#define IQ    1
#define AU    2
#define TXT   3

//display layout
#define RXTXPOSX          2
#define RXTXPOSY        250
#define TIMEPOSX        390
#define TIMEPOSY        250  
#define MODEPOSX          0
#define MODEPOSY        205
#define FREQPOSY        170                   // 50 high
#define STEPPOSY        FREQPOSY+50
#define BANDMENUX        90
#define BANDMENUY        48 
#define MODEMENUX        10
#define MODEMENUY        10 
#define FUNCMENUX        5
#define FUNCMENUY        5 
#define OKMENUX          140
#define OKMENUY          20
#define CWLINEX         240
#define CWLINEY         143                   // 144 - 160
#define SMETERX          20
#define SMETERY         145
#define FFTY            100
#define FFTH             40                   // 100..61
#define MAINH            48                   // main menu 0..48
#define SPANY            48
#define TEXTY           122

// prototypes
void init_ui();
void setMode(int mode);
void ui_irq(uint gpio, uint32_t events);
void ui_handler();
void ui_loop();
void setAgc(uint8_t agc);

void displaySpan();
void setupSmeter();
void displaySmeter();
void displayFreq();
void displayMain();
void displayMode();
void displayStep();
void displayTime();

#ifdef __cplusplus
}
#endif


