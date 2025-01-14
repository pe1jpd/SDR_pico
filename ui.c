#ifdef __cplusplus
 extern "C" {
#endif

#include "Arduino.h"
#include "pwm.h"
#include "adc.h"
#include "irq.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "pico/multicore.h"
#include "hardware/structs/bus_ctrl.h"

#include "tft.h"
#include "si5351.h"
#include "ui.h" 
#include "touch.h"
#include "ds3231.h"
#include "cw.h"

#define GP_MASK_UI  ((1<<GP_ENC_A)|(1<<GP_ENC_B)|(1<<GP_IRQ)|(1<<GP_TOUCH))
#define GPIO_IRQ_EDGE_ALL	  (GPIO_IRQ_EDGE_FALL|GPIO_IRQ_EDGE_RISE)

#define CHECKBYTE 0xde

// main menu
// main menu
#define SET  0
#define SPAN 1
#define FUNC 2
#define REF  3
#define BAND 4
#define MODE 5
#define TIME 6
#define RXTX 7

//settings menu
#define CAL  0

uint8_t state = S_TUNE;
uint8_t subState = SS_INIT;

uint8_t band, lastBand = -1;
extern uint8_t span, lastSpan = -1;
uint32_t freq, freqUpdate, lastFreq = -1;
extern uint8_t mode;
uint8_t lastMode = -1;
uint8_t step, lastStep = -1;
int inc = 0;
uint16_t x, y;
uint16_t xHoldL=480;
uint16_t xHoldH=0;
extern bool ptt;
bool lastPtt = true;
extern bool hold;
bool Smeter_init;
bool tuningOn= false;
volatile uint32_t updateTime = 0;
volatile uint32_t freqUpdateTime;
volatile uint32_t clockUpdateTime = 0;
volatile uint32_t tuningUpdateTime = 0;
bool calibrated = false;
int freqPosition, stepPosition, freqLen, lastFreqLen;
char freqString[12], lastFreqString[12];
char timeString[8];
uint8_t io;
uint8_t attOn = 0, decodeOn = 0;
uint8_t agc, lastAgc=-1;
int8_t fft_lvl, rf_lvl;

extern uint16_t rf_gain;
extern uint16_t fft_gain;
extern int8_t sql_val;
extern volatile uint32_t msec;
int smode = 0;                                            // 0: bar, 1: number
uint16_t display_mode=0;                                  // 0:FFT/waterfall, 1:IQ, 2:Audio

uint16_t fmark, fspace;
double baud;
int rtty_shift = 0;
int rtty_shift_val[3] = {170, 425, 850};
int rtty_baud = 0;
double rtty_baud_val[3] = {45.45, 50.0, 75.0};
#define MAXCHARS 60
#define MAXSCREEN 6
#define MAXFILE   64
char text[MAXCHARS][MAXFILE];
int xpos=0, ypos=0;             // position in file
int yscr=0;                     // line on screen

#define MAXMAIN 6
BUTTON mainMenu[MAXMAIN] = { 
  {"SET",    0, 94,2,37, BLACK, GREY, WHITE},
  {"SPAN",  96,190,2,37, BLACK, GREY, WHITE},
  {"FUNC", 192,286,2,37, BLACK, GREY, WHITE},
  {"REF",  288,382,2,37, BLACK, GREY, WHITE},
  {"BAND", 384,478,2,37, BLACK, GREY, WHITE},
  {"", MODEPOSX, MODEPOSX+80, MODEPOSY, MODEPOSY+37, BLUE, GREY, WHITE},          // mode button
};

#define MAXBAND 9
BUTTON bandMenu[MAXBAND] = {
  {"3.5", BANDMENUX,     BANDMENUX+95,  BANDMENUY+135, BANDMENUY+175, DARKGREY, GREY, WHITE},
  {"7",   BANDMENUX+100, BANDMENUX+195, BANDMENUY+135, BANDMENUY+175, DARKGREY, GREY, WHITE},
  {"10",  BANDMENUX+200, BANDMENUX+295, BANDMENUY+135, BANDMENUY+175, DARKGREY, GREY, WHITE},
  {"14",  BANDMENUX,     BANDMENUX+95,  BANDMENUY+90, BANDMENUY+130, DARKGREY, GREY, WHITE},
  {"18",  BANDMENUX+100, BANDMENUX+195, BANDMENUY+90, BANDMENUY+130, DARKGREY, GREY, WHITE},
  {"21",  BANDMENUX+200, BANDMENUX+295, BANDMENUY+90, BANDMENUY+130, DARKGREY, GREY, WHITE},
  {"24",  BANDMENUX,     BANDMENUX+95,  BANDMENUY+45, BANDMENUY+85, DARKGREY, GREY, WHITE},
  {"28",  BANDMENUX+100, BANDMENUX+195, BANDMENUY+45, BANDMENUY+85, DARKGREY, GREY, WHITE},
  {"144", BANDMENUX+200, BANDMENUX+295, BANDMENUY+45, BANDMENUY+85, DARKGREY, GREY, WHITE},
//  {"", BANDMENUX,     BANDMENUX+95,  BANDMENUY,  BANDMENUY+40, DARKGREY, GREY, WHITE},
//  {"  ",  BANDMENUX+100, BANDMENUX+195, BANDMENUY,  BANDMENUY+40, DARKGREY, GREY, WHITE},
//  {"  ",  BANDMENUX+200, BANDMENUX+295, BANDMENUY,  BANDMENUY+40, DARKGREY, GREY, WHITE},
};

#define MAXFUNC 8
BUTTON funcMenu[MAXFUNC] = {
  {"ATT",    5,  95, 150, 230, DARKGREY,GREY,WHITE},
  {"Meter",100, 190, 150, 230, DARKGREY,GREY,WHITE},
  {"Displ",195, 285, 150, 230, DARKGREY,GREY,WHITE},
  {"AGC",  290, 380, 150, 230, DARKGREY,GREY,WHITE},
  {"Shift",  5,  95,  65, 145, DARKGREY,GREY,WHITE},
  {"Baud", 100, 190,  65, 145, DARKGREY,GREY,WHITE},
  {"Decod",195, 285,  65, 145, DARKGREY,GREY,WHITE},
  {"Back", 385, 475,   5,  40, DARKGREY,GREY,WHITE},
};

#define MAXMODE 6
BUTTON modeMenu[MAXMODE] = {
  {"USB", MODEMENUX,     MODEMENUX+90,  MODEMENUY+50,  MODEMENUY+95,  DARKGREY, GREY, WHITE},
  {"LSB", MODEMENUX+100, MODEMENUX+190, MODEMENUY+50,  MODEMENUY+95,  DARKGREY, GREY, WHITE},
  {"AM",  MODEMENUX+200, MODEMENUX+290, MODEMENUY+50,  MODEMENUY+95,  DARKGREY, GREY, WHITE},
  {"CW",  MODEMENUX,     MODEMENUX+90,  MODEMENUY,     MODEMENUY+45,  DARKGREY, GREY, WHITE},
  {"FM",  MODEMENUX+100, MODEMENUX+190, MODEMENUY,     MODEMENUY+45,  DARKGREY, GREY, WHITE},
  {"RTTY", MODEMENUX+200, MODEMENUX+290, MODEMENUY,     MODEMENUY+45,  DARKGREY, GREY, WHITE}
};

#define MAXSETTINGS 2
BUTTON settingsMenu[MAXSETTINGS] = {
  {"CAL", 96, 190, 157, 192, BLACK},
  {"TIME", 192, 286, 157, 192, BLACK}
};

#define MAXOK 2
BUTTON okMenu[MAXOK] = {
  {"OK",     OKMENUX,     OKMENUX+90,  OKMENUY, OKMENUY+35, DARKGREY, GREY, WHITE},
  {"Cancel", OKMENUX+200, OKMENUX+190, OKMENUY, OKMENUY+35, DARKGREY, GREY, WHITE}
};

#define MAXKEYB 12
#define KEYBX  25
#define KEYBY  10
#define KEYBWX 60
#define KEYBWY 40
BUTTON keyboard[MAXKEYB] = {
  {"0",   KEYBX, KEYBX+KEYBWX,  KEYBY+3*KEYBWY+15,  KEYBY+4*KEYBWY+15, DARKGREY, GREY, WHITE},
  {"1",   KEYBX, KEYBX+KEYBWX,  KEYBY+2*KEYBWY+10,  KEYBY+3*KEYBWY+10, DARKGREY, GREY, WHITE},
  {"2",   KEYBX+KEYBWX+5, KEYBX+2*KEYBWX+5,  KEYBY+2*KEYBWY+10,  KEYBY+3*KEYBWY+10, DARKGREY, GREY, WHITE},
  {"3",   KEYBX+2*KEYBWX+10, KEYBX+3*KEYBWX+10,  KEYBY+2*KEYBWY+10,  KEYBY+3*KEYBWY+10, DARKGREY, GREY, WHITE},
  {"4",   KEYBX, KEYBX+KEYBWX,  KEYBY+KEYBWY+5,  KEYBY+2*KEYBWY+5, DARKGREY, GREY, WHITE},
  {"5",   KEYBX+KEYBWX+5, KEYBX+2*KEYBWX+5,  KEYBY+KEYBWY+5,  KEYBY+2*KEYBWY+5, DARKGREY, GREY, WHITE},
  {"6",   KEYBX+2*KEYBWX+10, KEYBX+3*KEYBWX+10,  KEYBY+KEYBWY+5,  KEYBY+2*KEYBWY+5, DARKGREY, GREY, WHITE},
  {"7",   KEYBX, KEYBX+KEYBWX,  KEYBY,  KEYBY+KEYBWY, DARKGREY, GREY, WHITE},
  {"8",   KEYBX+KEYBWX+5, KEYBX+2*KEYBWX+5,  KEYBY,  KEYBY+KEYBWY, DARKGREY, GREY, WHITE},
  {"9",   KEYBX+2*KEYBWX+10, KEYBX+3*KEYBWX+10,  KEYBY,  KEYBY+KEYBWY, DARKGREY, GREY, WHITE},
  {"BS",   KEYBX+KEYBWX+5, KEYBX+2*KEYBWX+5,  KEYBY+3*KEYBWY+15,  KEYBY+4*KEYBWY+15, DARKGREY, GREY, WHITE},
  {"ENT", KEYBX+2*KEYBWX+10, KEYBX+3*KEYBWX+10,  KEYBY+3*KEYBWY+15,  KEYBY+4*KEYBWY+15, DARKGREY, GREY, WHITE},
};

int stepSize[] = {10, 100, 1000, 10000, 100000};
int stepSizeFM[] = {10, 100, 12500, 25000, 100000};

int bandFreq[MAXBAND][3] = {
  {3500000, 3800000, 0},
  {7000000, 7200000, 0},
  {10000000, 10200000, 0},
  {14000000, 14300000, 0},
  {18000000, 18100000, 0},
  {21000000, 21500000, 0},
  {24000000, 24100000, 0},
  {28000000, 30000000, 0},
  {28000000, 32000000, 116},                // 144-148 MHz
};

char *modeName[] = {"USB", "LSB", "AM", "CW", "FM", "RTTY"};
char *agcName[] = {"OFF ", "SLOW", "FAST"};

// lin = sqrt(50/1000)*10^(dBm/20)
int16_t logtbl[] = {
  25,27,28,30,32,34,36,38,40,42,45,48,51,54,57,60,64,68,72,76,80,85,90,96,101,107,114,121,128,135,143,152,161,171,181,191,203,215,228,241,
  256,271,287,304,322,341,361,383,405,429,455,482,510,541,573,607,643,681,721,764,809,857,908,962,1019,1079,1143,1211,1283,1359,1439,1524,1615,1710,1812,1919,2033,2153,2281,2416,
  2560
};

long map(long x, long in_min, long in_max, long out_min, long out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void init_ui()
{
  // init input pins
	gpio_init_mask(GP_MASK_UI);
	
  // set internal pullup
  i2c_io_write(I2C_IO, 0xff);
  i2c_io_read(I2C_IO);
  i2c_io_write(I2C_RELAY, 0);

	// Enable pull-ups
	gpio_pull_up(GP_ENC_A);
	gpio_pull_up(GP_ENC_B);
	gpio_pull_up(GP_TOUCH);
	gpio_pull_up(GP_IRQ);

	gpio_set_irq_enabled(GP_ENC_A, GPIO_IRQ_EDGE_ALL, true);
	gpio_set_irq_enabled(GP_TOUCH, GPIO_IRQ_EDGE_ALL, true);
	gpio_set_irq_enabled(GP_IRQ, GPIO_IRQ_EDGE_FALL, true);

	gpio_set_irq_enabled_with_callback(GP_ENC_A, GPIO_IRQ_EDGE_FALL, true, ui_irq);
	gpio_set_irq_enabled_with_callback(GP_TOUCH, GPIO_IRQ_EDGE_ALL, true, ui_irq);
	gpio_set_irq_enabled_with_callback(GP_IRQ, GPIO_IRQ_EDGE_FALL, true, ui_irq);

/*
  if (!calibrated) {
    calibrateTouch();
    calibrated = true;
  }
*/

  uint8_t check = 0x00;
  band = 2;
  freq = 7000000;
  mode = MODE_LSB;
  span = 16;
  step = 2;
  fft_lvl = 20; // 10dB
  rf_lvl = 0;
  sql_val = 0;

  i2c_eeprom_read(0, &check, 1);
  if (check == CHECKBYTE) {
    i2c_eeprom_read(1, &band, 1);
    i2c_eeprom_read(2, &span, 1);
    i2c_eeprom_read(3, &step, 1);
    i2c_eeprom_read(4, &fft_lvl, 1);
    i2c_eeprom_read(5, &sql_val, 1);
    i2c_eeprom_read(6, &rf_lvl, 1);
    i2c_eeprom_read(7, &agc, 1);
    i2c_eeprom_read(8, &rtty_shift, 1);
    i2c_eeprom_read(9, &rtty_baud, 1);
  }
  else {
    check = CHECKBYTE;                                     // otherwise, save defaults in eeprom
    i2c_eeprom_write(0, &check, 1);
    delay(100);
    i2c_eeprom_write(1, &band, 1);
    delay(100);
    i2c_eeprom_write(2, &span, 1);
    delay(100);
    i2c_eeprom_write(3, &step, 1);
    delay(100);
    i2c_eeprom_write(4, &fft_lvl, 1);
    delay(100);
    i2c_eeprom_write(5, &sql_val, 1);
    delay(100);
    i2c_eeprom_write(6, &rf_lvl, 1);
    delay(100);
    i2c_eeprom_write(7, &agc, 1);
    delay(100);
    i2c_eeprom_write(8, &rtty_shift, 1);
    delay(100);
    i2c_eeprom_write(9, &rtty_baud, 1);
    delay(100);

    for (int i=0; i<MAXBAND; i++) {
      i2c_eeprom_write(16+16*i, &bandFreq[i][0], 4);     // save lower bandlimt of freq of each band
      delay(20);
      uint8_t tmp = 0;
      i2c_eeprom_write(20+16*i, &tmp, 1);                 // and default mode LSB (0)
      delay(20);
    }

  }

  i2c_eeprom_read(16+band*16, &freq, 4);                  // read freq and mode
  i2c_eeprom_read(20+band*16, &mode, 1);
  i2c_eeprom_read(21+band*16, &attOn, 1);

  rf_gain = logtbl[40+rf_lvl];                            // 0..+20dB step 0.5
  fft_gain = logtbl[40+fft_lvl];
  
  ptt = false;

  fmark = 1000;
  fspace = fmark + rtty_shift_val[rtty_shift];
  baud = rtty_baud_val[rtty_baud];
  calcFilter(fmark, fspace , baud);  // init rtty filters

  clearText();

  displayMain();
  displaySpan();
  displayMode();

  setBand(band);
  setMode(mode);
  setAgc(agc);
}

uint8_t event = EV_NOEVENT;
void ui_irq(uint gpio, uint32_t events) 
{
  uint8_t byte;
  char s[8];

	switch (gpio) {
	  case GP_ENC_A:
		if (events & GPIO_IRQ_EDGE_FALL) {
    	if (gpio_get(GP_ENC_B))
        inc++;
      else
        inc--;
    } 
    break;

	  case GP_TOUCH:
		if (events & GPIO_IRQ_EDGE_ALL) {
  		if (gpio_get(GP_TOUCH))
        event = EV_TOUCHUP;
      else
        event = EV_TOUCHDOWN;
    }
    break;

	  case GP_IRQ:
    event = EV_IO;
    break;

    default:
    break;
  }
}

int button = -1;
void buttonPress(BUTTON *buttonSet, int nSet)
{
  x = y = 0;
  getTouch(&x, &y);

  for (int i=0; i<nSet; i++) {
    if (x > buttonSet[i].x0 && x < buttonSet[i].x1 && y > buttonSet[i].y0 && y < buttonSet[i].y1) {
      if (buttonSet[i].press == false) {
        buttonSet[i].press = true;
        drawButton(&buttonSet[i]);
        button = i;
      }
    }
    else {
      if (buttonSet[i].press == true) {
        buttonSet[i].press = false;
        drawButton(&buttonSet[i]);
      }
    }
  }
}

uint8_t lastio = 0xff;
extern int noise_lvl;

void ui_handler()
{
  char s[24];

  displayRxtx();
  displayAgc();
  displayTuningRTTY();
  switch (mode) {
    case MODE_RTTY:
      if (display_mode == TXT) displayDecodedText();
      break;
    case MODE_CW:
      displayDecodedCW();
      break;
  }
  displaySmeter();
  displayTime();
/*
  // for debug puposes:
  setFont(1);
  sprintf(s, "%d  ", noise_lvl);
  drawString(s, CWLINEX, CWLINEY, 1, YELLOW, BLACK);
*/
  switch (state) {
    /*
     *  S_Tune is main loop 
     */

    case S_TUNE:
    // read tuning and update frequency
    if (inc>0) while (inc>0) {
      if (mode == MODE_FM) {
        freq += stepSizeFM[step];
        if (step==2 || step==3)
        // 12.5 or 25kHz in FM
          if (freq%stepSizeFM[step] != 0) freq = stepSizeFM[step]*(freq/stepSizeFM[step]);
      }
      else {
        freq += stepSize[step];
      }
      inc--;
    }
    else if (inc<0) while (inc<0) {
      if (mode == MODE_FM) {
        freq -= stepSizeFM[step];
        if (step==2 || step==3)
          if (freq%stepSizeFM[step] != 0) freq = stepSizeFM[step]*(freq/stepSizeFM[step]);
      }
      else {
        freq -= stepSize[step];
      }
      inc++;
    }

    if (freq != lastFreq) {
      SI_SETFREQ(0, freq);                                 // update si5351
      SI_SETPHASE(0, 1);
      si_evaluate(); 

      displayFreq();
      lastFreq = freq;
    }

    if (freq != freqUpdate && msec > freqUpdateTime + 2000) {
      // save new freq if left unchanged for 2 sec
      i2c_eeprom_write(16+band*16, &freq, 4); 
      freqUpdate = freq;
      freqUpdateTime = msec;
    }

    if (step != lastStep) {
      displayStep();
      i2c_eeprom_write(3, &step, 4);
      lastStep = step;
    }

    if (event == EV_IO) {
      // io event via pcf8574
      io = i2c_io_read(I2C_IO);
      if (io != lastio) {
        ptt = false;
        if (~io&1) {
          // bit0=ptt pulled low
          ptt = true;
        }

        if (~io&2) {
          // bit 1=rotary press pulled low
          fillRect(0, 0, 140, 60, LIGHTBLUE);
          drawRoundRect(0, 0, 140, 60, DARKGREY);
          setFont(1);
          drawString("SQL", 20, 40, 1, LIGHTGREY, LIGHTBLUE);
          state = S_SQL;
          updateTime = msec;
        }
        lastio=io;
      }
    }
    
    if (event == EV_TOUCHDOWN) { 
      getTouch(&x, &y);                           // get coorinates of touch

      if (y>FFTY-30 && y<FFTY+52) {               // fft area touched?
        int df = map(x, 0, 480, -span/2, span/2); // calculate new corresponding frequency
        if (abs(df) <= span/2) {
   	      freq += df*1000;
        }
      }
      else {
        subState = SS_TOUCHDOWN;                  // handle main menu in state machine
      }
    }

    if (subState == SS_TOUCHDOWN) {

      getTouch(&x, &y);
      if (y>FREQPOSY && y<FREQPOSY+50) {          // frequeny (step or band) touched?
        int xt = freqPosition;                    // end of string .xxx.xxp[-=]
        for (int i=0; i<5; i++) {                 // lower 5 digits?
          if (i==2) xt -= 8;                      // skip the dec point
          if (x>xt-32 && x<xt) {
            step = i;                             // if so, set step
          }
          xt -= 32;                               // otherwise, next digit
        }
        // MHz (aka Band) touched?
        if (event == EV_TOUCHUP) {
          if (x> xt-8+64 && x<xt-8) {
            state = S_BAND;
            subState = SS_INIT;
          }
        }
        break;
      }

      // handle main menu
      buttonPress(mainMenu, MAXMAIN);               // read button pressed

      if (event == EV_TOUCHUP) {
//        mainMenu[button].press = false;
        switch (button) {                           // handle button
        case SET:
          buttonPress(mainMenu, MAXMAIN);
//          state = S_SET;                          // not used yet
          break;
        case SPAN:                                  // toggle span 16 or 160kHz
          if (span==160)
            span = 16;
          else
            span = 160;
//          fft_gain = logtbl[40+fft_lvl];
          i2c_eeprom_write(2, &span, 1);
          for (int i=0; i<FFTH; i++) {              // clean up BPF lines in fft
            drawHLine(0, FFTY, 480, BLACK);
          }
          displayMain();                            // draw main menu
          displaySpan();                            // update new BPF lines
          break;
        case BAND:
          state = S_BAND;
          hold = true;                              // stop screen fft/decodes
          break;
        case MODE:
          state = S_MODE;
          break;
        case REF:
          if (display_mode == FFT) 
            state = S_REFLVL;                       // reflevel only useful in fft mode
          break;
        case FUNC:
          state = S_FUNC;
          hold = true;
          break;
        }
        button = -1;
        tuningOn = false;                           // update tuning indicator (if present)
        subState = SS_INIT;
      }
    }
    break;

   /*
     *  S_SQL to set squelch
     */

    case S_SQL:
    displaySquelch();
  
    io = i2c_io_read(I2C_IO);
    if (io != lastio) {
      if (~io&2) {                                          // rotary press again?
        drawString("RF GAIN", 5, 40, 1, LIGHTGREY, LIGHTBLUE);
        state = S_RFLVL;                                    // handle RF Gain
        updateTime = msec;
      }
      lastio = io;
    }
    if (msec > updateTime + 2000) {                         // after 2sec wait update eeprom
      i2c_eeprom_write(5, &sql_val, 1);
      state = S_TUNE;                                       // and back to tuning mode
    }
    if (state == S_TUNE) {                                  // if back to tuning, first clean up
      fillRect(0, 0, 140, 60, BLACK);                       // clean SQL/RF Gain screen area
      displayMain();
      displaySpan();
    }
    else {
      if (inc>0) while (inc>0) {
        if (++sql_val>80) sql_val = 80;
        inc--;
        updateTime = msec;
      }
      else if (inc<0) while (inc<0) {
        if (--sql_val<0) sql_val=0;
        inc++;
        updateTime = msec;
      }
    }
    break;

    case S_RFLVL:
    displayRfgain();
    if (io != lastio) {
      if (~io&2) {                                            // rotary pressed again?
        state = S_TUNE;                                       // do nothing
      }
      lastio = io;
    }
    if (msec > updateTime + 2000) {                           // if 2 sec passed
      i2c_eeprom_write(6, &rf_lvl, 1);                        // update ne RF gain
      state = S_TUNE;
    }
    if (state == S_TUNE) {
      fillRect(0, 0, 140, 60, BLACK);
      displayMain();
      displaySpan();
    }
    else {
      if (inc>0) while (inc>0) {
        if (++rf_lvl>40) rf_lvl = 40;
        inc--;
        updateTime = msec;
      }
      else if (inc<0) while (inc<0) {
        if (--rf_lvl<0) rf_lvl = 0;
        inc++;
        updateTime = msec;
      }
      rf_gain = logtbl[40+rf_lvl];                          // 0..+20dB step 0.5
    }
    break;

    /*
     *  S_REFLVL to set fft_gain
     */

    case S_REFLVL:
    if (subState == SS_INIT) {
      xHoldL = 250;                                       // prevent waterfall in REF area
      xHoldH = 390;
      fillRect(250, 38, 390, 98, LIGHTBLUE);              // and draw REF area
      drawRoundRect(250, 38, 390, 98, DARKGREY);
      setFont(1);
      drawString("REF", 270, 78, 1, LIGHTGREY, LIGHTBLUE);
      displayRef();
      subState = SS_WAIT;
    }
    if (subState == SS_WAIT) {
      displayRef();
      if (inc>0) while (inc>0) {
        if (++fft_lvl>40) fft_lvl = 40;                   // Pdbm = 10*log(Vrms^2*1000/50)--> Vrms=sqrt(50/1000)*10^(Pdbm/20)
        inc--;                                            // fft_lvl count 0.5dB so -20..20dB
        updateTime = msec;
      }
      else if (inc<0) while (inc<0) {
        if (--fft_lvl<-40) fft_lvl = -40;
        inc++;
        updateTime = msec;
      }
      fft_gain = logtbl[40+fft_lvl];                      // -20..+20dB step 0.5 with reflfl -40..+40
    }
    if (event == EV_TOUCHDOWN) {
      subState = SS_TOUCHDOWN;
    }
    if (subState == SS_TOUCHDOWN) {
      buttonPress(mainMenu, MAXMAIN);                     // read button pressed
      if (event == EV_TOUCHUP) {
        funcMenu[button].press = false;
        if (button==3) {                                  // if REF pressed save
          i2c_eeprom_write(4, &fft_lvl, 1);
        }
        fillRect(250,38, 390, 98, BLACK);
        xHoldL = 480;
        xHoldH = 0;
        displayMain();
        displaySpan();
        state = S_TUNE;
      }
    }
    break;
 
    /*
     *  S_FUNC to access functions
     */

    case S_FUNC:
    if (subState == SS_INIT) {
      hold = true;
      Smeter_init = false;
      fillRect(0, 0, 479, RXTXPOSY-5, BLACK);
      drawRoundRect(0, 0, 479, RXTXPOSY-5, DARKGREY);
      subState = SS_SHOW;
    }
    if (subState == SS_SHOW) {
      for (int i=0; i<MAXFUNC; i++) {
        drawButton(&funcMenu[i]);
        *s = 0;
        switch (i) {
          case 0: sprintf(s, "%s",  (attOn==0) ? "OFF" : "20dB"); break;
          case 1: sprintf(s, "%s", (smode==0) ? "Bar" : "dB "); break;
          case 2: sprintf(s, "%s", (display_mode==FFT) ? " FFT" : (display_mode==IQ) ? " IQ " : "Audio"); break;
          case 3: sprintf(s, "%s", agcName[agc]); break;
          case 4: sprintf(s, "%d", rtty_shift_val[rtty_shift]); break;
          case 5: sprintf(s, "%g", rtty_baud_val[rtty_baud]); break;
          case 6: sprintf(s, "%s", (decodeOn) ? "On" : "Off"); break;
        }
        setFont(1);
        drawString(s, funcMenu[i].x0+7, funcMenu[i].y0+20, 1, LIGHTGREY, DARKGREY);
      }
      subState = SS_WAIT;
    }
    // wait for touchdown
    if (subState == SS_WAIT) {                                // not nessecary
      if (event == EV_TOUCHDOWN) {
        subState = SS_TOUCHDOWN;
      }
    }
    if (subState == SS_TOUCHDOWN) {
      buttonPress(funcMenu, MAXFUNC);                         // button in Func menu pressed
      if (event == EV_TOUCHUP) {
        funcMenu[button].press = false;
        setFont(1);
        switch (button) {
        case 0:
          if (++attOn == 2) attOn = 0;
          i2c_eeprom_write(21+band*16, &attOn, 1);
          subState = SS_SHOW;
          break;
        case 1:
          smode = !smode;
          subState = SS_SHOW;
          break;
        case 2:
          if (++display_mode == 3) display_mode = 0;           // toggle display mode
          subState = SS_SHOW;
          break;
        case 3:
          if (++agc>2) agc = 0;
          setAgc(agc);
          i2c_eeprom_write(7, &agc, 1);
          subState = SS_SHOW;
          break;
        case 4:
          if (++rtty_shift > 2) rtty_shift = 0;
          i2c_eeprom_write(8, &rtty_shift, 1);
          fmark = 1000;
          fspace = fmark + rtty_shift_val[rtty_shift];
          calcFilter(fmark, fspace , baud);                       // init rtty filters
          clearText();
          subState = SS_SHOW;
          break;
        case 5:
          if (++rtty_baud > 2) rtty_baud = 0;
          i2c_eeprom_write(9, &rtty_baud, 1);
          baud = rtty_baud_val[rtty_baud];
          calcFilter(fmark, fspace , baud);                       // init rtty filters
          subState = SS_SHOW;
          break;
        case 6:
          if (decodeOn) decodeOn = 0;
          else decodeOn = 1;
          subState = SS_SHOW;
          break;
        case MAXFUNC-1:                                           // Back
          fillRect(0, FUNCMENUY-10, 479, RXTXPOSY-5, BLACK);
          displayMain();
          displaySpan();
          displayMode();
          displayFreq();
          displayStep();
          hold = false;
          state = S_TUNE;
          break;
        default:                                                  // no button pressed
          break;
        }
        button = -1;
      }
    }
    break;

    /*
     *  S_Band to set band
     */

    case S_BAND:
    if (subState == SS_INIT) {
      Smeter_init = false;
      fillRect(BANDMENUX-5, BANDMENUY+40, BANDMENUX+300, BANDMENUY+180, BLACK);
      drawRoundRect(BANDMENUX-5, BANDMENUY+40, BANDMENUX+300, BANDMENUY+180, DARKGREY);
      for (int i=0; i<MAXBAND; i++) {
        drawButton(&bandMenu[i]);
      }
      subState = SS_WAIT;
    }
    // wait for touchdown
    if (event == EV_TOUCHDOWN) {
      subState = SS_TOUCHDOWN;
    }
    if (subState == SS_TOUCHDOWN) {
      buttonPress(bandMenu, MAXBAND);                         // read button pressed
      if (event == EV_TOUCHUP) {
        bandMenu[button].press = false;
        if (button>=0) {
          lastBand = band;
          band = button;
          setBand(band);
          // check bounderies
          if (freq < bandFreq[lastBand][0] || freq > bandFreq[lastBand][1]) {
            freq = bandFreq[lastBand][0];                     // if out ouf band save default freq
          }
          i2c_eeprom_write(16+lastBand*16, &freq, 4);         // save freq former band
          delay(20);
          i2c_eeprom_write(1, &band, 1);                      // now save new band
          delay(20);
          i2c_eeprom_read(16+band*16, &freq, 4);              // and get freq of new band
          freqUpdate = freq;                                  // no need to update eeprom
          i2c_eeprom_read(20+band*16, &mode, 1);              // and get mode
          i2c_eeprom_read(21+band*16, &attOn, 1);             // attenuator setting
        }
        fillRect(BANDMENUX-5, BANDMENUY, BANDMENUX+300, BANDMENUY+180, BLACK);
//        displayMain();
        displaySpan();
        displayMode();
        displayFreq();
        displayStep();
        displayAgc();
//      set_bias();
        hold = false;
        state = S_TUNE;
        button = -1;
        subState = SS_INIT;
      }
    }
    break;
    
    /*
     *  S_MODE to set mode
     */

    case S_MODE:
    if (subState == SS_INIT) {
      hold = true;
      button = -1;
//      fillRect(0, MODEMENUY-10, 479, MODEMENUY+100, BLACK);
      fillRect(0, 0, 479, FFTY+30, BLACK);
      drawRoundRect(0, MODEMENUY-10, 479, MODEMENUY+100, DARKGREY);
      for (int i=0; i<MAXMODE; i++) {
        drawButton(&modeMenu[i]);
      }
      subState = SS_WAIT;
    }
    if (subState == SS_WAIT){
      if (event == EV_TOUCHDOWN) {
        subState = SS_TOUCHDOWN;
      }
    }
    if (subState == SS_TOUCHDOWN) {
      buttonPress(modeMenu, MAXMODE);                           // read button pressed
      if (event == EV_TOUCHUP) {
        if (button>=0) {
          modeMenu[button].press = false;
          lastMode = mode;
          mode = button;
          i2c_eeprom_write(20+band*16, &mode, 1);               // save mode for current band
          setMode(mode);                                        // set DSP filter
        }
        fillRect(0, 0, 479, FFTY+30, BLACK);                    // clean up area
        if (lastMode == MODE_CW) {                              // if last mode was CW
          for (int i=0; i<12; i++)
            drawHLine(CWLINEX, CWLINEY+i, 480-CWLINEX, BLACK);  // clean up CW text
        }
        if (mode == MODE_RTTY) {
            calcFilter(fmark, fspace , baud);                   // calculate rtty filters based on settings
        }
        displayMain();
        displaySpan();
        displayMode();
        hold = false;
        state = S_TUNE;
      }
    }
    break;

    /*
     *  S_SET to handle settings
     */

    case S_SET:
    if (subState == SS_INIT) {
      hold = true;
      fillRect(0, 0, 479, 271, BLACK);
      drawRoundRect(0, 0, 479, 271, DARKGREY);
      subState = SS_WAIT;
      button = -1;
    }
    if (event == EV_TOUCHDOWN) {
      subState = SS_TOUCHDOWN;
    }
    if (subState == SS_TOUCHDOWN) {
      buttonPress(settingsMenu, MAXSETTINGS);
      switch (button) {
        case CAL:
        state = S_CAL;
        subState = SS_INIT;
        break;
        state = S_TUNE;
        break;
      }
    }
    break;
  }
  event = EV_NOEVENT;
}

void setBand(band) {
  uint8_t sw;
  switch (band) {
    case 0: sw=0x02; break;     // 3.5
    case 1:                     // 7
    case 2: sw=0x04; break;     // 10
    case 3:                     // 14
    case 4:                     // 18
    case 5: sw=0x08; break;     // 21
    case 6:                     // 24
    case 7: sw=0x10; break;     // 28
    case 8: sw=0x50; break;     // 144 equals 28MHz plus relay to trv
  }
  i2c_io_write(I2C_RELAY, sw);
}

void displayRxtx()
{
  if (ptt != lastPtt) {
    setFont(1);
    if (ptt) {
      drawString("TX", RXTXPOSX, RXTXPOSY, 1, RED, BLACK);
      drawRoundRect(RXTXPOSX-2, RXTXPOSY-2, RXTXPOSX+34, RXTXPOSY+18, RED);
    }
    else {
      drawString("RX", RXTXPOSX, RXTXPOSY, 1, GREEN, BLACK);
      drawRoundRect(RXTXPOSX-2, RXTXPOSY-2, RXTXPOSX+34, RXTXPOSY+18, GREEN);
    }
    lastPtt = ptt;
  }
}

void displayAgc()
{
  int c;
  if (agc != lastAgc) {
    switch (agc) {
      case OFF:  c = GREY;  break;
      case SLOW: c = GREEN; break;
      case FAST: c = RED;   break;
    }
    drawString(agcName[agc], 300, RXTXPOSY, 1, c, BLACK);
  }
  lastAgc = agc;
}

void displaySquelch()
{
  char s[8];

  fillRect(20, 25, 20+sql_val, 33, YELLOW);
  fillRect(21+sql_val, 25, 100, 33, BLACK);
  drawHLine(20, 22, 80, DARKGREY);
  drawHLine(20, 36, 80, DARKGREY);
  setFont(1);
  sprintf(s, "%d ", sql_val);
  drawString(s, 20, 8, 1, LIGHTGREY, LIGHTBLUE);
}

void displayRfgain()
{
  char s[8];

  fillRect(20, 25, 20+2*rf_lvl, 33, YELLOW);
  fillRect(21+2*rf_lvl, 25, 100, 33, BLACK);
  drawHLine(20, 22, 80, DARKGREY);
  drawHLine(20, 36, 80, DARKGREY);
  setFont(1);
  sprintf(s, "%0.1f ", (double)rf_lvl/2);
  drawString(s, 20, 8, 1, LIGHTGREY, LIGHTBLUE);
}

void displayRef()
{
  char s[8];

  fillRect(270, 63, 310+fft_lvl, 71, YELLOW);
  fillRect(311+fft_lvl, 63, 350, 71, BLACK);
  drawHLine(270, 60, 80, DARKGREY);
  drawHLine(270, 74, 80, DARKGREY);
  setFont(1);
  sprintf(s, "%0.1f ", (double)fft_lvl/2);
  drawString(s, 270, 46, 1, LIGHTGREY, LIGHTBLUE);
}

void displaySpan()
{
  char s[24];

  if (display_mode != TXT) {
    // clear area
    for (int i=FFTY-FFTH; i>40; i--) {
      drawHLine(0, i, 480, BLACK);
    }

    // display new marker freqs
    int d = span/16;

    setFont(0);
    for (int i=1; i<8; i++) {
      sprintf(s, "%2.1f", (double)((2*i-8)*d));
      drawString(s, (i*480/8)-3*(strlen(s)), SPANY, 1, WHITE, BLACK);
    }
  }
}

int16_t last_dbm;
extern float rms;

void displaySmeter()
{
  char s[8];

  int16_t dbm = 20*log10(rms) - 127 - 10;               // 512 --> -73dBm=S9, 10dB SL560

  if (!hold) {
    if (!Smeter_init) {
      // clear area
      for (int i=0; i<16; i++) {
        drawHLine(SMETERX-10, SMETERY+i, 180, BLACK);
      }

      if (smode==0) {
        setFont(0);
        drawString("S", SMETERX-10, SMETERY+10, 1, WHITE, BLACK);
        drawHLine(SMETERX, SMETERY, 180, GREY);
        drawHLine(SMETERX, SMETERY+8, 180, GREY);
        for (int x=1; x<=9; x++) {
          drawVLine(SMETERX+12*x, SMETERY+8, 2, GREY);
          if (x%2 != 0) {
            sprintf(s, "%d", x);
            drawString(s, SMETERX+12*x-2, SMETERY+10, 1, GREY, BLACK);
          }
        }
        for (x=10; x<=30; x+=10) {
          drawVLine(SMETERX+9*12+2*x, SMETERY+8, 2, RED);
          sprintf(s, "%d", x);
          drawString(s, SMETERX+9*12+2*x-6, SMETERY+10, 1, RED, BLACK);
        }
      }
      else {
        for (int i=0; i<=8; i++) {
          drawHLine(SMETERX, SMETERY+i, 180, BLACK);
        }
      }

      Smeter_init = true;
    }

    if (!ptt) {
      if (smode == 0) {                                   // graphic s-meter
        int16_t bar = dbm + 73+54;                           // -73dBm --> bar 54
        if (bar>84) bar=84;                               // limit on S9+30dB
        int x;
        int color = BLUE;
        for (x=0; x<=bar; x++) {
          if (x>54) color = RED;                          // S9, 6dB/S-point
          drawVLine(SMETERX+2*x, SMETERY+2, 4, color);
          drawVLine(SMETERX+2*x+1, SMETERY+2, 4, BLACK);
        }
        for (; x<84; x++) {
          drawVLine(SMETERX+2*x, SMETERY+2, 4, BLACK);
          drawVLine(SMETERX+2*x+1, SMETERY+2, 4, BLACK);
        }
      }
      else {                                              // dBm meter
        char s[8];
        setFont(1);
        sprintf(s, "%d dBm  ", dbm);
        drawString(s, SMETERX, SMETERY, 1, YELLOW, BLACK);
      }
      last_dbm = dbm;
    }
  }
}

#define TUNX 300
#define TUNY 145
#define TUNW 130
#define TUNH 8

extern uint16_t tun1, tun2;

void displayTuningRTTY()
{
  char str[16];

  if (mode != MODE_RTTY) {
    if (tuningOn) {
      // clear tuning area
      for (int i=0; i<TUNH+20; i++) {
        drawHLine(TUNX-10, TUNY+i, TUNW+10, BLACK);
      }
      tuningOn = false;
    }
  }
  else {
    if (!hold) {
      if (!tuningOn) {
        // clear area
        for (int i=0; i<16; i++) {
          drawHLine(TUNX, TUNY+i, TUNW+4, BLACK);
        }
        // draw frame
        drawHLine(TUNX, TUNY, TUNW, GREY);
        drawHLine(TUNX, TUNY+8, TUNW, GREY);
        // show 'T'and mark & space
        setFont(0);
        drawString("T", TUNX-10, TUNY+10, 1, WHITE, BLACK);
        sprintf(str, "%d      %d", fmark, fspace);
        drawString(str, TUNX+10, TUNY+10, 1, GREY, BLACK);

        displayLastLines();

        tuningOn = true;
      }
      if (msec > tuningUpdateTime + 100) {
        tuningUpdateTime = msec;
        
        uint16_t t1=tun1, t2=tun2;

        if (t1>30) t1=30;
        if (t2>30) t2=30;

        for (x=0; x<=t1; x++) {
          drawVLine(TUNX+2*x, TUNY+2, 4, YELLOW);
//          drawVLine(TUNX+2*(x+1), TUNY+2, 4, BLACK);
        }
        for (; x<TUNW/4; x++) {
          drawVLine(TUNX+2*x, TUNY+2, 4, BLACK);
//          drawVLine(TUNX+2*(x+1), TUNY+2, 4, BLACK);
        }
        for (x=0; x<=t2; x++) {
          drawVLine(TUNX+TUNW-2*x, TUNY+2, 4, YELLOW);
//          drawVLine(TUNX+TUNW-2*(x+1), TUNY+2, 4, BLACK);
        }
        for (; x<TUNW/4; x++) {
          drawVLine(TUNX+TUNW-2*x, TUNY+2, 4, BLACK);
//          drawVLine(TUNX+TUNW-2*(x+1), TUNY+2, 4, BLACK);
        }

        t1 /= 2;
        t2 /= 2;
        tun1 = t1;
        tun2 = t2;
      }
    }
  }
}

#define CR  0xd
#define LF  0xa
#define NLINE 7

extern bool decode_event;
extern char ch;
extern int fontwidth;
extern int fontheight;

void displayDecodedText()
{
  if (decode_event) {
    if (ch == CR) {
      xpos = 0;
      ch = LF;
    }

    if (ch == LF) {
      // rotate file
      ypos++;
      ypos &= MAXFILE-1;

      // clear the last line
      for (x=0; x<MAXCHARS; x++) {
        text[x][ypos] = ' ';
      }

      // scroll screen
      if (++yscr == MAXSCREEN) {
        // already at last line on screen
        yscr = MAXSCREEN-1;

        if (display_mode == TXT && !hold) {

          displayLastLines();

          // clear last line on screen
          setFont(0);
          for (int i=0; i<fontheight; i++) {
            drawHLine(0, TEXTY-fontheight*(MAXSCREEN-1)+i, 480, BLACK);
          }
        }
      }
    }
    else {
      text[xpos][ypos] = ch;

      if (!hold && display_mode == TXT) {
        setFont(0);
        drawChar(ch, fontwidth*xpos, TEXTY-fontheight*yscr, 1, GREEN, BLACK);
      }

      if (++xpos == MAXCHARS)  xpos = MAXCHARS-1;
    }

    decode_event = false;
  }
}

void displayLastLines()
{
  if (display_mode == TXT) {
    // display last MAXSCREEN-1 lines
    setFont(0);
    uint16_t y = ypos - (MAXSCREEN-1);
    for (int i=0; i<MAXSCREEN; i++) {
      y &= (MAXFILE-1);
      for (x=0; x<MAXCHARS; x++) {
        ch = text[x][y];
        drawChar(ch, fontwidth*x, TEXTY-fontheight*i, 1, GREEN, BLACK);
      }
      y++;
    }
  }
}

void clearText()
{
  int x, y;

  for (y=0; y<MAXFILE; y++) {
    for (x=0; x<MAXCHARS; x++) {
      ch = text[x][y] = ' ';
    }
  }
//  for (x=0; x<10; x++) {
//    text[x][0] = 'A'+x;
//  }
}

int getNumber(char *title)
{
  int result;
  char str[16];

    if (subState == SS_INIT) {
      hold = true;
      fillRect(0, 0, 479, RXTXPOSY-5, BLACK);
      drawRoundRect(0, 0, 479, RXTXPOSY-5, DARKGREY);
      drawString(title, 10, RXTXPOSY-15, 1, YELLOW, BLACK);
      drawHLine(0, RXTXPOSY-17, 480, YELLOW);
      for (int i=0; i<MAXKEYB; i++) {
        drawButton(&keyboard[i]);
      }
      subState = SS_WAIT;
    }
    // wait for touchdown
    if (event == EV_TOUCHDOWN) {
      subState = SS_TOUCHDOWN;
    }
    if (subState == SS_TOUCHDOWN) {
      buttonPress(keyboard, MAXKEYB);                         // read button pressed
      if (event == EV_TOUCHUP) {
        keyboard[button].press = false;
        if (button>=0 && button<=9) {
          result *= 10;
          result += button;
          sprintf(str, "%d", result);
          drawString(str, 100, RXTXPOSY-15, 1, YELLOW, BLACK);
        }
        else if (button==10) {
          result /= 10;
        }
        else if (button==11) {
          return result;
        }
      }
    }
    return -1;
}

#define CWLINE  25
extern char cwLine[];
extern int wpm;

void displayDecodedCW()
{
  char s[8];

  if (decodeOn && decode_event) {
    if (!hold) {
      setFont(0);
      drawString(cwLine, CWLINEX, CWLINEY, 1, GREEN, BLACK);

      sprintf(s, "(%d) ", wpm);
      drawString(s, 440, CWLINEY, 1, GREEN, BLACK);
    }
  }
}

void displayFreq()
{
  setFont(2);
  uint64_t f = freq/10L + (uint64_t)bandFreq[band][2]*100000;   // ignore Hz

  sprintf(freqString, "%llu", f);                               // i.e. 14591875

  freqLen = strlen(freqString);

  if (freqLen != lastFreqLen) {                                 // if # digits changed
    for (int i=0; i<0x30; i++) {                                // fontheight=0x30
      drawHLine(82, FREQPOSY+i, 350, BLACK);                    // remove any left over junk
    }
    lastFreqLen = freqLen;
  }

  int x = 240-32*freqLen/2;                                     // start of char after freqString

  for (int i=0; i<freqLen; i++) {                               // draw chars left to right
    char c = freqString[i];
    if (i>0) {                                                  // do't start with a dec point
      if (freqLen-i==2 || freqLen-i==5) {                       // 9 876 543 21
        drawChar('.', x, FREQPOSY, 1, WHITE, BLACK);            // 3400.918.75
        x += 8;
      }
    }
    drawChar(c, x, FREQPOSY, 1, WHITE, BLACK);
    lastFreqString[i] = c;
    x += 32;                                                   // go to start of next char
  }
  freqPosition = x;
}

void displayStep()
{
  drawHLine(stepPosition-1, STEPPOSY, 3, BLACK);              // remove old step dot 
  drawHLine(stepPosition-1, STEPPOSY+1, 3, BLACK);
  drawHLine(stepPosition-1, STEPPOSY+2, 3, BLACK);
  stepPosition = freqPosition - 32*step - 16;                 // draw new step dot at top center of digit
  if (step>=2) stepPosition -= 8;                             // correct for dec point
  drawHLine(stepPosition-1, STEPPOSY, 3, WHITE);
  drawHLine(stepPosition-1, STEPPOSY+1, 3, WHITE);
  drawHLine(stepPosition-1, STEPPOSY+2, 3, WHITE);
}


void displayMode()
{
  // display mode in button title
  mainMenu[MODE].title = modeName[mode];
  drawButton(&mainMenu[MODE]);
}

/*
 * main menu buttons 0<y<MAINH
 */
void displayMain()
{
  fillRect(0, 0, 479, MAINH, BLACK);          // clear area around buttons
  for (int i=0; i<MAXMAIN; i++) {
    drawButton(&mainMenu[i]);
  }
}

void displayTime(){
  struct ts t;
  t.hour = t.min = 0;
  DS3231_get(&t);
  sprintf(timeString, "%02d:%02d", t.hour, t.min);
  setFont(1);
  drawString(timeString, TIMEPOSX+4, TIMEPOSY, 1, WHITE, BLACK);
}

extern uint16_t dac_iq;
int lastFreqMhz;
uint8_t biastbl[2][50];
uint8_t i_bias, q_bias, bias, lastBias;

void set_bias()
{
  char s[24];

  if (!ptt) {
    int freqMhz = freq/1000000;
    if (lastFreqMhz != freqMhz && freqMhz<50) {
      i2c_eeprom_read(500+freqMhz, &i_bias, 1);
      i2c_eeprom_read(501+freqMhz, &q_bias, 1);
      pwm_set_gpio_level(21, i_bias);
      pwm_set_gpio_level(20, q_bias);
      lastFreqMhz = freqMhz;
    }
  }
}

void cal_bias()
{
  char s[24];

  if (inc>0) while (inc>0) {
    if (++bias>255) bias=0;
      inc--;
  }
  if (inc<0) while (inc<0) {
    if (--bias<0) bias=255;
    inc++;
  }

  if (bias != lastBias) {
    if (subState == 0) {
      i_bias = bias;
      pwm_set_gpio_level(21, i_bias);
      sprintf(s, "bias I: %d  ", i_bias);
    }
    if (subState == 1) {
      q_bias = bias;
      pwm_set_gpio_level(20, q_bias);
      sprintf(s, "bias Q: %d  ", q_bias);
    }
    setFont(1);
    drawString(s, 240, FREQPOSY-16, 1, YELLOW, BLACK);
    lastBias = bias;
  }

  if (subState == 99) {
    int freqMhz = freq/1000000;
    i2c_eeprom_write(500+freqMhz, &i_bias, 1);
    i2c_eeprom_write(501+freqMhz, &q_bias, 1);
    setFont(1);
    drawString("               ", 240, FREQPOSY-16, 1, YELLOW, BLACK);  // clear bias textline
    subState = 0;
    state = S_TUNE;
  }
}


#ifdef __cplusplus
}
#endif
