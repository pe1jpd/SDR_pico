#ifdef __cplusplus
 extern "C" {
#endif

#include "Arduino.h"
#include "cw.h"

extern uint32_t msec;
bool decode_event;
volatile char ch;


#define CR  0xd
#define LF  0xa

const char morse[] = "~ ETIANMSURWDKGOHVF*L*PJBXCYZQ**54S3***2**+***J16=/***H*7*G*8*90************?_****\"**.****@***'**-********;!*)*****,****:****";
const char baudot[] = {'@','E',LF,'A',' ','S','I','U',CR,'D','R','J','N','F','C','K','T','Z','L','W','H','Y','P','Q','O','B','G',0,'M','X','V',0,
                       0,'3',LF,'-',' ','`','8','7',CR,'*','4',0,',','&',':','(','5','+',')','2','&','6','0','1','9','?','&',0,'.','/','=',0};

/*
void printChar()
{
  if(sym < 128) {
    ch = morse[sym]; 
    if (ch != '*') decode_event = true;
  }
  sym = 1; // space
}
*/

int32_t t11, t12, t21, t22, lp1, lp2;
int16_t periodcount = 0, bitperiod;          // =16000/8/45.45 baud
int16_t startbit = 0, bittime, bitcount;
uint8_t chr;
bool shift;
int16_t L1;                                         //  see https://www.micromodeler.com/dsp/#
int16_t K1;
int16_t L2;
int16_t K2;
int32_t tun1=0, tun2=0;

void calcFilter(uint16_t f1, uint16_t f2, double br)
{
  double lf1, kf1, lf2, kf2, tf;

  bitperiod = 2000.0/br + 0.5;                        // 44         2.1667
  double fbr = 2000/bitperiod + 50;                       // 95         973
  tf = 0.65/fbr;                                          // 0.0068     0.000668

  lf1 = exp(-2./tf/16000.);                                 // 0.98       0.82
  kf1 = 4.*(lf1/(lf1+1.0))*cos(2.0*PI*(double)f1/16000.0);           // 1.646
  K1 = 16383 * lf1;
  L1 = 16383 * kf1;

  lf2 = exp(-2./tf/16000.);
  kf2 = 4.*(lf2/(lf2+1.0))*cos(2.0*PI*(double)f2/16000.0);
  K2 = 16383 * lf2;
  L2 = 16383 * kf2;
}

/*
 * RTTY decode audio at 16kHz
 */

void rtty_decode(int16_t sample)
{
  int16_t d, tmp, t;

  // IIR band pass mark
  tmp = sample + ((L1*t11-K1*t12)>>14);
  t12 = t11;
  t11 = tmp;

  // IIR band pass space
  tmp = sample+((L2*t21-K2*t22)>>14);
  t22 = t21;
  t21 = tmp;

  // tuning indicator
  tun1 = MAX(tun1, ABS(t12>>6));
  tun2 = MAX(tun2, ABS(t22>>6));

  // demodulate
  d = abs(t11) - abs(t21);

  // post demodulation low pass
  lp1 = (d + lp1)/2;
  lp2 = (lp1 + lp2)/2;

  if (periodcount < bitperiod) {                    // baud rate subsampling: 8 samples/bit
    periodcount++;
    return;
  }
  periodcount = 0;

  if (startbit == 0) {                              // if waiting for startbit
    if (lp2 < -100) startbit = 1;                   // got one
  }
  else if (startbit > 0) {                          // wait 4 sample periods
    startbit++;
    if (lp2 > 0) startbit = 0;                      // if too short, start again
    if (startbit == 4) {
      bittime = 0;
      bitcount = 0;
      startbit = -1;                                // ready for data
    }
  }
  else {                                            // get data

    bittime++;
    bittime &= 7;

    if (bittime == 0) {
      if (bitcount++ < 5) {
        chr >>= 1;
        chr += (lp2 < 0) ? 0 : 0x10;                     // sample input
      }
      else {
        chr &= 0x1f;
        if (chr == 27)                          // shift to figures
          shift = true;
        else if (chr == 31)                     // unshiftt
          shift = false;
        else {
//          if (ch = ' ') shift = false;          // unshift on space          
          if (!shift)
            ch = baudot[chr];                   // letters
          else
            ch = baudot[32 + chr];              // figures
        }
        decode_event = true;
        startbit = 0;
      }
    }
  }
  
}

uint16_t sig, zerocross, loopfilter;
uint16_t decimate, clk;
int16_t raw, avg;
/*
 * CW decode audio at 8kHz
 */

void cw_decode(int16_t sample)
{
  // detect signal @ 8kHz
  sample <<= 4;
  raw += (abs(sample) - raw) >> 8;                  // IIR lowpass

  // decimate by 8 to 1kHz
  if ((++decimate & 7) != 0) return;

  // caculate threshold 
  avg += (raw - avg)>>8;                           

  // recover bit clock
  int16_t t = raw & 0x8000;                         // get sign of input
  if (sign ^ t)                                     // at zero cross ..
    corr = vco<<1;                                  // .. get correction to bit clock
  sign = t;                                         // update sign
  corr -= 0x2000;                                   // correct with phase offset
  pll += (corr - pll) >> 8;                         // pll loop filter
  vco += center + pll;                              // update vco (sawtooth)

  // sample bit
  t = vco & 0x8000;
  if () {
    bit = raw;
  }
  signvco = t;
}


/*
uint16_t t1, t2;
bool sign = LOW;
bool lastSign = LOW;
bool filteredState = LOW;
bool lastFilteredState = LOW;

// Noise Blanker time which shall be computed so this is initial 
int nbtime = 6;  /// ms noise blanker

uint32_t starttimehigh;
uint32_t highduration;
uint32_t lasthighduration;
uint32_t hightimesavg;
uint32_t lowtimesavg;
uint32_t startttimelow;
uint32_t lowduration;
uint32_t laststarttime = 0;

int  wpm;
int16_t avg = 50;
static uint8_t sym;

void cw_decode(int16_t sample)
{
  sample <<= 4;
  t1 += (abs(sample) - t1)>>8;                     // IIR lowpass

  avg += (t1-avg)>>10;                             // caculate moving average 
  sign = (t1>avg*2);                               // determine on/off

  // Clean up the state with a noise blanker
  if (sign != lastSign) {
    laststarttime = msec;
  }
  // option to scale timing to wpm
  if ((msec - laststarttime) > nbtime) {
    if (sign != filteredState) {
      filteredState = sign;
    }
  }
  else {
    avg += avg/100;
  }
  dec();
  lastSign = sign;
}

void dec()
{

  if (filteredState != lastFilteredState) {

    if (filteredState == HIGH) {
      starttimehigh = msec;
      lowduration = msec - startttimelow;
    }
    if (filteredState == LOW) {
      startttimelow = msec;
      highduration = msec - starttimehigh;

      if (highduration < 500) {
        if (highduration < (2*hightimesavg) || hightimesavg == 0) {
          hightimesavg = (highduration + 2*hightimesavg)/3; // now we know avg dit time ( rolling 3 avg)
        }
        if (highduration > (5*hightimesavg) && highduration < 10*hightimesavg) {
            hightimesavg = highduration/3;   // if speed decrease fast ..
//          hightimesavg = highduration + hightimesavg;   // if speed decrease fast ..
        }
      }
    }
  }

  // Now check the baud rate based on dit or dah duration either 1, 3 or 7 pauses
  if (filteredState != lastFilteredState) {
    if (filteredState == LOW) { 
      // we ended on a HIGH
      if (highduration < (hightimesavg*2) && highduration > (hightimesavg*6/10)) { /// 0.6 filter out false dits
        sym = (sym<<1|(0)); //insert dit (0)
      }
      if (highduration > (hightimesavg*2) && highduration < (hightimesavg*6)) {
        sym = (sym<<1|(1)); //insert dit (1)
        wpm = (wpm + (1200/(highduration/3) * 4/3))/2; //// the most precise we can do ;o)
      }
    }    
    else if (filteredState == HIGH) {
      // we ended on a LOW
      uint16_t lacktime = 20;
//      if (wpm > 10) lacktime = 20;
//      if (wpm > 15) lacktime = 25;

//      if (wpm > 25) lacktime = 10;
//      if (wpm > 30) lacktime = 12;
//      if (wpm > 35) lacktime = 15;
//      lacktime = 18;
      if (lowduration > (hightimesavg*(lacktime*7/80))) { // && lowduration < hightimesavg*(lacktime*5/10)) { // letter space
        printChar();
      }
      if (lowduration >= hightimesavg*(lacktime*5/10)) { // word space
        printChar();
        printChar();// add space
      }
    }
  }

  if ((msec - startttimelow) > (highduration*6) && sym > 1) {
    printChar(); // no more letters
  }

  lastFilteredState = filteredState;
}
*/
#ifdef __cplusplus
}
#endif

