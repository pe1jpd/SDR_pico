#ifdef __cplusplus
 extern "C" {
#endif

#include "Arduino.h"
#include "cw.h"

extern volatile uint32_t msec;
bool decode_event;
volatile char ch;


#define CR  0xd
#define LF  0xa

const char morse[] = "* ETIANMSURWDKGOHVF*L*PJBXCYZQ**54S3***2**+***J16=/***H*7*G*8*90************?_****\"**.****@***'**-********;!*)*****,****:****";
const char baudot[] = {'@','E',LF,'A',' ','S','I','U',CR,'D','R','J','N','F','C','K','T','Z','L','W','H','Y','P','Q','O','B','G',0,'M','X','V',0,
                       0,'3',LF,'-',' ','`','8','7',CR,'*','4',0,',','&',':','(','5','+',')','2','&','6','0','1','9','?','&',0,'.','/','=',0};

int32_t t11, t12, t21, t22, lp1, lp2;
int16_t periodcount = 0, bitperiod;
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

  bitperiod = 2000.0/br + 0.5;
  double fbr = 2000/bitperiod + 50;
  tf = 0.65/fbr;

  lf1 = exp(-2./tf/16000.);
  kf1 = 4.*(lf1/(lf1+1.0))*cos(2.0*PI*(double)f1/16000.0);
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

uint16_t raw, avg, avgh, avgl, decimate;
bool high_t0, high;

uint16_t starttimehigh;
uint16_t highduration;
uint16_t starttimelow;
uint16_t lowduration;
uint16_t avghigh;
uint16_t dittime;
static uint8_t sym;
int wpm;
uint16_t k=0, plotdata1[480], plotdata2[480], plotdata3[480];
bool klaar = false;
#define CWLINE  25
char cwLine[] = {"                              "};       // 30 long

/*
 * CW decode audio at 8kHz
 */

void printChar()
{
  if(sym  < 128) {
    char c = morse[sym]; 
    if (c != '*') {
      // shift line
      for (int i=1; i<CWLINE; i++) {
        cwLine[i-1] = cwLine[i];
      }
      // enter new char
      cwLine[CWLINE-1] = c;
      decode_event = true;
    }
  }
  sym = 1; // space
}


void cw_decode(int16_t sample)
{
  sample <<= 8;
  raw += (abs(sample) - raw)>>5;                    // IIR lowpass

  // decimate by 8 to 1kHz
  if ((++decimate & 7) == 0) {

    // determine on/off

    avg += (raw - avg)>>9;                          // find average
    high = (raw > avg);                             // dertermime signal on/off

    // check for low-high or high-low transition
    if (high != high_t0) {
  
      high_t0 = high;

      if (high) {
        // end of pause, start measuring high-time
        starttimehigh = msec;
        lowduration = msec - starttimelow;

        if (lowduration > 2*dittime) { 
          // > letter space: print character
          printChar();
        }
        if (lowduration > 4*dittime) { 
          // > word space print space
          printChar();
        }
      }
      else {
        // end of high, start measuring pause time
        starttimelow = msec;
        highduration = msec - starttimehigh;

        // filter out spikes 80/8000 = <10ms bittime or 120wpm
        if (highduration > 80) {
          // measure dittime
          if (highduration < 2*dittime || dittime == 0) {
          // was a dit, rolling average
          dittime = (highduration + 2*dittime)/3;
          }
          else if (highduration > 2*dittime) {
            // was a dah, 3*dittime
            dittime = highduration/3;
          }
 
          // decode
          if (highduration < 2*dittime) {
          //insert dit (0)
            sym = (sym<<1|(0)); 
          }
          if (highduration > 2*dittime && highduration < 5*dittime) {
            //insert dit (1)
            sym = (sym<<1|(1)); 
            wpm += (1200/dittime - wpm)/2;
          }
        }
      }
    }
  }
}

#ifdef __cplusplus
}
#endif

