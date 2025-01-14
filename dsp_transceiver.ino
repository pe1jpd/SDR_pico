#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include "pwm.h"
#include "adc.h"
#include "irq.h"
#include "hardware/dma.h"
#include "pico/multicore.h"
#include "hardware/structs/bus_ctrl.h"

#include "kiss_fftr.h"
#include "tft.h"
#include "filter.h"
#include "si5351.h"
#include "ui.h"
#include "touch.h"
#include "cw.h"
#include "ds3231.h"

#define NFFT 480                                  // width of tft
#define FSAMP 480000UL                            // freq AD sample / 3 channels = 160kHz
#define FSAMP_AUDIO 16000U                        // audio freq sample   32kHz=critical time
#define ADC_NSAMP  30                             // (FSAMP/FSAMP_AUDIO) block = 480k/16k = 30 samples
#define HILBERT_TAP_NUM 15

#define AGC_GAIN_MAX    (1<<AGC_GAIN_SHIFT)       //max attenuation agc can do   signal * agc_gain / AGC_GAIN_MAX
#define AGC_REF		      3u //6
#define AGC_DECAY	      8192u
#define AGC_ATTACK_FAST	  32u  //64
#define AGC_ATTACK_SLOW	 128u  //4096
#define AGC_GAIN_STEP      1u
#define AGC_OFF		     32766u
volatile uint16_t agc_gain=((AGC_GAIN_MAX/2)+1);  // AGC gain/attenuation - starts at the middle
volatile uint16_t rf_gain = 8;
uint16_t fft_gain;

int dma_chan=0;
volatile int16_t adcSample[8][ADC_NSAMP] = {0};
volatile int32_t adcAverage[3];                   // 3 channels I, Q and mic
volatile int32_t adcSampleDec[8][3];              // 8 blocks cyclic, for 3 channels I, Q and mic
volatile int16_t adcResult[3];                    //
volatile int16_t block=0;                         // current block
volatile uint16_t prevBlock = 0;                  // previous block
volatile uint16_t prevBlock1 = 0;                 // previous block
volatile uint16_t prevBlock2 = 0;                 // previous block
volatile uint16_t prevBlock3 = 0;                 // previous block

volatile int16_t fftQSample[NFFT+20];             // only I and Q samples for fft + extra for hilbert
volatile int16_t fftISample[NFFT+20];             // only I and Q samples for fft + extra for hilbert
int16_t last_i_plot[NFFT], last_q_plot[NFFT];     // I/Q plot
int16_t last_a_plot[NFFT], last_b_plot[NFFT], last_c_plot[NFFT];     // audio plot (plus second b-channel)
volatile uint16_t count = 0;    
volatile uint16_t fft_samples_ready = 0;          //all buffer filled
volatile uint16_t fft_display_ready = 0;
int16_t fftLine[NFFT];
int16_t fft_i_s[HILBERT_TAP_NUM], fft_q_s[HILBERT_TAP_NUM];          // Filtered I/Q samples
int16_t qh;  
bool toggle8khz;
uint16_t hanntbl[NFFT];
#define NFM 1024                                  // 1024
int8_t sintbl[NFM];
int8_t costbl[NFM];
volatile uint16_t dac_iq, dac_audio;
uint16_t mode_filter_tap_num = CW_BPF_TAP_NUM;
int16_t *mode_filter_taps = cw_bpf_taps;
int16_t pre_emp_filter;

#define FILTER_SHIFT  16  // 16 bits coef 
volatile int16_t i_s_raw[CW_BPF_TAP_NUM], q_s_raw[CW_BPF_TAP_NUM];      // Raw I/Q samples minus DC bias
volatile int16_t a_s_raw[CW_BPF_TAP_NUM];                               // Raw MIC samples, minus DC bias
volatile int16_t i_tmp[CW_BPF_TAP_NUM], q_tmp[CW_BPF_TAP_NUM];
volatile int16_t tmp[CW_BPF_TAP_NUM];                               // Raw MIC samples, minus DC bias

kiss_fftr_cfg fft_cfg;
kiss_fft_scalar fft_in_minus[NFFT]; // kiss_fft_scalar is defined int16_t
kiss_fft_scalar fft_in_plus[NFFT];
kiss_fft_cpx fft_in[NFFT], fft_out[NFFT];

uint16_t sptr = 0;
int16_t spectrum[64][480];
extern int16_t x_block_min;
extern int16_t x_block_max;

volatile uint16_t tick=0;
volatile uint32_t msec=0;

uint16_t mode;
extern int freq;
uint8_t span = 16;
bool ptt = 0;
bool hold = false;

#define ABS(x)    ((x)<0 ? -(x) : (x))
#define MAG(i,q)  (abs(i)>abs(q) ? abs(i)+abs(q)/4 : abs(q)+abs(i)/4) // approximation of: magnitude = sqrt(i*i + q*q); error 0.95dB

//#define FIFO_START_FFT  10
//#define FIFO_FFT_READY  20
#define FIFO_IQ_SAMPLE  30


/**************************************************************************************
 * AGC reference level is log2(0x40) = 6, where 0x40 is the MSB of half DAC_RANGE
 * 1/AGC_DECAY and 1/AGC_ATTACK are multipliers before agc value integrator
 * These values should ultimately be set by the HMI.
 * The time it takes to a gain change is the ( (Set time)/(signal delta) ) / samplerate
 * So when delta is 1, and attack is 64, the time is 64/15625 = 4msec (fast attack)
 * The decay time is about 100x this value
 * Slow attack would be about 4096
 **************************************************************************************/
#define AGC_REF		3u //6
#define AGC_DECAY	8192u
#define AGC_ATTACK_FAST	 32u  //64
#define AGC_ATTACK_SLOW	 128u  //4096
#define AGC_OFF		32766u
volatile uint16_t agc_decay  = AGC_OFF;       // Decay time, gain correction in case of low level
volatile uint16_t agc_attack = AGC_OFF;       // Attack time, gain correction in case of high level

void setAgc(uint8_t agc)
{
	switch(agc) {
	case SLOW:
		agc_attack = AGC_ATTACK_SLOW;
		agc_decay  = AGC_DECAY;
		break;
	case FAST:
		agc_attack = AGC_ATTACK_FAST;
		agc_decay  = AGC_DECAY;
		break;
	default:
		agc_attack = AGC_OFF;
		agc_decay  = AGC_OFF;
		break;
	}
}

void __not_in_flash_func(dma_handler)(void)
{
  int16_t i, j;
  uint16_t i_dac, q_dac;

  dma_hw->ints0 = 1<<dma_chan;                                        // clear IRQ
  dma_channel_set_write_addr(dma_chan, &adcSample[block][0], true);   // next buffer & retrigger

  // remove bias: https://www.embedded.com/dsp-tricks-dc-removal/ (filter d)

  adcSampleDec[prevBlock][0] = 0;
  adcSampleDec[prevBlock][1] = 0;
  adcSampleDec[prevBlock][2] = 0;

  for (i=0 ; i<ADC_NSAMP; ) {
    //I
    adcAverage[0] += adcSample[prevBlock][i] - (adcAverage[0]>>8); 
    adcSample[prevBlock][i] -= adcAverage[0]>>8;
    i++;
    //Q
    adcAverage[1] += adcSample[prevBlock][i] - (adcAverage[1]>>8);
    adcSample[prevBlock][i] -= adcAverage[1]>>8;
    i++;
    //MIC
    adcAverage[2] += adcSample[prevBlock][i] - (adcAverage[2]>>8);
    adcSample[prevBlock][i] -= adcAverage[2]>>8;

    adcSampleDec[prevBlock][2] += adcSample[prevBlock][i];          // for mic a simple decimating /10 (=low pass)
    i++;
  }

  // lowpass filter 4kHz, samplerate 160kHz
  adcSampleDec[prevBlock][0] = (int16_t)((
                      (adcSample[prevBlock3][21] * 29L) +
                      (adcSample[prevBlock3][24] * 59L) +
                      (adcSample[prevBlock3][27] * 114L) +
                      (adcSample[prevBlock2][0] * 195L) + 
                      (adcSample[prevBlock2][3] * 307L) + 
                      (adcSample[prevBlock2][6] * 453L) +
                      (adcSample[prevBlock2][9] * 635L) +
                      (adcSample[prevBlock2][12] * 849L) +
                      (adcSample[prevBlock2][15] * 1091L) +
                      (adcSample[prevBlock2][18] * 1353L) +
                      (adcSample[prevBlock2][21] * 1622L) +
                      (adcSample[prevBlock2][24] * 1885L) +
                      (adcSample[prevBlock2][27] * 2127L) +
                      (adcSample[prevBlock1][0] * 2334L) + 
                      (adcSample[prevBlock1][3] * 2493L) + 
                      (adcSample[prevBlock1][6] * 2593L) +
                      (adcSample[prevBlock1][9] * 2627L) +
                      (adcSample[prevBlock1][12] * 2593L) +
                      (adcSample[prevBlock1][15] * 2493L) +
                      (adcSample[prevBlock1][18] * 2334L) +
                      (adcSample[prevBlock1][21] * 2127L) +
                      (adcSample[prevBlock1][24] * 1885L) +
                      (adcSample[prevBlock1][27] * 1622L) +
                      (adcSample[prevBlock][0] * 1353L) +
                      (adcSample[prevBlock][3] * 1091L) +
                      (adcSample[prevBlock][6] * 849L) +
                      (adcSample[prevBlock][9] * 635L) +
                      (adcSample[prevBlock][12] * 453L) +
                      (adcSample[prevBlock][15] * 307L) +
                      (adcSample[prevBlock][18] * 195L) +
                      (adcSample[prevBlock][21] * 114L) +
                      (adcSample[prevBlock][24] * 59L) +
                      (adcSample[prevBlock][27] * 29L)   ) >> 13u);  // >>16  *8 to give some gain (on average sum is *10)

  adcSampleDec[prevBlock][1] = (int16_t)((
                      (adcSample[prevBlock3][22] * 29L) +
                      (adcSample[prevBlock3][25] * 59L) +
                      (adcSample[prevBlock3][28] * 114L) +
                      (adcSample[prevBlock2][1] * 195L) + 
                      (adcSample[prevBlock2][4] * 307L) + 
                      (adcSample[prevBlock2][7] * 453L) +
                      (adcSample[prevBlock2][10] * 635L) +
                      (adcSample[prevBlock2][13] * 849L) +
                      (adcSample[prevBlock2][16] * 1091L) +
                      (adcSample[prevBlock2][19] * 1353L) +
                      (adcSample[prevBlock2][22] * 1622L) +
                      (adcSample[prevBlock2][25] * 1885L) +
                      (adcSample[prevBlock2][28] * 2127L) +
                      (adcSample[prevBlock1][1] * 2334L) + 
                      (adcSample[prevBlock1][4] * 2493L) + 
                      (adcSample[prevBlock1][7] * 2593L) +
                      (adcSample[prevBlock1][10] * 2627L) +
                      (adcSample[prevBlock1][13] * 2593L) +
                      (adcSample[prevBlock1][16] * 2493L) +
                      (adcSample[prevBlock1][19] * 2334L) +
                      (adcSample[prevBlock1][22] * 2127L) +
                      (adcSample[prevBlock1][25] * 1885L) +
                      (adcSample[prevBlock1][28] * 1622L) +
                      (adcSample[prevBlock][1] * 1353L) +
                      (adcSample[prevBlock][4] * 1091L) +
                      (adcSample[prevBlock][7] * 849L) +
                      (adcSample[prevBlock][10] * 635L) +
                      (adcSample[prevBlock][13] * 453L) +
                      (adcSample[prevBlock][16] * 307L) +
                      (adcSample[prevBlock][19] * 195L) +
                      (adcSample[prevBlock][22] * 114L) +
                      (adcSample[prevBlock][25] * 59L) +
                      (adcSample[prevBlock][28] * 29L)   ) >> 13u);  // >>16  *8 to give some gain (on average sum is *10)

  if (!ptt && mode == MODE_CW) {
    // RX CW with samplerate 8kHz
    if (toggle8khz) {
      adcResult[0] = (uint32_t)(adcSampleDec[0][0] + 
                                adcSampleDec[1][0])>>1;                       // = 10x
      adcResult[1] = (uint32_t)(adcSampleDec[0][1] +
                                adcSampleDec[1][1])>>1;                       // = 10x
      adcResult[2] = adcSampleDec[prevBlock][2]>>3;                           // = /8

      multicore_fifo_push_blocking(FIFO_IQ_SAMPLE);
    }
    toggle8khz = !toggle8khz;
  }
  else {
    // default RX and TX samplerate 16kHz
    adcResult[0] = adcSampleDec[prevBlock][0];                                // = 10x input signal, 12bits x 10 = 16bits   (FFF * 10 = 0x9FF6)
    adcResult[1] = adcSampleDec[prevBlock][1];                                // = 10x input signal, 12bits x 10 = 16bits   (FFF * 10 = 0x9FF6)
    adcResult[2] = adcSampleDec[prevBlock][2]>>3u;                            // = /8 for mic 

    multicore_fifo_push_blocking(FIFO_IQ_SAMPLE);
  }

  // collect fft block with samples
  if (!ptt && fft_samples_ready == 0) {
    if (span == 160) {
      // collect all raw samples (160 kHz)
      for (i=0; i<ADC_NSAMP; ) {
        fftQSample[count] = adcSample[prevBlock][i++];
        fftISample[count] = adcSample[prevBlock][i++];
        i++;
        count++;
      }
    }
    else {
      // samples after decimation (16kHz)    
      fftQSample[count] = adcSampleDec[prevBlock][0]>>3;
      fftISample[count] = adcSampleDec[prevBlock][1]>>3;
      count++;
    }

    if (count > 495) {        // 480 + hilbert
      // start fft
      fft_samples_ready = 1;
      count = 0;
    }
  }
  else if (fft_samples_ready == 2) {
    // fft finished
    fft_samples_ready = 0;
  }

  prevBlock3 = prevBlock2;
  prevBlock2 = prevBlock1;
  prevBlock1 = prevBlock;
  prevBlock = block;
  block++;              // next block
  block &= 7;           // 8 blocks cyclic

  // msec in ms
  if (++tick >= 16) {
    tick = 0;
    msec++;
  }
}

#define __UA   1024 // corresponds to 2PI
//#define __atan2(z)  (__UA/8  + __UA/22) * z                 // very much of a simplification...not accurate at all, but fast
#define __atan2(z)  (__UA/8 - __UA/22 * z + __UA/22) * z  //derived from (5) [1]
inline int16_t _arctan3(int16_t q, int16_t i)               // error ~ 0.8 degree
{
  int16_t r;
  if(abs(q) > abs(i))
    r = __UA/4 - __atan2(abs(i)/abs(q));                    // arctan(z) = 90-arctan(1/z)
  else
    r = (i == 0) ? 0 : __atan2(abs(q) / abs(i));            // arctan(z)
  r = (i < 0) ? __UA/2 - r : r;                             // arctan(-z) = -arctan(z)
  return (q < 0) ? -r : r;                                  // arctan(-z) = -arctan(z)
}

volatile int16_t i_s[HILBERT_TAP_NUM], q_s[HILBERT_TAP_NUM];
volatile int16_t q_sample, i_sample, a_sample;
int16_t iq_peak = 0;
int16_t z[2];
volatile int16_t dphi, lp;
uint16_t s_meter_delay;
uint16_t tuning_delay;
int16_t dc;
int32_t lasta_sample, a_noise;
volatile int32_t peak_avg_shifted=0;                        // signal level detector after AGC = average of positive values
volatile int16_t peak_avg_diff_accu=0;                      // Log peak level integrator
int8_t sql_val;
float rms;
int n_sample, n_sample0, noise, noise_lvl;

void rx()
{
	int32_t q_accu=0, i_accu=0;
	uint16_t i;
	uint16_t k;

  i_sample = (rf_gain * adcResult[0]) >> RF_GAIN_SHIFT;
  q_sample = (rf_gain * adcResult[1]) >> RF_GAIN_SHIFT;

  // FIR Filter dependent of mode
  for (i=0; i<(mode_filter_tap_num-1u); i++) {
    i_s_raw[i] = i_s_raw[i+1];                            // raw samples shift register
    q_s_raw[i] = q_s_raw[i+1];
  }
  i_s_raw[(mode_filter_tap_num-1u)] = i_sample;           // feed new sample
  q_s_raw[(mode_filter_tap_num-1u)] = q_sample;

  for (i=0; i<mode_filter_tap_num; i++) {
    i_accu += (int32_t)i_s_raw[i] * mode_filter_taps[i];  // calculate fir
    q_accu += (int32_t)q_s_raw[i] * mode_filter_taps[i];
  }
  i_accu = i_accu >> FILTER_SHIFT;                        // scale
  q_accu = q_accu >> FILTER_SHIFT;

  // S-meter without AGC
  int16_t iq_tmp = MAG(i_accu, q_accu);                   // |i,q|
  iq_peak = MAX(iq_peak, iq_tmp);
  if (++s_meter_delay % 2048 == 0) {
    rms = (float)iq_peak;
    iq_peak /= 2;
  }

  // perform AGC
  q_accu = (agc_gain * q_accu) >> AGC_GAIN_SHIFT;
  i_accu = (agc_gain * i_accu) >> AGC_GAIN_SHIFT;

  // prepare Hilbert transform on Q, delay for I for SSB/CW
  for (i=0; i<(HILBERT_TAP_NUM-1u); i++) {
    q_s[i] = q_s[i+1];
    i_s[i] = i_s[i+1];
  }
	q_s[HILBERT_TAP_NUM-1] = q_accu;
	i_s[HILBERT_TAP_NUM-1] = i_accu;

  // demodulaters
	switch (mode) {
  case MODE_RTTY:
	case MODE_USB:
    // I[7] - Qh
		q_accu = (q_s[0]-q_s[14])*315L + (q_s[2]-q_s[12])*440L + (q_s[4]-q_s[10])*734L + (q_s[6]-q_s[8])*2202L;
		qh = q_accu >> 12;
		a_sample = i_s[7] - qh;
		break;

	case MODE_LSB:
    // I[7] + Qh
		q_accu = (q_s[0]-q_s[14])*315L + (q_s[2]-q_s[12])*440L + (q_s[4]-q_s[10])*734L + (q_s[6]-q_s[8])*2202L;
		qh = q_accu >> 12;
		a_sample = i_s[7] + qh;
		break;

	case MODE_CW:
    q_accu = (q_s[0]-q_s[14])*315L + (q_s[2]-q_s[12])*440L + (q_s[4]-q_s[10])*734L + (q_s[6]-q_s[8])*2202L;
    qh = q_accu >> 12;  
    a_sample = i_s[7] - qh;
    break;

	case MODE_AM:
    a_sample = MAG(i_accu, q_accu);
    break;

  case MODE_FM:
    z[0] = _arctan3(q_sample, i_sample);              // calc momentary phase
    dphi = z[1]-z[0];                                 // freq = dPhase/dt, dt=1/16000
    z[1] = z[0];
    if (dphi < -__UA/2) dphi += __UA;
	  if (dphi > __UA/2) dphi -= __UA;
    a_sample = dphi>>2;

    // squelch only for FM
    n_sample = dphi<<2;                               // scale input for noise detector
    noise += (n_sample - n_sample0)*0.8;              // high pass 1-z^-1/1-0.8z^-1
    n_sample0 = n_sample;
    noise_lvl += (abs(noise)-noise_lvl)/256;          // detect and low pass
    if (noise_lvl>>3 > 80-sql_val)                    // sql level 0=open, 80=closed
      a_sample = 0;

    break;
  }

  // agc
    peak_avg_shifted += ABS(a_sample) - (peak_avg_shifted>>2);  // result is average*4
  
    k=0;
    i = peak_avg_shifted>>2;     
	  if (i & 0xff00) {k+=8; i>>=8;}				          // Logarithmic peak detection
	  if (i & 0x00f0) {k+=4; i>>=4;}                  // k=log2(peak), find highest bit set 
	  if (i & 0x000c) {k+=2; i>>=2;}                  // results k = 0 - 15
	  if (i & 0x0002) {k+=1;}

    if(k>3 && peak_avg_diff_accu<0) {
      peak_avg_diff_accu = 0;                       //start attack from fixed point = fast
    }
    peak_avg_diff_accu += k - AGC_REF;              // Add difference with target to integrator (Acc += Xn - R)  AGC_REF=6
 
	  if (peak_avg_diff_accu > agc_attack) {
      if(agc_gain>1) 
        agc_gain -= AGC_GAIN_STEP;                  // Decrease gain
      else 
        agc_gain = 1;
		  peak_avg_diff_accu -= agc_attack;						  // Reset integrator
	  } 
	  else if (peak_avg_diff_accu < -agc_decay)	{
      if(agc_gain < (AGC_GAIN_MAX-AGC_GAIN_STEP)) 
        agc_gain += AGC_GAIN_STEP;                  // Increase gain
      else 
        agc_gain = AGC_GAIN_MAX;
		  peak_avg_diff_accu += agc_decay;						  // Reset integrator
	  }

  // decode data
  if (mode == MODE_RTTY) {
    rtty_decode(a_sample);                          // @16kHz
  }
  else if (mode == MODE_CW) {
    cw_decode(a_sample);                            // @8kHz
  }

  //output audio
	a_sample += 127;                                  // make unsigned
	if (a_sample > 255)	
    a_sample = 255;                                 // limit to DAC range
	else if (a_sample < 0)
    a_sample = 0;
  
  pwm_set_chan_level(dac_audio, PWM_CHAN_A, a_sample);
}

// 666Hz cw tone @ 16kHz sample freq 
#define CW_TONE_NUM  24
int16_t cw_tone_pos = 0;
// ADC_RANGE 4095  >>4 = 255 255
int16_t cw_tone[CW_TONE_NUM] = {0,  529, 1023,  1447,  1773,  1977,  2047,  1977,  1773,  1447,  1023,  529, -1,  -530,  -1024, -1448, -1774, -1978, -2048, -1978, -1774, -1448, -1024, -530}; 
volatile int16_t a_s[HILBERT_TAP_NUM];
volatile int16_t phase_accu;

void tx()
{
  int32_t a_accu, q_accu=0, i_accu=0;
  int16_t qh=0;
  uint i;
  uint16_t i_dac, q_dac;

  // filter audio dependent on mode
	for (i=0; i<(mode_filter_tap_num-1u); i++) {
    a_s_raw[i] = a_s_raw[i+1];
  }
  a_s_raw[mode_filter_tap_num-1u] = adcResult[2];                 // mic input

  if(mode != MODE_CW) {                                           // no filter for CW  - direct generated
    a_accu = 0;
    for (i=0; i<mode_filter_tap_num; i++) {                       // audio FIR filter, using raw samples
      a_accu += (int32_t)a_s_raw[i] * mode_filter_taps[i];        // filters audio sample
    }
    a_accu >>= FILTER_SHIFT;

    if (mode != MODE_FM) {
      for (i=0; i<(HILBERT_TAP_NUM-1); i++)                       //prepare hilbert
        a_s[i] = a_s[i+1];                                        // Shift decimated samples
      a_s[(HILBERT_TAP_NUM-1)] = a_accu;
    }
  }

  switch (mode) {

    case MODE_USB:
    q_accu = (a_s[0]-a_s[14])*315L + (a_s[2]-a_s[12])*440L + (a_s[4]-a_s[10])*734L + (a_s[6]-a_s[ 8])*2202L;
    qh = -(q_accu >> 12);	                    // USB: sign is negative
    break;

    case MODE_LSB:
    q_accu = (a_s[0]-a_s[14])*315L + (a_s[2]-a_s[12])*440L + (a_s[4]-a_s[10])*734L + (a_s[6]-a_s[ 8])*2202L;
    qh = (q_accu >> 12);	// MODE_LSB: sign is positive
    break;    

    case MODE_AM:
    qh = a_s[7];                              // I and Q values are identical
    break;

    case MODE_CW:
    // Tx MODE_CW I=0 Q=tone
    if(++cw_tone_pos >= CW_TONE_NUM) {
      cw_tone_pos = 0;
    }
    qh = cw_tone[cw_tone_pos]>>2;
    i = cw_tone_pos + (CW_TONE_NUM/4);        // 90 degrees
    if(i >= CW_TONE_NUM) {
      i -= CW_TONE_NUM;
    }
    a_s[7] = cw_tone[i]>>2;                   //audio side tone
    pwm_set_chan_level(dac_audio, PWM_CHAN_A, (cw_tone[cw_tone_pos]>>8)+127);
    break;

    default:
    break;
  }

  if (mode != MODE_FM) {
    a_accu = 127 - (qh>>4);                   //(qh/16);
    if (a_accu<0) q_dac = 0;
    else if (a_accu>255) q_dac = 255;
    else q_dac = a_accu;
	
    a_accu = 127 + (a_s[7]>>4);               //(a_s[7]/16);
    if (a_accu<0) i_dac = 0;
    else if (a_accu>255) i_dac = 255;
    else i_dac = a_accu;

    pwm_set_both_levels(dac_iq, i_dac, q_dac);	// Set both channels of the IQ slice simultaneously
  }
  else {
    // deviation = M*RefClk/2^N = (0x13fe/2)*16000/2048 = 10kHz (M=max tuning word, N length of phase accu)
    // https://www.analog.com/en/technical-articles/low-power-iq-modulators-for-generating-fm.html

    phase_accu += adcResult[2];
    phase_accu &= (NFM-1);

    q_dac = costbl[phase_accu];
    i_dac = sintbl[phase_accu];

    pwm_set_both_levels(dac_iq, i_dac, q_dac);	// Set both channels of the IQ slice simultaneously
  }

}

void core0_irq_handler() 
{
  multicore_fifo_clear_irq();

  if(multicore_fifo_rvalid()) {
    // pop the data from FIFO stack
    (void)multicore_fifo_pop_blocking();

    if (ptt) 
      tx();
    else
      rx();
  }
}

void core1()
{
  int16_t i, j;

  //bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS; // Set Core 1 prio high
  // Grant high bus priority to the DMA, so it can shove the processors out
  // of the way. This should only be needed if you are pushing things up to
  // >16bits/clk here, i.e. if you need to saturate the bus completely.
  bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

  // setup fft
  fft_cfg = kiss_fftr_alloc(NFFT, false, 0, 0);

  // init ADCs
  adc_gpio_init(26);                // GP26 is ADC 0  Q
  adc_gpio_init(27);                // GP27 is ADC 1  I
  adc_gpio_init(28);                // GP28 is ADC 2  MIC
  adc_init();                       // Initialize ADC to known state
  adc_select_input(0);              // Start with ADC0  (AINSEL = 0)

  adc_set_round_robin(0x01+0x02+0x04);      // Sequence ADC 0-1-2 (GP 26, 27, 28) free running
  adc_fifo_setup(
      true,    // Write each completed conversion to the sample FIFO
      true,    // Enable DMA data request (DREQ)
      1,       // DREQ (and IRQ) asserted when at least 1 sample present
      false,   // We won't see the ERR bit because of 8 bit reads; disable.
      false    // 12 bits in FIFO
  );

  adc_set_clkdiv(100);     //48Mhz / 480Khz = 100
  irq_set_enabled(22, true); // FIFO IRQ number

  dma_chan = dma_claim_unused_channel(true);
  dma_channel_config cfg = dma_channel_get_default_config(dma_chan);

  // Reading from constant address, writing to incrementing byte addresses
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
  channel_config_set_read_increment(&cfg, false);
  channel_config_set_write_increment(&cfg, true);

  channel_config_set_dreq(&cfg, DREQ_ADC);
  dma_channel_configure(
      dma_chan, 
      &cfg,
      &adcSample[block][0],     // dst
      &adc_hw->fifo,            // src
      ADC_NSAMP,                // block size
      true                      // start immediately
  );
  dma_channel_set_irq0_enabled(dma_chan, true);
  // on interrupt, call handler
  irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
  dma_hw->ints0 = 1u << dma_chan;                     // clear irq
  irq_set_enabled(DMA_IRQ_0, true);

  for (i=0; i<10000; i++) {  j++; }   //wait core0 to be ready

  adc_run(true);

  while (true) {
    
    if (fft_samples_ready == 1) {

      int16_t k = 0;
      for (j=0; j<HILBERT_TAP_NUM; j++) {
        fft_q_s[j] = (int32_t)(rf_gain * fft_gain * fftQSample[k]) >> (FFT_GAIN_SHIFT + RF_GAIN_SHIFT);
        fft_i_s[j] = (int32_t)(rf_gain * fft_gain * fftISample[k]) >> (FFT_GAIN_SHIFT + RF_GAIN_SHIFT);
        k++;
      }

      // Hilbert transform
      for(j=0; j<NFFT; j++) {

        // shift samples
        for (i=0; i<(HILBERT_TAP_NUM-1); i++) {
          fft_q_s[i] = fft_q_s[i+1];
          fft_i_s[i] = fft_i_s[i+1];
        }

        // feed new samples
        fft_q_s[(HILBERT_TAP_NUM-1)] = (int32_t)(rf_gain * fft_gain * fftQSample[k]) >> (FFT_GAIN_SHIFT + RF_GAIN_SHIFT);
        fft_i_s[(HILBERT_TAP_NUM-1)] = (int32_t)(rf_gain * fft_gain * fftISample[k]) >> (FFT_GAIN_SHIFT + RF_GAIN_SHIFT);
        k++;

        // do transform
        qh = ((int32_t)(fft_q_s[0]-fft_q_s[14])*315L + (int32_t)(fft_q_s[2]-fft_q_s[12])*440L + 
             (int32_t)(fft_q_s[4]-fft_q_s[10])*734L + (int32_t)(fft_q_s[6]-fft_q_s[8])*2202L) >> 12;  // / 4096L      

        // calc upper and lower sideband
        fft_in_minus[j] = fft_i_s[7] - qh;  //USB
        fft_in_plus[j] = fft_i_s[7] + qh;   //LSB
      }

      // lower half of spectrum
      for (i=0; i<NFFT; i++) {
        fft_in_minus[i] = (int32_t)(fft_in_minus[i]*hanntbl[i])>>12;      // WAS 11!
      }
      kiss_fftr(fft_cfg , fft_in_minus, fft_out);

      // calculate magnitude
      for (i=0; i<240; i++) {
        fftLine[240-i] = MAG(fft_out[i].r, fft_out[i].i);
      }

      // upper half of spectrum
      for (i=0; i<NFFT; i++) {
        fft_in_plus[i] = (int32_t)(fft_in_plus[i]*hanntbl[i])>>12;
      }
      kiss_fftr(fft_cfg , fft_in_plus, fft_out);

      // calculate magnitude
      for (i=0; i<240; i++) {
        fftLine[240+i] = MAG(fft_out[i].r, fft_out[i].i);
      }

      // fft done
      fft_display_ready = 1;
    }
  }
}

void plotIQ()
{
  uint16_t x;
  int16_t i_plot, q_plot;

  // draw I and Q
  for (x=0; x<480; x++) {
    drawPixel(x, last_i_plot[x], BLACK);
    drawPixel(x, last_q_plot[x], BLACK);

    i_plot = FFTY + (fftISample[x]>>6);      // 0x0 < ii < 0x13fe/64 => 0x0 < ii < 0x4f=80
    q_plot = FFTY + (fftQSample[x]>>6);
    drawPixel(x, i_plot, YELLOW);
    drawPixel(x, q_plot, CYAN);

    last_i_plot[x] = i_plot;
    last_q_plot[x] = q_plot;
  }
}

extern uint16_t plotdata1[480], plotdata2[480], plotdata3[480];
extern bool klaar;
void plotAudio()
{
  uint16_t x;
  int16_t a_plot, b_plot, c_plot;

  for (x=0; x<480; x++) {
    a_plot = 70+(a_sample>>2);
    drawPixel(x, last_a_plot[x], BLACK);
    drawPixel(x, a_plot, YELLOW);
    last_a_plot[x] = a_plot;
/*  
    b_plot = 70+(plotdata2[x]>>6);
    drawPixel(x, last_b_plot[x], BLACK);
    drawPixel(x, b_plot, RED);
    last_b_plot[x] = b_plot;

    c_plot = 70+(plotdata3[x]>>6);
    drawPixel(x, last_c_plot[x], BLACK);
    drawPixel(x, c_plot, GREEN);
    last_c_plot[x] = c_plot;
  */
  }
}

extern uint16_t xHoldL, xHoldH;
int16_t intens;
void plotSpectrum()
{
	int16_t i, x, y;

  // insert new spectrum line
  uint16_t optr = sptr++;
  sptr &= 0x3f;
  for (x=0; x<480; x++) {
    // dBm scale
    spectrum[sptr][x] = 10.0*log10(fftLine[x]);
  }

  drawHLine(0, FFTY+5, 480, DARKGREY);
  drawHLine(0, FFTY+10, 480, DARKGREY);
  drawHLine(0, FFTY+15, 480, DARKGREY);
  drawHLine(0, FFTY+20, 480, DARKGREY);
  drawHLine(0, FFTY+25, 480, DARKGREY);
  drawHLine(0, FFTY+30, 480, DARKGREY);

  drawVLine(1, FFTY, 30, DARKGREY);
  drawVLine(60, FFTY, 30, DARKGREY);
  drawVLine(120, FFTY, 30, DARKGREY);
  drawVLine(180, FFTY, 30, DARKGREY);
  drawVLine(240, FFTY, 30, DARKGREY);
  drawVLine(300, FFTY, 30, DARKGREY);
  drawVLine(360, FFTY, 30, DARKGREY);
  drawVLine(420, FFTY, 30, DARKGREY);
  drawVLine(479, FFTY, 30, DARKGREY);

  // plot spectrum
	for (x=1; x<480; x++) {
    drawVLine(x, FFTY+spectrum[optr][x-1], spectrum[optr][x]-spectrum[optr][x-1], BLACK);  // remove last fft
    drawVLine(x, FFTY+spectrum[sptr][x-1], spectrum[sptr][x]-spectrum[sptr][x-1], WHITE);  // draw new fft
	}

  switch (mode) {
    case MODE_LSB:
    drawVLine(240, FFTY, 30, GREEN);
    drawVLine( (span==160) ? 229 : 132, FFTY, 30, GREEN);       // 240-480*3600/160000 or 240-480*3600/16000
    break;
    case MODE_USB:
    drawVLine(240, FFTY, 30, GREEN);
    drawVLine( (span==160) ? 251 : 348, FFTY, 30, GREEN);
    break;
    case MODE_CW:
    drawVLine( (span==160) ? 241 : 256, FFTY, 30, GREEN);
    drawVLine( (span==160) ? 243 : 263, FFTY, 30, GREEN);
    break;
    case MODE_AM:
    drawVLine( (span==160) ? 228 : 120, FFTY, 30, GREEN);
    drawVLine( (span==160) ? 252 : 360, FFTY, 30, GREEN);
    break;
    case MODE_FM:
    drawVLine( (span==160) ? 217 : 150, FFTY, 30, GREEN);
    drawVLine( (span==160) ? 263 : 330, FFTY, 30, GREEN);
    break;
  }

  // plot waterfall
//  for (y=0; y<50; y++) {
  y = sptr;
  for (int i=0; i<FFTH; i++) {
    y--;
    y &= 0x3f;
    for (x=1; x<480; x++) {
      // check if waterfall is partly blocked by pop-up
      if (x<xHoldL || x>xHoldH) {
        intens = spectrum[y][x];
        if (intens>30)
          drawPixel(x, FFTY-i, RED);
        else if (intens>20)
          drawPixel(x, FFTY-i, YELLOW);
        else if (intens>0)
          drawPixel(x, FFTY-i, CYAN);
        else {
          drawPixel(x, FFTY-i, BLUE);
        }
      }
    }
  }
}

void setMode(int mode)
{
  switch (mode) {
  case MODE_LSB:
  case MODE_USB:
  case MODE_RTTY:
    mode_filter_tap_num = SSB_LPF_TAP_NUM;   // band pass filter for SSB
    mode_filter_taps = ssb_lpf_taps;
    break;
  case MODE_AM:
    mode_filter_tap_num = AM_LPF_TAP_NUM;      // band pass filter for AM
    mode_filter_taps = am_lpf_taps;
    break;
  case MODE_FM:
    mode_filter_tap_num = FM_BPF_TAP_NUM;   // band pass filter for FM TX
    mode_filter_taps = fm_bpf_taps;
    break;
  case MODE_CW:
    mode_filter_tap_num = CW_BPF_TAP_NUM;      // band pass filter for CW
    mode_filter_taps = cw_bpf_taps;
    break;
  default:
    break;
  }
}

void setup()
{
  char s[80];
  int i;

  Serial.begin(115200);
//  while(!Serial);

  Wire.begin();
  Wire.setClock(400000);                    // Fast mode
  SPI.begin();

  // init DACs  I/Q out 
  gpio_set_function(20, GPIO_FUNC_PWM);     // GP20 is PWM for Q DAC (Slice 2, Channel A)
  gpio_set_function(21, GPIO_FUNC_PWM);     // GP21 is PWM for I DAC (Slice 2, Channel B)
  dac_iq = pwm_gpio_to_slice_num(20);       // Get PWM slice for GP20 (Same for GP21)
  pwm_set_clkdiv_int_frac (dac_iq, 1, 0);   // clock divide by 1 = 125MHz
  pwm_set_wrap(dac_iq, 255);                // Set cycle length; nr of counts until wrap, 125MHz / 255 = 490kHz
  pwm_set_enabled(dac_iq, true);            // Set the PWM running

  // init DAC audio out
  gpio_set_function(22, GPIO_FUNC_PWM);     // GP22 is PWM for Audio DAC (Slice 3, Channel A)
  dac_audio = pwm_gpio_to_slice_num(22);    // Find PWM slice for GP22
  pwm_set_clkdiv_int_frac (dac_audio, 1, 0);// clock divide by 1 = 125MHz
  pwm_set_wrap(dac_audio, 255);             // Set cycle length; nr of counts until wrap, 125MHz / 255 = 490kHz
  pwm_set_enabled(dac_audio, true);         // Set the PWM running

  init_tft();
  si_init();
  init_ui();

  for (i=0; i<NFFT; i++) {
    hanntbl[i] = 4096.0*(1.0+cos(2.0*M_PI*(i-250)/(float)NFFT))/2.0;
  }

  for (i=0; i<NFM; i++) {
    sintbl[i] =127+127*sin(2.0*M_PI*i/(float)NFM);
    costbl[i] =127+127*cos(2.0*M_PI*i/(float)NFM);
  }

  // start core1
  delay(500);  //required to run core1 - after tests
  multicore_launch_core1(core1);
  delay(5);

  irq_set_exclusive_handler(SIO_IRQ_PROC0, core0_irq_handler);

  // clear the interrupt flag to be sure
  multicore_fifo_clear_irq();

  // enable interrupt
  irq_set_enabled(SIO_IRQ_PROC0, true);
}

/*
 * Main loop
 */

extern uint16_t display_mode;

void loop() 
{
  ui_handler();
  cat_handler();

  if (fft_display_ready == 1) {
    if (!hold) {
      switch (display_mode) {
      case IQ:
        plotIQ();
        break;
      case AU:
        plotAudio();
        break;
      case FFT:
        plotSpectrum();
        break;
      default:
        break;
      }
    }
    else {
      delay(100);                                       // no plot, just delay for touch mechanism
    }
    fft_samples_ready = 2;
    fft_display_ready = 0;
  }
}

/*
 * CAT routines
 */

char buf[16];
int catptr = 0;
extern uint8_t state, subState;

void cat_handler()
{
  char c;

  if (Serial.available()) {

    c = Serial.read();

    if (c>='0' && c<='Z') {
      buf[catptr++] = c;
    }
    else if (c == 0x0d) {
      Serial.println(buf);

      // set or get frequency
      if (buf[0]== 'F' && buf[1] == 'A') {
        if (buf[2]!=';') {
          for (int i=2; i<20; i++) {
            c = buf[i];
            if (c==';') break;
            freq *= 10;
            freq += c-'0';
          }
        }
        else {
          Serial.println(freq);
        }
      }

      // set or get band
      if (buf[0]== 'B' && buf[1] == 'S') {
        extern uint8_t band;
        int b = atoi(&buf[2]);
        switch (b) {
          case 0: band=0; break;
          case 1: band=1; break;
          case 2: break;
          case 3: band=2; break;
          case 4: band=3; break;
          case 5: band=4; break;
          case 6: band=5; break;
          case 7: band=6; break;
          case 8: band=7; break;
          case 9: band=8; break;
          case 10: band=9; break;
          case 15: band=11; break;
          default: break;
        }
      }

      // set date/time
      if (buf[0]== 'D' && buf[1] == 'T') {
        struct ts t;
        DS3231_get(&t);
        switch (buf[2]) {
          case '0':
            t.mday = getByte(&buf[9]);
            buf[9] = '\0';
            t.mon  = getByte(&buf[7]) & 0x7f;
            buf[7] = '\0';
            t.year = getByte(&buf[3]);
            DS3231_set(t);
            break;
          case '1':
            t.sec = getByte(&buf[7]);
              buf[7] = '\0';
            t.min  = getByte(&buf[5]);
            buf[5] = '\0';
            t.hour = getByte(&buf[3]) & 0x3f;
            DS3231_set(t);
            break;
          default:
            break;
        }
        displayTime();
      }

      // TX on/off
      if (buf[0]== 'T' && buf[1] == 'X') {
        if (buf[2] == '1') ptt = true;
        else ptt = false;
      }

      // Calibrate bias: CI and CQ
      if (buf[0] == 'C') {
        if (buf[1] == 'I') {
          state = S_IQCAL;
          subState = 0;
        }
        else if (buf[1] == 'Q') {
          state = S_IQCAL;
          subState = 1;
        }
        else if (buf[1] == 'W') {
          state = S_IQCAL;
          subState = 99;
        }
      }
      catptr = 0;
    }
  }
}


