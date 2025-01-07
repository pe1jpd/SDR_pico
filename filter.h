#ifdef __cplusplus
 extern "C" {
#endif

//#include "Arduino.h"
#include "pwm.h"
#include "adc.h"
#include "irq.h"
#include "hardware/dma.h"
#include "pico/multicore.h"
#include "hardware/structs/bus_ctrl.h"
#include "kiss_fftr.h"

/**************************************************************************************

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 160000 Hz

fixed point precision: 16 bits

* 0 Hz - 4000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 13000 Hz - 80000 Hz
  gain = 0
  desired attenuation = -63 dB
  actual attenuation = n/a

**************************************************************************************/

#define FILTER_TAP_NUM 33

static int16_t lp4khz[] = {
  29,
  59,
  114,
  195,
  307,
  453,
  635,
  849,
  1091,
  1353,
  1622,
  1885,
  2127,
  2334,
  2493,
  2593,
  2627,
  2593,
  2493,
  2334,
  2127,
  1885,
  1622,
  1353,
  1091,
  849,
  635,
  453,
  307,
  195,
  114,
  59,
  29
};

/**************************************************************************************

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 160000 Hz

fixed point precision: 16 bits

* 0 Hz - 6000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 16000 Hz - 80000 Hz
  gain = 0
  desired attenuation = -42 dB  = 0.8% Vin
  actual attenuation = -42.49 dB  = 0.75% Vin

**************************************************************************************/

#define FILTER_TAP_NUM 19

static const int16_t lp6khz[] = {
  304,
  523,
  897,
  1366,
  1901,
  2454,
  2972,
  3394,
  3671,
  3768,
  3671,
  3394,
  2972,
  2454,
  1901,
  1366,
  897,
  523,
  304
};

/**************************************************************************************

FIR filter designed with
http://t-filter.engineerjs.com/

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 3600 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 5000 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

**************************************************************************************/
/*
#define SSB_LPF_TAP_NUM 13

static int16_t ssb_lpf_taps[] = {
  1326,
  2635,
  460,
  -3277,
  -360,
  10353,
  16726,
  10353,
  -360,
  -3277,
  460,
  2635,
  1326
};
*/
/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 2400 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 3400 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

*/

#define SSB_LPF_TAP_NUM 19

static int16_t ssb_lpf_taps[SSB_LPF_TAP_NUM] = {
  568,
  1097,
  1009,
  -256,
  -2129,
  -2877,
  -864,
  3775,
  8677,
  10777,
  8677,
  3775,
  -864,
  -2877,
  -2129,
  -256,
  1009,
  1097,
  568
};

/**************************************************************************************

FIR filter designed with
http://t-filter.engineerjs.com/

sampling frequency: 8000 Hz

fixed point precision: 16 bits

* 0 Hz - 300 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

* 550 Hz - 750 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 1000 Hz - 4000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

**************************************************************************************/

#define CW_BPF_TAP_NUM   49     // band pass filter for CW

static int16_t cw_bpf_taps[CW_BPF_TAP_NUM] = {
  99,
  27,
  20,
  -48,
  -197,
  -385,
  -515,
  -487,
  -245,
  194,
  734,
  1218,
  1461,
  1311,
  715,
  -240,
  -1317,
  -2196,
  -2570,
  -2259,
  -1285,
  119,
  1572,
  2658,
  3059,
  2658,
  1572,
  119,
  -1285,
  -2259,
  -2570,
  -2196,
  -1317,
  -240,
  715,
  1311,
  1461,
  1218,
  734,
  194,
  -245,
  -487,
  -515,
  -385,
  -197,
  -48,
  20,
  27,
  99
};

/**************************************************************************************

FIR filter designed with
http://t-filter.engineerjs.com/

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 4000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 5000 Hz - 8000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

**************************************************************************************/

#define AM_LPF_TAP_NUM 19

static int16_t am_lpf_taps[AM_LPF_TAP_NUM] = {
  422,
  -380,
  -2325,
  -1917,
  1341,
  1276,
  -3105,
  -1395,
  10293,
  17806,
  10293,
  -1395,
  -3105,
  1276,
  1341,
  -1917,
  -2325,
  -380,
  422
};

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 16000 Hz

fixed point precision: 16 bits

* 0 Hz - 50 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a

* 500 Hz - 3000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a

* 3500 Hz - 8000 Hz
  gain = 0
  desired attenuation = -30 dB
  actual attenuation = n/a

*/

#define FM_BPF_TAP_NUM 29

static int16_t fm_bpf_taps[FM_BPF_TAP_NUM] = {
  -664,
  -156,
  -27,
  -633,
  -2018,
  -2964,
  -2184,
  -191,
  708,
  -1005,
  -3633,
  -3342,
  1689,
  8539,
  11737,
  8539,
  1689,
  -3342,
  -3633,
  -1005,
  708,
  -191,
  -2184,
  -2964,
  -2018,
  -633,
  -27,
  -156,
  -664
};


#ifdef __cplusplus
}
#endif
