#ifdef __cplusplus
extern "C" {
#endif

// event for CW char received
#define EV_CW         20

// prototypes
void cw_decode(int16_t sample);
//void calcFilter(int f1, int f2, int b);
void rtty_decode(int16_t sample);

#define ABS(x)    ((x)<0 ? -(x) : (x))
#define MAX(x,y)  ((x)>(y) ? (x) : (y))

#ifdef __cplusplus
}
#endif


