#ifdef __cplusplus
 extern "C" {
#endif

// Pico pins used 
#define TFT_CS    0  // Chip select control pin
#define TFT_RST   0  // Reset pin
#define TFT_RD    0
#define TFT_WR    7  // Write strobe control pin - must use a pin in the range 0-31
#define TFT_DC    6  // Data Command control pin - must use a pin in the range 0-31

#define TFT_D0    8 
#define TFT_D1    9 
#define TFT_D2   10 
#define TFT_D3   11 
#define TFT_D4   12 
#define TFT_D5   13 
#define TFT_D6   14 
#define TFT_D7   15 


/*********************************************************************
* Overview: Some basic colors definitions.
*********************************************************************/
#define BLACK               0x000000
#define RED                 0xff0000
#define BLUE                0x0000ff
#define LIGHTBLUE           0x00007f
#define GREEN               0x00ff00
#define CYAN                0x00ffff
#define MAGENTA             0xff00ff
#define YELLOW              0xffff00
#define DARKGREY            0x3f3f3f
#define GREY                0x7f7f7f
#define LIGHTGREY           0xcfcfcf
#define WHITE               0xffffff

typedef struct BUTTON {
  char *title;
  int x0; int x1; int y0; int y1;
  int color;
  int border;
  int textcolor;
  bool press;
} BUTTON;

void writeData(unsigned int);
void writeCommand(unsigned int);
void init_tft(void);
void setxy(unsigned int, unsigned int, unsigned int, unsigned int);
unsigned int Color565(unsigned int r, unsigned int g, unsigned int b);
void cls(int);
void setFont(int f);
void drawChar(char c, int x, int y, int size, int color, int bcolor);
void drawString(char *s, int x, int y, int size, int color, int bcolor);
void drawPixel(unsigned int x, unsigned int y, int color);
void drawLine(int x0, int y0, int x1, int y1, int color);
void drawHLine(int x, int y, int l, int color);
void drawVLine(int x, int y, int l, int color);
void drawRect(int x1, int y1, int x2, int y2, int color);
void drawRoundRect(int x1, int y1, int x2, int y2, int color);
void fillRect(int x1, int y1, int x2, int y2, int color);
void fillRoundRect(int x1, int y1, int x2, int y2, int color);
void drawButton(BUTTON *b);

#ifdef __cplusplus
}
#endif
