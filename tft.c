#ifdef __cplusplus
 extern "C" {
#endif

#include "Arduino.h"
//#include <Wire.h>
#include "pwm.h"
#include "adc.h"
#include "tft.h" 
#include "fonts.h"

#define TFT_WIDTH 480
#define TFT_HEIGHT 272

#define GP_MASK_TFT	((1<<TFT_D0)|(1<<TFT_D1)|(1<<TFT_D2)|(1<<TFT_D3)|(1<<TFT_D4)|(1<<TFT_D5)|(1<<TFT_D6)|(1<<TFT_D7)|(1<<TFT_DC)|(1<<TFT_WR))
int col, row;

void swap(int a, int b) {int t=a; a=b; b=t;} 

void writeData(unsigned int data) {
  gpio_put(TFT_CS, 0);
  gpio_put(TFT_DC, 1);
  gpio_put(TFT_RD, 1);
  gpio_put(TFT_WR, 1);
  gpio_put_masked(0xff00, data<<8);
  gpio_put(TFT_WR, 0);
  gpio_put(TFT_WR, 0);
  gpio_put(TFT_WR, 1);
  gpio_put(TFT_CS, 1);
}

void writeCommand(unsigned int cmd) {
  gpio_put(TFT_CS, 0);
  gpio_put(TFT_DC, 0);
  gpio_put(TFT_RD, 1);
  gpio_put(TFT_WR, 1);
  gpio_put_masked(0xff00, cmd<<8);
  gpio_put(TFT_WR, 0);
  gpio_put(TFT_WR, 0);
  gpio_put(TFT_WR, 1);
  gpio_put(TFT_DC, 1);
  gpio_put(TFT_CS, 1);
}

/* 480 x 272 */
void init_tft()
{
  gpio_init_mask(GP_MASK_TFT);  
  gpio_set_dir_out_masked(GP_MASK_TFT);

  writeCommand(0xe2);
  writeData(0x23);
  writeData(0x02);
  writeData(0x54);

  writeCommand(0xe0);
  writeData(0x01);
  delay(10);

  writeCommand(0xe0);
  writeData(0x03);
  delay(10);

  writeCommand(0x01);
  delay(100);

  writeCommand(0xe6);
  writeData(0x01);
  writeData(0x1f);
  writeData(0xff);

  writeCommand(0xb0);
  writeData(0x20);
  writeData(0x00);
  writeData(0x01);
  writeData(0xdf);
  writeData(0x01);
  writeData(0x0f);
  writeData(0x00);

  writeCommand(0xb4);
  writeData(0x02);
  writeData(0x13);
  writeData(0x00);
  writeData(0x08);
  writeData(0x2b);
  writeData(0x00);
  writeData(0x02);
  writeData(0x00);

  writeCommand(0xb6);
  writeData(0x01);
  writeData(0x20);
  writeData(0x00);
  writeData(0x04);
  writeData(0x0c);
  writeData(0x00);
  writeData(0x02);

  writeCommand(0xba);
  writeData(0x0f);

  writeCommand(0xb8);
  writeData(0x07);
  writeData(0x01);

  writeCommand(0x36);
  //writeData(0x82);
  writeData(0x81);        // vertical flip 

  writeCommand(0xf0);
  writeData(0x00);
  delay(1);

  setxy(0,479,0,271);
  writeCommand(0x29);

  writeCommand(0xbe);
  writeData(0x06);
  writeData(0xf0);
  writeData(0x01);
  writeData(0xf0);
  writeData(0x00);
  writeData(0x00);

  writeCommand(0xd0);
  writeData(0x0d);

  writeCommand(0x2c);

  cls(BLACK);
}

void setxy(unsigned int x1,unsigned int x2,unsigned int y1,unsigned int y2)
{
  writeCommand(0x2a);
  writeData(x1>>8);
  writeData(x1&0xff);
  writeData(x2>>8);
  writeData(x2&0xff);

  writeCommand(0x2b);
  writeData(y1>>8);
  writeData(y1&0xff);
  writeData(y2>>8);
  writeData(y2&0xff);

  writeCommand(0x2c);
}

void fastfill(int color, int n)
{
  for (; n>0; n--) {
    writeData((color>>16)&0xff);
    writeData((color>>8)&0xff);
    writeData((color)&0xff);
  }
}

unsigned int Color565(unsigned int r, unsigned int g, unsigned int b) {
  unsigned int c;
  c = r >> 3;
  c <<= 6;
  c |= g >> 2;
  c <<= 5;
  c |= b >> 3;
  return c;
}

void cls(int color)
{
  setxy(0, 479, 0, 271);
  fastfill(color, 480*272);
  col = row = 0;
}

char *font;
int fontwidth, fontheight, fontstart;
void setFont(int f)
{
  switch (f) {
    case 0:
      font = &Font8x12[0];
      break;
    case 1:
      font = &Font16x16[0];
      break;
    case 2:
      font = &calibribold[0];
      break;
    case 3:
      font = &font8x9[0];
      break;
  }
  fontwidth = font[0];
  fontheight = font[1];
  fontstart = font[2];
}

void drawChar(char c, int x, int y, int size, int color, int bcolor)
{
  uint16_t i, j;
  uint32_t w=0, w1;

  int ptr = 4 + (c-fontstart)*fontheight*(fontwidth>>3);        // offset in chartable
  int32_t msb = 1<<(fontwidth-1);

  setxy(x, x+fontwidth*size-1, y, y+fontheight*size-1);         // allocate room on screen

  for (j=0; j<fontheight; j++) {
    int16_t fw = fontwidth>>3;                                  // #bytes/line: 1, 2 or 4
    do { 
      w <<= 8;
      w += font[ptr++];                                         // get all bytes of line in w
      fw--;
    } while (fw>0);
    w1 = w;
    for (uint16_t sy=0; sy<size; sy++) {
      w = w1;
      for (i=0; i<fontwidth; i++) {
        for (uint16_t sx=0; sx<size; sx++) {
          if (w & msb) {
            writeData((color>>16)&0xff);
            writeData((color>>8)&0xff);
            writeData((color)&0xff);
          }
          else {
            writeData((bcolor>>16)&0xff);
            writeData((bcolor>>8)&0xff);
            writeData((bcolor)&0xff);
          }
        }
        w <<= 1;
      }
    }
  }
}

void drawString(char *s, int x, int y, int size, int color, int bcolor)
{
  while(*s) {
    drawChar(*s++, x, y, size, color, bcolor);
    x += fontwidth*size;
  }
}


void drawPixel(unsigned int x, unsigned int y, int color)
{
  setxy(x,x,y,y);
  writeData((color>>16)&0xff);
  writeData((color>>8)&0xff);
  writeData((color)&0xff);
}

void drawHLine(int x, int y, int l, int color) 
{
	if (l<0) {
		l = -l;
		x -= l;
	}
	setxy(x, x+l, y, y);
  fastfill(color, l+4);
}

void drawVLine(int x, int y, int l, int color)
{
	if (l<0) {
		l = -l;
		y -= l;
	}
  if (l==0) {
    l = 1;
  }
	setxy(x, x, y, y+l);
  fastfill(color, l);
}

void drawLine(int x0, int y0, int x1, int y1, int color)
{
	if (y0==y1)
		drawHLine(x0, y0, x1-x0, color);
	else if (x0==x1)
		drawVLine(x0, y0, y1-y0, color);
	else {
    int dx =  abs(x1-x0), sx = x0<x1 ? 1 : -1;
    int dy = -abs(y1-y0), sy = y0<y1 ? 1 : -1;
    int err = dx+dy, e2;

    for (;;) {
      drawPixel(x0, y0, color);
      if (x0==x1 && y0==y1) break;
      e2 = 2*err;
      if (e2>=dy) { err+=dy; x0+=sx;}
      if (e2<=dx) { err+=dx; y0+=sy;}
    }
  }
}

void drawRect(int x1, int y1, int x2, int y2, int color)
{
	if (x1>x2) swap(x1, x2);
	if (y1>y2) swap(y1, y2);

	drawHLine(x1, y1, x2-x1, color);
	drawHLine(x1, y2, x2-x1, color);
	drawVLine(x1, y1, y2-y1, color);
	drawVLine(x2, y1, y2-y1, color);
}

void drawRoundRect(int x1, int y1, int x2, int y2, int color)
{
  if (x1>x2) swap(x1, x2);
  if (y1>y2) swap(y1, y2);

	if ((x2-x1)>4 && (y2-y1)>4) {
		drawPixel(x1+1,y1+1, color);
		drawPixel(x2-1,y1+1, color);
		drawPixel(x1+1,y2-1, color);
		drawPixel(x2-1,y2-1, color);
		drawHLine(x1+2, y1, x2-x1-4, color);
		drawHLine(x1+2, y2, x2-x1-4, color);
		drawVLine(x1, y1+2, y2-y1-4, color);
		drawVLine(x2, y1+2, y2-y1-4, color);
	}
}

void fillRect(int x1, int y1, int x2, int y2, int color)
{
	if (x1>x2) swap(x1, x2);
	if (y1>y2) swap(y1, y2);
	
	setxy(x1, x2, y1, y2);
	fastfill(color, ((long)(x2-x1)+1) * ((long)(y2-y1)+1) );
}

void fillRoundRect(int x1, int y1, int x2, int y2, int color)
{
	if (x1>x2) swap(x1, x2);
	if (y1>y2) swap(y1, y2);

	if ((x2-x1)>4 && (y2-y1)>4) {
		for (int i=0; i<((y2-y1)/2)+1; i++) {
			switch(i) {
			case 0:
				drawHLine(x1+2, y1+i, x2-x1-4, color);
				drawHLine(x1+2, y2-i, x2-x1-4, color);
				break;
			case 1:
				drawHLine(x1+1, y1+i, x2-x1-2, color);
				drawHLine(x1+1, y2-i, x2-x1-2, color);
				break;
			default:
				drawHLine(x1, y1+i, x2-x1, color);
				drawHLine(x1, y2-i, x2-x1, color);
			}
		}
	}
}

void drawCircleSegment(int xc, int yc, int x, int y, int color)
{
    drawPixel(xc+x, yc+y, color);
    drawPixel(xc-x, yc+y, color);
    drawPixel(xc+x, yc-y, color);
    drawPixel(xc-x, yc-y, color);
    drawPixel(xc+y, yc+x, color);
    drawPixel(xc-y, yc+x, color);
    drawPixel(xc+y, yc-x, color);
    drawPixel(xc-y, yc-x, color);
}

void drawCircle(int xc, int yc, int r, int color)
{
    int x = 0, y = r;
    int d = 3 - 2 * r;

    drawCircleSegment(xc, yc, x, y, color);
    while (y >= x) {
        // for each pixel we will
        // draw all eight pixels
         
        x++;
 
        // check for decision parameter
        // and correspondingly
        // update d, x, y
        if (d > 0)
        {
            y--;
            d = d + 4 * (x - y) + 10;
        }
        else
            d = d + 4 * x + 6;
        drawCircleSegment(xc, yc, x, y, color);
    }
}

void drawButton(BUTTON *b)
{
  int x0 = b->x0;
  int y0 = b->y0;
  int x1 = b->x1;
  int y1 = b->y1;

  if (!b->press)  
    fillRoundRect(x0, y0, x1, y1, b->color);
  drawRoundRect(x0, y0, x1, y1, b->press ? CYAN : b->border);
  drawRoundRect(x0+1, y0+1, x1-1, y1-1, b->press ? CYAN : b->color);

  setFont(1);
  drawString(b->title, ( (x0+x1-16*strlen(b->title))/2 ), y1-25, 1, b->textcolor, b->color); //(y1+y0)/2-8
}


#ifdef __cplusplus
}
#endif

