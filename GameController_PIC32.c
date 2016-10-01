/*
 * File:   ECE 477 Spring 2015.4
 * Author: Team 5 (Di Tang, Yinuo Li, Chengzhang Zhong, Ji Ma)
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <p32xxxx.h>
#include <plib.h>	// Include the PIC32 Peripheral Library.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


#define FCY 72000000L
#define FPB 36000000L
#define PERIOD  3906
#define SYS_FREQ (40000000L)
#pragma config POSCMOD=XT, FNOSC=PRIPLL
#pragma config FPLLIDIV=DIV_2, FPLLMUL=MUL_18, FPLLODIV=DIV_1
#pragma config FPBDIV=DIV_2, FWDTEN=OFF, CP=OFF, BWP=OFF

#if defined (__32MX360F512L__) || (__32MX460F512L__) || (__32MX795F512L__) || (__32MX430F064L__) || (__32MX450F256L__) || (__32MX470F512L__)
// Configuration Bit settings
// SYSCLK = 80 MHz (8MHz Crystal / FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 10 MHz (SYSCLK / FPBDIV)
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// Other options are don't care

#endif

//Timing constants
#define PERIOD  3906
#define TIMERFREQ 10
#define TENFOLD 10
#define HUNDREDFOLD 100
#define THOUSANDFOLD 1000

//Port pin define
#define LED _RD1
#define LPB _RB5
#define RPB _RB10
#define LTG _RA0
#define RTG _RD4
#define GX _RB2
#define GY _RB1


//State constant define
    //--states(running & setting & waiting)
#define WAT W
#define SET S
#define RUN R
    //--control mode(normal control & climb control)
#define NORM N
#define CLIM C

    //--LCD NOKIA
#define SPI_CONF 0x8120 // SPI on, 8-bit master,CKE=1,CKP=0
#define SPI_BAUD 15 // clock divider Fpb/(2 * (15+1))
#define LCD_SCE _RD12//pin 3 on LCD
#define LCD_RESET _RD3 //pin 4 on LCD
#define LCD_DC _RD2 //pin 5 on LCD
#define LCD_SDIN _RG8 //pin 6 on LCD
#define LCD_SCLK _RG6 //pin 7 on LCD
#define LCD_C 0
#define LCD_D 1
#define LCD_X 84
#define LCD_Y 48

static const char ASCII[][5] = {
  {0x00, 0x00, 0x00, 0x00, 0x00} // 20
  ,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
  ,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
  ,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
  ,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
  ,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
  ,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
  ,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
  ,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
  ,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
  ,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
  ,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
  ,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
  ,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
  ,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
  ,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
  ,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
  ,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
  ,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
  ,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
  ,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
  ,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
  ,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
  ,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
  ,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
  ,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
  ,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
  ,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
  ,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
  ,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
  ,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
  ,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
  ,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
  ,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
  ,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
  ,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
  ,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
  ,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
  ,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
  ,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
  ,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
  ,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
  ,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
  ,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
  ,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
  ,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
  ,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
  ,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
  ,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
  ,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
  ,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
  ,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
  ,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
  ,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
  ,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
  ,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
  ,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
  ,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
  ,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
  ,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
  ,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c \
  ,{}
  ,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
  ,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
  ,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
  ,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
  ,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
  ,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
  ,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
  ,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
  ,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
  ,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
  ,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
  ,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
  ,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
  ,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
  ,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
  ,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
  ,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
  ,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
  ,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
  ,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
  ,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
  ,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
  ,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
  ,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
  ,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
  ,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
  ,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
  ,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
  ,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
  ,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
  ,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
  ,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
  ,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
  ,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ~
  ,{0x78, 0x46, 0x41, 0x46, 0x78} // 7f DEL
};
const unsigned char meyer [] = {
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18,
0x1C, 0x1C, 0x0C, 0x0E, 0x0C, 0x0E, 0x1A, 0x1C, 0x1C, 0x1E, 0x1E, 0x1C, 0x9C, 0x18, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0xE0, 0xE0,
0x68, 0x74, 0x3C, 0xBC, 0x9E, 0x9E, 0x9F, 0xDF, 0xDF, 0xCF, 0xCF, 0xCF, 0x8F, 0xC7, 0x87, 0x97,
0x9F, 0x9F, 0xFF, 0xFF, 0xFF, 0x7F, 0x7F, 0xFF, 0xFF, 0x77, 0xBF, 0xFF, 0xFB, 0xFF, 0xFF, 0xFF,
0xFF, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xF8, 0x3E, 0x40, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x06, 0x06, 0x07, 0x03, 0x03, 0x05, 0x07, 0x07,
0x07, 0x03, 0x01, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x09, 0xF4, 0x86, 0x06, 0x43, 0xC5, 0x07, 0x23, 0x01, 0x01, 0x02, 0x03, 0x03, 0x03, 0x03,
0x01, 0x01, 0x01, 0x03, 0x07, 0x1F, 0x0F, 0x0F, 0x15, 0x00, 0x82, 0x83, 0x3F, 0xFF, 0xFB, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0x79, 0xE3, 0xFF, 0xEF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFE, 0xF8, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x88, 0xF1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x80, 0x43, 0x77, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xF0, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x60, 0x20, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x08, 0x47, 0xEE, 0xD8, 0xF0,
0x20, 0x60, 0xC0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0xC0, 0x80, 0xBB, 0x81, 0xF1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0xFC,
0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x30, 0x18, 0x04, 0x02, 0x01, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x04, 0x0C, 0x1C, 0x1C, 0x0C, 0x0E, 0x07, 0x03, 0x0F,
0x07, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x04, 0x00, 0x00, 0x00, 0x00, 0x80,
0x00, 0x00, 0x18, 0xFD, 0xFC, 0xFF, 0x9F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xF8, 0xE0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x10, 0x10, 0x10, 0x10, 0x70, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x40, 0x40, 0x40, 0x60,
0x60, 0x60, 0xE0, 0xE0, 0x60, 0x60, 0x60, 0x60, 0x70, 0xF0, 0xF8, 0xFC, 0x76, 0x3D, 0x3E, 0x24,
0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xC7, 0xFF, 0xB6, 0x23, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0x7F, 0xFF, 0xFF, 0x8F, 0x0F, 0x3F, 0x7F, 0xFF,
};

const unsigned char carlogo [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0xE0, 0xF4, 0xF4, 0xFE, 0xFE, 0xDE, 0xFC,
0xFC, 0xFC, 0xF8, 0xF8, 0xE8, 0xA0, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0,
0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xAD, 0x59, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x80, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFE, 0xFF, 0xBB, 0x3B, 0x3F, 0x7F, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF3, 0xC7, 0x87, 0x8A, 0x02, 0x02, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFC, 0xF6, 0x99, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x20, 0xE0, 0xE0, 0x20, 0x20, 0x20, 0x20, 0xA0, 0xA0, 0xA0, 0xA0, 0x20, 0x20, 0xA0, 0xA0, 0xA0,
0xA0, 0xA0, 0xA0, 0x20, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0x20, 0x20, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0,
0x20, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0x00, 0xA0, 0xA0, 0xA1, 0x00, 0x03, 0x07, 0x1E, 0x3C,
0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x05, 0x0F, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFE, 0xFE, 0xFC, 0xF0, 0xE0, 0xE0, 0xF0, 0xF8, 0x37, 0x1F, 0x07, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x30, 0x3F, 0x3F, 0x33, 0x03, 0x00, 0x0F, 0x3F, 0x22, 0x22, 0x33, 0x13,
0x00, 0x00, 0x3F, 0x3F, 0x01, 0x03, 0x03, 0x00, 0x30, 0x3F, 0x31, 0x01, 0x03, 0x03, 0x00, 0x1D,
0x1C, 0x36, 0x3F, 0x3F, 0x20, 0x00, 0x3F, 0x3F, 0x21, 0x03, 0x03, 0x00, 0x20, 0x3F, 0x3F, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x23, 0x67,
0x7F, 0x7F, 0x3F, 0x7F, 0xFF, 0xFF, 0x0F, 0x07, 0x07, 0x01, 0x00, 0x0F, 0x03, 0x01, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x60, 0x70, 0x78, 0x3C, 0x0C, 0x07, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

void outchar(char inputdata);//sending out charactor through UART2 TX
char inchar(void);//getting charactor from UART2 RX
void initADC(void);//initialize analog input ports
int readADC( int ch);//read ch's analog input
void AnalogInput(char enable);//read 4 linearized analog inputs from the joystick
char LineConv(int num,int offset);//linearize and convert a integer input to char
void DigitalInput(char enable);//debouncing all digital input and set the flags
char JSstate(char js);//detect joystick digital states
void CommandSending(char enable);
void BatDisp(void);

//Global variables
    //--timing
int TimeTrack = 0;//frequency divider counter
char TenthFlag = 0;//set every 0.1 second
char MiliFlag = 0;//set every 0.1 second
   //--car control variables
unsigned char LJSX;//left joystick left/right direction
unsigned char LJSY;//left joystick left/right direction
unsigned char RJSX;//right joystick left/right direction
unsigned char RJSY;//right joystick left/right direction
unsigned char Digi;//receive digital bits
unsigned int Gx;//gravity sensor X axis
unsigned int Gy;//gravity sensor Y axis

    //--Digital debouncing variables and flags
char prevlpb = 1;//previous left pushbuttom state(0,1)
char prevrpb = 1;//previous right pushbuttom state(0,1)
char prevltg = 1;//previous left trigger state(0,1)
char prevrtg = 1;//previous right trigger state(0,1)
char prevljs = 0;//previous left joystick state(0,U,D,L,R)
char prevrjs = 0;//previous right joystick state(0,U,D,L,R)
char LPBF = 0;//left pushbotton flag(0,1)
char RPBF = 0;//right pushbotton flag(0,1)
char LTGF = 0;//left trigger flag(0,1)
char RTGF = 0;//right trigger flag(0,1)
char LJSF = 0;//left joystick flag(0,U,D,L,R)
char RJSF = 0;//right joystick flag(0,U,D,L,R)
char FAU;//front arm up     Digi 0xccc10000 16
char FAD;//front arm down   Digi 0xccc01000 8
char BAU;//back arm up      Digi 0xccc00100 4
char BAD;//back arm down    Digi 0xccc00010 2
char ALM;//alarm trigger    Digi 0xccc00001 1
char GUN;//gun trigger      Digi 0xccc00001 1 (substituting the alarm if neccessary)

//--State variables & enables
            //NOTE: States are the "pages" user can see.
            //          SET-set parameters, manually calibrate battery life, or check connections;
            //          RUN-control the car;
            //          WAT-trap in when timeout; wait until any input occur
            //      Modes are the way the car responds to the user inputs
            //          NORM-normal way of operation;
            //          CLIM-climbing operation;
char SYSstate = 'W';//initially in waiting state
char CTLMode = 'N';//initially in normal control mode
char DIGenable = 'e';
char ANAenable = 'e';
char GUNenable = 0;
char COMenable = 0;
char LTtoggler = 0;
char str[3] = {'a','a','\0'};
int bl;
int i = 4;
//Receiced data
char bat;//battery life
char cmd;


char cc = 'a';

//====================================================start of the program==========
int main() {
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    mPORTESetPinsDigitalOut( BIT_7 | BIT_6 | BIT_5 | BIT_5 | BIT_4 | \
                                                     BIT_3 | BIT_2 | BIT_1 | BIT_0 );
    mPORTDSetPinsDigitalOut( BIT_1 );
    mJTAGPortEnable(DEBUG_JTAGPORT_OFF);
    mPORTASetPinsDigitalIn( BIT_7 | BIT_6 | BIT_5 | BIT_5 | BIT_4 | \
                                                     BIT_3 | BIT_2 | BIT_1 | BIT_0 );
    UARTConfigure(UART1,0);
    UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
    UARTSetDataRate(UART1,4000000L,9600);
    init_Timer1();
    initADC();
    DIGenable = 0;
    COMenable = 0;
    ANAenable = 0;
    SPI2CON = SPI_CONF; // select mode and enable SPI2
    SPI2BRG = SPI_BAUD; // select clock speed
    LCDInit();
    LCDClear();
    //LCDBitmap(carlogo);
    DIGenable = 'e';
    COMenable = 'e';
    ANAenable = 'e';
    LED = 0;
    LPBF = 0;
    RPBF = 0;
    LJSF = 0;
    RJSF = 0;
//=====================================================end of initialization=========
    while(1){
        //============WAT STATE BELOW==============================
        if (SYSstate == 'W'){
            gotoXY(0, 11);
            LCDString("Push any button to start!");
        }
        while(SYSstate == 'W'){
            LED = 0;
            COMenable = 0;
            if(!prevlpb||!prevrpb){
                SYSstate = 'S';
                LJSF = 0;
                RJSF = 0;
                LPBF = 0;
                RPBF = 0;
            }
        }

        if (SYSstate == 'S'){

            LED = 1;//no matter when, leaving W indicates awakening, thus LED on
        }
        while(SYSstate == 'S'){
            if(LJSF == 'L'){
                LJSF = 0;
                LED = 1;
            }
            if(LJSF == 'R'){
                LJSF = 0;
                LED = 0;
            }


            if(LPBF||RPBF){
                SYSstate = 'R';
                LJSF = 0;
                RJSF = 0;
                LPBF = 0;
                RPBF = 0;
            }
            //str = "aa";
        }
        if(SYSstate = 'R'){
            LCDClear();
            LCDBitmap(carlogo);
            gotoXY(0, 22);
            LCDString("Running Mode");
            COMenable = 'e';
        }
        while(SYSstate = 'R'){
            if (LPBF){
                LPBF = 0;
                LTtoggler++;
                CTLMode = LTtoggler%2 ? 'C' : 'N';
            }
//            if (LTGF){
//                LTGF = 0;
//                //cmd = 0;
//               // while(!UARTReceivedDataIsAvailable(UART1));
//               // cmd = inchar()&0x0f;
//                LCDClear();
//                LED = 1;
//                LCDCharacter(cc);
//
//                delay(10);
//                /*
//                for(bl = 0; bl<cmd;bl++){
//                    LCDString("x");
//                    delay(10);
//                }*/
//            }


        }

    }//while(1)
//==================================================================end of main loop====
}//main

// send one byte of data and receive one back at the same time
int writeSPI2( int i)
{
    SPI2BUF = i; // write to buffer for TX
    while( !SPI2STATbits.SPIRBF); // wait for transfer complete
    return SPI2BUF; // read the received value
}//writeSPI2

void LCDInit(void){
    TRISD = 0x0000; // set RA1 to the only input pin
    LCD_RESET = 0;
    LCD_RESET = 1;
    LCDWrite(LCD_C, 0x21); //Tell LCD that extended commands follow
    LCDWrite(LCD_C, 0xBF); //Set LCD Vop (Contrast): Try 0xB1(good @ 3.3V) or 0xBF if your display is too dark
    LCDWrite(LCD_C, 0x04); //Set Temp coefficent
    LCDWrite(LCD_C, 0x14); //LCD bias mode 1:48: Try 0x13 or 0x14

    LCDWrite(LCD_C, 0x20); //We must send 0x20 before modifying the display control mode
    LCDWrite(LCD_C, 0x0C); //Set display control, normal mode. 0x0D for inverse
}

void LCDWrite(char dc, char data){
    LCD_DC = dc;
    LCD_SCE = 0;
    writeSPI2( data);
    LCD_SCE = 1;
}

void LCDBitmap(char my_array[]){
    int index;
    int i;
    for (index = 0 ; index < (LCD_X * LCD_Y / 8) ; index++)
    LCDWrite(LCD_D, my_array[index]);
    delay(2);
}

void LCDClear(void) {
    int index;
    for (index = 0 ; index < (LCD_X * LCD_Y / 8) ; index++)
    LCDWrite(LCD_D, 0x00);
    gotoXY(0, 0); //After we clear the display, return to the home position
}

void gotoXY(int x, int y) {
    LCDWrite(0, 0x80 | x);  // Column
    LCDWrite(0, 0x40 | y);  // Row
}

void LCDString(char *characters) {
    while (*characters) LCDCharacter(*characters++);
}

void LCDCharacter(char character) {
    int index;
    LCDWrite(LCD_D, 0x00); //Blank vertical line padding
    for (index = 0 ; index < 5 ; index++) LCDWrite(LCD_D, ASCII[character - 0x20][index]);
    LCDWrite(LCD_D, 0x00); //Blank vertical line padding
}

void __ISR(0 , ipl1) Timer1Handler(void)
{
    mT1ClearIntFlag();// 1 times the basic frequency
    TimeTrack = TimeTrack? TimeTrack - 1: 999;
    TenthFlag = !(TimeTrack%(10000/TIMERFREQ));
    MiliFlag =  !(TimeTrack%(100/TIMERFREQ));


    if (!(TimeTrack % TENFOLD)){// 1/10 times the basic frequency

    }
    if (!(TimeTrack % HUNDREDFOLD)){// 1/100 times the basic frequency
        DigitalInput(DIGenable);
    }
    if (!(TimeTrack % THOUSANDFOLD)){// 1/1000 times the basic frequency
        AnalogInput(ANAenable);
        CommandSending(COMenable);

    }
}


void init_Timer1(void){
    //DDPCONbits.JTAGEN = 0;
    PR1 = 4000/TIMERFREQ;
    T1CON = 0x8000;
    // 2.3 init interrupts
    mT1SetIntPriority(1);
    mT1ClearIntFlag();
    INTEnableSystemSingleVectoredInt();
    mT1IntEnable(1);
}

void outchar(char a){
    while(!UARTTransmitterIsReady(UART1));
    UARTSendDataByte(UART1, a);
    while(!UARTTransmissionHasCompleted(UART1));
}

char inchar(void){
    while(!UARTReceivedDataIsAvailable(UART1));
    return UARTGetDataByte(UART1);
}

void initADC(void)
{
    AD1PCFG = 0; // select analog input pins
    AD1PCFG |= 0x0420; //0000 0100 0010 0000 pin 5 and 10 digital
    AD1CON1 = 0; // manual conversion sequence control
    AD1CSSL = 0; // no scanning required
    AD1CON2 = 0; // use MUXA, AVss/AVdd used as Vref+/-
    AD1CON3=0x1F02; // Tad=2+1) x 2 x Tpb=6x27 ns>75 ns
    AD1CON1bits.ADON=1; // turn on the ADC
} //initADC

int readADC( int ch)
{
    AD1CHSbits.CH0SA = ch; // 1. select analog input
    AD1CON1bits.SAMP = 1; // 2. start sampling
    T1CON = 0x8000; TMR1 = 0; // 3. wait for sampling time
    while (TMR1 < 100); //
    AD1CON1bits.SAMP = 0; // 4. start the conversion
    while (!AD1CON1bits.DONE); // 5. wait conversion complete
    return ADC1BUF0; // 6. read result
}

void AnalogInput(char enabled){
    //enable check
    if (!enabled) return;
    //Linearize the analog inputs
    LJSX =  LineConv(readADC(3),50)>>3;
    LJSY =  LineConv(readADC(4),50)>>3;
    RJSX =  LineConv(readADC(9),50)>>3;
    RJSY =  LineConv(readADC(8),50)>>3;
    //Raw readout of the gravity sensor
    Gx =  readADC(2);
    Gy =  readADC(1);
    if(CTLMode == 'N') return;
    //Linearize the gravity sensor inputs within a valid range(0~31), then mapping to a char size
    Gx = (((Gx<451)?(451):((Gx>578)?(578):(Gx)))-451)>>2;
    Gy = (((Gy<451)?(451):((Gy>578)?(578):(Gy)))-451)>>2;
}

char LineConv(int num,int offset){
    if (num > 1023 - offset){
        num = num + 255 - 1023;
    }else if (num < offset){
    }else{
        num = (num - offset) * (256 - 2 * offset) / (1024 - 2 * offset) + offset;
    }
}

void DigitalInput(char enabled){
    if (!enabled) return;
    int buff;
    LPBF = !LPB && prevlpb;
    RPBF = !RPB && prevrpb;
    LTGF = !LTG && prevltg;
    RTGF = !RTG && prevrtg;
    char LJSFcur = JSstate('L');//detect current state
    char RJSFcur = JSstate('R');//detect current state
    //LJSF = (!prevljs && (prevljs != LJSFcur)) ?  LJSFcur : LJSF;
    //RJSF = (!prevrjs && (prevrjs != RJSFcur)) ?  RJSFcur : RJSF;
    LJSF = (!prevljs && LJSFcur) ?  LJSFcur : LJSF;
    RJSF = (!prevrjs && RJSFcur) ?  RJSFcur : RJSF;

    prevlpb = LPB;
    prevrpb = RPB;
    prevltg = LTG;
    prevrtg = RTG;
    prevljs = JSstate('L');
    prevrjs = JSstate('R');
    //Encoding "Digi"
    if(CTLMode == 'N'){
        GUN = !RPB;
        ALM = !RPB;
        Digi = 0xe0 + GUN|ALM;
    }
    if(CTLMode == 'C'){
        FAU = prevljs == 'U';
        FAD = prevljs == 'D';
        BAU = prevrjs == 'U';
        BAD = prevrjs == 'D';
        Digi = 0xe0 + FAU*16 + FAD*8 + BAU*4 + BAD*2 + 0;
    }
}

char JSstate(char js){
    int x,y,s;
    int sensitivity = 15;
    if (js == 'L'){
        x = LJSX;
        y = LJSY & 0b00011111;
    }
    else if (js == 'R'){
        x = RJSX & 0b00011111;
        y = RJSY & 0b00011111;
    }
    s = (x-sensitivity)*(x-sensitivity)+(y-sensitivity)*(y-sensitivity);
    if (s < 100) return 0;
    else if (x>y && x+y>30) return 'L';
    else if (x<y && x+y<30) return 'R';
    else if (x<y && x+y>30) return 'U';
    else return 'D';
}

void CommandSending(char enabled){
    //enable check

    //Code Lookup
    //    function      N mode  C mode
    //000   LX          L/R     F arms dif
    //001   LY          F/B     F arms u/d
    //010   RX        CCW/CW    R arms dif
    //011   RY          U/D     R arms u/d
    //100   State
    //101   dog         feed    feed
    //110
    //111   DigCMD        alarm/gun/user inputs


    if (!enabled) return;
    outchar(Digi);

    //Rule: outchar(ch) to send command, ch should be 0~31 char
    //      +0  left/right shift
    //      +32 forward/backward shift
    //      +64 CCW/CW rotate
    //      +96 up/down tilt
    if(CTLMode == 'N'){
        outchar(0x80);//1000 0000 Normal state
        outchar(LJSX+0);//Send L/R shift speed
        outchar(LJSY+32);//Send F/B shift speed
        outchar(RJSX+64);//Send CCW/CW rotate speed
        outchar(RJSY+96);//Send U/D tilt speed
    }
    if(CTLMode == 'C'){
        outchar(0x81);//1000 0001 Climb state
        if (Gx<20 && Gx>11) Gx = 15;
        if (Gy<20 && Gy>11) Gy = 15;
        outchar(Gy+0);//LR
        outchar(Gx+32);//FB
        outchar(RJSX+64);//shoudl we enable rotation in this state?
        outchar(RJSY+96);//arm
    }
    outchar(0xa0);//feed the dog (101ddddd)
}

void BatDisp(void){
    LCDClear();
    char a = 123;
    gotoXY(0, 22);
    LCDCharacter(48+(a/100)%10);
    LCDCharacter(48+(a/10)%10);
    LCDCharacter(48+a%10);
}

void delay(int n){
    int i;
    for(i = 0;i<1024*n;i++);
}