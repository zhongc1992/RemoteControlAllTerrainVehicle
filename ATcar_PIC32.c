/*
 * File:   uart_main.c
 * Author: Team 5 (Di Tang, Yinuo Li, Chengzhang Zhong, Ji Ma)
 *
 * Created on February 23, 2015, 8:01 PM
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

#define PERIOD  3906
#define TIMERFREQ 20
#define TENFOLD 10
#define HUNDREDFOLD 100
#define THOUSANDFOLD 1000

void drive(int y, int x, int r);//translate direction and rotation info into motor duty cycles
void SpeedControl(char num);// Realize the psudo-PWM duty cycle outputs
void outchar(char inputdata);//sending command through UART2 TX
char inchar(void);//getting command from UART2 RX
void initADC(void);//initialize the ADC module
int readADC( int ch);//read integer value of the ADC conversion from channal ch
void SelfBalance(char enabled);
void FuelGauge(void);

//Global variables
    //--timing
int TimeTrack = 0;//frequency divider counter
char TenthFlag = 0;//set every 0.1 second
char MiliFlag = 0;//set every 0.1 second
char Wdog = 0;//watch dog
    //--motor duty cycle control variables;
char dty0 = 0x00;//Mecanum Wheel duty cycle BR-F
char dty1 = 0x00;//Mecanum Wheel duty cycle BR-B
char dty2 = 0x00;//Mecanum Wheel duty cycle BL-F
char dty3 = 0x00;//Mecanum Wheel duty cycle BL-B
char dty4 = 0x00;//Mecanum Wheel duty cycle FR-F
char dty5 = 0x00;//Mecanum Wheel duty cycle FR-B
char dty6 = 0x00;//Mecanum Wheel duty cycle FL-F
char dty7 = 0x00;//Mecanum Wheel duty cycle FL-B

char Adty0 = 0x00;//Arm duty cycle FL-F
char Adty1 = 0x00;//Arm duty cycle FL-B
char Adty2 = 0x00;//Arm duty cycle FR-F
char Adty3 = 0x00;//Arm duty cycle FR-B
char Adty4 = 0x00;//Arm duty cycle BL-F`
char Adty5 = 0x00;//Arm duty cycle BL-B
char Adty6 = 0x00;//Arm duty cycle BR-F
char Adty7 = 0x00;//Arm duty cycle BR-B

char Tdty0 = 0x00;//Track duty cycle FL-F
char Tdty1 = 0x00;//Track duty cycle FL-B
char Tdty2 = 0x00;//Track duty cycle FR-F
char Tdty3 = 0x00;//Track duty cycle FR-B
char Tdty4 = 0x00;//Track duty cycle BL-F
char Tdty5 = 0x00;//Track duty cycle BL-B
char Tdty6 = 0x00;//Track duty cycle BR-F
char Tdty7 = 0x00;//Track duty cycle BR-B
char Gdty  = 0x00;//Gun trigger
char pcnt = 0x00;
char pmod = 0x00;
int delaycnt=0x0fff;
    //--speed control variables
unsigned char Xdir,Ydir,Rdir,Rspd,Digi;
/*left thumbstick, x direction, maybe left right;left thumssrick, y direction;right thumstick, rotation;right thumstick, arm;receive digital bits*/

    //--self-balance platform feedback loop variables
char Ref;//reference angle
char Fed;//feedback angle
char Ksys = 1;//error gain
char Ssys = 0;//sensityvity of the system
char Sout = 0;//system output angle command
int Scnt = 0;
int Sdty = 28;//servo duty cycle  21---28---36
int temp = 1350;
int MID = 512;

    //--state machine variables
char state = 0;
    //--communication
char cmd;

    //--fuel gauge variables
int vol1;
int vol2;
int voltage;
int current;
int power;
int energy = 0;
char bl;


int main() {
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    mPORTESetPinsDigitalOut( BIT_7 | BIT_6 | BIT_5 | BIT_5 | BIT_4 | \
                                                     BIT_3 | BIT_2 | BIT_1 | BIT_0 );
    mPORTCSetPinsDigitalOut(BIT_1);
    mPORTDSetPinsDigitalOut(BIT_12 | BIT_3 | BIT_2 | BIT_1 | BIT_0);
    mPORTGSetPinsDigitalOut(BIT_6 | BIT_7 | BIT_8 | BIT_9 | BIT_12 | BIT_13 | BIT_14 | BIT_15);
    mJTAGPortEnable(DEBUG_JTAGPORT_OFF);
    mPORTASetPinsDigitalOut(BIT_5 | BIT_4 | BIT_3 | BIT_2);
    UARTConfigure(UART1,0);
    UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
    UARTSetDataRate(UART1,4000000L,9600);
    init_Timer1();
    initADC();
    state = 'N';

    while(1){
        if(UARTReceivedDataIsAvailable(UART1))
        cmd = inchar();
        switch (cmd & 0b11100000){
            case (0b00000000):
              Xdir = cmd & 0b00011111;
              break;
            case (0b00100000):
              Ydir = cmd & 0b00011111;
              break;
            case (0b01000000):
              Rdir = cmd & 0b00011111;
              break;
            case (0b01100000):
              Rspd = cmd & 0b00011111;
              break;
            case (0b11100000):
              Digi = cmd;
              break;
            case (0b10100000):
              Wdog = 1;//feed the dog
              break;
            case (0b10000000):
              state = (cmd&0x01)?'C':'N';
              break;
        }//switch
        if(Xdir>12 && Xdir<18) Xdir = 15;
        if(Ydir>12 && Ydir<18) Ydir = 15;
        if(Rdir>12 && Rdir<18) Rdir = 15;
        drive(Xdir,Ydir,Rdir);
        Gdty = (Digi&0x01)?200:0;
        //outchar(bl);
    }//while(1)

}//main


void __ISR(0 , ipl1) Timer1Handler(void)
{
    mT1ClearIntFlag();// 1 times the basic frequency
    TimeTrack = TimeTrack? TimeTrack - 1: 999;
    SpeedControl('e');
    TenthFlag = !(TimeTrack%(10000/TIMERFREQ));
    MiliFlag =  !(TimeTrack%(100/TIMERFREQ));
    
    
    if (!(TimeTrack % TENFOLD)){// 1/10 times the basic frequency

    }// 1/10
    if (!(TimeTrack % HUNDREDFOLD)){// 1/100 times the basic frequency
    }// 1/100
    if (!(TimeTrack % THOUSANDFOLD)){// 1/1000 times the basic frequency
        if (!Wdog){
            dty0 = 0;dty1 = 0;dty2 = 0;dty3 = 0;
            dty4 = 0;dty5 = 0;dty6 = 0;dty7 = 0;// don't tu cao me!!!!!!!!!!!
            Adty0 = 0;Adty1 = 0;Adty2 = 0;Adty3 = 0;
            Adty4 = 0;Adty5 = 0;Adty6 = 0;Adty7 = 0;// don't tu cao me!!!!!!!!!!!
            Tdty0 = 0;Tdty1 = 0;Tdty2 = 0;Tdty3 = 0;
            Tdty4 = 0;Tdty5 = 0;Tdty6 = 0;Tdty7 = 0;// don't tu cao me!!!!!!!!!!!
            Gdty = 13;
        }
        else Wdog = 0;// starve the dog to die...........................hehe

        //Sdty = (31 - Rspd)*5/32*3+8;
        /*
        if (state == 'N'){
            if(Rspd<10) Sdty = Sdty>36?36:Sdty+1;
            if(Rspd>20) Sdty = Sdty<21?21:Sdty-1;
        }*/
        //Sdty = (energy>>8)*100/255;
        FuelGauge();
        SelfBalance(1);
    }// 1/1000
}



void init_Timer1(void){
    //DDPCONbits.JTAGEN = 0;
    PR1 = 4000/TIMERFREQ;
    T1CON = 0x8000;
    // 2.3 init interrupts
    mT1SetIntPriority( 1);
    mT1ClearIntFlag();
    INTEnableSystemSingleVectoredInt();
    mT1IntEnable( 1);
}

void drive(int y, int x, int r) {
    int X,Y,R;
    int FL,FR,BL,BR;

      X = 15 - x;
      Y = 15 - y;
      R = -15 + r;
      X *= 8;
      Y *= 8;
      R *= 8;

      FL = X - Y - R;
      FR = X + Y + R;
      BL = X + Y - R;
      BR = X - Y + R;

      if(FL > 0){
        dty0 = FL;
        dty1 = 0;
      }else{
        dty0 = 0;
        dty1 = -FL;
      }
      if(FR > 0){
        dty2 = FR;
        dty3 = 0;
      }else{
        dty2 = 0;
        dty3 = -FR;
      }
      if(BL > 0){
        dty4 = BL;
        dty5 = 0;
      }else{
        dty4 = 0;
        dty5 = -BL;
      }
      if(BR > 0){

        dty6 = BR;
        dty7 = 0;
      }else{
        dty6 = 0;
        dty7 = -BR;
      }
      delaycnt=0xAfff;
      //cmd = 0;


      if (state == 'C'){
          Adty0 = (Rspd>18)?(Rspd-15)*16-1:0;
          Adty1 = (Rspd<12)?(15-Rspd)*16-1:0;
          Adty2 = (Rspd>18)?(Rspd-15)*16-1:0;
          Adty3 = (Rspd<12)?(15-Rspd)*16-1:0;

          _RG6 = (Digi & 0x10)?1:0;
          _RG8 = (Digi & 0x10)?1:0;
          _RG7 = (Digi & 0x08)?1:0;
          _RG9 = (Digi & 0x08)?1:0;

          _RA5 = _RD0;
          _RA4 = _RD1;
          _RA3 = _RD2;
          _RA2 = _RD3;

          Tdty0 = (Ydir>18)?(Ydir-15)*16-1:0;
          Tdty1 = (Ydir<12)?(15-Ydir)*16-1:0;
          Tdty2 = (Ydir>18)?(Ydir-15)*16-1:0;
          Tdty3 = (Ydir<12)?(15-Ydir)*16-1:0;

      }

}

void SpeedControl(char enabled){
    if (!enabled) return;

    //regular 255 resolution PWM
    pcnt++;
    if (!pcnt){
        pmod = 0xff;//mecanum wheels motors(4)
        PORTG |= 0xf000;//front arms motors(2)
        //PORTG |= 0x03c0;//back arms motors(2)
        PORTD |= 0x000f;//front track motors(2)
        _RD12= 1;
    }
    if (pcnt == dty0) pmod &= ~(1<<0);//_RDG12 = 0;
    if (pcnt == dty1) pmod &= ~(1<<1);
    if (pcnt == dty2) pmod &= ~(1<<2);
    if (pcnt == dty3) pmod &= ~(1<<3);
    if (pcnt == dty4) pmod &= ~(1<<4);
    if (pcnt == dty5) pmod &= ~(1<<5);
    if (pcnt == dty6) pmod &= ~(1<<6);
    if (pcnt == dty7) pmod &= ~(1<<7);
    PORTE = pmod;

    if (pcnt == Adty0) _RG12 = 0;
    if (pcnt == Adty1) _RG13 = 0;
    if (pcnt == Adty2) _RG14 = 0;
    if (pcnt == Adty3) _RG15 = 0;

    if (pcnt == Gdty) _RD12 = 0;

    if (pcnt == Tdty0) _RD0 = 0;
    if (pcnt == Tdty1) _RD1 = 0;
    if (pcnt == Tdty2) _RD2 = 0;
    if (pcnt == Tdty3) _RD3 = 0;



    Scnt = (Scnt + 1)%180;
    if (!Scnt) _RC1 = 1;
    _RC1 =  Scnt == Sdty ? 0 : _RC1;
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
    AD1PCFG = 0; // select analog input pins ALL
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



void SelfBalance(char enabled){
    if (Digi&0x01) return;//don't tilt while shooting
    if(!enabled) return;
    int GY;//temp raw readouts
    if (state == 'N'){
        if(Rspd>26) MID += (MID>512+80)?0:5;
        if(Rspd<5) MID -= (MID<512-80)?0:5;
    }
    //else MID = 512;
    GY = readADC(0);
    GY = GY<448-20?448-20:(GY>575+20?575+20  :GY);
    Sdty += (GY > MID && Sdty < 35)?((GY-MID)/15*(GY-MID)/40):0;
    Sdty -= (GY < MID && Sdty > 21)?((MID-GY)/15*(MID-GY)/40):0;
    //Sdty = Sdty<21?21:(Sdty>35?35:Sdty);
}

void FuelGauge(void){
   // int voltage;
    vol1 = readADC(3)*12;//node 1 voltage devided(mV)
    vol2 = readADC(4)*12;//node 2 voltage devided(mV)
    voltage = vol1;//battery voltage(mV)
    current = 5 * (vol1 - vol2);//power resistor current(mA)
    power = voltage / 1000 * current;//Battery output power (mW)
    //energy = ;//mJ
    Table(voltage);
}

void Table(int v){
	if(v > 11500){
		bl = 10;
	}else if (v > 11500){
		bl = 9;
	}else if (v > 11450){
		bl = 8;
	}else if (v > 11400){
		bl = 7;
	}else if (v > 11350){
		bl = 6;
	}else if (v > 11300){
		bl = 5;
	}else if (v > 11250){
		bl = 4;
	}else if (v > 90000){
		bl = 3;
	}else if (v > 80000){
		bl = 2;
	}else {
		bl = 1;
	}
}