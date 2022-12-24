/*
 * File:   main.c
 * Author: hiro
 *
 * Created on 2022/10/25, 18:29
 */

// PIC16F1459 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover Mode (Internal/External Switchover Mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config CPUDIV = CLKDIV6 // CPU System Clock Selection Bit (CPU system clock divided by 6)
#pragma config USBLSCLK = 48MHz // USB Low SPeed Clock Selection bit (System clock expects 48 MHz, FS/LS USB CLKENs divide-by is set to 8.)
#pragma config PLLMULT = 3x     // PLL Multipler Selection Bit (3x Output Frequency Selected)
#pragma config PLLEN = ENABLED  // PLL Enable Bit (3x or 4x PLL Enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = ON        // Low-Voltage Programming Enable

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

#define _XTAL_FREQ 1000000

//定義
//        ピン定義
#define A_COL_1 RC4
#define A_COL_2 RC3
#define A_COL_3 RC2
#define A_COL_4 RC1
#define A_COL_5 RC0
#define C_ROW_1 RC5
#define C_ROW_2 RB5
#define C_ROW_3 RA5
#define C_ROW_4 RC7
#define C_ROW_5 RB7
#define C_ROW_6 RC6
#define C_ROW_7 RA4
#define EN RA3
#define D0 RB6
#define D1 RB4
#define D2 RA1
#define D3 RA0

//変数定義
char anode[7]; //        表示バッファ（配列, 5bitx7）
int value_LED;

//表示パターンのデータ
char font[16][7] = {{0xe,0x11,0x11,0x11,0x11,0x11,0xe}, //0 
                    {0x4,0x4,0x4,0x4,0x4,0x4,0x4}, //1
                    {0xe,0x11,0x1,0xe,0x10,0x10,0x1f}, //2
                    {0xe,0x11,0x1,0x6,0x1,0x11,0xe}, //3
                    {0x2,0x12,0x12,0x1f,0x2,0x2,0x2}, //4
                    {0x1f,0x10,0x10,0x1e,0x1,0x11,0xe}, //5
                    {0xe,0x10,0x10,0x1e,0x11,0x11,0xe}, //6
                    {0x1f,0x1,0x1,0x2,0x2,0x2,0x2}, //7
                    {0xe,0x11,0x11,0xe,0x11,0x11,0xe}, //8
                    {0xe,0x11,0x11,0xf,0x1,0x1,0xe}, //9
                    {0,0,0,0,0,0,0}, //A
                    {0,0,0,0,0,0,0}, //B
                    {0,0,0,0,0,0,0}, //C
                    {0,0,0,0,0,0,0}, //D
                    {0,0,0,0,0,0,0}, //E
                    {0,0,0,0,0,0,0}}; //F
// 16進数表示をさせるときは、A-Fをこれと入れ替える。
// ただし、1010-1111の値を入力しても消灯できなくなる。
//                    {0x4,0xa,0x11,0x11,0x1f,0x11,0x11}, //A
//                    {0x1e,0x11,0x11,0x1e,0x11,0x11,0x1e}, //B
//                    {0xf,0x10,0x10,0x10,0x10,0x10,0xf}, //C
//                    {0x1c,0x12,0x11,0x11,0x11,0x12,0x1c}, //D
//                    {0x1f,0x10,0x10,0x1f,0x10,0x10,0x1f}, //E
//                    {0x1f,0x10,0x10,0x1f,0x10,0x10,0x10}}; //F

//関数プロトタイプ
void LED_on(int);
void LED_off(void);
int readio(void);

void main(void){
    // I/O 入出力設定と初期化
    SSPEN=0;
    PORTA = 0;
    TRISA = 0b00001011;
    ANSELA = 0;
    PORTB = 0;
    TRISB = 0b01010000;
    ANSELB = 0;
    PORTC = 0;
    TRISC = 0b00000000;
    ANSELC = 0;
    IOCAP = 0b00001011;
    IOCAN = 0b00001011;
    IOCBP = 0b01010000;
    IOCBN = 0b01010000;
    C_ROW_1 = 1;
    C_ROW_2 = 1;
    C_ROW_3 = 1;
    C_ROW_4 = 1;
    C_ROW_5 = 1;
    C_ROW_6 = 1;
    C_ROW_7 = 1;
    IOCIE = 1;
    GIE = 0;1;

    //ここでループ
    while(1){
    //　IOCで割込み発生
    //  割込みが発生したら表示する値が切り替わる  
        value_LED = readio();
        LED_on(value_LED);
    }
    return;
}

void __interrupt() isr(void){
    if(IOCIF){
        value_LED = readio();
        IOCAF = 0;
        IOCBF = 0;
        LED_on(value_LED);
        __delay_ms(1000);
        LED_off();
        __delay_ms(1000);
        }
    return;
    }

int readio(void){
    int myvalue;
    myvalue = 8*D3 + 4*D2 + 2*D1 + D0;
    return myvalue;
}

void LED_on(int number){  //ダイナミック点灯してその状態を保持

    //        点灯させるアノードのパターンを読み込む
    for (int i = 1; i < 8 ; i++){
        anode[i] = font[number][i-1];
    }
    
    for(int i = 1; i < 8 ; i++){
        PORTC &= 0b11100000;
        PORTC |= anode[i];   //        点灯させるアノードをHIにする
        switch (i){         //        選択したカソードをLOにして点灯する
            case 1:
                C_ROW_1 = 0;
                break;
            case 2:
                C_ROW_2 = 0;
                break;
            case 3:
                C_ROW_3 = 0;
                break;
            case 4:
                C_ROW_4 = 0;
                break;
            case 5:
                C_ROW_5 = 0;
                break;
            case 6:
                C_ROW_6 = 0;
                break;
            case 7:
                C_ROW_7 = 0;
                break;
        }            

        switch (i){         //        選択したカソードをHIにして消灯する
            case 1:
                C_ROW_1 = 1;
                break;
            case 2:
                C_ROW_2 = 1;
                break;
            case 3:
                C_ROW_3 = 1;
                break;
            case 4:
                C_ROW_4 = 1;
                break;
            case 5:
                C_ROW_5 = 1;
                break;
            case 6:
                C_ROW_6 = 1;
                break;
            case 7:
                C_ROW_7 = 1;
                break;
            }            
        }
}

void LED_off(void){
    for(int i = 1; i < 8 ; i++){
        PORTC &= 0b11100000;
        switch (i){         //        選択したカソードをHIにして消灯する
            case 1:
                C_ROW_1 = 1;
                break;
            case 2:
                C_ROW_2 = 1;
                break;
            case 3:
                C_ROW_3 = 1;
                break;
            case 4:
                C_ROW_4 = 1;
                break;
            case 5:
                C_ROW_5 = 1;
                break;
            case 6:
                C_ROW_6 = 1;
                break;
            case 7:
                C_ROW_7 = 1;
                break;
        }            
    }
}       