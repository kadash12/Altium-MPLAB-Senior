/*
 * movement functions.
 */
#include "movement.h"
#include "lcd.h"

/*
 * The forward function.
 */
void forward(gostop){
    LR=stop;
    LF=gostop;
    RR=stop;
    RF=gostop;
}

/*
 * The backward function.
 */
void backward(){
    CCPR1H = 0x00;
    CCPR1L = 0x50;
    CCPR2H = 0x00;
    CCPR2L = 0x50;
    LF=stop;
    LR=go;
    RF=stop;
    RR=go;
    __delay_ms(110);
    LF=stop;
    LR=stop;
    RF=stop;
    RR=stop;
}

/*
 * The right turn function.
 */
void rturn(){
   //Static
    CCPR1H = 0x00;
    CCPR1L = 0xF0;
    CCPR2H = 0x00;
    CCPR2L = 0xF0;
    LF=go;
    LR=stop;
    RF=stop;
    RR=go;
    __delay_ms(110);  
    LF=stop;
    LR=stop;
    RF=stop;
    RR=stop; 
    CCPR1H = 0x00;
    CCPR1L = 0x50;
    CCPR2H = 0x00;
    CCPR2L = 0x50;
}

/*
 * The left turn function.
 */
void lturn(){
    //Static
    CCPR1H = 0x00;
    CCPR1L = 0xF0;
    CCPR2H = 0x00;
    CCPR2L = 0xF0;
    LF=stop;
    LR=go;
    RF=go;
    RR=stop;
    __delay_ms(110);  
    LF=stop;
    LR=stop;
    RF=stop;
    RR=stop;    
    CCPR1H = 0x00;
    CCPR1L = 0x50;
    CCPR2H = 0x00;
    CCPR2L = 0x50;
}

/*
 * Reverse left 180
 */
void reversel(){
    CCPR1H = 0x00;
    CCPR1L = 0xF0;
    CCPR2H = 0x00;
    CCPR2L = 0xF0;
    LF=stop;
    LR=go;
    RF=go;
    RR=stop;
    __delay_ms(225);  
    LF=stop;
    LR=stop;
    RF=stop;
    RR=stop;  
    CCPR1H = 0x00;
    CCPR1L = 0x50;
    CCPR2H = 0x00;
    CCPR2L = 0x50;
}

/*
 * Reverse right 180
 */
void reverser(){
    CCPR1H = 0x00;
    CCPR1L = 0xF0;
    CCPR2H = 0x00;
    CCPR2L = 0xF0;
    LF=go;
    LR=stop;
    RF=stop;
    RR=go;
    __delay_ms(225);  
    LF=stop;
    LR=stop;
    RF=stop;
    RR=stop;  
    CCPR1H = 0x00;
    CCPR1L = 0x50;
    CCPR2H = 0x00;
    CCPR2L = 0x50;

}