
/* 
 * File:   senior_main.c
 * Author: Johnny Li
 * Created on October 7, 2020
 * Description:
 * Senior design is to build a battery powered autonomous robotic car and use it
 * as a platform for testing proximity detection and collision avoidance. The 
 * system allows the car to detect obstructions in its paths and proceed to 
 * automatically adjust its movement to avoid a collision. This will be demo
 * through machine learning of the car solving a maze and then going back to the
 * maze and taking the more efficient path out. The car will use an IR beacon and
 * ultrasonic object detection sensor mounted on a servo to sense 180° of its 
 * surrounding for obstacles in its path and will produce various warning, both 
 * visually and audial, if an object is within its comfort proximity for the 
 * user?s benefit. An LCD mounted on the car will display detected distance, 
 * time, and obstructions from last scan.
 */

//Includes files
#include "adc.h"
#include "lcd.h"
#include "i2c.h"
#include "timer.h"

//Configuration
#pragma config WDTE = OFF   //Disable watch dog timer
#pragma config LVP = ON      //Enable low voltage programming mode
#define Trigger RB1 //34 is Trigger
#define Echo RB2//35 is Echo 

//Global Variable
unsigned long value0 = 0;   //Where to store the ADC result

/*
 * Setup timer1
 */
void t1_init(){
     //Timer setup
    OSCCON1 = 0x70;
    TMR1 = 0;   //Initialize to 0
    //T1CKPS0 = 0;
    //T1CKPS1 = 0;
    T1CON = 0x20;
   // T1CONbits.NOT_SYNC = 0;
   // T1CONbits.RD16 = 1;
    TMR1CLKbits.CS=2;
    TMR1IF=0;
}

/*
 * Get distance from ultrasound.
 */
int ultra_dist(){
    //Ultrasonic
    TMR1 =0; //clear the timer bits

    Trigger = 1;       //Set trigger
    __delay_us(10);           
    Trigger = 0;       //Turn off trigger

    while (!Echo);    //Wait for echo
    TMR1ON = 1;     //Turn on timer
    while (Echo);    //Get echo
    TMR1ON = 0;     //Turn off timer

    //Calculate distance
    int time_taken = (TMR1L | (TMR1H<<8)); 
    int distance = (time_taken/58.82)+1;    //In cm

    char d1 = (distance/100)%10;
    char d2 = (distance/10)%10;
    char d3 = (distance/1)%10;

    char d[1]; //Distance Storage array
    //Clear array
    for(int i=0; i<1; i++){
        d[i]=0;
    }

    lcd_char('D');
    lcd_char(':');
    lcd_char(' ');

    sprintf(d, "%u", d1);    //Convert to char
    char dd = d[0];
    lcd_char(dd);
    sprintf(d, "%u", d2);    //Convert to char
    dd = d[0];
    lcd_char(dd);
    sprintf(d, "%u", d3);    //Convert to char
    dd = d[0];
    lcd_char(dd);
    lcd_char('c');
    lcd_char('m');
    
    return distance;
}

/*
 *
 */
void main() {
    //Initialization
    adc_init();     //Initialized ADC ports
    lcd_init();     //Initialize LCD screen, note this takes care of PORTC I/0 direction  
    i2c_init();     //Initialized i2c
    rtc_init();     //Initialized rtc
    t1_init();      //Initialized timer1
    
    //IR setup
    TRISAbits.TRISA1 = 0;   //IR LED output signal
    
    //Ultrasonic setup
    TRISBbits.TRISB0 = 1;   //Define the RB0 pin as input to use as interrupt pin
    TRISBbits.TRISB1 = 0;   //Trigger pin of US sensor is sent as output pin
    TRISBbits.TRISB2 = 1;   //Echo pin of US sensor is set as input pin  
    TRISBbits.TRISB3 = 0;   //RB3 is output pin for LED
    ANSELB = 0x0;
    
    //RTC setup
    TRISCbits.TRISC0 = 0;   //Configure PORTC pin 0 as output
    PORTCbits.RC0 = 0;  //Clear and enable
    TRISCbits.TRISC1 = 0;   //Configure PORTC pin 1 as output
    PORTCbits.RC1 = 0;  //Clear and enable
    TRISCbits.TRISC2 = 1;   //Configure PORTC pin 2 as input
    PORTCbits.RC2 = 0;  //Clear and enable
    TRISCbits.TRISC5 = 0;   //Configure PORTC pin 5 as output
    PORTCbits.RC5 = 0;  //Clear and enable
    
    //Infinite loop to output the time. 
    while(1){
        lcd_command(0x02);  //Clear Home
        //Display 'time'
        lcd_char('T');
        lcd_char(':');
        lcd_char(' ');
        
        char array[2]; //Time Storage array
        //Clear array
        for(int i=0; i<2; i++){
            array[i]=0;
        }
        //Print format HH/MM/SS
        //Covert to char array
        sprintf(array, "%u", get_hours());
        char temp1 = array[0];   //Temporary storage
        char temp2 = array[1];
        if((temp1 != 0) && (temp2 != 0)){  //Check if not null
            lcd_char(temp1);
            lcd_char(temp2);
        }
        else if((temp1 != 0) && (temp2 == 0)){
            lcd_char('0');
            lcd_char(temp1); //Print digit
        }
        else{
            lcd_char('0');
            lcd_char('0');
        }
        lcd_char(':');
        //Covert to char array
        sprintf(array, "%u", get_minutes());
        temp1 = array[0];   //Temporary storage
        temp2 = array[1];
        if((temp1 != 0) && (temp2 != 0)){  //Check if not null
            lcd_char(temp1);
            lcd_char(temp2);
        }
        else if((temp1 != 0) && (temp2 == 0)){
            lcd_char('0');
            lcd_char(temp1); //Print digit
        }
        else{
            lcd_char('0');
            lcd_char('0');
        }
        lcd_char(':');
        //Covert to char array
        sprintf(array, "%u", get_seconds());
        temp1 = array[0];   //Temporary storage
        temp2 = array[1];
        if((temp1 != 0) && (temp2 != 0)){  //Check if not null
            lcd_char(temp1);
            lcd_char(temp2);
        }
        else if((temp1 != 0) && (temp2 == 0)){
            lcd_char('0');
            lcd_char(temp1); //Print digit
        }
        else{
            lcd_char('0');
            lcd_char('0');
        }
        lcd_command(0xC0);  //Next row
        
        //Check if close to IR beacon
        value0 = adcNum0(); //Get ADC value from IR detector
     
        if(value0<921){
            PORTAbits.RA1 = 0;  //IR LED off
            
            int dist = ultra_dist();
            
        }
        else{
            PORTAbits.RA1 = 1;  //IR LED on
        }
    }
    
    return;
}