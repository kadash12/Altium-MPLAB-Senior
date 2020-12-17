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
#include "movement.h"

//Configuration
#pragma config WDTE = OFF   //Disable watch dog timer
#pragma config LVP = ON      //Enable low voltage programming mode
#define Trigger RD6     //34 is Trigger
#define Echo RB2    //35 is Echo 

//Global Variable
unsigned long value0 = 0;   //Where to store the ADC result
int distance=0;     //Ultrasound distance
int maze[30];      //Direction Storage array  1.Forward 2.Left 3.Right 4.U-turn
int index = 0;      //Maze index

/*
 * Setup timer1
 */
void t1_init(){
    //Timer setup
    OSCCON1 = 0x70;
    TMR1 = 0;   //Initialize to 0
    T1CON = 0x20;
    TMR1CLKbits.CS=2;
    TMR1IF=0;
    
    //Interrupt setup
    INTCONbits.GIE = 1;     //Set Global Interrupt Enable bit
    INTCONbits.IPEN=0;  //Interrupt priority disabled
    INTCONbits.PEIE = 1; // enable peripheral interrupts.
    PIE0bits.IOCIE = 1;     //Interrupt on change enable
    IOCBPbits.IOCBP2 = 1;    //Set PORTB pin 2 On-Change Interrupt Enable Bit
    IOCBN = 51;     //Falling edge detection on B5 and B4
    IOCBF = 0x00;  //Set flag off
    
    //CLKRMD CLKR enabled; SYSCMD SYSCLK enabled; SCANMD SCANNER enabled; FVRMD FVR enabled; IOCMD IOC enabled; CRCMD CRC enabled; NVMMD NVM enabled; 
    PMD0 = 0x00;
}

/*
 * Get and display distance from ultrasound.
 */
int ultra_dist(){
    //Ultrasonic
    TMR1 =0; //clear the timer bits

    Trigger = 1;       //Set trigger
    __delay_us(10);           
    Trigger = 0;       //Turn off trigger
    __delay_ms(2);   //Waiting for ECHO
    
    return distance;
}

/*
 * Echo interrupt
 */
void us_irs(void){
    while (!Echo);    //Wait for echo
    TMR1ON = 1;     //Turn on timer
    while (Echo);    //Get echo
    TMR1ON = 0;     //Turn off timer

    //Calculate distance
   int time_taken = (TMR1L | (TMR1H<<8)); 
   distance = (time_taken/58.82)+1;

   if(distance <= 25 && PORTAbits.RA1 == 0){
       forward(stop);
   }
   
    if(distance >= 25 && PORTAbits.RA1 == 1){
       forward(stop);
   }
}

/*
 * IR left
 */
void IRL(){
    while((PORTBbits.RB5==0 || PORTBbits.RB0==0) && IOCBFbits.IOCBF2==0){   //Left error   
        //Speed up
        CCPR1H = 0x00;
        CCPR1L = 0x50;
        //Static
        CCPR2H = 0x00;
        CCPR2L = 0x00;
        forward(go);
    }
    //Static
    CCPR1H = 0x00;
    CCPR1L = 0x50;
    CCPR2H = 0x00;
    CCPR2L = 0x50;
}

/*
 * IR right
 */
void IRR(){   
    while((PORTBbits.RB4==0 || PORTBbits.RB1==0) && IOCBFbits.IOCBF2==0){   //Right error        
        //Speed up
        CCPR2H = 0x00;
        CCPR2L = 0x50;
        //Static
        CCPR1H = 0x00;
        CCPR1L = 0x00;
        forward(go);
    }
    CCPR1H = 0x00;
    CCPR1L = 0x50;
    CCPR2H = 0x00;
    CCPR2L = 0x50;
}

/*
 * Echo location interrupt.
 */
void __interrupt () echo(){
    if(IOCBFbits.IOCBF2==1){
        IOCBPbits.IOCBP2 = 0;   //Disable interrupt
        us_irs();
        //Reset
        IOCBFbits.IOCBF2 = 0;
        IOCBPbits.IOCBP2 = 1;
    }
    
    //IR interrupt
    // interrupt on change for pin IOCBF4 IR right
    if((IOCBFbits.IOCBF4 == 1 && IOCBNbits.IOCBN4 == 1 && IOCBFbits.IOCBF2==0) || (IOCBFbits.IOCBF1 == 1 && IOCBNbits.IOCBN1 == 1 && IOCBFbits.IOCBF2==0)){
        if(IOCBFbits.IOCBF4 == 1 && IOCBNbits.IOCBN4 == 1){
            IOCBNbits.IOCBN4 = 0;  //Disable interrupt
            IOCBFbits.IOCBF4 = 0;
        }
        else if(IOCBFbits.IOCBF1 == 1 && IOCBNbits.IOCBN1 == 1){
            IOCBNbits.IOCBN1 = 0;  //Disable interrupt
            IOCBFbits.IOCBF1 = 0;
        }
        IRR();
        IOCBNbits.IOCBN4 = 1;  //Enable interrupt
        IOCBNbits.IOCBN1 = 1;  //Enable interrupt
    }
    // interrupt on change for pin IOCBF5 IR left
    if((IOCBFbits.IOCBF5 == 1 && IOCBNbits.IOCBN5 == 1 && IOCBFbits.IOCBF2==0)||(IOCBFbits.IOCBF0 == 1 && IOCBNbits.IOCBN0 == 1 && IOCBFbits.IOCBF2==0)){
        if(IOCBFbits.IOCBF5 == 1 && IOCBNbits.IOCBN5 == 1){
            IOCBNbits.IOCBN5 = 0;  //Disable interrupt
            IOCBFbits.IOCBF5 = 0;
        }
        else if(IOCBFbits.IOCBF0 == 1 && IOCBNbits.IOCBN0 == 1){
            IOCBNbits.IOCBN0 = 0;  //Disable interrupt
            IOCBFbits.IOCBF0 = 0;
        }
        IRL();
        IOCBNbits.IOCBN5 = 1;  //Enable interrupt
        IOCBNbits.IOCBN0 = 1;  //Enable interrupt
    }
}

/*
 * Setup pwm servo
 */
void pwm_init(){
    RA4PPS = 0x07;   //RA4->PWM3:PWM3;   
    
    // CSWHOLD may proceed; SOSCPWR Low power; 
    OSCCON3 = 0x00;
    // MFOEN disabled; LFOEN disabled; ADOEN disabled; SOSCEN disabled; EXTOEN disabled; HFOEN disabled; 
    OSCEN = 0x00;
    // TUN 0; 
    OSCTUNE = 0x00;
    
    // Select timer
    CCPTMRSbits.P3TSEL = 1;
    
    // TMR0MD TMR0 enabled; TMR1MD TMR1 enabled; TMR4MD TMR4 enabled; TMR5MD TMR5 enabled; TMR2MD TMR2 enabled; TMR3MD TMR3 enabled; TMR6MD TMR6 enabled; 
    PMD1 = 0x00;
    // CCP2MD CCP2 enabled; CCP1MD CCP1 enabled; PWM4MD PWM4 enabled; PWM3MD PWM3 enabled; 
    PMD3 = 0x00;
    
    //PWMPeriod = (T2PR+1)*4*Tosc*(TMR2 PrescaleValue)
    //CLK =4MHZ     Tosc = 1/Fosc
    // T2CS FOSC/4; 
    T2CLKCON = 0x01;
    // T2PSYNC Synchronized; T2MODE Software control; T2CKPOL Rising Edge; T2CKSYNC Synchronized; 
    T2HLT = 0xA0;
    // T2RSEL T2CKIPPS pin; 
    T2RST = 0x00;
    // PR2 155; 
    T2PR = 0x9B;
    // TMR2 0; 
    T2TMR = 0x00;
    // T2CKPS 1:128; T2OUTPS 1:1; TMR2ON on; 
    T2CON = 0xF0;
}

/*
 * ccp pwm setup speed
 */
void ccp_init() {
    RC6PPS = 0x05; //RC6->CCP1:PWM; left
    RC5PPS = 0x06; //RC5->CCP2:PWM; right
    
    // Set the PWM1 to the options selected in the User Interface
	// MODE PWM; EN enabled; FMT right_aligned; 
	CCP1CON = 0x8C;      
    
	// Selecting Timer 4
	CCPTMRSbits.C1TSEL = 2;
    
    // T4CS FOSC/4; 
    T4CLKCON = 0x01;
    // T2PSYNC Not Synchronized; T2MODE Software control; T2CKPOL Rising Edge; T2CKSYNC Not Synchronized; 
    T4HLT = 0x00;
    // T2RSEL T2CKIPPS pin; 
    T4RST = 0x00;
    // PR2 155; 
    T4PR = 0x9B;
    // TMR4 0; 
    T4TMR = 0x00;
    // T4CKPS 1:128; T4OUTPS 1:1; TMR4ON on; 
    T4CON = 0xF0;
    
    //Duty cycle
    CCPR1H = 0x00;
    CCPR1L = 0x50;
    
    /* MODE PWM; EN enabled; FMT right_aligned */    
    CCP2CONbits.MODE = 0x0C;    
    CCP2CONbits.FMT = 0;    
    CCP2CONbits.EN = 1;     
    
    //Duty cycle   
    CCPR2H = 0x00;
    CCPR2L = 0x50;
}

/*
 * LCD print statements
 */
void lcdprint(){    
    lcd_command(0x02);  //Clear Home

    char array[2]; //Time Storage array
    char temp1 = array[0];   //Temporary storage
    char temp2 = array[1];
    //Clear array
    /*for(int i=0; i<2; i++){
        array[i]=0;
    }*/
    
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

    //Distance
    char d1 = (distance/100)%10;
    char d2 = (distance/10)%10;
    char d3 = (distance/1)%10;

    char d[1]; //Distance Storage array
    //Clear array
    for(int i=0; i<1; i++){
        d[i]=0;
    }
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
}

/*
 * IR path method 1 - manual
 */
void IRpath1(){
    int check = 1;
    int turncheck = 1;
    while(check){
        //Scan left
        //Left position
        if(turncheck){
            PWM3DCH = 0x12; //Duty cycle 
            PWM3DCL = 0xC0; 
            T2TMR = 0x00;   //Timer2 reset
            PWM3CONbits.PWM3EN = 1;     //PWM3 Enable
            __delay_ms(800);
            PWM3CONbits.PWM3EN = 0;     //PWM3 Disable
            turncheck = 0;
            lcdprint();
        }
        int distlr = ultra_dist();

        if(distlr < 60){
            forward(go);    //Go forward
        }
        else{ //Turn left
            __delay_ms(200);
            forward(stop);
            lcdprint();
            lturn();
            check = 0;
        }
    }
    
    while(check==0){
        if(turncheck == 0){
            //Scan Center
            PWM3DCH = 0x0B;  //Duty cycle 
            PWM3DCL = 0xC0;   
            T2TMR = 0x00;   //Timer2 reset
            PWM3CONbits.PWM3EN = 1;     //PWM3 Enable 
            __delay_ms(800);
            PWM3CONbits.PWM3EN = 0;     //PWM3 Disable
            turncheck=1;
            lcdprint();
        }
        int distcr = ultra_dist();    //Get distance

        if(distcr > 25){
            forward(go);    //Go forward
        }
        else{
            forward(stop);  //Stop forward
            lcdprint();
            IOCBNbits.IOCBN4 = 0; 
            IOCBNbits.IOCBN1 = 0; 
            IOCBNbits.IOCBN0 = 0;
            IOCBNbits.IOCBN5 = 0; 

            PORTBbits.RB3 = 1;  //Ultrasound LED on

            PORTDbits.RD5 = 1;  //Buzzer on
            for(int i=0; i<200; i++){
                //Speaker tone
                LATDbits.LATD5=1;
                __delay_ms(0.25);
                LATDbits.LATD5=0;
                __delay_ms(0.25);
            }
            PORTDbits.RD5 = 0;  //Buzzer off

            //Scan left
            //Left position
            PWM3DCH = 0x12; //Duty cycle 
            PWM3DCL = 0xC0; 
            T2TMR = 0x00;   //Timer2 reset
            PWM3CONbits.PWM3EN = 1;     //PWM3 Enable
            __delay_ms(500);
            int dist1 = ultra_dist();
            __delay_ms(100);
            PWM3CONbits.PWM3EN = 0;     //PWM3 Disable
            lcdprint();

            //Scan right
            //Right position
            PWM3DCH = 0x04;     //Duty cycle 
            PWM3DCL = 0x10;  
            T2TMR = 0x00;   //Timer2 reset
            PWM3CONbits.PWM3EN = 1;     //PWM3 Enable
            __delay_ms(500);
            int dist2 = ultra_dist();
            __delay_ms(100);
            PWM3CONbits.PWM3EN = 0;     //PWM3 Disable
            lcdprint();

            //Scan Center
            PWM3DCH = 0x0B;  //Duty cycle 
            PWM3DCL = 0xC0;   
            T2TMR = 0x00;   //Timer2 reset
            PWM3CONbits.PWM3EN = 1;     //PWM3 Enable 
            __delay_ms(800);
            int dist3 = ultra_dist();    //Get distance
            __delay_ms(100);
            PWM3CONbits.PWM3EN = 0;     //PWM3 Disable
            lcdprint();

            PORTDbits.RD5 = 1;  //Buzzer on
            for(int i=0; i<200; i++){
                //Speaker tone
                LATDbits.LATD5=1;
                __delay_ms(0.25);
                LATDbits.LATD5=0;
                __delay_ms(0.25);
            }
            PORTDbits.RD5 = 0;  //Buzzer off

            if(dist3 <=15){
                backward();
            }
            if(dist2<30 && dist1<30 && dist3<30){
                if(dist1>dist2){
                    reverser();
                }
                if(dist2>dist1){
                   reversel(); 
                }
            }
            else if(dist1>30 && dist1>=dist3){
                lturn();
            }
            else if(dist2>30 && dist2>dist3){
                rturn();
                check = 1;
            }
        IOCBNbits.IOCBN4 = 1; 
        IOCBNbits.IOCBN1 = 1; 
        IOCBNbits.IOCBN0 = 1;
        IOCBNbits.IOCBN5 = 1; 
        PORTBbits.RB3 = 0;  //Ultrasound LED off
        }
    }
}

/*
 * IR path method 1 - auto
 */
void IRpath2(int num){
    if(num==2){
        PWM3DCH = 0x12; //Duty cycle 
        PWM3DCL = 0xC0; 
        T2TMR = 0x00;   //Timer2 reset
        PWM3CONbits.PWM3EN = 1;     //PWM3 Enable
        __delay_ms(800);
        PWM3CONbits.PWM3EN = 0;     //PWM3 Disable
        lcdprint();
    }
     
    int check=1;
    while(check==1){
        int distlr = ultra_dist();
        
        if(distlr < 60){
            forward(go);    //Go forward
        }
        else{ //Turn left
            __delay_ms(200);
            forward(stop);
            lcdprint();
            lturn();
            check=0;
        }
    }

    if(num == 3){
        //Scan Center
        PWM3DCH = 0x0B;  //Duty cycle 
        PWM3DCL = 0xC0;   
        T2TMR = 0x00;   //Timer2 reset
        PWM3CONbits.PWM3EN = 1;     //PWM3 Enable 
        __delay_ms(800);
        PWM3CONbits.PWM3EN = 0;     //PWM3 Disabl
        lcdprint();
    }
    
    check=1;
    while(check==1){
       int distcr = ultra_dist();    //Get distance

       if(distcr > 25){
           forward(go);    //Go forward
       }
       else{
           forward(stop);  //Stop forward
           lcdprint();
           IOCBNbits.IOCBN4 = 0; 
           IOCBNbits.IOCBN1 = 0; 
           IOCBNbits.IOCBN0 = 0;
           IOCBNbits.IOCBN5 = 0; 

           PORTBbits.RB3 = 1;  //Ultrasound LED on

           PORTDbits.RD5 = 1;  //Buzzer on
           for(int i=0; i<200; i++){
               //Speaker tone
               LATDbits.LATD5=1;
               __delay_ms(0.25);
               LATDbits.LATD5=0;
               __delay_ms(0.25);
           }
           PORTDbits.RD5 = 0;  //Buzzer off

           //Scan left
           //Left position
           PWM3DCH = 0x12; //Duty cycle 
           PWM3DCL = 0xC0; 
           T2TMR = 0x00;   //Timer2 reset
           PWM3CONbits.PWM3EN = 1;     //PWM3 Enable
           __delay_ms(500);
           int dist1 = ultra_dist();
           __delay_ms(100);
           PWM3CONbits.PWM3EN = 0;     //PWM3 Disable
           lcdprint();

           //Scan right
           //Right position
           PWM3DCH = 0x04;     //Duty cycle 
           PWM3DCL = 0x10;  
           T2TMR = 0x00;   //Timer2 reset
           PWM3CONbits.PWM3EN = 1;     //PWM3 Enable
           __delay_ms(500);
           int dist2 = ultra_dist();
           __delay_ms(100);
           PWM3CONbits.PWM3EN = 0;     //PWM3 Disable
           lcdprint();

           //Scan Center
           PWM3DCH = 0x0B;  //Duty cycle 
           PWM3DCL = 0xC0;   
           T2TMR = 0x00;   //Timer2 reset
           PWM3CONbits.PWM3EN = 1;     //PWM3 Enable 
           __delay_ms(800);
           int dist3 = ultra_dist();    //Get distance
           __delay_ms(100);
           PWM3CONbits.PWM3EN = 0;     //PWM3 Disable
           lcdprint();

           PORTDbits.RD5 = 1;  //Buzzer on
           for(int i=0; i<200; i++){
               //Speaker tone
               LATDbits.LATD5=1;
               __delay_ms(0.25);
               LATDbits.LATD5=0;
               __delay_ms(0.25);
           }
           PORTDbits.RD5 = 0;  //Buzzer off

           if(dist3 <=15){
               backward();
           }
           if(dist2<30 && dist1<30 && dist3<30){
               if(dist1>dist2){
                   reverser();
               }
               if(dist2>dist1){
                  reversel(); 
               }
           }
           else if(dist1>30 && dist1>=dist3){
               lturn();
               lturn();
           }
           else if(dist2>30 && dist2>dist3){
               rturn();
               check = 0;
           }
        IOCBNbits.IOCBN4 = 1; 
        IOCBNbits.IOCBN1 = 1; 
        IOCBNbits.IOCBN0 = 1;
        IOCBNbits.IOCBN5 = 1; 
        PORTBbits.RB3 = 0;  //Ultrasound LED off
        lcdprint();
        }
    }
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
    pwm_init();     //Initialized pwm
    ccp_init();     //Initialized pwm on ccp
    
    //Movement Control
    TRISEbits.TRISE0 = 0;   //LeftReverse output signal
    TRISCbits.TRISC2 = 0;   //LeftForward output signal
    TRISCbits.TRISC0 = 0;   //RightReverse output signal
    TRISCbits.TRISC1 = 0;   //RightForward output signal
    
    //IR setup
    TRISAbits.TRISA1 = 0;   //IR LED output signal
    TRISBbits.TRISB4 = 1;   //IR Right input signal Active low
    TRISBbits.TRISB5 = 1;   //IR Left input signal  Active low
    TRISBbits.TRISB1 = 1;   //IR Right 2 input signal  Active low
    TRISBbits.TRISB0 = 1;   //IR Left 2 input signal  Active low
    
    //PWM setup
    TRISAbits.TRISA4 = 0;   //PWM servo output signal
    TRISCbits.TRISC6 = 0;   //PWM H left ccp1 output signal
    TRISCbits.TRISC5 = 0;   //PWM H right ccp2 output signal
    
    //Ultrasonic setup
    TRISDbits.TRISD5 = 0;   //RD5 is output pin for Buzzer
    TRISDbits.TRISD6 = 0;   //Trigger pin of US sensor is sent as output pin
    TRISBbits.TRISB2 = 1;   //Echo pin of US sensor is set as input pin  
    TRISBbits.TRISB3 = 0;   //RB3 is output pin for LED
    ANSELB = 0x0;
    
    //Zero maze
    for(int i=0; i<30; i++){
        maze[i]=0;
    }
    
    //Set timer
    int time = 0;
    set_minutes(time);
    //time = 0;
    set_seconds(time);
    
    int distl;
    int distr;
    int distcc;
    
    //Infinite loop to output the time. 
    while(1){    
        //Check if close to IR beacon
        value0 = adcNum0(); //Get ADC value from IR detector
     
        if(value0<716){
            PORTAbits.RA1 = 0;  //IR LED off
            
            lcdprint();
            
            IOCBNbits.IOCBN4 = 0; 
            IOCBNbits.IOCBN1 = 0; 
            IOCBNbits.IOCBN0 = 0;
            IOCBNbits.IOCBN5 = 0;
            int distc = ultra_dist();    //Get distance
            IOCBNbits.IOCBN4 = 1; 
            IOCBNbits.IOCBN1 = 1; 
            IOCBNbits.IOCBN0 = 1;
            IOCBNbits.IOCBN5 = 1;
            
            if(distc > 25){
                PORTBbits.RB3 = 0;  //Ultrasound LED off
                PORTDbits.RD5 = 0;  //Buzzer off
                
                forward(go);    //Go forward
                maze[index]=1;
                index++;
            }
            else{
                forward(stop);  //Stop forward
                IOCBNbits.IOCBN4 = 0; 
                IOCBNbits.IOCBN1 = 0; 
                IOCBNbits.IOCBN0 = 0;
                IOCBNbits.IOCBN5 = 0; 
                
                PORTBbits.RB3 = 1;  //Ultrasound LED off
                
                PORTDbits.RD5 = 1;  //Buzzer on
                for(int i=0; i<200; i++){
                    //Speaker tone
                    LATDbits.LATD5=1;
                    __delay_ms(0.25);
                    LATDbits.LATD5=0;
                    __delay_ms(0.25);
                }
                PORTDbits.RD5 = 0;  //Buzzer off
                
                //Scan left
                //Left position
                PWM3DCH = 0x12; //Duty cycle 
                PWM3DCL = 0xC0; 
                T2TMR = 0x00;   //Timer2 reset
                PWM3CONbits.PWM3EN = 1;     //PWM3 Enable
                __delay_ms(500);
                distl = ultra_dist();
                __delay_ms(100);
                PWM3CONbits.PWM3EN = 0;     //PWM3 Disable
                lcdprint();
                
                //Scan right
                //Right position
                PWM3DCH = 0x04;     //Duty cycle 
                PWM3DCL = 0x10;  
                T2TMR = 0x00;   //Timer2 reset
                PWM3CONbits.PWM3EN = 1;     //PWM3 Enable
                __delay_ms(500);
                distr = ultra_dist();
                __delay_ms(100);
                PWM3CONbits.PWM3EN = 0;     //PWM3 Disable
                lcdprint();
                
                //Scan Center
                PWM3DCH = 0x0B;  //Duty cycle 
                PWM3DCL = 0xC0;   
                T2TMR = 0x00;   //Timer2 reset
                PWM3CONbits.PWM3EN = 1;     //PWM3 Enable 
                __delay_ms(800);
                distcc = ultra_dist();    //Get distance
                __delay_ms(100);
                PWM3CONbits.PWM3EN = 0;     //PWM3 Disable
                lcdprint();
                
                PORTDbits.RD5 = 1;  //Buzzer on
                for(int i=0; i<200; i++){
                    //Speaker tone
                    LATDbits.LATD5=1;
                    __delay_ms(0.25);
                    LATDbits.LATD5=0;
                    __delay_ms(0.25);
                }
                PORTDbits.RD5 = 0;  //Buzzer off
                
                if(distcc <=15){
                    backward();
                }
                if(distr<30 && distl<30 && distcc<30){
                    if(distl>distr){
                        reverser();
                    }
                    if(distr>distl){
                       reversel(); 
                    }
                    maze[index]=4;
                    index++;
                }
                else if(distl>30 && distl>=distcc){
                    lturn();
                    maze[index]=2;
                    index++;
                }
                else if(distr>30 && distr>distcc){
                    rturn();
                    maze[index]=3;
                    index++;
                }
            }
            IOCBNbits.IOCBN4 = 1; 
            IOCBNbits.IOCBN1 = 1; 
            IOCBNbits.IOCBN0 = 1;
            IOCBNbits.IOCBN5 = 1; 
        }
        else{
            forward(stop);  //Stop forward
            PORTAbits.RA1 = 1;  //IR LED on 
            
            int spf[15];    //shortest path first
            index=0;
            //Zero maze
            for(int i=0; i<15; i++){
                spf[i]=0;
            }
            int to3 = 0;    //Swap 2 to 3
            //Get Shortest path
            for(int j=29; j>0; j--){
                if(maze[j]==2 && maze[j+1]==1 ){
                    if(to3==0){
                        spf[index]=2;   //Left turn
                        index++;
                        to3=1;
                    }
                    else if (to3==1){
                        spf[index]=3;   //right turn
                        index++;
                        to3=0;
                    }
                }
            }
            
            while(1){
                IRpath1(); //Reverse path travel
                
                /*for(int i=0; i>(sizeof spf/sizeof spf[0]); i++){
                    if(spf[i]==2 || spf[i]==3){
                        IRpath2(spf[i]); //Reverse path travel
                    }
                }*/
            }
        }
    }
    return;
}

