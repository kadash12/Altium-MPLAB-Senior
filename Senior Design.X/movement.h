/* 
 * Header for movement function.
 */

#ifndef MOVEMENT_H
#define	MOVEMENT_H

#include <xc.h>     //Contain the PIC C commands
#include <stdio.h>
#include <stdlib.h>

#define LR RE0    //Left Reverse
#define LF RC2    //Left Forward
#define RR RC0    //Right Reverse
#define RF RC1    //Right Forward

const int go = 1;
const int stop = 0;

void forward(int gostop);
void backward();
void rturn();
void lturn();
void reversel();
void reverser();

#endif	/* MOVEMENT_H */

