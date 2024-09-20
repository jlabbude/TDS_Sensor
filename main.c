#include <xc.h>

#include "config.h"
#include "mcc_generated_files/system/clock.h"

void button_press() {
    if(RC4 == 0) { 
        LATCbits.LATC5 = 1; 
    } else { 
        LATCbits.LATC5 = 0; 
    }
}

void touch_press() {
    
    if (RC1 == 0) {
        
    }
    
}

void main() {
    
    OSCCON = 0b01110000;
    OSCTUNE = 0b01000000;
    
    ANSELAbits.ANSA2 = 0;
    
    TRISAbits.TRISA2 = 0;
    TRISAbits.TRISA5 = 0;
    TRISCbits.TRISC5 = 0;
    
    TRISCbits.TRISC4 = 1;
    TRISCbits.TRISC1 = 1;
    
    while(1) {
        LATAbits.LATA5 = 1;
        LATAbits.LATA2 = 1;
        button_press();
    }
}
    
