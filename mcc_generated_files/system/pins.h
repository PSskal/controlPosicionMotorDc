/**
 * Generated Pins header File
 * 
 * @file pins.h
 * 
 * @defgroup  pinsdriver Pins Driver
 * 
 * @brief This is generated driver header for pins. 
 *        This header file provides APIs for all pins selected in the GUI.
 *
 * @version Driver Version  3.1.1
*/

/*
© [2025] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

#ifndef PINS_H
#define PINS_H

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set RA0 aliases
#define pot_TRIS                 TRISAbits.TRISA0
#define pot_LAT                  LATAbits.LATA0
#define pot_PORT                 PORTAbits.RA0
#define pot_WPU                  WPUAbits.WPUA0
#define pot_OD                   ODCONAbits.ODCA0
#define pot_ANS                  ANSELAbits.ANSELA0
#define pot_SetHigh()            do { LATAbits.LATA0 = 1; } while(0)
#define pot_SetLow()             do { LATAbits.LATA0 = 0; } while(0)
#define pot_Toggle()             do { LATAbits.LATA0 = ~LATAbits.LATA0; } while(0)
#define pot_GetValue()           PORTAbits.RA0
#define pot_SetDigitalInput()    do { TRISAbits.TRISA0 = 1; } while(0)
#define pot_SetDigitalOutput()   do { TRISAbits.TRISA0 = 0; } while(0)
#define pot_SetPullup()          do { WPUAbits.WPUA0 = 1; } while(0)
#define pot_ResetPullup()        do { WPUAbits.WPUA0 = 0; } while(0)
#define pot_SetPushPull()        do { ODCONAbits.ODCA0 = 0; } while(0)
#define pot_SetOpenDrain()       do { ODCONAbits.ODCA0 = 1; } while(0)
#define pot_SetAnalogMode()      do { ANSELAbits.ANSELA0 = 1; } while(0)
#define pot_SetDigitalMode()     do { ANSELAbits.ANSELA0 = 0; } while(0)

// get/set RA1 aliases
#define EN_1A_TRIS                 TRISAbits.TRISA1
#define EN_1A_LAT                  LATAbits.LATA1
#define EN_1A_PORT                 PORTAbits.RA1
#define EN_1A_WPU                  WPUAbits.WPUA1
#define EN_1A_OD                   ODCONAbits.ODCA1
#define EN_1A_ANS                  ANSELAbits.ANSELA1
#define EN_1A_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define EN_1A_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define EN_1A_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define EN_1A_GetValue()           PORTAbits.RA1
#define EN_1A_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define EN_1A_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define EN_1A_SetPullup()          do { WPUAbits.WPUA1 = 1; } while(0)
#define EN_1A_ResetPullup()        do { WPUAbits.WPUA1 = 0; } while(0)
#define EN_1A_SetPushPull()        do { ODCONAbits.ODCA1 = 0; } while(0)
#define EN_1A_SetOpenDrain()       do { ODCONAbits.ODCA1 = 1; } while(0)
#define EN_1A_SetAnalogMode()      do { ANSELAbits.ANSELA1 = 1; } while(0)
#define EN_1A_SetDigitalMode()     do { ANSELAbits.ANSELA1 = 0; } while(0)

// get/set RA2 aliases
#define EN_1B_TRIS                 TRISAbits.TRISA2
#define EN_1B_LAT                  LATAbits.LATA2
#define EN_1B_PORT                 PORTAbits.RA2
#define EN_1B_WPU                  WPUAbits.WPUA2
#define EN_1B_OD                   ODCONAbits.ODCA2
#define EN_1B_ANS                  ANSELAbits.ANSELA2
#define EN_1B_SetHigh()            do { LATAbits.LATA2 = 1; } while(0)
#define EN_1B_SetLow()             do { LATAbits.LATA2 = 0; } while(0)
#define EN_1B_Toggle()             do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define EN_1B_GetValue()           PORTAbits.RA2
#define EN_1B_SetDigitalInput()    do { TRISAbits.TRISA2 = 1; } while(0)
#define EN_1B_SetDigitalOutput()   do { TRISAbits.TRISA2 = 0; } while(0)
#define EN_1B_SetPullup()          do { WPUAbits.WPUA2 = 1; } while(0)
#define EN_1B_ResetPullup()        do { WPUAbits.WPUA2 = 0; } while(0)
#define EN_1B_SetPushPull()        do { ODCONAbits.ODCA2 = 0; } while(0)
#define EN_1B_SetOpenDrain()       do { ODCONAbits.ODCA2 = 1; } while(0)
#define EN_1B_SetAnalogMode()      do { ANSELAbits.ANSELA2 = 1; } while(0)
#define EN_1B_SetDigitalMode()     do { ANSELAbits.ANSELA2 = 0; } while(0)

// get/set RB1 aliases
#define M1pin_TRIS                 TRISBbits.TRISB1
#define M1pin_LAT                  LATBbits.LATB1
#define M1pin_PORT                 PORTBbits.RB1
#define M1pin_WPU                  WPUBbits.WPUB1
#define M1pin_OD                   ODCONBbits.ODCB1
#define M1pin_ANS                  ANSELBbits.ANSELB1
#define M1pin_SetHigh()            do { LATBbits.LATB1 = 1; } while(0)
#define M1pin_SetLow()             do { LATBbits.LATB1 = 0; } while(0)
#define M1pin_Toggle()             do { LATBbits.LATB1 = ~LATBbits.LATB1; } while(0)
#define M1pin_GetValue()           PORTBbits.RB1
#define M1pin_SetDigitalInput()    do { TRISBbits.TRISB1 = 1; } while(0)
#define M1pin_SetDigitalOutput()   do { TRISBbits.TRISB1 = 0; } while(0)
#define M1pin_SetPullup()          do { WPUBbits.WPUB1 = 1; } while(0)
#define M1pin_ResetPullup()        do { WPUBbits.WPUB1 = 0; } while(0)
#define M1pin_SetPushPull()        do { ODCONBbits.ODCB1 = 0; } while(0)
#define M1pin_SetOpenDrain()       do { ODCONBbits.ODCB1 = 1; } while(0)
#define M1pin_SetAnalogMode()      do { ANSELBbits.ANSELB1 = 1; } while(0)
#define M1pin_SetDigitalMode()     do { ANSELBbits.ANSELB1 = 0; } while(0)

// get/set RB2 aliases
#define M2pin_TRIS                 TRISBbits.TRISB2
#define M2pin_LAT                  LATBbits.LATB2
#define M2pin_PORT                 PORTBbits.RB2
#define M2pin_WPU                  WPUBbits.WPUB2
#define M2pin_OD                   ODCONBbits.ODCB2
#define M2pin_ANS                  ANSELBbits.ANSELB2
#define M2pin_SetHigh()            do { LATBbits.LATB2 = 1; } while(0)
#define M2pin_SetLow()             do { LATBbits.LATB2 = 0; } while(0)
#define M2pin_Toggle()             do { LATBbits.LATB2 = ~LATBbits.LATB2; } while(0)
#define M2pin_GetValue()           PORTBbits.RB2
#define M2pin_SetDigitalInput()    do { TRISBbits.TRISB2 = 1; } while(0)
#define M2pin_SetDigitalOutput()   do { TRISBbits.TRISB2 = 0; } while(0)
#define M2pin_SetPullup()          do { WPUBbits.WPUB2 = 1; } while(0)
#define M2pin_ResetPullup()        do { WPUBbits.WPUB2 = 0; } while(0)
#define M2pin_SetPushPull()        do { ODCONBbits.ODCB2 = 0; } while(0)
#define M2pin_SetOpenDrain()       do { ODCONBbits.ODCB2 = 1; } while(0)
#define M2pin_SetAnalogMode()      do { ANSELBbits.ANSELB2 = 1; } while(0)
#define M2pin_SetDigitalMode()     do { ANSELBbits.ANSELB2 = 0; } while(0)

// get/set RB4 aliases
#define pulsador_TRIS                 TRISBbits.TRISB4
#define pulsador_LAT                  LATBbits.LATB4
#define pulsador_PORT                 PORTBbits.RB4
#define pulsador_WPU                  WPUBbits.WPUB4
#define pulsador_OD                   ODCONBbits.ODCB4
#define pulsador_ANS                  ANSELBbits.ANSELB4
#define pulsador_SetHigh()            do { LATBbits.LATB4 = 1; } while(0)
#define pulsador_SetLow()             do { LATBbits.LATB4 = 0; } while(0)
#define pulsador_Toggle()             do { LATBbits.LATB4 = ~LATBbits.LATB4; } while(0)
#define pulsador_GetValue()           PORTBbits.RB4
#define pulsador_SetDigitalInput()    do { TRISBbits.TRISB4 = 1; } while(0)
#define pulsador_SetDigitalOutput()   do { TRISBbits.TRISB4 = 0; } while(0)
#define pulsador_SetPullup()          do { WPUBbits.WPUB4 = 1; } while(0)
#define pulsador_ResetPullup()        do { WPUBbits.WPUB4 = 0; } while(0)
#define pulsador_SetPushPull()        do { ODCONBbits.ODCB4 = 0; } while(0)
#define pulsador_SetOpenDrain()       do { ODCONBbits.ODCB4 = 1; } while(0)
#define pulsador_SetAnalogMode()      do { ANSELBbits.ANSELB4 = 1; } while(0)
#define pulsador_SetDigitalMode()     do { ANSELBbits.ANSELB4 = 0; } while(0)

// get/set RC0 aliases
#define LED0_TRIS                 TRISCbits.TRISC0
#define LED0_LAT                  LATCbits.LATC0
#define LED0_PORT                 PORTCbits.RC0
#define LED0_WPU                  WPUCbits.WPUC0
#define LED0_OD                   ODCONCbits.ODCC0
#define LED0_ANS                  ANSELCbits.ANSELC0
#define LED0_SetHigh()            do { LATCbits.LATC0 = 1; } while(0)
#define LED0_SetLow()             do { LATCbits.LATC0 = 0; } while(0)
#define LED0_Toggle()             do { LATCbits.LATC0 = ~LATCbits.LATC0; } while(0)
#define LED0_GetValue()           PORTCbits.RC0
#define LED0_SetDigitalInput()    do { TRISCbits.TRISC0 = 1; } while(0)
#define LED0_SetDigitalOutput()   do { TRISCbits.TRISC0 = 0; } while(0)
#define LED0_SetPullup()          do { WPUCbits.WPUC0 = 1; } while(0)
#define LED0_ResetPullup()        do { WPUCbits.WPUC0 = 0; } while(0)
#define LED0_SetPushPull()        do { ODCONCbits.ODCC0 = 0; } while(0)
#define LED0_SetOpenDrain()       do { ODCONCbits.ODCC0 = 1; } while(0)
#define LED0_SetAnalogMode()      do { ANSELCbits.ANSELC0 = 1; } while(0)
#define LED0_SetDigitalMode()     do { ANSELCbits.ANSELC0 = 0; } while(0)

// get/set RC1 aliases
#define pwm1_TRIS                 TRISCbits.TRISC1
#define pwm1_LAT                  LATCbits.LATC1
#define pwm1_PORT                 PORTCbits.RC1
#define pwm1_WPU                  WPUCbits.WPUC1
#define pwm1_OD                   ODCONCbits.ODCC1
#define pwm1_ANS                  ANSELCbits.ANSELC1
#define pwm1_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define pwm1_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define pwm1_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define pwm1_GetValue()           PORTCbits.RC1
#define pwm1_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define pwm1_SetDigitalOutput()   do { TRISCbits.TRISC1 = 0; } while(0)
#define pwm1_SetPullup()          do { WPUCbits.WPUC1 = 1; } while(0)
#define pwm1_ResetPullup()        do { WPUCbits.WPUC1 = 0; } while(0)
#define pwm1_SetPushPull()        do { ODCONCbits.ODCC1 = 0; } while(0)
#define pwm1_SetOpenDrain()       do { ODCONCbits.ODCC1 = 1; } while(0)
#define pwm1_SetAnalogMode()      do { ANSELCbits.ANSELC1 = 1; } while(0)
#define pwm1_SetDigitalMode()     do { ANSELCbits.ANSELC1 = 0; } while(0)

// get/set RC2 aliases
#define pwm2_TRIS                 TRISCbits.TRISC2
#define pwm2_LAT                  LATCbits.LATC2
#define pwm2_PORT                 PORTCbits.RC2
#define pwm2_WPU                  WPUCbits.WPUC2
#define pwm2_OD                   ODCONCbits.ODCC2
#define pwm2_ANS                  ANSELCbits.ANSELC2
#define pwm2_SetHigh()            do { LATCbits.LATC2 = 1; } while(0)
#define pwm2_SetLow()             do { LATCbits.LATC2 = 0; } while(0)
#define pwm2_Toggle()             do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define pwm2_GetValue()           PORTCbits.RC2
#define pwm2_SetDigitalInput()    do { TRISCbits.TRISC2 = 1; } while(0)
#define pwm2_SetDigitalOutput()   do { TRISCbits.TRISC2 = 0; } while(0)
#define pwm2_SetPullup()          do { WPUCbits.WPUC2 = 1; } while(0)
#define pwm2_ResetPullup()        do { WPUCbits.WPUC2 = 0; } while(0)
#define pwm2_SetPushPull()        do { ODCONCbits.ODCC2 = 0; } while(0)
#define pwm2_SetOpenDrain()       do { ODCONCbits.ODCC2 = 1; } while(0)
#define pwm2_SetAnalogMode()      do { ANSELCbits.ANSELC2 = 1; } while(0)
#define pwm2_SetDigitalMode()     do { ANSELCbits.ANSELC2 = 0; } while(0)

// get/set RF0 aliases
#define IO_RF0_TRIS                 TRISFbits.TRISF0
#define IO_RF0_LAT                  LATFbits.LATF0
#define IO_RF0_PORT                 PORTFbits.RF0
#define IO_RF0_WPU                  WPUFbits.WPUF0
#define IO_RF0_OD                   ODCONFbits.ODCF0
#define IO_RF0_ANS                  ANSELFbits.ANSELF0
#define IO_RF0_SetHigh()            do { LATFbits.LATF0 = 1; } while(0)
#define IO_RF0_SetLow()             do { LATFbits.LATF0 = 0; } while(0)
#define IO_RF0_Toggle()             do { LATFbits.LATF0 = ~LATFbits.LATF0; } while(0)
#define IO_RF0_GetValue()           PORTFbits.RF0
#define IO_RF0_SetDigitalInput()    do { TRISFbits.TRISF0 = 1; } while(0)
#define IO_RF0_SetDigitalOutput()   do { TRISFbits.TRISF0 = 0; } while(0)
#define IO_RF0_SetPullup()          do { WPUFbits.WPUF0 = 1; } while(0)
#define IO_RF0_ResetPullup()        do { WPUFbits.WPUF0 = 0; } while(0)
#define IO_RF0_SetPushPull()        do { ODCONFbits.ODCF0 = 0; } while(0)
#define IO_RF0_SetOpenDrain()       do { ODCONFbits.ODCF0 = 1; } while(0)
#define IO_RF0_SetAnalogMode()      do { ANSELFbits.ANSELF0 = 1; } while(0)
#define IO_RF0_SetDigitalMode()     do { ANSELFbits.ANSELF0 = 0; } while(0)

// get/set RF1 aliases
#define IO_RF1_TRIS                 TRISFbits.TRISF1
#define IO_RF1_LAT                  LATFbits.LATF1
#define IO_RF1_PORT                 PORTFbits.RF1
#define IO_RF1_WPU                  WPUFbits.WPUF1
#define IO_RF1_OD                   ODCONFbits.ODCF1
#define IO_RF1_ANS                  ANSELFbits.ANSELF1
#define IO_RF1_SetHigh()            do { LATFbits.LATF1 = 1; } while(0)
#define IO_RF1_SetLow()             do { LATFbits.LATF1 = 0; } while(0)
#define IO_RF1_Toggle()             do { LATFbits.LATF1 = ~LATFbits.LATF1; } while(0)
#define IO_RF1_GetValue()           PORTFbits.RF1
#define IO_RF1_SetDigitalInput()    do { TRISFbits.TRISF1 = 1; } while(0)
#define IO_RF1_SetDigitalOutput()   do { TRISFbits.TRISF1 = 0; } while(0)
#define IO_RF1_SetPullup()          do { WPUFbits.WPUF1 = 1; } while(0)
#define IO_RF1_ResetPullup()        do { WPUFbits.WPUF1 = 0; } while(0)
#define IO_RF1_SetPushPull()        do { ODCONFbits.ODCF1 = 0; } while(0)
#define IO_RF1_SetOpenDrain()       do { ODCONFbits.ODCF1 = 1; } while(0)
#define IO_RF1_SetAnalogMode()      do { ANSELFbits.ANSELF1 = 1; } while(0)
#define IO_RF1_SetDigitalMode()     do { ANSELFbits.ANSELF1 = 0; } while(0)

/**
 * @ingroup  pinsdriver
 * @brief GPIO and peripheral I/O initialization
 * @param none
 * @return none
 */
void PIN_MANAGER_Initialize (void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt on Change Handling routine
 * @param none
 * @return none
 */
void PIN_MANAGER_IOC(void);


#endif // PINS_H
/**
 End of File
*/