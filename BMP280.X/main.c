#include <xc.h>

// PIC18F87J11 Configuration Bit Settings
// CONFIG1L
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT disabled (control is placed on SWDTEN bit))
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Reset on stack overflow/underflow enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG1H
#pragma config CP0 = OFF        // Code Protection bit (Program memory is not code-protected)

// CONFIG2L
#pragma config FOSC = HSPLL     // Oscillator Selection bits (HS oscillator, PLL enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Two-Speed Start-up (Internal/External Oscillator Switchover) Control bit (Two-Speed Start-up disabled)

// CONFIG2H
#pragma config WDTPS = 32768    // Watchdog Timer Postscaler Select bits (1:32768)

// CONFIG3L
#pragma config EASHFT = ON      // External Address Bus Shift Enable bit (Address shifting enabled, address on external bus is offset to start at 000000h)
#pragma config MODE = MM        // External Memory Bus Configuration bits (Microcontroller mode - External bus disabled)
#pragma config BW = 16          // Data Bus Width Select bit (16-bit external bus mode)
#pragma config WAIT = OFF       // External Bus Wait Enable bit (Wait states on the external bus are disabled)

// CONFIG3H
#pragma config CCP2MX = DEFAULT // ECCP2 MUX bit (ECCP2/P2A is multiplexed with RC1)
#pragma config ECCPMX = DEFAULT // ECCPx MUX bit (ECCP1 outputs (P1B/P1C) are multiplexed with RE6 and RE5; ECCP3 outputs (P3B/P3C) are multiplexed with RE4 and RE3)
#pragma config PMPMX = DEFAULT  // PMP Pin Multiplex bit (PMP port pins connected to EMB (PORTD and PORTE))
#pragma config MSSPMSK = MSK7   // MSSP Address Masking Mode Select bit (7-Bit Address Masking mode enable)

//Project includes
#include "LCD.h"
#include "../src/bmp280.h"
#include "../src/pic18_i2c.h"
#include <stdio.h>
/*
Connections:
        Master RD5 <-> SDA
        Master RD6 <-> SCL
 */

char lcd[20];
struct bmp280_dev myDev;
struct bmp280_config config;
struct bmp280_uncomp_data ucomp_data;

void error(char err) {
    sprintf(lcd, "Error = %d", err);
    LCDClearLine(0);
    LCDWriteLine(lcd, 0);
    while (1);
}

void main(void) {
    OSCTUNEbits.PLLEN = 1;
    TRISDbits.TRISD0 = 0;
    LATDbits.LATD0 = 1;
    LCDInit();
    LCDClear();

    pic18_i2c_enable();

    myDev.read = pic18_i2c_read;
    myDev.write = pic18_i2c_write;
    myDev.delay_ms = pic18_delay_ms;
    myDev.intf = BMP280_I2C_INTF;
    myDev.dev_id = BMP280_I2C_ADDR_PRIM;

    LCDWriteLine("Starting", 0);
    char result;
    result = bmp280_init(&myDev);

    if (result == BMP280_OK) {
        sprintf(lcd, "Chip id 0x%x", myDev.chip_id);
        LCDWriteLine(lcd, 1);
    } else {
        LCDWriteLine("Init failed", 1);
    }


    /* Always read the current settings before writing, especially when
     * all the configuration is not modified 
     */
    result = bmp280_get_config(&config, &myDev);
    if (result) {
        error(result);
    }
    /* Overwrite the desired settings */
    config.filter = BMP280_FILTER_COEFF_4;
    config.os_pres = BMP280_OS_16X;
    config.os_temp = BMP280_OS_2X;
    config.odr = BMP280_ODR_500_MS;
    result = bmp280_set_config(&config, &myDev);
    if (result) {
        error(result);
    }
    /* Always set the power mode after setting the configuration */
    result = bmp280_set_power_mode(BMP280_NORMAL_MODE, &myDev);
    if (result) {
        error(result);
    }
    while (1) {
        __delay_ms(500);
        result = bmp280_get_uncomp_data(&ucomp_data, &myDev);
        if (result) {
            error(result);
        }
        int32_t temp32 = bmp280_comp_temp_32bit(ucomp_data.uncomp_temp, &myDev);
        uint32_t pres64 = bmp280_comp_pres_64bit(ucomp_data.uncomp_press, &myDev);
        sprintf(lcd, "T:%.2f C", temp32 / 100.0);
        LCDClearLine(0);
        LCDWriteLine(lcd, 0);
        sprintf(lcd, "P:%.1f Pa", pres64 / 256.0);
        LCDClearLine(1);
        LCDWriteLine(lcd, 1);
        LATDbits.LATD0 ^= 1;
    }
}



