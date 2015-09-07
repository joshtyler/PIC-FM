/*
* File: main.c
* Project: LDPS IV FM Radio project
* Created: Spring 2014
* Created by: Lab group 8 - Josh Tyler, Vilma Wilke, Umair Hassan and Le Huy Hoang (based on skeletal code provided by RAC)
* Purpose: Communicate with an AR1010 FM receiver chip and an LCD screen to act as an FM radio
* Language: C (Complied with the XC8 compiler on MPLABX)
* Target: PIC16F84A
*/



#pragma config OSC = INTIO7     // Internal osc, RA6=CLKO, RA7=I/O
#pragma config FCMEN = OFF		// Fail-Safe Clock Monitor disabled 
#pragma config IESO = OFF		// Oscillator Switchover mode disabled 
#pragma config WDT = OFF        // WDT disabled (control through SWDTEN bit)
#pragma config PWRT = OFF       // racmod  -> PWRT disabled
#pragma config MCLRE = ON       // MCLR pin enabled; RG5 input pin disabled
#pragma config XINST = OFF      // Instruction set extension disabled
#pragma config BOREN = OFF      // Brown-out controlled by software
#pragma config BORV = 3         // Brown-out voltage set for 2.0V, nominal
#pragma config STVREN = OFF		// Stack full/underflow will not cause Reset
#pragma config CP = OFF			// Program memory block not code-protected 



#include "i2c.h" // Library containing I2C functions

#include "fm.h" // Accompanying header file contaning constants and funciton definitions




// FM register bank default values -
const unsigned int regDflt[18] = {
	0xFFFF,     // R0 -- the first writable register .  (disable xo_en)   
	0x5B15,     // R1.   
	0xD0B9,     // R2.   
	0xA010,     // R3   seekTHD = 16   
	0x0780,     // R4   
	0x28AB,     // R5   
	0x6400,     // R6   
	0x1EE7,     // R7   
	0x7141,     // R8   
	0x007D,     // R9   
	0x82C6,     // R10  disable wrap   
	0x4F55,     // R11. <--- (disable xo_output)   
	0x970C,     // R12.   
	0xB845,     // R13   
	0xFC2D,     // R14   
	0x8097,     // R15   
	0x04A1,     // R16   
	0xDF6A      // R17
};

unsigned int regImg[18];	// FM register bank images

// The words to put into LCDDATAx to display each number, see Josh's log book for working.
const unsigned char segData[10] = { 0b00111111, // Zero
                                    0b00000110, // One
                                    0b01011011, // Two
                                    0b01001111, // Three
                                    0b01100110, // Four
                                    0b01101101, // Five
                                    0b01111101, // Six
                                    0b00000111, // Seven
                                    0b01111111, // Eight
                                    0b01101111};// Nine

// The words to put into LCDDATAx to display each letter, see Vilma's log book for working.
const unsigned char LCDLetters[] = {0b01110111, // A
                                    0b01111100, // B
                                    0b00111001, // C
                                    0b01011110, // D
                                    0b01111001, // E
                                    0b01110001, // F
                                    0b01101111, // G
                                    0b01110110, // H
                                    0b00110000, // I
                                    0b00011110, // J
                                    0b01110110, // K
                                    0b00111000, // L
                                    0b00010101, // M
                                    0b01010100, // N
                                    0b00111111, // O
                                    0b01110011, // P
                                    0b01100111, // Q
                                    0b01010000, // R
                                    0b01101101, // S
                                    0b01111000, // T
                                    0b00111110, // U
                                    0b00011100, // V
                                    0b00101010, // W
                                    0b01110110, // X
                                    0b01011011, // Y
                                    0b01101110,  // Z
                                    0b00000000};// SPACE

/*
 * Test input ports for changes.
 * Returns 0 on no change, 1 if a button is pushed, 2 if a button is released
 * butn stores the previous button state.
 */
unsigned char butnEvent(unsigned char *butn) {


    unsigned char newPin, oldPin, returnVal; /* Variable to hold previous and new input states, and the return value */

    /* Test Button 1 */
    newPin = PORTB & 0b00000001; /* Mask off RB0 */
    newPin ^= 0b00000001; /* Toggle active bit */
    oldPin = *butn & 0b00000001; /* Mask off LSB */
    returnVal = testPinState(oldPin, newPin, butn);
    if(returnVal !=0)
        return returnVal;

    /* Test Button 2 */
    newPin = PORTB & 0b00100000; /* Mask off RB5 */
    newPin ^= 0b00100000; /* Toggle active bit */
    newPin >>= 4; /* Shift to correct place */
    oldPin = *butn & 0b00000010; /* Mask off second bit */
    returnVal = testPinState(oldPin, newPin, butn);
    if(returnVal !=0)
        return returnVal;

    /* Test Button 3 */
    newPin = PORTA & 0b00000001; /* Mask off RA0 */
    newPin ^= 0b00000001; /* Toggle active bit */
    newPin <<= 2; /* Shift to correct place */
    oldPin = *butn & 0b00000100; /* Mask off second bit */
    returnVal = testPinState(oldPin, newPin, butn);
    if(returnVal !=0)
        return returnVal;

    /* Test Button 4 */
    newPin = PORTA & 0b00000010; /* Mask off RA1 */
    newPin ^= 0b00000010; /* Toggle active bit */
    newPin <<= 2; /* Shift to correct place */
    oldPin = *butn & 0b00001000; /* Mask off second bit */
    returnVal = testPinState(oldPin, newPin, butn);
    if(returnVal !=0)
        return returnVal;

    /* Test Button 5 */
    newPin = PORTG & 0b00000001; /* Mask off RG0 */
    newPin ^= 0b00000001; /* Toggle active bit */
    newPin <<= 4; /* Shift to correct place */
    oldPin = *butn & 0b00010000; /* Mask off second bit */
    returnVal = testPinState(oldPin, newPin, butn);
    if(returnVal !=0)
        return returnVal;

    /* Test Button 6 */
    newPin = PORTG & 0b00000010; /* Mask off RG1 */
    newPin ^= 0b00000010; /* Toggle active bit */
    newPin <<= 4; /* Shift to correct place */
    oldPin = *butn & 0b00100000; /* Mask off second bit */
    returnVal = testPinState(oldPin, newPin, butn);
    if(returnVal !=0)
        return returnVal;

    /* Test Button 7 */
    newPin = PORTG & 0b00000100; /* Mask off RG2 */
    newPin ^= 0b00000100; /* Toggle active bit */
    newPin <<= 4; /* Shift to correct place */
    oldPin = *butn & 0b01000000; /* Mask off second bit */
    returnVal = testPinState(oldPin, newPin, butn);
    if(returnVal !=0)
        return returnVal;

    /* Test Button 8 */
    newPin = PORTG & 0b00001000; /* Mask off RG3 */
    newPin ^= 0b00001000; /* Toggle active bit */
    newPin <<= 4; /* Shift to correct place */
    oldPin = *butn & 0b10000000; /* Mask off second bit */
    returnVal = testPinState(oldPin, newPin, butn);

    return returnVal; /* This time we return returnVal regardless of a bit change */
}

/*
 * Tests for a change in an individual pin.
 *
 * @param butn Which button changed.  See fm.h.
 *
 * @return 	0 if no button has changed state,
 *			1 if button is pushed,
 *			2 if button is released.
 *
 */
unsigned char testPinState(unsigned char oldPin, unsigned char newPin, unsigned char *butn){

    /* Test for change*/
    if( newPin == oldPin)
        return 0;

    *butn = newPin; /* Set butn to new input */

    if(newPin == 0) /* If this is TRUE a button was RELEASED */
        return 2;

    return 1; /* The only other option is that a button was pressed */
}




/* Simple delay loop*/
void dly(int d) {

	int i = 0;

	for ( ; d; --d) 
		for (i = 100;  i;  --i) ;
}


/*
 * Set all LCD segments to Either off or on, depending on state.
 *     TRUE = All on, FALSE = All off
 *
 */

void setscn(unsigned char state) {
    unsigned char data;

    if(state == TRUE)
        data = 0xFF;
    else
        data = 0x00;

    LCDDATA0 = data;
    LCDDATA1 = data;
    LCDDATA2 = data;
    data &= 0b00000111;
    LCDDATA3 = data;

    return;

}
/* This is the stock setscn function, replaced by our function above */
//void setscn(unsigned char state) {
//
//	int i = 0;
//	unsigned char *CLEARptr;        // Pointer used to clear all LCDDATA
//
//
//	for (   i = 0, CLEARptr = (unsigned char *) &LCDDATA0;  // Point to first segment
//		i < 24;
//		i++)		// Turn off all segments
//            if (state)
//                *CLEARptr++ = 0xFF; // Turn on segments
//            else
//		*CLEARptr++ = 0x00; // Turn off segments
//}



/* Initialisation procedure */
void Init() {

	int i;

	OSCCON = 0b01110010;        	// Select 8 MHz internal oscillator
	LCDSE0 = 0b11111111;        	// Enable  LCD segments 07-00
	LCDSE1 = 0b11111111;        	// Enable  LCD segments 15-08
	LCDSE2 = 0b11111111;        	// Enable  LCD segments 23-16
	LCDSE3 = 0b00000111;        	// Disable LCD segments 31-24
	LCDCON = 0b10001000;         	// Enab LC controller. Static mode. INTRC clock
	LCDPS  = 0b00110110;         	// 37 Hz frame frequency
	ADCON1 = 0b00111111;        	// Make all ADC/IO pins digital
	TRISA = 0b00000011;             // RA0 and RA1 pbutton
	TRISB = 0b00100001;				// RB0 and RB5 pbutton 
        TRISC = 0b00011000;				// RC3 and RC4 do the I2C bus
	TRISG = 0b11101111;				// RG0, RG1, RG2 & RG3 pbutton, RG4 is LCD
	PORTA = 0;
	PORTB = 0;
	PORTC = 0;
    INTCONbits.TMR0IF = 0;          // Clear timer flag
	//T0CON = 0b00000011;				// Prescale by 16
    T0CON = 0b00001000;             // No prescale
    TMR0H = 0;                      // Clear timer count
    TMR0L = 0;
    T0CONbits.TMR0ON = 1;           // Start timer
	OpenI2C( MASTER, SLEW_OFF);
	SSPADD = 0x3F;

}


/*
 * Write an individual LCD segment.  
 *
 * @param segOrd The segment ordinal.  Between 0 and 26.
 *
 * @param state Whether to turn the segment dark (true) or clear (false).
 *
 */
void segWrt(unsigned char segOrd,  unsigned char state) {

	unsigned char bitSelect;
	unsigned char *LCReg;

	if (segOrd > 26)    // Return if invalid argument
            return;

	LCReg = (unsigned char *)&LCDDATA0 + (segOrd >> 3);
	bitSelect = 1 << (segOrd & 0x07);

	if (state)
            *LCReg  |=  bitSelect;		// Segment on
	else
            *LCReg &= ~bitSelect;				// Segment off
}


/*  Writes a character to one of the 7 segments on the LCD
    NumToDisp is the number to display in the segment
    DigitNo is the digit to write it to */

void charWrt(unsigned char numToDisp, unsigned char DigitNo) {

    numToDisp = segData[numToDisp]; // Set numToDisp to the binary word needed to display that number

    switch(DigitNo)
    {
        case 1:
            LCDDATA2 = numToDisp;
            break;
        case 2:
            LCDDATA1 = numToDisp;
            break;
        case 3:
            LCDDATA0 = numToDisp;
            break;
        default:   // This will never happen (hopefully)
            errfm();
            break;
    }

}

/*
 * manualTune() -  Manually tunes the channel
 *
 * @param dir Sets which way the frequency is changed, zero for up,
 *  nonzero for down.
 *
 * @param freq holds the frequency value.
 *
 * @return frequency on success or XF on error.
 *
 */
unsigned int manualTune(unsigned int freq, unsigned char dir) {


    switch (dir)
    {
	case TRUE : //manual up
            if(freq < FREQMAX) //checks that frequency is valid
            {
                freq++;
                FMfrequenc(freq); //sends incremented new value to FM chip
            } else {
                freq = FREQMIN;
                FMfrequenc(freq);
            }
            break;


        case FALSE : //manual down
            if(freq > FREQMIN)
            {
                freq--;
                FMfrequenc(freq);
            } else {
                freq = FREQMAX;
                FMfrequenc(freq);
            }
            break;

	default :
            errfm(); // Call error subroutine
            break;
    }

    return freq;
}


/*
 * FMwrite() -  Write a two byte word to the FM module.  The new 
 * register contents are obtained from the image bank.
 *
 * @param adr The address of the register in the FM module that needs 
 * to be written.
 *
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char FMwrite(unsigned char adr) {

	unsigned int  regstr;
	unsigned char firstByt;
	unsigned char secndByt;
	unsigned char rpy;

	firstByt = regImg[adr] >> 8;
	secndByt = regImg[adr];

	StartI2C();					// Begin I2C communication
	IdleI2C();

	// Send slave address of the chip onto the bus
	if (WriteI2C(FMI2CADR)) return XF;
	IdleI2C();
	WriteI2C(adr);				// Adress the internal register
	IdleI2C();
	WriteI2C(firstByt);			// Ask for write to FM chip
	IdleI2C();
	WriteI2C(secndByt);
	IdleI2C();
	StopI2C();
	IdleI2C();
	return XS;
}




/*
 * FMread - Read a two byte register from the FM module.
 *
 * @param regAddr The address of the register in the module that needs 
 *        to be read.
 *
 * @param data Where to store the reading.
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char FMread(unsigned char regAddr, unsigned int *data) {

	unsigned char firstByt;
	unsigned char secndByt;

	StartI2C();					// Begin I2C communication
	IdleI2C();					// Allow the bus to settle

	// Send address of the chip onto the bus
	if (WriteI2C(FMI2CADR)) return XF;	
	IdleI2C();
	WriteI2C(regAddr);			// Adress the internal register
	IdleI2C();
	RestartI2C();				// Initiate a RESTART command
	IdleI2C();
	WriteI2C(FMI2CADR + DEVRD);	// Ask for read from FM chip
	IdleI2C();
	firstByt = ReadI2C(); 		// Returns the MSB byte
	IdleI2C();
	AckI2C();					// Send back Acknowledge
	IdleI2C();
	secndByt = ReadI2C();		// Returns the LSB of the temperature
	IdleI2C();
	NotAckI2C();
	IdleI2C();
	StopI2C();
	IdleI2C();
	*data = firstByt;
	*data <<= 8;
	*data = *data | secndByt;

	return XS;
}




/*
 * FMready - See if the FM module is ready.
 *
 * @param rdy Where to store the busy/ready status.  Will become
 * non-zero if the chip is ready, zero if busy.
 * 
 *
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char FMready(unsigned int *rdy) {

	unsigned int sts;

	if (FMread(FMCHIPSTSADR, &sts)  != XS) return XF;
	sts &= FMASKSTATUS;
	*rdy = sts ? TRUE : FALSE;
	return XS;
}



/*
 * FMinit() -  Initialise the FM module.  
 *
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char FMinit() {

	unsigned char ad;
	unsigned int dat;

	// Copy default FM register values to the image set -
	for(ad = 0; ad < 18; ad++) regImg[ad] = regDflt[ad];

	dat = regImg[0];
	regImg[0] &= ~1;
	if (FMwrite(0) != XS) return  XF;
	for(ad = 1; ad < 18; ad++) {
		if (FMwrite(ad) != XS)return XF;
	}

	regImg[0] = dat | 1;
	if (FMwrite(0) != XS) return XF;
	dly(20);
	while (FMready(&dat), !dat) dly(2);
	return XS;
}





/*
 * FMfrequenc(f) -  Tune the FM module to new frequency.  
 *
 *
 * @param f The new frequency as a multiple of 100 kHz.
 *
 * @return XS on success or XF on error.
 *
 */
unsigned char FMfrequenc(unsigned int f) {

	unsigned int dat;
	unsigned int cn;		// AR1010 channel number

	cn = f - 690;

	// NB AR1010 retunes on 0 to 1 transition of TUNE bit -	
	regImg[2] &= ~FMASKTUNE;
	if (FMwrite(2) != XS)
            return XF;
	regImg[2] &= 0xfe00; 
	regImg[2] |= (cn | FMASKTUNE);
	if (FMwrite(2) != XS)
            return XF;
	do {
		dly(2);
		if (FMready(&dat) != XS)
                    return XF;
	} while (!dat);

        return XF;  // Code will never get to here.
}



/*
 * FMvers - Obtain the FM chip version.
 *
 * @param vsn Where to store the version number.  Will become
 * 0x65B1 for vintage 2009 devices.
 *
 * @return XS on success or XF on error. *
 */
unsigned char FMvers(unsigned int *vsn) {
	if (FMread(FMCHIPVERSADR, vsn)  != XS) return XF;
	return XS;
}



/*
 * FMid - Obtain the FM chip ID.
 * * @param id Where to store the ID number.  Will become
 * 0x1010 for AR1010 devices.
 *
 * @return XS on success or XF on error. *
 */
unsigned char FMid(unsigned int *id) {

	if (FMread(FMCHIPIDADR, id)  != XS) return XF;
	return XS;
}


/*
 * volSet() -  Changes the volume
 *
 * @param dir Sets which way the volume is changed, TRUE for up,
 *  FALSE for down.
 *
 * @param vol holds the volume value.
 *
 * @return volume on success. On failure, enter error loop.
 *
 */

unsigned int volSet(unsigned int vol, unsigned char dir) {

    

    switch (dir)
    {
	case TRUE : //volume up
            if(vol < 18) //checks that new volume is valid
            {
                vol++;
                
                /* Clear the volume bits but leave other bits intact */
                regImg[3] &= VOL1_MASK;
                regImg[14] &= VOL2_MASK;

                /* OR the mask with the current regImg value */
                regImg[3] |= volumePair[0][vol];
                regImg[14] |= volumePair[1][vol];

                FMwrite(3); /*Calls function FMwrite to write new volume values to the FM chip*/
                FMwrite(14);
            }
            break;

        case FALSE :
            if(vol > 0)
            {
                vol--;

                /* Clear the volume bits but leave other bits intact */
                regImg[3] &= VOL1_MASK;
                regImg[14] &= VOL2_MASK;

                regImg[3] |= volumePair[0][vol];
                regImg[14] |= volumePair[1][vol];
                FMwrite(3);
                FMwrite(14);
            }
            break;


	default :
            errfm();
            break;
    }

    return vol;
}



/*
 * nextChan() -  Tune to the next channel.
 *
 * @param up Set to TRUE for next channel up,
 *  FALSE for preset down.
 *
 * @return XS on success or XF on error.
 *
 */
unsigned int nextChan(unsigned int chan, unsigned char dir) {

        switch (dir)
    {
	case TRUE : //manual up
            if(chan < 5) //checks that the selection is valid ***NOTE CHANGE****
            {
                chan++;
                FMfrequenc(station[chan]); //sends stored value to FM chip
            } else {
                chan = 0;
                FMfrequenc(station[chan]);
            }
            break;


        case FALSE : //manual down
            if(chan > 0)
            {
                chan--;
                FMfrequenc(station[chan]);
            } else {
                chan = 5;
                FMfrequenc(station[chan]);
            }
            break;

	default :
            errfm();
            break;
    }

    return chan;


}



/*
 * errfm() -  Firmware error.   Call this on a showstopper.
 *
 *
 * @return Never!
 *
 */
void errfm(void) {

    LCDDATA2 = 0b01111001; // Display E
    LCDDATA1 = 0b00110001; // Display r
    LCDDATA0 = 0b00110001; // Display r
	for(;;) ;          // Loop until reset
}


/*
 * Display the frequency that the receiver chip is set to.
 *
 * @return XS if successful, else XF on failure.
 *
 * frequency is the frequency to display
 */
unsigned char showFreq(unsigned int frequency) {

    unsigned char display[3];

    if (frequency >= 1000)   // Set the '1'
    {
        segWrt(LCD_K, TRUE);
        frequency -= 1000; // Subtract 1000 from the frequency for subsequent digits
    }
    else
        segWrt(LCD_K, FALSE);

    display[0] = (frequency/100);// The hundreds digit, this works as C integer division truncates towards 0

    frequency -= (frequency/100)*100; // Remove the hundreds digit

    display[1] = (frequency/10);

    frequency -= (frequency/10)*10; // Remove the tens digit

    display[2] = frequency;

    charWrt(display[0], 1);
    charWrt(display[1], 2);
    segWrt(LCD_DP3, TRUE); // Display the decimal point
    charWrt(display[2], 3);

    return XS;
}


/*
 * Display the volume.
 *
 * @return XS if successful, else XF on failure.
 * volume is the volume to display
 *
 */
unsigned char showVol(unsigned int volume) {

    setscn(FALSE); // All segments off

    if (volume >= 10)
    {
        charWrt(1,2); // Set segment 2 to '1'
        volume -= 10;
    }

    charWrt(volume, 3);

    return XS;
}

/*
 * Display the channel number.
 *
 * @return XS if successful, else XF on failure.
 * channel is the number to display
 *
 */
unsigned char showChan(unsigned int channel)
{
    setscn(FALSE); // All segments off
    charWrt(channel, 1);     // Write channel to segment 1

    return XS;
}

/*
 * setScan() -  Scan up or down
 *
 * @param dir Sets which way to scan, zero for up,
 *  nonzero for down.
 *
 * Returns the new frequency
 */
unsigned int setScan(unsigned char dir) {

   unsigned int dat;
   unsigned int chanRgstr;
   int reg3Wrd;

   regImg[1] |= 0x0002; // Set hmute
   FMwrite(1);

   regImg[2] &=0xFDFF; //Clear Tune bit
   FMwrite(2);


   regImg[3] &=0xBFFF; // Clear Seek Bit
   FMwrite(3);

   regImg[3] |= 0x4000; // Set SEEK bit

   regImg[3] |= 0x2000; // Set spacing

    if(dir == 1)
        regImg[3] |= 0x8000;  // If dir is 1, set SEEKUP bit
    else
        regImg[3] &= 0x7FFF; // Otherwise clear to seek down

   regImg[3] &= 0xE7FF; // Clear BAND

   regImg[3] &= 0xFF80; // Mask all but seek threshold
   regImg[3] |= 0x0040; // Set seek threshold

   regImg[10] |= 0x0008; // Set wrap bit
   FMwrite(10); // Write the bits to memory

   FMwrite(3);

   /* Now wait until finished scanning */
    do {
            dly(2);
            if (FMready(&dat) != XS) return XF;
    } while (!dat);

    /* Read new channel */
   if ( FMread(READCHAN_ADR, &chanRgstr) == XF)
       errfm();

   chanRgstr >>= 7;
   chanRgstr &= 0x01FF; // Ensure that all bits apart from READCHAN are clear

   regImg[2] &= 0xFE00; //Mask off CHAN
   regImg[2] |= chanRgstr; // OR with current channel
   FMwrite(2);

   chanRgstr += 690;

   regImg[1] &= 0xFFFD; // Clear hmute
   FMwrite(1);

   return chanRgstr;

}

/*
 * displays three charactes of the string str on the screen.
 * pos gives the starting position (i.e so the next three are displayed
 * the return value is the new position
 */
int displayText(char str[], int pos)
{
    unsigned char display[3] = {0x00,0x00,0x00};

    unsigned char flag = FALSE;

    segWrt(LCD_K, FALSE); // Turn '1' off

    int i;
    for(i=0; (flag!= TRUE)&&(i<3); i++)
    {

        if( str[pos +i] == '\0')
            flag = TRUE;
        else
            display[i] = LCDLetters[ str[pos +i] - 'A'];
    }

        LCDDATA2 = display[0];
        LCDDATA1 = display[1];
        LCDDATA0 = display[2];

        if(str[pos] == '\0')
            return 0;

        pos++;
        return pos;
}

/* The main routine */
void main(void) {

	unsigned char btn = 0b00000000; // Initialise btn to no buttons being pressed.
	unsigned char evt;
	unsigned int ui;
        unsigned int freq = 875; //Stores frequency
        unsigned int vol = 11; //Stores volume
        unsigned int chan = 0; //Stores channel
        unsigned int counter = 0;
        unsigned int charDispCtr = CHAR_DISP_DELAY;
        unsigned char toggle = FALSE;

        unsigned char disp = 'F'; // Stores display state

        dly(20);        // Delay to allow power lines to stabilise?
	Init();

        char welcomeMessage[] = "HELLO";

        int dispOfst;
        for(dispOfst=0; dispOfst<6; dispOfst++)
        {
            displayText(welcomeMessage, dispOfst);
            dly(1000);
        }


	FMvers(&ui);	// Check we have comms with FM chip
	if (ui != 0x1010)
            errfm();
	if (FMinit() != XS)
            errfm();

        FMfrequenc(freq);    // Initialise frequency


        vol=volSet(vol,FALSE);   // Initialise volume to 10

	for (;;) {
                    dly(20);    // delay to debounce switches

                    switch(disp)
                    {
                        case 'F':
                            showFreq(freq);
                            break;

                        case 'C':
                            showChan(chan);
                            break;

                        case 'V':
                            showVol(vol);
                            break;

                        case 'S':
                            if(charDispCtr == 0)
                            {
                                dispOfst = displayText(stationName[chan],dispOfst);
                                charDispCtr = CHAR_DISP_DELAY;

                                if(dispOfst == 0)
                                    disp = 'F';
                            }
                            else
                                charDispCtr--;
                            break;

                        default:
                            errfm();
                            break;
                    }

                     if (toggle == TRUE) // Volume counter loop
                        {
                            counter = 100;
                            toggle = FALSE;
                        }
                        else if(disp == 'V')
                        {
                            if (counter != 0)
                                counter--;
                            else
                                disp = 'F';
                        }

                  
                    evt = butnEvent(&btn);
                    if (evt == 1) switch (btn) {
												case BUTN2 : //preset up
                            chan=nextChan(chan, TRUE);
                            freq = station[chan];
                            disp = 'S';
                            //toggle = TRUE;
                            dispOfst = 0;
                            break;

                        case BUTN7 : // preset down
                            chan=nextChan(chan, FALSE);
                            freq = station[chan];
                            disp = 'S';
                            //toggle = TRUE;
                            dispOfst = 0;
                            break;

                        case BUTN3 : //manual tuning up
                            freq=manualTune(freq,TRUE);
                            disp = 'F';
                            break;

                        case BUTN6 : //manual tuning down
                            freq=manualTune(freq,FALSE);
                            disp = 'F';
                            break;

                        case BUTN4 : // volume up
                            vol=volSet(vol,TRUE);
                            disp = 'V';
                            toggle = TRUE;
                            break;
                            
                        case BUTN5 : // volume down
                            vol=volSet(vol,FALSE);
                            disp = 'V';
                            toggle = TRUE;
                            break;

                        case BUTN1 : // scan up

                            freq = setScan(TRUE);
                            disp = 'F';

                            vol = volSet(vol,TRUE); // Reset Volume
                            vol = volSet(vol,FALSE);

                            break;

                        case BUTN8 : // Scan down
                            freq = setScan(FALSE);
                            disp = 'F';

                            vol = volSet(vol,TRUE); // Reset Volume
                            vol = volSet(vol,FALSE);

                            break;

                        default:
                            break;

		}
	}
}

