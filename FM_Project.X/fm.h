/*
   File: fm.h
   Project: LDPS IV FM Radio project
   Created: Spring 2014
   Created by: Lab group 8 - Josh Tyler, Vilma Wilke, Umair Hassan and Le Huy Hoang (based on skeletal code provided by RAC)
   Purpose: Provide constants and function definitions used by main.c
   Language: C (Complied with the XC8 compiler on MPLABX)
   Target: PIC16F84A
*/


#define XS			0			// Exit success
#define XF			1			// Exit fail

#define FMI2CADR		0x20		// Address (for writes) of FM module on I2C bus


#define DEVRD			0x01		// Read not write an I2C device 
#define FMCHIPVERSADR           0x1C		// Address of FM chip version
#define FMCHIPIDADR		0x1B		// Address of FM chip ID  
#define FMCHIPSTSADR            0x13		// Address of FM chip status

#define READCHAN_ADR            0x13

#define FMASKMUTE		0x0001		// Register 1, bit 1
#define FMASKTUNE		0x0200		// Register 2, bit 9
#define FMASKSTATUS		0x0020		// Register 0x13, bit 5
#define FMASKSEEK		0x4000		// Register 3, bit 14
#define FMASKRDCHAN		0xFF80		// Register 2, channel number bits

#define BUTN1			0b00000001	// Button number one
#define BUTN2			0b00000010	// Button number two
#define BUTN3			0b00000100
#define BUTN4			0b00001000
#define BUTN5			0b00010000
#define BUTN6			0b00100000
#define BUTN7			0b01000000
#define BUTN8			0b10000000

#define LCDSEGDP3		22			// LCD segment for decimal point
#define LCDSEGZ 		23			// LCD segment for Z

#define FREQMAX                 1080            // Maximum frequency
#define FREQMIN                 875             // Minimum frequency

#define FMHIGHCHAN		(1080-690)	// Highest FM channel number
#define FMLOWCHAN		(875-690)
#define FALSE			0
#define TRUE			1

#define LCD_3A                  0               // LCDDATA0
#define LCD_3B                  1
#define LCD_3C                  2
#define LCD_3D                  3
#define LCD_3E                  4
#define LCD_3F                  5
#define LCD_3G                  6
#define LCD_DP1                 7
#define LCD_2A                  8               // LCDDATA1
#define LCD_2B                  9
#define LCD_2C                  10
#define LCD_2D                  11
#define LCD_2E                  12
#define LCD_2F                  13
#define LCD_2G                  14
#define LCD_DP2                 15
#define LCD_1A                  16               // LCDDATA2
#define LCD_1B                  17
#define LCD_1C                  18
#define LCD_1D                  19
#define LCD_1E                  20
#define LCD_1F                  21
#define LCD_1G                  22
#define LCD_DP3                 23
#define LCD_K                   24
#define LCD_COL                 25
#define LCD_Z                   26

#define CHAR_DISP_DELAY         60              // Delay used for displaying text

/* Masks used to clear volume bits */
#define VOL1_MASK               0xF87F
#define VOL2_MASK               0x0FFF

/* Masks used to clear scan bits*/
#define SCAN_REG3_MASK          0x1FFF
#define SCAN_REG10_MASK         0xFFF7

/* Recommended values for volume, obtained from data sheet, repackaged for actual location in memory */
const int volumePair[2][19] = {            
    {0x0780, 0x0780, 0x0780, 0x0780, 0x0580, 0x0580, 0x0580, 0x0500, 0x0480, 0x0400, 0x0380, 0x0300, 0x0300, 0x0300, 0x0180, 0x0180, 0x0100, 0x0080, 0x0000} ,   /*  Values for R3  */
    {0x0000, 0xC000, 0xD000, 0xF000, 0xC000, 0xD000, 0xF000, 0xF000, 0xF000, 0xF000, 0xF000, 0xD000, 0xE000, 0xF000, 0xE000, 0xF000, 0xF000, 0xF000, 0xF000}    /*  Values for R14 */
};

const unsigned int station[6] = {875, 881, 903, 964, 1046, 1129};

char stationName[6][15] = { "UNKNOWN", "BBC[TWO", "BBC[THREE", "EAGLE", "BBC[ONE", "UNKNOWN" };

enum {							// Global error numbers
	GERNONE, 					// No error
	GERWCOL,					// I2C write collision
	GERFINT,					// Could not initialize FM module
	GERFMID						// Could not read chip ID (0x1010)
};

void Init();								// Processor initialisation.
void dly(int d);
unsigned char butnEvent(unsigned char *butn);
unsigned char testPinState(unsigned char oldPin, unsigned char newPin, unsigned char *butn);
void setscn(unsigned char state);
void segWrt(unsigned char segOrd,  unsigned char state);
void charWrt(unsigned char numToDisp, unsigned char DigitNo);
unsigned int manualTune(unsigned int freq, unsigned char dir);
unsigned char FMread(unsigned char regAddr, unsigned int *data);
unsigned char FMwrite(unsigned char adr);				// Write a new value to a register
unsigned char FMinit();									// Initialise the chip
unsigned char FMfrequenc(unsigned int f);
unsigned char FMready(unsigned int *rdy);				// Status is ready or busy
unsigned char FMid(unsigned int *id);					// Obtain ID number
unsigned char showFreq(unsigned int frequency);						// Display the current f in MHz
unsigned char FMvers(unsigned int *vsn);				// Obtain version number
unsigned int volSet(unsigned int vol, unsigned char dir);
unsigned int nextChan(unsigned int chan, unsigned char dir);
void errfm(void);
unsigned char showVol(unsigned int volume);
unsigned char showChan(unsigned int channel);
unsigned int setScan(unsigned char dir);
int displayText(char str[], int pos);


//
// end receiveFM.h ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//
//
