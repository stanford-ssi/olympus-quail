/*
 * ads1248.h
 *
 * Function prototypes and device definitions for the ADS1248
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
//******************************************************************************
//   ADS1248 Function File for Demo Functions
//
//   Description: SPI master talks to SPI slave (ADS1248) using 4-wire mode. LaunchPad
//   is used for this example.
//   ACLK = n/a, MCLK = SMCLK = DCO ~24MHz, BRCLK = SMCLK/7
//
//
//
//                                         MSP430F5529
//                                      -----------------
//                                     |             P1.3|<- Data Interrupt (DRDY)
//                                     |                 |
//                             START <-|P6.0         P6.2|-> Device Select (CS)
//                                     |                 |
//                             RESET <-|P6.1         P3.0|-> Data Out (UCB0SIMO -> DIN)
//                                     |                 |
//                                   <-|P1.6         P3.1|<- Data In (DOUT -> UCB0SOMI)
//                                     |                 |
// Serial Clock Out (UCB0CLK - SCLK) <-|P3.2         P2.7|->
//                                     |                 |
//                 I2C SCL (UCB1SCL) <-|P4.2         P8.1|->
//                                     |                 |
//                 I2C SDA (UCB1SDA)<>-|P4.1         P2.6|->
//                                     |                 |
//                                     |             P3.7|->
//                                     |                 |
//
//
//******************************************************************************

#ifndef ADS1248_H_
#define ADS1248_H_

#ifdef __cplusplus
extern "C" {
#endif

#if defined (__MSP430F5529__)
/* Definition of GPIO Port Bits Used for Communication */
// P6.2
#define ADS1248_CS      	0x04
// P3.0
#define ADS1248_DIN     	0x01
// P3.1
#define ADS1248_DOUT    	0x02
// P1.3
#define ADS1248_DRDY    	0x08
// P3.2
#define ADS1248_SCLK    	0x04
// P6.0
#define ADS1248_START    	0x01
// P6.1
#define ADS1248_RESET    	0x02

#elif defined (PART_TM4C1294NCPDT)
/// The clock rate of the internal XTAL...
#define DEVICE_ICLK 16384000
/// 2 MHz
#define SPI_SPEED 2000000
/// The TIVA has a 4 to 16 bit word size, to handle  32 bit words the TIVA needs to assemble
/// two groups of words together.  32 bits comprises of 2x16 words.
#define SPI_WORD_SIZE 8
/// This defines the clock polarity and phase.  See the datasheet for more info.
#define SPI_MOTO_MODE SSI_FRF_MOTO_MODE_1
#define SPI_BASE SSI3_BASE
#define SYSCTL_SSI SYSCTL_PERIPH_SSI3
#define ADS1248_SCLK GPIO_PQ0_SSI3CLK
#define ADS1248_DOUT GPIO_PQ3_SSI3XDAT1
#define ADS1248_DIN GPIO_PQ2_SSI3XDAT0
#define SPI_MODE SSI_MODE_MASTER
// Chip select feedthrough GPIO - we hand feed chip select so overages may be pumped out.
#define NCS_PORT GPIO_PORTK_BASE
#define ADS1248_CS  GPIO_PIN_0
#define SPI_GPIO_PORT GPIO_PORTQ_BASE
#define SPI_GPIO_PINS (GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3)
// GPIO definitions.
#define DRDY_PORT GPIO_PORTM_BASE
#define DRDY_PIN  GPIO_PIN_1
#define nDRDY_REG (((((volatile uint32_t*) 0x40063008))[0]))
// PB
#define START_PORT GPIO_PORTB_BASE
#define ADS1248_START  GPIO_PIN_4
#define RESET_PORT GPIO_PORTB_BASE
#define ADS1248_RESET  GPIO_PIN_5
#endif

/* Error Return Values */
#define ADS1248_NO_ERROR        0
#define ADS1248_ERROR			-1

/* Command Definitions */
// System Control
#define ADS1248_CMD_WAKEUP    	0x00
#define ADS1248_CMD_SLEEP     	0x03
#define ADS1248_CMD_SYNC     	0x05
#define ADS1248_CMD_RESET    	0x07
#define ADS1248_CMD_NOP    		0xFF
// Data Read
#define ADS1248_CMD_RDATA    	0x13
#define ADS1248_CMD_RDATAC    	0x15
#define ADS1248_CMD_SDATAC    	0x17
// Read Register
#define ADS1248_CMD_RREG    	0x20
// Write Register
#define ADS1248_CMD_WREG    	0x40
// Calibration
#define ADS1248_CMD_SYSOCAL    	0x60
#define ADS1248_CMD_SYSGCAL    	0x61
#define ADS1248_CMD_SELFOCAL    0x62


/* ADS1248 Register Definitions */
#define ADS1248_0_MUX0   		0x00
#define ADS1248_1_VBIAS     	0x01
#define ADS1248_2_MUX1	     	0x02
#define ADS1248_3_SYS0	    	0x03
#define ADS1248_4_OFC0	    	0x04
#define ADS1248_5_OFC1	    	0x05
#define ADS1248_6_OFC2	    	0x06
#define ADS1248_7_FSC0	    	0x07
#define ADS1248_8_FSC1	    	0x08
#define ADS1248_9_FSC2	    	0x09
#define ADS1248_10_IDAC0	   	0x0A
#define ADS1248_11_IDAC1	   	0x0B
#define ADS1248_12_GPIOCFG    	0x0C
#define ADS1248_13_GPIODIR    	0x0D
#define ADS1248_14_GPIODAT    	0x0E

/* ADS1248 Register 0 (MUX0) Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
//--------------------------------------------------------------------------------------------
//        BCS[1:0]       |             MUX_SP[2:0]           |            MUX_SN[2:0]
//
// Define BCS (burnout current source)
#define ADS1248_BCS_OFF			0x00
#define ADS1248_BCS_500nA		0x40
#define ADS1248_BCS_2uA			0x80
#define ADS1248_BCS_10uA		0xC0
// Define Positive MUX Input Channels
#define ADS1248_AINP0			0x00
#define ADS1248_AINP1			0x08
#define ADS1248_AINP2			0x10
#define ADS1248_AINP3			0x18
#define ADS1248_AINP4			0x20
#define ADS1248_AINP5			0x28
#define ADS1248_AINP6			0x30
#define ADS1248_AINP7			0x38
// Define Negative Mux Input Channels
#define ADS1248_AINN0			0x00
#define ADS1248_AINN1			0x01
#define ADS1248_AINN2			0x02
#define ADS1248_AINN3			0x03
#define ADS1248_AINN4			0x04
#define ADS1248_AINN5			0x05
#define ADS1248_AINN6			0x06
#define ADS1248_AINN7			0x07

/* ADS1248 Register 1 (VBIAS) Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
//--------------------------------------------------------------------------------------------
//                                         VBIAS[7:0]
//
#define ADS1248_VBIAS_OFF		0x00
#define ADS1248_VBIAS0			0x01
#define ADS1248_VBIAS1			0x02
#define ADS1248_VBIAS2			0x04
#define ADS1248_VBIAS3			0x08
#define ADS1248_VBIAS4			0x10
#define ADS1248_VBIAS5			0x20
#define ADS1248_VBIAS6			0x40
#define ADS1248_VBIAS7			0x80


/* ADS1248 Register 2 (MUX1) Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
//--------------------------------------------------------------------------------------------
//  CLKSTAT  |       VREFCON[1:0]    |      REFSELT[1:0]     |            MUXCAL[2:0]
//
// Define Internal Reference
#define ADS1248_INT_VREF_OFF	0x00
#define ADS1248_INT_VREF_ON		0x20
#define ADS1248_INT_VREF_CONV	0x40
// Define Reference Select
#define ADS1248_REF0			0x00
#define ADS1248_REF1			0x08
#define ADS1248_INT				0x10
#define ADS1248_INT_REF0		0x18
// Define System Monitor
#define ADS1248_MEAS_NORM		0x00
#define ADS1248_MEAS_OFFSET		0x01
#define ADS1248_MEAS_GAIN		0x02
#define ADS1248_MEAS_TEMP		0x03
#define ADS1248_MEAS_REF1		0x04
#define ADS1248_MEAS_REF0		0x05
#define ADS1248_MEAS_AVDD		0x06
#define ADS1248_MEAS_DVDD		0x07

/* ADS1248 Register 3 (SYS0) Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
//--------------------------------------------------------------------------------------------
//     0     |              PGA[2:0]             |                   DOR[3:0]
//
// Define Gain
#define ADS1248_GAIN_1			0x00
#define ADS1248_GAIN_2			0x10
#define ADS1248_GAIN_4			0x20
#define ADS1248_GAIN_8			0x30
#define ADS1248_GAIN_16			0x40
#define ADS1248_GAIN_32			0x50
#define ADS1248_GAIN_64			0x60
#define ADS1248_GAIN_128		0x70
//Define data rate
#define ADS1248_DR_5			0x00
#define ADS1248_DR_10			0x01
#define ADS1248_DR_20			0x02
#define ADS1248_DR_40			0x03
#define ADS1248_DR_80			0x04
#define ADS1248_DR_160			0x05
#define ADS1248_DR_320			0x06
#define ADS1248_DR_640			0x07
#define ADS1248_DR_1000			0x08
#define ADS1248_DR_2000			0x09

/* ADS1248 Register 4 (OFC0) Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
//--------------------------------------------------------------------------------------------
//                                         OFC0[7:0]
//

/* ADS1248 Register 5 (OFC1) Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
//--------------------------------------------------------------------------------------------
//                                         OFC1[7:0]
//

/* ADS1248 Register 6 (OFC2) Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
//--------------------------------------------------------------------------------------------
//                                         OFC2[7:0]
//

/* ADS1248 Register 7 (FSC0) Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
//--------------------------------------------------------------------------------------------
//                                         FSC0[7:0]
//

/* ADS1248 Register 8 (FSC1) Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
//--------------------------------------------------------------------------------------------
//                                         FSC1[7:0]
//

/* ADS1248 Register 9 (FSC2) Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
//--------------------------------------------------------------------------------------------
//                                         FSC2[7:0]
//

/* ADS1248 Register A (IDAC0) Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
//--------------------------------------------------------------------------------------------
//                     ID[3:0]                   | DRDY_MODE |              IMAG[2:0]
//
// Define DRDY mode on DOUT
#define ADS1248_DRDY_OFF		0x00
#define ADS1248_DRDY_ON			0x08
//Define IDAC Magnitude
#define ADS1248_IDAC_OFF		0x00
#define ADS1248_IDAC_50			0x01
#define ADS1248_IDAC_100		0x02
#define ADS1248_IDAC_250		0x03
#define ADS1248_IDAC_500		0x04
#define ADS1248_IDAC_750		0x05
#define ADS1248_IDAC_1000		0x06
#define ADS1248_IDAC_1500		0x07

/* ADS1248 Register B (IDAC1) Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
//--------------------------------------------------------------------------------------------
//                   I1DIR[3:0]                  |                   I2DIR[3:0]
//
// Define IDAC1 Output
#define ADS1248_IDAC1_A0		0x00
#define ADS1248_IDAC1_A1		0x10
#define ADS1248_IDAC1_A2		0x20
#define ADS1248_IDAC1_A3		0x30
#define ADS1248_IDAC1_A4		0x40
#define ADS1248_IDAC1_A5		0x50
#define ADS1248_IDAC1_A6		0x60
#define ADS1248_IDAC1_A7		0x70
#define ADS1248_IDAC1_EXT1		0x80
#define ADS1248_IDAC1_EXT2		0x90
#define ADS1248_IDAC1_OFF		0xF0
// Define IDAC2 Output
#define ADS1248_IDAC2_A0		0x00
#define ADS1248_IDAC2_A1		0x01
#define ADS1248_IDAC2_A2		0x02
#define ADS1248_IDAC2_A3		0x03
#define ADS1248_IDAC2_A4		0x04
#define ADS1248_IDAC2_A5		0x05
#define ADS1248_IDAC2_A6		0x06
#define ADS1248_IDAC2_A7		0x07
#define ADS1248_IDAC2_EXT1		0x08
#define ADS1248_IDAC2_EXT2		0x09
#define ADS1248_IDAC2_OFF		0x0F

/* ADS1248 Register C (GPIOCFG) Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
//--------------------------------------------------------------------------------------------
//                                         IOCFG[7:0]
//
// Define GPIO (0-Analog; 1-GPIO)
#define ADS1248_GPIO_0			0x01
#define ADS1248_GPIO_1			0x02
#define ADS1248_GPIO_2			0x04
#define ADS1248_GPIO_3			0x08
#define ADS1248_GPIO_4			0x10
#define ADS1248_GPIO_5			0x20
#define ADS1248_GPIO_6			0x40
#define ADS1248_GPIO_7			0x80

/* ADS1248 Register D (GPIODIR) Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
//--------------------------------------------------------------------------------------------
//                                         IODIR[7:0]
//
// Define GPIO Direction (0-Output; 1-Input)
#define ADS1248_IO_0			0x01
#define ADS1248_IO_1			0x02
#define ADS1248_IO_2			0x04
#define ADS1248_IO_3			0x08
#define ADS1248_IO_4			0x10
#define ADS1248_IO_5			0x20
#define ADS1248_IO_6			0x40
#define ADS1248_IO_7			0x80

/* ADS1248 Register E (GPIODAT) Definition */
//   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0
//--------------------------------------------------------------------------------------------
//                                         IOIDAT[7:0]
//
#define ADS1248_OUT_0			0x01
#define ADS1248_OUT_1			0x02
#define ADS1248_OUT_2			0x04
#define ADS1248_OUT_3			0x08
#define ADS1248_OUT_4			0x10
#define ADS1248_OUT_5			0x20
#define ADS1248_OUT_6			0x40
#define ADS1248_OUT_7			0x80

void ADS1148_Quail_Config(void);
/* Low Level ADS1248 Device Functions */
void InitConfig(void);							// Device initialization, setting a default starting condition
int ADS1248WaitForDataReady(int Timeout);		// DRDY polling, with a maximum polling wait time if needed for break from loop; Timeout=0 is no timeout
void ADS1248AssertCS(int fAssert);				// Assert/deassert CS (0=low, 1=high)
void ADS1248SendByte(unsigned char cData );		// Send byte to the ADS1248
unsigned char ADS1248ReceiveByte(void);			// Receive byte from the ADS1248

/* ADS1248 Higher Level Functions */
// Commands as given in datasheet
void ADS1248SendWakeup(void);					// Send device Wakeup Command
void ADS1248SendSleep(void);					// Set device to Sleep
void ADS1248SendSync(void);						// Sync device
void ADS1248SendResetCommand(void);				// Send a device Reset Command
long ADS1248ReadData(void);						// Read the data results by command
void ADS1248ReadRegister(int StartAddress, int NumReg, unsigned * pData);	 // Read the register(s) (NumReg is the total number of registers read)
void ADS1248WriteRegister(int StartAddress, int NumReg, unsigned * pData);   // Write the register(s) (NumReg is the total number of registers written)
void ADS1248WriteSequence(int StartAddress, int NumReg, unsigned * pData);   // Write the register(s) (NumReg is the total number of registers written)
void ADS1248SendRDATAC(void);					// Read data continuous mode
void ADS1248SendSDATAC(void);					// Stop read data continuous mode
void ADS1248SendSYSOCAL(void);					// System offset calibration
void ADS1248SendSYSGCAL(void);					// System gain calibration
void ADS1248SendSELFOCAL(void);					// Self offset calibration

/* Register Set Value Commands */
// Relate to MUX0
int ADS1248SetBurnOutSource(int BurnOut);
int ADS1248SetChannel(int pMux, int vMux);		// pMux (0=AINP. 1=AINN); vMux (value of mux channel selection)
// Relate to VBIAS
int ADS1248SetBias(unsigned char vBias);
// Relate to MUX1
int ADS1248SetIntRef(int sRef);
int ADS1248SetVoltageReference(int VoltageRef);
int ADS1248SetSystemMonitor(int Monitor);
// Relate to SYS0
int ADS1248SetGain(int Gain);
int ADS1248SetDataRate(int DataRate);
// Relate to OFC (3 registers)
int ADS1248SetOFC(long RegOffset);
// Relate to FSC (3 registers)
int ADS1248SetFSC(long RegGain);
// Relate to IDAC0
int ADS1248SetDRDYMode(int DRDYMode);
int ADS1248SetCurrentDACOutput(int CurrentOutput);
// Relate to IDAC1
int ADS1248SetIDACRouting(int IDACroute, int IDACdir);		// IDACRoute (0 = I1DIR, 1 = I2DIR)
// Relate to GPIOCFG
int ADS1248SetGPIOConfig(unsigned char cdata);
// Relate to GPIODIR
int ADS1248SetGPIODir(unsigned char cdata);
// Relate to GPIODAT
int ADS1248SetGPIO(unsigned char cdata);

/* Register Get Value Commands */
// Relate to MUX0
int ADS1248GetBurnOutSource(void);
int ADS1248GetChannel(int cMux);
// Relate to VBIAS
unsigned char ADS1248GetBias(void);
//Relate to MUX1
int ADS1248GetCLKSTAT(void);
int ADS1248GetIntRef(void);
int ADS1248GetVoltageReference(void);
int ADS1248GetSystemMonitor(void);
// Relate to SYS0
int ADS1248GetGain(void);
int ADS1248GetDataRate(void);
// Relate to OFC (3 registers)
long ADS1248GetOFC(void);
// Relate to FSC (3 registers)
long ADS1248GetFSC(void);
// Relate to IDAC0
int ADS1248GetID(void);
int ADS1248GetDRDYMode(void);
int ADS1248GetCurrentDACOutput(void);
// Relate to IDAC1
int ADS1248GetIDACRouting(int WhichOne); 		// IDACRoute (0 = I1DIR, 1 = I2DIR)
// Relate to GPIOCFG
unsigned char ADS1248GetGPIOConfig(void);
// Relate to GPIODIR
unsigned char ADS1248GetGPIODir(void);
// Relate to GPIODAT
unsigned char ADS1248GetGPIO(void);

/* Miscellaneous Commands */
long ADS1248RDATACRead(void);		// reads data directly based on RDATAC mode (writes NOP) and 32 SCLKs

/* Hardware Control Functions for Device Pin Control */
// Possible Need for Reset, Start (power down) (0-low, 1-high, 2-pulse)
int ADS1248SetStart(int nStart);
int ADS1248SetReset(int nReset);

void InitSPI(void);
void InitDevice(void);

#ifdef __cplusplus
}
#endif

#endif /* ADS1248_H_ */
