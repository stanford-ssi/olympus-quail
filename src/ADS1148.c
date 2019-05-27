/*
 * David Muller; Germán Alfaro
 * davidmuller10@mittymonarch.com; alfaro.germanevera@gmail.com
 */



#include "ADS1148.h"
#include <Arduino.h>





/*
 * ADS1148.c is the "driver" for the ADS1148.  It assumes 20SPS is
 * desired and a 50MHZ processor clock is used.
 *
 * openADS1148(): gets the ADS1148 ready for conversions; POWERS THE ADC, ties
 * start and reset high, sets up the SSI lines, sets ADC to 20SPS,
 * default channel, and SDATAC mode. 24 bit counts can be asked for immediately
 * upon openADS1148()s return.
 * pollADS1148(short channel, int trials): gets trials # conversions from the specified
 * channel (0,1,2 or 3).  The counts from all the trials are averaged and returned
 * as a float.
 * closeADS1148(): POWERS OFF THE ADC, ties start and reset to low, and disables
 * SSI3.
 *
 * ADS1148GetValue() returns just 1 count. ADS1148GetValue() does not include
 * any delays...the user should be sure to give appropriate delays between conversions.
 * ADS1148ChangeChannel(...) allows you to switch between the 4 differential
 * channels (0,1,2,3).
 */





/*
 * openADS1148() performs initial set up of the ADS1148.
 * POWERS THE ADC, ties start and reset high, sets up the SSI, sets
 * the sampling rate to 20SPS, sets up the internal
 * reference for use, and puts the ADS1148 in SDATAC mode. Conversions
 * can be asked for immediately after this function returns (no additional
 * delays are needed).
 *
 * For safety, it assumes the ADS1148 was just booted and gives the necessary
 * delays as the ADS1148 boots up.
 */
void openADS1148(void)
{
	//Assume we're at default 5SPS and START pin just went high, this delay lets the filter reset
        delay(SPS5DELAY);

        uint8_t out[13];

	//Send the stop reading data continuously command (so we can read registers and get conversions by sending RDATA)
	// ROM_SSIDataPut(SSI3_BASE, SDATAC);
        out[0] = send_load_cell(SDATAC);

	//let the filter reset
        delay(SPS5DELAY);

	//write to SYS0 to change SPS
	// ROM_SSIDataPut(SSI3_BASE, 0x43);
	// ROM_SSIDataPut(SSI3_BASE, 0x00);  //0x00 is # of bytes to write minus 1
	// ROM_SSIDataPut(SSI3_BASE, 0x02);  //20 SPS
        out[1] = send_load_cell(0x43);
        out[2] = send_load_cell(0x00);
        out[3] = send_load_cell(0x02);

	//let digital filter reset
	delay(SPS20DELAY);

	//read back that register
	// ROM_SSIDataPut(SSI3_BASE, 0x23);
	// ROM_SSIDataPut(SSI3_BASE, 0x00); //0x00 is # of bytes to write minus 1
	// ROM_SSIDataPut(SSI3_BASE, NOP);  //1st NOP issues 8 SCLK's to get 1 byte (should get 0xFF back)
        out[4] = send_load_cell(0x23);
        out[5] = send_load_cell(0x00);
        out[6] = send_load_cell(NOP);



	//write to MUX1 to set up internal reference
	// ROM_SSIDataPut(SSI3_BASE, 0x42);
	// ROM_SSIDataPut(SSI3_BASE, 0x00); //0x00 is # of bytes to write minus 1
	// ROM_SSIDataPut(SSI3_BASE, 0x30); //0x30 sets up internal reference for use
        out[7] = send_load_cell(0x42);
        out[8] = send_load_cell(0x00);
        out[9] = send_load_cell(0x30);

	//let filter reset before trying to get conversions. THIS DELAY MAY BE BREAKING EVERYTHING
	//ROM_SysCtlDelay(SPS20DELAY);

	//read back that register
	// ROM_SSIDataPut(SSI3_BASE, 0x22);
	// ROM_SSIDataPut(SSI3_BASE, 0x00); //0x00 is # of bytes to write minus 1
	// ROM_SSIDataPut(SSI3_BASE, NOP);  //1st NOP issues 8 SCLK's to get 1 byte (should get 0xFF back)
        out[10] = send_load_cell(0x22);
        out[11] = send_load_cell(0x00);
        out[12] = send_load_cell(NOP);

        print_ADS1148_init(out);


	/*
	 * USE REF1 Input pair
	 */
	/*
	//write to MUX1
	ROM_SSIDataPut(SSI3_BASE, 0x42);
	ROM_SSIDataPut(SSI3_BASE, 0x00); //0x00 is # of bytes to write minus 1
	ROM_SSIDataPut(SSI3_BASE, 0x28); //0x28 sets up Ref1 for the ref, otherwise normal operation

	//let filter reset before trying to get conversions
	ROM_SysCtlDelay(SPS20DELAY);
	 */

	/*
	 * TEST READ REGISTER
	 */
/*
	//write to MUX1
	ROM_SSIDataPut(SSI3_BASE, 0x42);
	ROM_SSIDataPut(SSI3_BASE, 0x00); //0x00 is # of bytes to write minus 1
	ROM_SSIDataPut(SSI3_BASE, 0x30); //should read ox30 back

	//let filter reset before trying to get conversions
	//ROM_SysCtlDelay(SPS20DELAY);




	//send read register command
	ROM_SSIDataPut(SSI3_BASE, 0x22);
	ROM_SSIDataPut(SSI3_BASE, 0x00); //0x00 is # of bytes to write minus 1
	ROM_SSIDataPut(SSI3_BASE, NOP);  //1st NOP issues 8 SCLK's to get 1 byte (should get 0xFF back)

	ROM_SysCtlDelay(SPS20DELAY);
*/





	/*
	 * TEST WITH THE SYSTEM MONITOR.
	 */
/*
	//write to MUX1
	ROM_SSIDataPut(SSI3_BASE, 0x42);
	ROM_SSIDataPut(SSI3_BASE, 0x00); //0x00 is # of bytes to write minus 1
	ROM_SSIDataPut(SSI3_BASE, 0x33); //0x33 uses int ref and does temp diode

	//let filter reset before trying to get conversions
	ROM_SysCtlDelay(SPS20DELAY);



	UARTprintf("SysMonitor: \n");
	UARTprintf("Taking 3 measurements...\n");
	signed long w;
	int x = 3;
	while(x)
	{
		SysCtlDelay(ONESEC);
		w = ADS1148GetValue();
		UARTprintf("%d\n",w);
		--x;
	}

	closeADS1148();
	while(1)
	{
	}
*/



	/*
	 * Calibration commands
	 */
	/*
    //send the SELFOCAL command for calibration
    ROM_SSIDataPut(SSI3_BASE, SELFOCAL);

    //allow the ADS1148 proper calibration time
    ROM_SysCtlDelay(SPS20CALIBRATIONDELAY);
	 */

}




/*
 * pollADS1148() returns the average number of counts from a user
 * specified number of conversions (trials).  User can specify
 * the channel (0,1,2, or 3).
 * This function ASSUMES THE ADC IS RUNNING AT 20SPS and
 * delays appropriately between conversions.
 */
float pollADS1148(short channel, int trials)
{
	//change to the specified channel
	ADS1148ChangeChannel(channel);

	long sample = 0;
	int i;


	//make trials # of conversions
	for (i = 0; i < trials; i++)
	{
		sample += ADS1148GetValue();

		//next conversion will be ready after about 50ms
		// ROM_SysCtlDelay(SPS20DELAY);
		delay(SPS20DELAY);
	}

	return (float) (sample/trials);
}





/*
 * closeADS1148 turns off start, reset, power and disables
 * SSI3.
 */
void closeADS1148(void)
{
}









/*
 * Selects between the 4 differential channels and delays to let
 * the filter reset (ASSUMES 20SPS OPERATION).
 * The only acceptable arguments are 0, 1, 2, and 3:
 * 0 selects AIN0 as positive, AIN1 as negative.
 * 1 selects AIN2 as positive, AIN3 as negative.
 * 2 selects AIN4 as positive, AIN5 as negative...
 */
void ADS1148ChangeChannel(short channel)
{
	switch(channel)
	{
		//+AIN0, -AIN1
		case 0:
		{
			// ROM_SSIDataPut(SSI3_BASE, 0x40); //write to MUX0 register to change channels
			// ROM_SSIDataPut(SSI3_BASE, 0x00); //0x00 is the # of bytes to write minus 1
			// ROM_SSIDataPut(SSI3_BASE, 0x01); //0x00 means AIN0 is positive, AIN1 is negative (default)
			send_load_cell(0x40);
			send_load_cell(0x00);
			send_load_cell(0x01);
			break;
		}

		//+AIN2, -AIN3
		case 1:
		{
			// ROM_SSIDataPut(SSI3_BASE, 0x40);
			// ROM_SSIDataPut(SSI3_BASE, 0x00);
			// ROM_SSIDataPut(SSI3_BASE, 0x13); //0x13 means +AIN2, -AIN3
			send_load_cell(0x40);
			send_load_cell(0x00);
			send_load_cell(0x13);
			break;
		}

		//+AIN4, -AIN5
		case 2:
		{
			// ROM_SSIDataPut(SSI3_BASE, 0x40);
			// ROM_SSIDataPut(SSI3_BASE, 0x00);
			// ROM_SSIDataPut(SSI3_BASE, 0x25); //0x25 means +AIN4, -AIN5
			send_load_cell(0x40);
			send_load_cell(0x00);
			send_load_cell(0x25);
			break;
		}

		//+AIN6, -AIN7
		case 3:
		{
			// ROM_SSIDataPut(SSI3_BASE, 0x40);
			// ROM_SSIDataPut(SSI3_BASE, 0x00);
			// ROM_SSIDataPut(SSI3_BASE, 0x37); //0x37 means +AIN6, -AIN7
			send_load_cell(0x40);
			send_load_cell(0x00);
			send_load_cell(0x37);
			break;
		}

	}

	//let the digital filter reset
	// ROM_SysCtlDelay(SPS20DELAY);
        delay(SPS20DELAY);

}



/*
 * ADS1148GetValue() retrieves a conversion from the ADC.  It assumes
 * the ADC is operating in SDATAC mode.  It does not include any delays;
 * instead it assumes the ADC will be given proper wait/set-up delays
 * elsewhere in the code.
 */
signed long ADS1148GetValue(void)
{
	//the last 3 cells each store a byte of the 24 bit count
	unsigned long counts[4];


	unsigned long garbage[1];

	//our 24 bit count
	signed long result;

	int i;


	// Flush receive FIFO
	// while(ROM_SSIDataGetNonBlocking(SSI3_BASE, &garbage[0] ))
	// {
	// }

	//send RDATA command
	// ROM_SSIDataPut(SSI3_BASE, RDATA);
	// ROM_SSIDataPut(SSI3_BASE, NOP);  //1st NOP issues 8 SCLK's to get 1st byte of the count
	// ROM_SSIDataPut(SSI3_BASE, NOP);  //2nd NOP for 2nd...
	// ROM_SSIDataPut(SSI3_BASE, NOP);
	counts[0] = send_load_cell(RDATA);
	counts[1] = send_load_cell(NOP);
	counts[2] = send_load_cell(NOP);
	counts[3] = send_load_cell(NOP);


	// Wait until SSI is done transferring all the data in the transmit FIFO.
	// while(ROM_SSIBusy(SSI3_BASE))
	// {
	// }

	//get the 4 bytes that come back (0x00 is received after we send RDATA--it is stored in counts[0])
	for(i = 0; i < 4; ++i)
	{
		//This fcn waits until there is data in the receive FIFO before returning.
		// ROM_SSIDataGet(SSI3_BASE, &counts[i]);

		//Since we are using 8-bit data, mask off the MSB.
		counts[i] &= 0x00FF;
	}



	//concatenate the 3 bytes of the 24 bit count (the 6 nibbles from the ADC)
	result =  (counts[1] << 16) | (counts[2] << 8 ) | counts[3];


	// we need to sign the extra two nibbles from the 32 bit long in case of being a signed number,  because bit 23 could be the signed "-" of the negative voltage
	//the bit to check either 1 or 0
  //  long result_;
//
/*	long bitcheck;
	bitcheck = 0x00800000;


		if ( result & (1<<23) == bitcheck )
	{

*/
		result = 0xff000000 | result;



//	}


	return(result);
}
