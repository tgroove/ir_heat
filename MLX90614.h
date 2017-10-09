/*************************************************************************
 * DESCRIPTION: A driver module for the MLX90614 IR sensors on the IIC.
 * MCU: 9S12DP512, fE = 24MHz, IIC
 * AUTHOR: Scott Corbridge
 * 			-Modified from Todd Morton's DS3232IIC driver  
*************************************************************************
* MLX90614SMBusIICInit - Initialize IIC for the MLX90614
*************************************************************************/
extern void MLX90614SMBusIICInit(void);

/*************************************************************************
* RTCWr - Write one byte to MLX90614.
*         waddr - MLX90614 register address to write
* 		  daddr - addres of MLX90614 to be written to
*         wdata - Data to be written to waddr
* 		  pec - CRC8 data byte for message to be sent
* 		  cmnd - ram or eeprom write command
* 		Returns ERROR or NOERROR
*************************************************************************/
extern INT8U MLX90614Wr(INT8U waddr, INT8U daddr, INT16U wdata, INT8U pec, INT8U cmnd);

/*************************************************************************
* RTCRd - Read one byte from the MLX90614. 
*         raddr - MLX90614 register address to be read
* 		  daddr - MLX90614 device address to read from
* 		  cmnd - ram or eeprom read command
* 		  error - ERROR or NOERROR
* 		Returns 2 byte result
*************************************************************************/
extern INT16U MLX90614Rd(INT8U raddr, INT8U daddr, INT8U cmnd, INT8U *error);

/*************************************************************************
* GetTemp - Gets the temperature in degrees Fahrinheight
* 			temps - reference to the struct were temp data is stored
*************************************************************************/
void GetTemp(TEMPERATURES *temps);

/*************************************************************************
* SetAddr - Sets the addresses of the sensors.
* 			waddr - MLX90614 register address to be written to		//dont need always the same
* 			daddr - MLX90614 device to be written to
* 			cmnd - eeprom or ram write command                     //dont need always the same
* 			new_addr - the new address of the MLX90614
* 		Returns ERROR or NOERROR
*************************************************************************/
extern INT8U SetAddr(INT8U waddr, INT8U daddr, INT8U cmnd, INT16U new_addr);

/*********************************************************************************************
									CALCULATION PEC PACKET	
**********************************************************************************************
Name:			PEC_calculation
Function:		Calculates the PEC of received bytes
Parameters:	unsigned char pec[]		
					-A 6 char array initilized to zero (only works for 5 bytes)
					-has the last byte sent in postion 1(pre-shifted).
					-has the last byte received at zero.	
Return:		pec[0]-this byte contains calculated crc value
Comments: 	Refer to "System Managment BUS(SMBus) specification Version 2.0" and
				AN "SMBus comunication with MLX90614"
*********************************************************************************************/
unsigned char PEC_calculation(unsigned char pec[]);

/*************************************************************************
* MLX90614 Defines - Read/Write addresses, etc.
*************************************************************************/
#define INSIDESLVADDR  		0x5A<<1   /* Inside MLX90614 slave address */
#define OUTSIDESLVADDR 		0x5B<<1   /* Outside MLX90614 slave address */
#define MIDDLESLVADDR  		0x5C<<1   /* Middle MLX90614 slave address */
#define RAM		       		0x00
#define EEPROM	     		0x20
#define AMBIENTTEMPREG     	0x06
#define OBJTEMPREG       	0x07
#define CONFIGREG      		0x05
#define SLVADDRREG         	0x0E
#define NACK				1
#define ACK					0
#define ERASE				0x0000
#define NEWINSIDEADDR		0x005A
#define NEWOUTSIDEADDR		0x005B
#define NEWMIDDLEADDR		0x005C
#define OLDADDR				0x00
#define TIMEOUT				0xFFFF





