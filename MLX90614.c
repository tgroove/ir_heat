/*************************************************************************
 * DESCRIPTION: A driver module for the MLX90614 IR sensors on the IIC.
 * MCU: 9S12DP512, fE = 24MHz, IIC
 * AUTHOR: Scott Corbridge
 * 			-Modified from Todd Morton's DS3232IIC driver   
**************************************************************************
* Master header file
*************************************************************************/
#include "includes.h"
/*************************************************************************
 * IIC definitions
*************************************************************************/
#define TCF     0x80
#define IBB     0x20
#define IBIF    0x02
#define RXAK    0x01
#define IBEN    0x80
#define MSSL    0x20
#define TXRX    0x10
#define TXAK    0x08
#define RSTA    0x04
/*************************************************************************
* Function prototypes (Public)
*************************************************************************/
void MLX90614IICInit(void);
INT16U MLX90614Rd(INT8U raddr, INT8U daddr, INT8U cmnd, INT8U *error);
INT8U MLX90614Wr(INT8U waddr, INT8U daddr, INT16U wdata, INT8U pec, INT8U cmnd);
static INT8U SMBusIICWr(INT8U dout);
static INT8U SMBusIICRd(INT8U ack, INT8U *err);
static void SMBusIICStop(void);
static void SMBusIICStart(void);
static void MLX90614Dly5us(void);
void GetTemp(TEMPERATURES *temps);
INT8U SetTemp(INT8U waddr, INT8U daddr, INT8U cmnd, INT16U new_addr);
INT8U crc8 (INT8U *data_in, INT8U number_of_bytes_to_read);


/*************************************************************************
* MLX90614SMBusIICInit - Initialize IIC for the MLX90614
*************************************************************************/
void MLX90614SMBusIICInit(void){
    IBFD = 0x1F;                  /* 1F Set ICC rate, 100kbps, w/ fE=24MHz */
    while((IBSR & IBB) == IBB){}  /* wait for IIC bus not to be busy */
    IBCR = IBEN;      /* Enable IIC*/
}                                                

/*************************************************************************
* MLX90614Wr - Standard MLX90614 EEPROM or RAM write
*************************************************************************/
INT8U MLX90614Wr(INT8U waddr, INT8U daddr, INT16U wdata, INT8U pec, INT8U cmnd){
	INT8U *ptr = &wdata;
	INT8U err;
	
    SMBusIICStart();					/* Send start condition */
    err = SMBusIICWr(daddr);               	/* Send slave device address */
    if(err)
    	return err;
    err = SMBusIICWr(waddr | cmnd);			/* Send register address */
    if(err)
    	return err;
    err = SMBusIICWr(ptr[1]);					/* Send the low byte */
    if(err)
    	return err;
    err = SMBusIICWr(ptr[0]);					/* Send the high byte */
    if(err)
    	return err;
    err = SMBusIICWr(pec);					/* Send the PEC byte */
    if(err)
    	return err;
    SMBusIICStop();						/* Read the stop condition */
    OSTimeDly(10);				/* Time needed for cell to be written */
    
    return 0;
}
/*************************************************************************
* MLX90614Rd - Standard MLX90614 RAM or EEPROM Read.
*************************************************************************/
INT16U MLX90614Rd(INT8U raddr, INT8U daddr, INT8U cmnd, INT8U *error){
	INT16U temp;
	INT8U TempHighByte = 0;
	INT8U TempLowByte = 0;
	INT8U err = 0;
	INT8U pec;
	INT8U pec_read[6];
    INT8U *ptr = &temp;
    *error = 0;
    
    SMBusIICStart();							/* Send start condition */
    err = SMBusIICWr(daddr);              			/* Send device address */
    if(err)
    {
    	*error = ERROR;
    	return 0;
    }	
    err = SMBusIICWr(raddr | cmnd);					/* Send register address */
    if(err)
    {
    	*error = ERROR;
    	return 0;
    }
    IBCR |= RSTA;								/* Send repeated start */
    err = SMBusIICWr(daddr);							/* Re-send device address */
    if(err)
    {
    	*error = ERROR;
    	return 0;
    }
    TempLowByte = SMBusIICRd(ACK, &err);		/* Read the low byte */
    if(err)
    {
    	*error = ERROR;
    	return 0;
    }
    TempHighByte = SMBusIICRd(ACK, &err);		/* Read the high byte */
    if(err)
    {
    	*error = ERROR;
    	return 0;
    }
    pec = SMBusIICRd(NACK, &err);						/* Read the PEC byte */
    if(err)
    {
    	*error = ERROR;
    	return 0;
    }
    ptr[0] = TempHighByte;						/* Store the 16 bit value */
    ptr[1] = TempLowByte;
    
    
	//data for PEC read object temp.
    pec_read[5] = daddr;
    pec_read[4] = cmnd + raddr;
    pec_read[3] = daddr;
    pec_read[2] = ptr[1];
    pec_read[1] = ptr[0];
    pec_read[0] = 0;

	//return error if PEC error
	if(pec != PEC_calculation(pec_read))
	{
		*error = ERROR;
        return 0;
	}
	
    return temp;						/* Return that value */
}
/*************************************************************************
* SMBusIICWr - Write one byte to IIC.
*************************************************************************/
static INT8U SMBusIICWr(INT8U dout){
	INT16U timeout = 0;
    IBDR = dout;                    /* Send data/address */
    
    //timeout for failed communication SMBUS defined 25ms
    for(timeout = TIMEOUT;;timeout--)
    {
    	if((IBSR & IBIF) != 0)						/* Wait for completion */
    	{
    		IBSR |= IBIF;
    		break;
    	}
    	else if(timeout == 0)
    	{
    		SMBusIICStop();
    		return ERROR;
    	}
    	
    }
    
    //timeout for failed communication SMBUS defined 25ms
    for(timeout = TIMEOUT;;timeout--)
    {
    	if((IBSR & RXAK) == 0)				/* Check for Acknowledge */
    	{
    		break;
    	}
    	else if(timeout == 0)
    	{
    		SMBusIICStop();
    		return ERROR;
    	}
    	
    }
    
    //No error
    return NOERROR;
}

/*************************************************************************
* SMBusIICRd - Read one byte to IIC.
*************************************************************************/
static INT8U SMBusIICRd(INT8U ack, INT8U *err){
    INT8U din;
    INT16U timeout;
    
    //clear error
    *err = NOERROR;
    
    IBCR &= ~TXRX;        /* Set to master receive mode */
    if(ack)				  /* if last byte read */
    {
        IBCR |= TXAK;
    }
    din = IBDR;           /* dummy read */  
    
    //timeout for failed communication SMBUS defined 25ms
    for(timeout = TIMEOUT;;timeout--)
    {
    	if((IBSR & IBIF) != 0)					/* Wait for completion */
    	{
    		IBSR |= IBIF;
    		break;
    	}
    	else if(timeout == 0)
    	{
    		*err = ERROR;
    		//SMBusIICStop();
    		return 0;
    	}
    	
    }

    if(ack)					/* if last byte read */
    {
        SMBusIICStop();
    }
    din = IBDR;           /* real read */
    return din;
}

/*************************************************************************
* SMBusIICStop - Generate a stop to free the IIC bus.
*************************************************************************/
static void SMBusIICStop(void){
    IBCR &= ~MSSL;					/* generate stop, clear MSSL*/
    MLX90614Dly5us();                /* Needed to meet tBUF */
}
/*************************************************************************
* SMBusIICStart - Generate a start to grab the IIC bus.
*************************************************************************/
static void SMBusIICStart(void){
    IBCR &= ~TXAK; 
    IBCR |= (MSSL|TXRX);
}
/********************************************************************
* void RtcDly5us(void)
*
*  DESCRIPTION: Delays, at least, 5us. Assumes 24MHz E-clock.
********************************************************************/
static void MLX90614Dly5us(void){
    INT8U cnt;
    for(cnt=120;cnt > 0;cnt--){}
}
/*************************************************************************/

/*************************************************************************
* GetTemps - Gets the temperatures in degrees Fahrinheight
*************************************************************************/
void GetTemp(TEMPERATURES *temps)
{	
	float  InsideTemp = 0;
	float  OutsideTemp = 0;
	float  MiddleTemp= 0;
	INT8U errcount = 0;
	INT8U err;
	
    /* get inside temp */
	InsideTemp = MLX90614Rd(OBJTEMPREG, INSIDESLVADDR, RAM, &err);
    while(err)
    {
    	errcount++;
    	if(errcount == 2)
    	{
    		break;
    	}
    	InsideTemp = MLX90614Rd(OBJTEMPREG, INSIDESLVADDR, RAM, &err);
    }
    if(errcount == 2)
    {
    	//temps->InsideTemperature = (INT16U)0;
    }
    else
    {
    	/* convert temp */
    	InsideTemp = (InsideTemp * .02) - 273.15;
    	InsideTemp = ((InsideTemp * 9) / 5) + 32;
    	/* return Temp */
	    temps->InsideTemperature = (INT16U)(InsideTemp*10);
    }
    errcount = 0;  		//reset error counter for next sensor
    
    
    /* get middle temp */
    MiddleTemp = MLX90614Rd(OBJTEMPREG, MIDDLESLVADDR, RAM, &err);
    while(err)
    {
    	errcount++;
    	if(errcount == 2)
    	{
    		break;
    	}
    	MiddleTemp = MLX90614Rd(OBJTEMPREG, MIDDLESLVADDR, RAM, &err);
    }
    if(errcount == 2)
    {
    	//temps->MiddleTemperature = (INT16U)0;
    }
    else
    {
    	/* convert temp */
    	MiddleTemp = (MiddleTemp * .02) - 273.15;
    	MiddleTemp = ((MiddleTemp * 9) / 5) + 32;
    	/* return Temp */
    	temps->MiddleTemperature = (INT16U)(MiddleTemp*10);
    }
    errcount = 0;  		//reset error counter for next sensor
    
    
    /* get outside temp */
    OutsideTemp = MLX90614Rd(OBJTEMPREG, OUTSIDESLVADDR, RAM, &err);
    while(err)
    {
    	errcount++;
    	if(errcount == 2)
    	{
    		break;
    	}
    	OutsideTemp = MLX90614Rd(OBJTEMPREG, OUTSIDESLVADDR, RAM, &err);
    }
    if(errcount == 2)
    {
    	//temps->OutsideTemperature = (INT16U)0;
    }
    else
    {
    	/* convert temp */
    	OutsideTemp = (OutsideTemp * .02) - 273.15;
    	OutsideTemp = ((OutsideTemp * 9) / 5) + 32;
    	/* return Temp */
    	temps->OutsideTemperature = (INT16U)(OutsideTemp*10);
    }
    	
}


/*************************************************************************
* SetAddr - Sets the addresses of the sensors.
*************************************************************************/
INT8U SetAddr(INT8U waddr, INT8U daddr, INT8U cmnd, INT16U new_addr)
{
	INT8U errcount = 0;
	INT8U pec_erase[6];
	INT8U pec_write[6];
	INT8U pec;
	INT8U *ptr = &new_addr;
	
	//data for PEC send erase command
	pec_erase[1] = ERASE;
	pec_erase[2] = ERASE;
	pec_erase[3] = cmnd + waddr;
	pec_erase[4] = daddr;
	pec_erase[0] = 0;
	pec_erase[5] = 0;

	//data for PEC send new address
	pec_write[1] = ptr[0];
	pec_write[2] = ptr[1];
	pec_write[3] = cmnd + waddr;
	pec_write[4] = daddr;
	pec_write[0] = 0;
	pec_write[5] = 0;

	//erase addr //write new one
	pec = PEC_calculation(pec_erase);
	while(MLX90614Wr(waddr, daddr, ERASE, pec, cmnd))
    {
    	errcount++;
    	if(errcount == 3)
    	{
    		return ERROR;
    	}
    }
	//write new one
	pec = PEC_calculation(pec_write);
	while(MLX90614Wr(waddr, daddr, new_addr, pec, cmnd))
    {
    	errcount++;
    	if(errcount == 3)
    	{
    		return ERROR;
    	}
    }
}

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
INT8U PEC_calculation(INT8U pec[])
{
	INT8U 	crc[6];
	INT8U	BitPosition=47;
	INT8U	shift;
	INT8U	i;
	INT8U	j;
	INT8U	temp;

	do{
		/*Load pattern value 0x000000000107*/
		crc[5]=0;				
		crc[4]=0;
		crc[3]=0;
		crc[2]=0;
		crc[1]=0x01;
		crc[0]=0x07;
		
		/*Set maximum bit position at 47 ( six bytes byte5...byte0,MSbit=47)*/
		BitPosition=47;	
		
		/*Set shift position at 0*/		
		shift=0;
				
		/*Find first "1" in the transmited message beginning from the MSByte byte5*/
		i=5;					
		j=0;
		while((pec[i]&(0x80>>j))==0 && i>0){
			BitPosition--;
			if(j<7){
				j++;
			}
			else{
				j=0x00;
				i--;
			}
		}/*End of while */
		
		/*Get shift value for pattern value*/
		shift=BitPosition-8;	
		
		/*Shift pattern value */
		while(shift){
			for(i=5; i<0xFF; i--){
				if((crc[i-1]&0x80) && (i>0)){
					temp=1;
				}
				else{
					temp=0;
				}
				crc[i]<<=1;
				crc[i]+=temp;
			}/*End of for*/
			shift--;
		}/*End of while*/
		
		
		/*Exclusive OR between pec and crc*/		
		for(i=0; i<=5; i++){
			pec[i] ^=crc[i];
		}/*End of for*/
		
	}while(BitPosition>8);/*End of do-while*/
	
	return pec[0];
}/*End of PEC_calculation*/
