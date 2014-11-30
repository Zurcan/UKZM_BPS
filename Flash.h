//*----------------------------------------------------------------------------
//*         ATMEL Microcontroller Software Support  -  ROUSSET  -
//*----------------------------------------------------------------------------
//* The software is delivered "AS IS" without warranty or condition of any
//* kind, either express, implied or statutory. This includes without
//* limitation any warranty or condition with respect to merchantability or
//* fitness for any particular purpose, or against the infringements of
//* intellectual property rights of others.
//*----------------------------------------------------------------------------
//* File Name           : Flash.h
//* Object              : Flash constan description
//* Creation            : JPP  30/Jun/2004
//*
//*----------------------------------------------------------------------------


#ifndef Flash_h
#define Flash_h

/*-------------------------------*/
/* Flash Status Field Definition */
/*-------------------------------*/

#define 	AT91C_MC_FSR_MVM 		((unsigned int) 0xFF << 8)		// (MC) Status Register GPNVMx: General-purpose NVM Bit Status
#define 	AT91C_MC_FSR_LOCK 		((unsigned int) 0xFFFF << 16)	// (MC) Status Register LOCKSx: Lock Region x Lock Status


#define	 ERASE_VALUE 		0xFFFFFFFF

/*-----------------------*/
/* Flash size Definition */
/*-----------------------*/
/* 64 Kbytes of Internal High-speed Flash, Organized in 512 Pages of 128 Bytes */

#define  FLASH_PAGE_SIZE_BYTE	128
#define  FLASH_PAGE_SIZE_LONG	32

#define  FLASH_LOCK_BITS_SECTOR	16
#define  FLASH_SECTOR_PAGE		32
#define  FLASH_LOCK_BITS		16    /* 16 lock bits, each protecting 16 sectors of 32 pages*/
#define  FLASH_BASE_ADDRESS		0x00100000
/*------------------------------*/
/* External function Definition */
/*------------------------------*/

/* Flash function */
extern void AT91F_Flash_Init(void);
extern int AT91F_Flash_Check_Erase(unsigned int * start, unsigned int size);
extern int AT91F_Flash_Erase_All(void);
extern int AT91F_Flash_Write( unsigned int Flash_Address ,int size ,unsigned int * buff);
extern int AT91F_Flash_Write_all( unsigned int Flash_Address ,int size ,unsigned int * buff);

/* Lock Bits functions */
extern int AT91F_Flash_Lock_Status(void);
extern int AT91F_Flash_Lock (unsigned int Flash_Lock);
extern int AT91F_Flash_Unlock(unsigned int Flash_Lock);

/* NVM bits functions */
extern int AT91F_NVM_Status(void);
extern int AT91F_NVM_Set (unsigned char NVM_Number);
extern int AT91F_NVM_Clear(unsigned char NVM_Number);

/* Security bit function */
extern int AT91F_SET_Security_Status (void);
extern int AT91F_SET_Security (void);

#endif /* Flash_h */
