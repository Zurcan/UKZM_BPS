//*----------------------------------------------------------------------------
//*         ATMEL Microcontroller Software Support  -  ROUSSET  -
//*----------------------------------------------------------------------------
//* The software is delivered "AS IS" without warranty or condition of any
//* kind, either express, implied or statutory. This includes without
//* limitation any warranty or condition with respect to merchantability or
//* fitness for any particular purpose, or against the infringements of
//* intellectual property rights of others.
//*----------------------------------------------------------------------------
//* File Name           : Flash.c
//* Object              : Flash routine
//* Creation            : JPP   30/Jun/2004
//* Modif               : JPM   16/Nov/2004 Flash write status
//* 1.1   12/Sep/05 JPP : Change MC_FMR Setting
//*----------------------------------------------------------------------------

// Include Standard files
#include "Board.h"
#include "Flash.h"

extern  void AT91F_disable_interrupt(void);
extern  void AT91F_enable_interrupt(void);


//*----------------------------------------------------------------------------
//* \fn    AT91F_Flash_Init
//* \brief Flash init
//*----------------------------------------------------------------------------
void AT91F_Flash_Init (void)
{
    //* Set number of Flash Waite sate
    //  SAM7S64 features Single Cycle Access at Up to 30 MHz
    //  if MCK = 47923200, 72 Cycles for 1 µseconde ( field MC_FMR->FMCN)
        AT91C_BASE_MC->MC_FMR = ((AT91C_MC_FMCN)&(72 <<16)) | AT91C_MC_FWS_1FWS ;
}
//*----------------------------------------------------------------------------
//* \fn    AT91F_Flash_Init
//* \brief Flash init
//*----------------------------------------------------------------------------
void AT91F_NVM_Init (void)
{
    //* Set number of Flash Waite sate
    //  SAM7S64 features Single Cycle Access at Up to 30 MHz
    //  if MCK = 47923200, 48 Cycles for 1 µseconde ( field MC_FMR->FMCN)
        AT91C_BASE_MC->MC_FMR = ((AT91C_MC_FMCN)&(48 <<16)) | AT91C_MC_FWS_1FWS ;
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_Flash_Ready
//* \brief Wait the flash ready
//*----------------------------------------------------------------------------
__ramfunc int AT91F_Flash_Ready (void)
{
    unsigned int status;
    status = 0;

    //* Wait the end of command
        while ((status & AT91C_MC_FRDY) != AT91C_MC_FRDY )
        {
          status = AT91C_BASE_MC->MC_FSR;
        }
        return status;
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_Flash_Lock_Status
//* \brief Get the Lock bits field status
//*----------------------------------------------------------------------------
__ramfunc int AT91F_Flash_Lock_Status(void)
{
  return (AT91C_BASE_MC->MC_FSR & AT91C_MC_FSR_LOCK);
}
//*----------------------------------------------------------------------------
//* \fn    AT91F_Flash_Lock
//* \brief Write the lock bit and set at 0 FSR Bit = 1
//* \input page number (0-1023)
//* \output Region
//*----------------------------------------------------------------------------
__ramfunc int AT91F_Flash_Lock (unsigned int Flash_Lock_Page)
{
    //* set the Flash controller base address
        AT91PS_MC ptMC = AT91C_BASE_MC;

        AT91F_NVM_Init();
    //* write the flash
	//* Protect
		AT91F_disable_interrupt();
    //* Write the Set Lock Bit command
        ptMC->MC_FCR = AT91C_MC_CORRECT_KEY | AT91C_MC_FCMD_LOCK | (AT91C_MC_PAGEN & (Flash_Lock_Page << 8) ) ;

    //* Wait the end of command
         AT91F_Flash_Ready();
    //* Protect
		AT91F_enable_interrupt();


  return (AT91F_Flash_Lock_Status());
}
//*----------------------------------------------------------------------------
//* \fn    AT91F_Flash_Unlock
//* \brief Clear the lock bit and set at 1 FSR bit=0
//* \input page number (0-1023)
//* \output Region
//*----------------------------------------------------------------------------
__ramfunc int AT91F_Flash_Unlock(unsigned int Flash_Lock_Page)
{
	    AT91F_NVM_Init();

	//* Protect
		AT91F_disable_interrupt();
    //* Write the Clear Lock Bit command
        AT91C_BASE_MC->MC_FCR = AT91C_MC_CORRECT_KEY | AT91C_MC_FCMD_UNLOCK | (AT91C_MC_PAGEN & (Flash_Lock_Page << 8) ) ;

    //* Wait the end of command
        AT91F_Flash_Ready();
    //* Protect
		AT91F_enable_interrupt();

  return (AT91F_Flash_Lock_Status());
}


//*----------------------------------------------------------------------------
//* \fn    AT91F_Flash_Erase_All
//* \brief Send command erase all flash
//*----------------------------------------------------------------------------
__ramfunc int AT91F_Flash_Erase_All(void)
{
        AT91F_Flash_Init();
	//* Protect
		AT91F_disable_interrupt();
    //* set the Flash controller base address
        AT91PS_MC ptMC = AT91C_BASE_MC;
    //* Write the Erase All command
        ptMC->MC_FCR = AT91C_MC_CORRECT_KEY | AT91C_MC_FCMD_ERASE_ALL ;
    //* Wait the end of command
        AT91F_Flash_Ready();
   //* Protect
		AT91F_enable_interrupt();
    //* Check the result
        return ( (ptMC->MC_FSR & ( AT91C_MC_PROGE | AT91C_MC_LOCKE ))==0) ;
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_Flash_Write
//* \brief Write in one Flash page located in AT91C_IFLASH,  size in 32 bits
//* \input Flash_Address: start at 0x0010 0000 size: in byte
//*----------------------------------------------------------------------------
__ramfunc int AT91F_Flash_Write( unsigned int Flash_Address ,int size ,unsigned int * buff)
{
    //* set the Flash controller base address
    AT91PS_MC ptMC = AT91C_BASE_MC;
    unsigned int i, page, status;
    unsigned int * Flash;
    //* init flash pointer
        Flash = (unsigned int *) Flash_Address;

        AT91F_Flash_Init();
   //* Get the Flash page number
        page = ((Flash_Address - (unsigned int)AT91C_IFLASH ) /FLASH_PAGE_SIZE_BYTE);
   //* copy the new value
	for (i=0; (i < FLASH_PAGE_SIZE_BYTE) & (size > 0) ;i++, Flash++,buff++,size-=4 ){
	//* copy the flash to the write buffer ensuring code generation
	    *Flash=*buff;
	}
	//* Protect
		AT91F_disable_interrupt();
    //* Write the write page command
        ptMC->MC_FCR = AT91C_MC_CORRECT_KEY | AT91C_MC_FCMD_START_PROG | (AT91C_MC_PAGEN & (page <<8)) ;
    //* Wait the end of command
        status = AT91F_Flash_Ready();
    //* Protect
		AT91F_enable_interrupt();

    //* Check the result
    if ( (status & ( AT91C_MC_PROGE | AT91C_MC_LOCKE ))!=0) return false;
  return true;
}



//*----------------------------------------------------------------------------
//* \fn    AT91F_NVM_Status
//* \brief Get the NVM field status
//*----------------------------------------------------------------------------
__ramfunc int AT91F_NVM_Status(void)
{
  return (AT91C_BASE_MC->MC_FSR & AT91C_MC_FSR_MVM);
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_NVM_Set
//* \brief Write the Non Volatile Memory Bits and set at 0 FSR Bit = 1
//*----------------------------------------------------------------------------
__ramfunc int AT91F_NVM_Set (unsigned char NVM_Number)
{
        AT91F_NVM_Init();
    //* set the Flash controller base address
        AT91PS_MC ptMC = AT91C_BASE_MC;
	//* Protect
		AT91F_disable_interrupt();

	 //* write the flash
    //* Write the Set NVM Bit command
        ptMC->MC_FCR = AT91C_MC_CORRECT_KEY | AT91C_MC_FCMD_SET_GP_NVM | (AT91C_MC_PAGEN & (NVM_Number << 8) ) ;

    //* Wait the end of command
        AT91F_Flash_Ready();
    //* Protect
		AT91F_enable_interrupt();

  return (AT91F_NVM_Status());
}
//*----------------------------------------------------------------------------
//* \fn    AT91F_NVM_Clear
//* \brief Clear the Non Volatile Memory Bits and set at 1 FSR bit=0
//*----------------------------------------------------------------------------
__ramfunc int AT91F_NVM_Clear(unsigned char NVM_Number)
{
        AT91F_NVM_Init();
    //* set the Flash controller base address
        AT91PS_MC ptMC = AT91C_BASE_MC;

	//* Protect
		AT91F_disable_interrupt();
	 //* write the flash
    //* Write the Clear NVM Bit command
        ptMC->MC_FCR = AT91C_MC_CORRECT_KEY | AT91C_MC_FCMD_CLR_GP_NVM | (AT91C_MC_PAGEN & (NVM_Number << 8) ) ;

    //* Wait the end of command
       AT91F_Flash_Ready();
    //* Protect
		AT91F_enable_interrupt();


  return (AT91F_NVM_Status());
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_SET_Security_Status
//* \brief Get Flash Security Bit Status
//*----------------------------------------------------------------------------
__ramfunc int AT91F_SET_Security_Status (void)
{
  return (AT91C_BASE_MC->MC_FSR & AT91C_MC_SECURITY);
}

//*----------------------------------------------------------------------------
//* \fn AT91F_SET_Security
//* \brief Set Flash Security Bit
//*----------------------------------------------------------------------------
__ramfunc int AT91F_SET_Security (void)
{
        AT91F_NVM_Init();
	//* Protect
		AT91F_disable_interrupt();
	 //* write the flash
    //* Write the Set Security Bit command
        AT91C_BASE_MC->MC_FCR = ( AT91C_MC_CORRECT_KEY | AT91C_MC_FCMD_SET_SECURITY ) ;

    //* Wait the end of command
       AT91F_Flash_Ready();
    //* Protect
		AT91F_enable_interrupt();

  return (AT91F_SET_Security_Status());
}
//*----------------------------------------------------------------------------
//* \fn    AT91F_Flash_Check_Erase
//* \brief Check the memory at 0xFF in 32 bits access
//*----------------------------------------------------------------------------
int AT91F_Flash_Check_Erase (unsigned int * start, unsigned int size)
{
	unsigned int i;
    //* Check if flash is erased
	for (i=0; i < (size/4) ; i++ )
	{
	    if ( start[i] != ERASE_VALUE ) return  false;
	}
	return true ;
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_Flash_Write_all
//* \brief Write in one Flash page located in AT91C_IFLASH,  size in byte
//* \input Start address (base=AT91C_IFLASH) size (in byte ) and buff address
//*----------------------------------------------------------------------------
int AT91F_Flash_Write_all( unsigned int Flash_Address ,int size ,unsigned int * buff)
{

    int   next, status;
    unsigned int  dest;
    unsigned int * src;

    dest = Flash_Address;
    src = buff;
    status = true;

    while( (status == true) & (size > 0) )
	{
        //* Check the size
        if (size <= FLASH_PAGE_SIZE_BYTE) next = size;
        else next = FLASH_PAGE_SIZE_BYTE;

        //* Unlock current sector base address - current address by sector size
        AT91F_Flash_Unlock((dest - (unsigned int)AT91C_IFLASH ) /FLASH_PAGE_SIZE_BYTE);

        //* Write page and get status
        status = AT91F_Flash_Write( dest ,next ,src);
        // * get next page param
        size -= next;
        src += FLASH_PAGE_SIZE_BYTE/4;
        dest +=  FLASH_PAGE_SIZE_BYTE;
	}
    return status;
}
