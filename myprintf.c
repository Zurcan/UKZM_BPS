
void DBGU_SendBuf(int Count)
{
  
  pDBGU->DBGU_TPR = (unsigned int)&(DBGU_Buf[0]);
  pDBGU->DBGU_TCR = Count;
  
  AT91F_PIO_SetOutput( AT91C_BASE_PIOA, AT91C_PIO_PA1); //Led off

  while (pDBGU->DBGU_TCR != 0) {}; //wait end of transmission

  AT91F_PIO_ClearOutput( AT91C_BASE_PIOA,  AT91C_PIO_PA1); //Led on
}

#define myprintf(...) DBGU_SendBuf(sprintf(&DBGU_Buf[0],__VA_ARGS__));


void US0_Send(int Count)
{
  
  pUS0->US_TPR = (unsigned int)&(US0_SendBuf[0]);
  pUS0->US_TCR = Count;
  
  AT91F_PIO_SetOutput( AT91C_BASE_PIOA, AT91C_PIO_PA1); //Led off

  while (pUS0->US_TCR != 0) {}; //wait end of transmission

  AT91F_PIO_ClearOutput( AT91C_BASE_PIOA,  AT91C_PIO_PA1); //Led on
}

#define US0printf(...) US0_Send(sprintf(&US0_SendBuf[0],__VA_ARGS__));

