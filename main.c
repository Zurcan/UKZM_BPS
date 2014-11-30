
#include "..\AT91SAM7S256.h"
#define __inline inline
#include "..\lib_AT91SAM7S256.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "board.h"
//#include "frequency.c"
#include "cossintable.h" 

//*************************************************************
// Controller blocks pointers

AT91PS_PMC  pPMC  = AT91C_BASE_PMC;
AT91PS_PIO  pPIO  = AT91C_BASE_PIOA;
AT91PS_TC   pTC0  = AT91C_BASE_TC0;
AT91PS_DBGU pDBGU = AT91C_BASE_DBGU;  
AT91PS_ADC  pADC  = AT91C_BASE_ADC;
AT91PS_SPI  pSPI  = AT91C_BASE_SPI;
AT91PS_USART pUS0 = AT91C_BASE_US0;

//************************************************
// Common global variables 

#define ADC_Buf_Shift 9
#define ADC_Buf_Size (1 << ADC_Buf_Shift)
__no_init signed short ADC_Buf [ADC_Buf_Size];

#define DBGU_Buf_Size 1000
__no_init char DBGU_Buf [DBGU_Buf_Size];
#define US0_BufSize 256
__no_init char US0_SendBuf[US0_BufSize];
__no_init char US0_RecvBuf[US0_BufSize];

#include "myprintf.c" //function myprintf, uses DBGU_Buf anf Led control

signed char Gain; //current gain power of 2 = gain, 0 - gain 1, 3 - gain 8
unsigned char GainTable[9] = {0, 128, 192, 224, 240, 248, 252, 254, 255};
inline void SetGain(unsigned int a) {Gain=a; pSPI->SPI_TDR = (0x010D << 16)|GainTable[a]; while (!(pSPI->SPI_SR & (1<<9))) {};}

inline void SetGainCode(unsigned char c) {pSPI->SPI_TDR = (0x010D << 16)|c; while (!(pSPI->SPI_SR & (1<<9))) {};}
unsigned char GainCode = 0;
int first_pass=1;
#define Max_ValFreqCount 18
typedef unsigned short SKZ_ARRAY[Max_ValFreqCount]; //0 - DC level, 19*250 - 4.75KHz level
SKZ_ARRAY SKZ;
float f_SKZ[Max_ValFreqCount];
unsigned int  ReadSKZ1[Max_ValFreqCount], ReadSKZ2[Max_ValFreqCount],MinSKZ_val=30,Object_on_level=40, Object_on=0;
signed int RelValSKZ;
char RelValSKZ1toSKZ2;
unsigned short ValSKZ;
unsigned int UARTRateTable[] = {300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
//timeout set in bits for modbus. for speed <=19200 it equal 3,5 char = 
unsigned int UARTTimeoutTable[] = {39, 39, 39, 39, 39, 39, 39, 39*2, 39*3, 39*6};


typedef struct //struct to interprete data in config
{
  unsigned int UARTRate;
  unsigned int ModbusAddr;
  unsigned int ValFreqCount;
  SKZ_ARRAY SKZ_1, SKZ_2,Change;
  unsigned short ValSKZ_1, ValSKZ_2;
  unsigned char FreqOrder[Max_ValFreqCount];
} CONFIG, *PCONFIG;

const CONFIG DefaultConfig = {5, 1, 3}; //rate 9600, addr 1, 1 frequency (Василий Владимирович меняйте адрес ТУТ!!!)

#define FPageSize 64
#pragma data_alignment=256
const unsigned int FlashConfig [FPageSize] = {0xFF}; //flash page for config

__no_init union
{
  unsigned int RAMConfig [FPageSize]; //ram page for config
  CONFIG Config;
};

inline void US0_SetBaudRate (unsigned int Rate)
{
  pUS0->US_BRGR = 3225600 / UARTRateTable [Rate];
  pUS0->US_RTOR = UARTTimeoutTable [Rate];//bits
}
__ramfunc void WriteConfig()             //функция записи во flash память значений SKZ1 SKZ2
{
  unsigned int i;
  unsigned int * Flash;
  unsigned int * RAMBuf;
  unsigned int Page;
  AT91PS_MC pMC = AT91C_BASE_MC;
  
  Flash = (unsigned int *)&FlashConfig[0];
  RAMBuf   = (unsigned int *)&RAMConfig[0];
  
  Page = ((unsigned int)Flash >> 8) & 0x3FF;

  //copy flash to ram 
  for (i=0; i<FPageSize; i++, Flash++, RAMBuf++) *Flash = *RAMBuf; 
  
//  pMC->MC_FMR = 0x00500100;
  pMC->MC_FCR = ((0x5A000000) | 0x01 | (Page << 8));
  
  //* Wait the end of command
  while ((pMC->MC_FSR & 0x01) != 1) {};
 
  myprintf ("Config write OK!\n\r");
}
void ReadInitConfig()
{
  unsigned int i;
  for (i=0; i<FPageSize; i++) RAMConfig[i] = FlashConfig[i]; //copy flash to ram
  myprintf ("Config read...\n\r");
  if (Config.UARTRate > 9) //wrong rate - no config
  {
    Config = DefaultConfig;
    myprintf ("Config wrong, use default config...\n\r");
    WriteConfig();
    return;
  }
  myprintf ("Config read OK!\n\r");
}


void MyprintfConfig()
{
  myprintf ("UART: %6d, Addr: %3d = 0x%2X Freq: %2d (Enter to save)\n\r",
            UARTRateTable [Config.UARTRate],
            Config.ModbusAddr, Config.ModbusAddr,
            Config.ValFreqCount );
}

void SetUARTRate (unsigned char Rate) { Config.UARTRate = Rate; US0_SetBaudRate(Rate);};
void SetModbusAddr (unsigned char Addr) { Config.ModbusAddr = Addr; };
void SetValFreqCount (unsigned char Count) {Config.ValFreqCount = Count; };

inline unsigned short HiLow (unsigned char Hi, unsigned char Low) 
                            {return ((Hi<<8)|Low);}
inline unsigned char Hi (unsigned short Val) {return (Val>>8);}
inline unsigned char Low (unsigned short Val) {return (Val&0xFF);}

#include "Frequency.c"

unsigned char ViewModbus = 0;
#define modbus_printf(...) if (ViewModbus) DBGU_SendBuf(sprintf(&DBGU_Buf[0],__VA_ARGS__));

//*******************************************************************
//* crc for Buf
unsigned short ModbusCRC (unsigned char *Buf, unsigned int Count)
{
  unsigned short s,j,t,i;
  s = 0xFFFF;
  for(i=0; i<Count; i++)
  {
    t = Buf[i];
    s = t^s;
    for(j=0; j<8; j++)
    {  
      if(s & 0x0001 == 1)  {s >>= 1;  s ^= 0xA001;}
      else { s >>= 1;} 
    } 
  }
  return s;
}

//*******************************************************************
//* reads  logical modbus registers as physical variables 
unsigned short ReadInputReg (unsigned short Addr)
{
  if ((Addr>=0x00) & (Addr<=0x12)) return (SKZ[Addr]);// SKZ values
  if ((Addr>=0x20) & (Addr<=0x32)) return (Config.SKZ_1[Addr-0x20]);// SKZ1 values
  if ((Addr>=0x40) & (Addr<=0x52)) return (Config.SKZ_2[Addr-0x40]);// SKZ2 values
  if ((Addr>=0x60) & (Addr<=0x72)) return (Config.FreqOrder[Addr-0x60]);//order
  
  if (Addr==0x80) return Config.ValFreqCount;
  if (Addr==0x83) return Config.ValSKZ_1;
  if (Addr==0x84) return Config.ValSKZ_2;
  if (Addr==0x85) return ValSKZ;
  if (Addr==0x86) return RelValSKZ;
  if (Addr==0x87) return RelValSKZ1toSKZ2;
  if (Addr==0x88) return Object_on;
  return 0; 
}

//*******************************************************************
//* func code 0x0C - функция, которую мы используем для сохранения обоих массивов SKZ в памяти еепром БКИ  
inline void Read_SKZ_func (unsigned char *Frame, unsigned char FrameCount)
{ 
Config.ValSKZ_1=((unsigned int)Frame[2]<<8)+Frame[3];
Config.ValSKZ_2=((unsigned int)Frame[4]<<8)+Frame[5];
CalcSKZ(); 
first_pass=0; 
  return;
}

inline void Read_freq_count (unsigned char *Frame, unsigned char FrameCount)
{
Config.ValFreqCount=Frame[2];
return; 
}

inline void Read_object_on_level (unsigned char *Frame, unsigned char FrameCount)
{
//Object_on_level=(int);  
Object_on_level=0;  
Object_on_level=(int)(Frame[2]<<8)|Frame[3];
Object_on_level = Object_on_level/sqrt(Config.ValFreqCount);
return; 
}

inline void Read_MINSKZ_value (unsigned char *Frame, unsigned char FrameCount)
{
MinSKZ_val=Frame[2]<<8;  
MinSKZ_val=MinSKZ_val+Frame[3];

return; 
}
//* func code 0xCC - функция, которую мы используем для сохранения обоих массивов SKZ в памяти еепром БКИ 
/*inline void Write_SKZ_arrays_func (unsigned char *Frame,unsigned char FrameCount, 
                                  unsigned char *Answ,unsigned char *AnswCount )
{ unsigned int m=0;
  Answ[0] = Frame[0]; //address
  Answ[1] = Frame[1]; //func code
  for (m=0;m<19;m++)
  {Answ[2+m*2] = Config.SKZ_1[m]>>8;
   Answ[3+m*2] = Config.SKZ_1[m];}
  for (m=0;m<19;m++)
  {Answ[40+m*2] = Config.SKZ_2[m]>>8;
  Answ[41+m*2] = Config.SKZ_2[m];}
  unsigned short crc = ModbusCRC(Answ, 78);
  Answ[78] = Low(crc);
  Answ[79] = Hi(crc);
  *AnswCount = 80;
  return;}*/
//* func code 0xCD - функция, которую мы используем для чтения обоих массивов SKZ из памяти еепром БКИ
inline void SKZ_arrays_read (unsigned char *Frame,unsigned char FrameCount 
                                  )
{ unsigned int m=0;

  for (m=0;m<19;m++)
  {Config.SKZ_1[m] = ((unsigned int)Frame[2+m*2]<<8)+Frame[3+m*2];//сохраняем 1й массив
 }
// SetSKZ_1();
  for (m=0;m<19;m++)
  {Config.SKZ_2[m] = ((unsigned int)Frame[40+m*2]<<8)+Frame[41+m*2];//сохраняем 2й массив
 }


 
SetSKZ_2_ ();
if((Config.ValSKZ_1==0xfffe)&&(Config.ValSKZ_2==0xfffe)){
for (m=0;m<19;m++){  Config.SKZ_1[m]=0;Config.SKZ_2[m]=0;}}
SetSKZ_2_ ();  
  return;}

inline void Freq_Order_Read (unsigned char *Frame,unsigned char FrameCount 
                                  )
{ unsigned int m=0;

  for (m=0;m<19;m++)
  {Config.FreqOrder[m] = Frame[2];//сохраняем 1й массив
 }
// SetSKZ_1();

 return;}

inline void ParseRead_InputReg (unsigned char *Frame, unsigned char FrameCount,
                                  unsigned char *Answ, unsigned char *AnswCount)
{
  if (FrameCount != 8) // addr + funccode + 2regaddr + 2regcount + 2crc
    { modbus_printf("Wrong 04func frame length %d\n\r", FrameCount); return;}
  
  unsigned short StartAddr = HiLow (Frame[2], Frame[3]);
  unsigned short RegCount  = HiLow (Frame[4], Frame[5]);
  
  if ((RegCount<1) || (RegCount>125)) 
    { modbus_printf("Wrong 04func reg count %d\n\r", RegCount); return;}

  // 1) form header of frame
  Answ[0] = Frame[0]; //address
  Answ[1] = Frame[1]; //func code
  Answ[2] = (unsigned char) (2 * RegCount); //byte count
  unsigned char Count = 3;

  // 2) reading registers and appending them to answer
  unsigned short Addr;
  for (Addr = StartAddr; Addr < StartAddr+RegCount; Addr++)
  {
    signed int Reg = ReadInputReg(Addr);
    modbus_printf ("Read input reg 0x%X = %d\n\r", Addr, Reg);
    Answ[Count] = Hi(Reg);
    Count++;
    Answ[Count] = Low(Reg);
    Count++;
  }
  
  // 3) appending CRC
  unsigned short crc = ModbusCRC(Answ, Count);
  Answ[Count] = Low(crc);
  Count++;
  Answ[Count] = Hi(crc);
  Count++;
  
  // 4) returning answer frame length
  *AnswCount = Count;
  return;
}
//*******************************************************************
//* writes physical variables as logical modbus registers в этом виде отправляются регистры 
void WriteHoldingReg( unsigned short Addr, unsigned short Value)
{
  modbus_printf ("Write holding reg 0x%X = %d\n\r", Addr, Value)
  if (Addr==0x80) {Config.ValFreqCount = Value; WriteConfig();}
  if (Addr==0x81) {SetSKZ_1(); return;}
  if (Addr==0x82) {SetSKZ_2(); 
  RelValSKZ1toSKZ2 =100*(double)(Config.ValSKZ_2)/(double)(Config.ValSKZ_1)-100; //relation value of levels (first calibration point to second calibration point)
  if(RelValSKZ1toSKZ2 <= 0)RelValSKZ1toSKZ2=0;
  if(RelValSKZ1toSKZ2 > 100)RelValSKZ1toSKZ2=100;
  return; }
  return;
}

//*******************************************************************
//* func code 0x06 ф-я записывает регистр ожидания одиночный
inline void ParseWriteSingle_HoldingReg (unsigned char *Frame, unsigned char FrameCount,
                                         unsigned char *Answ, unsigned char *AnswCount)
{
  if (FrameCount != 8) // addr + funccode + 2regaddr + 2regvalue + 2crc
    { modbus_printf("Wrong 06func frame length %d\n\r", FrameCount); return;}

  unsigned short RegAddr = HiLow (Frame[2], Frame[3]);
  unsigned short RegValue = HiLow (Frame[4], Frame[5]);

  // 1) form header of frame
  Answ[0] = Frame[0]; //address
  Answ[1] = Frame[1]; //func code
  Answ[2] = Frame[2]; //addr hi
  Answ[3] = Frame[3]; //addr low
  Answ[4] = Frame[4]; //val hi
  Answ[5] = Frame[5]; //val low

  // 2) writing reg
  WriteHoldingReg( RegAddr, RegValue);

  // 3) appending CRC
  unsigned short crc = ModbusCRC(Answ, 6);
  Answ[6] = Low(crc);
  Answ[7] = Hi(crc);
  
  // 4) returning answer frame length
  *AnswCount = 8;
  return;
}

//*******************************************************************
//* func code 0x10 ф-я записывает множественный регистр ожидания
inline void ParseWriteMulty_HoldingReg (unsigned char *Frame, unsigned char FrameCount,
                                        unsigned char *Answ, unsigned char *AnswCount)
{
  unsigned short StartAddr = HiLow (Frame[2], Frame[3]);
  unsigned short RegCount = HiLow (Frame[4], Frame[5]);
  unsigned char ByteCount = Frame[6];
  
  if ((RegCount<1) || (RegCount>123) || (ByteCount!=RegCount*2))
      { modbus_printf("Wrong 10func reg count %d or wrong bytecount%d\n\r", RegCount, ByteCount); return;}
  
  if (FrameCount != (9+RegCount*2)) // addr + funccode + 2regaddr + 2regcount + bytecount + 2crc +2*RegCount
  { modbus_printf("Wrong 10func frame length %d\n\r", FrameCount); return;}

  // 1) form header of frame
  Answ[0] = Frame[0]; //address
  Answ[1] = Frame[1]; //func code
  Answ[2] = Frame[2]; //addr hi
  Answ[3] = Frame[3]; //addr low
  Answ[4] = Frame[4]; //count hi
  Answ[5] = Frame[5]; //count low

  // 2) writing regs
  unsigned short RegIndex, RegValue;
  for (RegIndex = 0; RegIndex < RegCount; RegIndex++)
  {
    RegValue = HiLow ( Frame[7+RegIndex*2], Frame[8+RegIndex*2] );
    WriteHoldingReg( StartAddr + RegIndex, RegValue);
  }

  // 3) appending CRC
  unsigned short crc = ModbusCRC(Answ, 6);
  Answ[6] = Low(crc);
  Answ[7] = Hi(crc);
  
  // 4) returning answer frame length
  *AnswCount = 8;
  return;
}

//*******************************************************************
//* if AnswCount = 0, then no answer.
void ProcessModbusFrame (unsigned char *Frame, unsigned char FrameCount,
                       unsigned char *Answ, unsigned char *AnswCount)
{
  *AnswCount = 0;
  //check  address
  unsigned char Addr = Frame[0];
  if (Addr!=Config.ModbusAddr) 
  { modbus_printf("Wrong modbus addr\n\r"); return;}
  //check crc
  unsigned short crc;
  crc = ModbusCRC (Frame, FrameCount);
  if (crc!=0) { modbus_printf("Wrong modbus crc\n\r"); return;}
  //parsing func code
  unsigned char FuncCode = Frame[1];
  switch (FuncCode)
  {
  case 0xCD: SKZ_arrays_read (Frame, FrameCount);break;  
  case 0xCE: Freq_Order_Read (Frame, FrameCount);break;  
  case 0xCC: Read_freq_count(Frame, FrameCount); break;
  case 0xCF: Read_object_on_level(Frame, FrameCount); break;
  case 0xCB: Read_MINSKZ_value(Frame, FrameCount);break;
  case 0x0C: Read_SKZ_func(Frame, FrameCount);break;  
  case 0x04: ParseRead_InputReg(Frame, FrameCount, Answ, AnswCount); break; //читаем регистр MODBUS
  case 0x06: ParseWriteSingle_HoldingReg(Frame, FrameCount, Answ, AnswCount); break;//читаем регистр MODBUS и отправляем значение на БКИ
  case 0x10: ParseWriteMulty_HoldingReg(Frame, FrameCount, Answ, AnswCount); break;
  default: modbus_printf ("Wrong func code %d\n\r", FuncCode)//AnswerIllegalFuncCode(Frame, FrameCount, Answ, AnswCount);
  }
}

//*********************************************************
//  USART0 waiting frame - setting pdc, writing sttto
inline void StartTimeout ()
{
  pUS0->US_RPR=(unsigned int)&US0_RecvBuf[0]; 
  pUS0->US_RCR = US0_BufSize; 
  pUS0->US_CR |= (1<<11);
}

//**************************************************
//* Function  main
int main( void )
{ 
  //*******************************************************
  //  Configuring PMC controller
  pPMC->PMC_PCER = (1 << AT91C_ID_SYS) |
                   (1 << AT91C_ID_PIOA) |
                   (1 << AT91C_ID_TC0) |
                   (1 << AT91C_ID_SPI) |
                   (1 << AT91C_ID_US0) ;
  
  //*******************************************************
  //  Configuring PIO Controller
  
  #define ConfigPIOA(a) pPIO->PIO_ASR |= a; pPIO->PIO_PDR |=a;
  #define ConfigPIOB(a) pPIO->PIO_BSR |= a; pPIO->PIO_PDR |=a;
  #define ConfigGPIO_Out(a) pPIO->PIO_PER |= a; pPIO->PIO_OER |= a;
  
  ConfigPIOA (AT91C_PIO_PA9 |  //drxd
              AT91C_PIO_PA10 | //dtxd
              AT91C_PIO_PA12 | //spi miso
              AT91C_PIO_PA13 | //spi mosi
              AT91C_PIO_PA14 | //spi sck
              AT91C_PIO_PA5| //usart0 rxd
              AT91C_PIO_PA6| //usart0 txd
              AT91C_PIO_PA7| //usart0 rts (for 485)
              AT91C_PIO_PA31 ); //spi npcs1 - for attenuator
  ConfigPIOB (AT91C_PIO_PA0); //tioa0 output
  ConfigGPIO_Out (AT91C_PIO_PA1);//led

  //********************************************************
  //  Configuring Timer 0
//  unsigned int dummy;
  pTC0 = AT91C_BASE_TC0;

  pTC0->TC_CCR = 0; //disable clock
  pTC0->TC_IDR = 0xFFFFFFFF ; //disable all interrupt
//  dummy = pTC0->TC_SR;
//  dummy = dummy;
  
  pTC0->TC_CMR = 0x0009C000; //MCK/2, wave with trigger on rc
  pTC0->TC_RA = 500; //800;
  pTC0->TC_RC = 1008; //25600Hz  //1613; //16KHz
  pTC0->TC_CCR = 1; //enable clock
  pTC0->TC_CCR = 4; //start
  
  //*********************************************************
  //  Configuring DBGU for printing 115200 8 bit no parity
  pDBGU->DBGU_BRGR = 28;
  pDBGU->DBGU_MR = 0x800;
  pDBGU->DBGU_CR = (1<<6) | (1<<4); //TXEN & RXEN
  pDBGU->DBGU_PTCR = (1 << 8); //TXTEN for PDC

  //*********************************************************
  //  Configuring ADC
  pADC->ADC_MR = 0x00013001; //trigger on TIOA0 clock 500K
  pADC->ADC_CHER = (1<<4);
  pADC->ADC_PTCR = 1; //RXTEN

  //*********************************************************
  //  Configuring SPI
  pSPI->SPI_CSR[1] = 0x0000FF02; //200KHz, 8 bit, rising edge
  pSPI->SPI_MR = 0xFF000013; 
  pSPI->SPI_CR = 1;//enabled

  //**********************************************************
  // Application initialization
  myprintf("\n\r>Start of work\n\r");
  SetGain(0);
  myprintf (">Debug interface ready\n\r");

  ReadInitConfig();

  //*********************************************************
  //  Configuring USART0 for speed
  pUS0->US_MR = 0x000000C1;  //8 bit EVEN parity, RS485
  pUS0->US_CR = (1<<6) | (1<<4); //TXEN & RXEN
  pUS0->US_PTCR = (1 << 8) | (1); //TXTEN & RXTEN for PDC
  US0_SetBaudRate(Config.UARTRate);
//  Config.ValFreqCount = Value;
//  SetSKZ_2 ();
  StartTimeout();
//  SetGainCode(GainCode);
  myprintf ("Entering main endless loop\n\r");
// CalcSKZ();
for (;;) //********* MAIN ENDLES LOOP *************************
{   
    //sampling buffer from ADC
    pADC->ADC_RPR = (unsigned int)&ADC_Buf[0]; 
    pADC->ADC_RCR = ADC_Buf_Size;
        
    while (pADC->ADC_RCR != 0) {} //wait ADC buf recieving
//    if(!first_pass){
      CalcSKZ();//} 
    
    myprintf("\rSKZ1:%6d SKZ2:%6d SKZ:%6d=%3d SKZ1/SKZ2:%3d EnVal:%6d MinVal:%6d", Config.ValSKZ_1, Config.ValSKZ_2, ValSKZ, RelValSKZ,RelValSKZ1toSKZ2,(int)(Object_on_level/sqrt(Config.ValFreqCount)),MinSKZ_val);//SKZ1/SKZ2:%3d должно быть таким иначе всё ломается//(int)(Object_on_level*sqrt(Config.ValFreqCount))

  unsigned int i;
  unsigned char Count;
    
  //*******************************************************************
  // checking MODBUS frame recieved, processing it
  
  if (pUS0->US_CSR & (1<<8)) //is timeout
  {
    Count = US0_BufSize - pUS0->US_RCR; //save count
    pUS0->US_RCR = 0;                    //no more recieve

    modbus_printf ("Recieved %d chars\n\r", Count); //test print buf
    for (i=0; i<Count; i++) modbus_printf ("%2X ", US0_RecvBuf[i]);
    modbus_printf ("\n\r");

    //parsing frame and sending answer
    ProcessModbusFrame ((unsigned char*)&US0_RecvBuf[0], Count, (unsigned char*)&US0_SendBuf, &Count);    
    US0_Send(Count);
    
    StartTimeout(); //start waiting new frame
  }

  unsigned char Sym;

  //if symbol recieved trough DBGU, then handle it
  if (pDBGU->DBGU_CSR & 1) //symbol recieved
  {
    Sym = pDBGU->DBGU_RHR;
    myprintf("\n\r");
//    myprintf ("Recieved symbol <%c> with code <%2X>\n\r", Sym, Sym)
    switch (Sym)
    {
    //******************  configuration commands  ********************//
    case '0': 
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8': 
    case '9': SetUARTRate (Sym-'0'); MyprintfConfig(); break;

    case 0x3D: //Config.ModbusAddr++; 
               //if (Config.ModbusAddr>255) Config.ModbusAddr = 255; 
               MyprintfConfig ();
               break; 
    case 0x2D: //Config.ModbusAddr--; 
               //if (Config.ModbusAddr<1) Config.ModbusAddr = 1; 
               MyprintfConfig ();
               break; 
    case ']':  Config.ValFreqCount++; 
               if (Config.ValFreqCount>Max_ValFreqCount) Config.ValFreqCount = Max_ValFreqCount;
               MyprintfConfig ();
               break; 
    case '[':  Config.ValFreqCount--; 
               if (Config.ValFreqCount<1) Config.ValFreqCount = 1; 
               MyprintfConfig ();
               break; 
    case 0x0D: WriteConfig (); 
               break; 
    case 'c':  MyprintfConfig (); 
               break; 
    case 'v':  myprintf("Current firmware virsion is 1.7.3\n\r"); 
               break; 
    case 'i':  myprintf("Identify number of the device is 55485A4D\n\r"); 
               break; 
    //******************  calibration commands  ********************//
    
    case 'z':  SetSKZ_1(); 
               myprintf("SET UP SKZ_1\n\r"); 
               break; 
    case 'x':  SetSKZ_2();
               myprintf("SET UP SKZ_2 AND FREQUENCY ORDER\n\r"); 
               break; 
    case 'q':  myprintfSKZ_1(); 
               break; 
    case 'w':  myprintfSKZ_2(); 
               break; 
    case 'f':  myprintfFreqOrder(); 
               break; 
    case 's':  myprintfSKZSpectr(); 
               break; 
    case 'a':  myprintfSKZ(); 
               break; 

    //******************  calibration commands  ********************//

    case 'h': myprintf ("0 ... 9 - Set UART speed 300 ... 115200\n\r"); 
              myprintf ("- / +  - Decrement / increment MODBUS address\n\r"); 
              myprintf ("[ / ]  - Decrement / increment frecuency count\n\r"); 
              myprintf ("c      - Display current configuration\n\r"); 
              myprintf ("Enter  - Save currrent configuration to flash memory\n\r"); 
              myprintf ("z / x  - Save current SKZ as first / second calibration point\n\r"); 
              myprintf ("q / w  - Display first / second calibration point\n\r"); 
              myprintf ("f      - View frequency order\n\r"); 
              myprintf ("s      - View current SKZ as spectre\n\r"); 
              myprintf ("a      - View current SKZ as values\n\r"); 
              myprintf ("h      - This help screen\n\r"); 
              myprintf ("g      - Display current gain\n\r"); 
              myprintf ("t      - Test message to MODBUS port\n\r"); 
              myprintf ("m      - Toggle on/off displaying MODBUS command processing\n\r"); 
              myprintf ("v      - View version of firmware\n\r"); 
              myprintf ("i      - View identify number of the device\n\r"); 
              myprintf ("\n\r"); 
              break;
    case 'g': myprintf ("Current gain <%d>\n\r", Gain); 
              break; 
    case 't': myprintf("Test UART for send\n\r"); 
              US0printf("Test UART for send\n\r");
              break; 
    case 'm': ViewModbus ^= 0x01; myprintf("Modbus is %d\n\r", ViewModbus); 
              break; 

/*    case '/': CalcSKZ(); 
              break; 
*/
/*    case ',': GainCode--; SetGainCode(GainCode); myprintf("Gain Code %3d\n\r", GainCode);
              break;
    case '.': GainCode++; SetGainCode(GainCode); myprintf("Gain Code %3d\n\r", GainCode);
              break;
*/
    default: myprintf ("Recieved symbol <%c> with code <%2X>\n\r", Sym, Sym);
    }
  }
}
}
