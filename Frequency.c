
unsigned short CalcValSKZ(SKZ_ARRAY _SKZ)
{
  int i;
/*    for (i=0; i<Max_ValFreqCount; i++) 
    {
      if ((i%10)==0) myprintf ("\n\r");
      myprintf ("%6d", _SKZ[i]);
    }
    myprintf ("\n\r");*/

  signed long long d_ValSKZ;
  d_ValSKZ = 0;
  for (i=0; i<Config.ValFreqCount; i++)
       d_ValSKZ += ((signed long long)_SKZ[Config.FreqOrder[i]] * (signed long long)_SKZ[Config.FreqOrder[i]]);
//       d_ValSKZ += ( (signed long long)_SKZ[Config.FreqOrder[i]]);
//  myprintf ("%lld \n\r", d_ValSKZ);
 return (unsigned short)(sqrt((float)d_ValSKZ) / Config.ValFreqCount);
//    return (unsigned short)(d_ValSKZ / Config.ValFreqCount);
//  return (unsigned short)(sqrt((float)d_ValSKZ));
// return (unsigned short)(sqrt((float)d_ValSKZ / Config.ValFreqCount));
}



void CalcSKZ()
{
#define Stripes 5
    //signed int sign=0;
    unsigned int i;
    signed long long sll_SKZ [Max_ValFreqCount * Stripes];
    unsigned long long DCLevel;
  
    //DC level substraction 
    DCLevel = 0;
    for (i=0; i<ADC_Buf_Size; i++)  DCLevel += ADC_Buf[i];
    DCLevel >>= ADC_Buf_Shift;
    for (i=0; i<ADC_Buf_Size; i++)  ADC_Buf[i] -= DCLevel;
    
    
/*    for (i=0; i<ADC_Buf_Size; i++) 
    {
      if ((i%16)==0) myprintf ("\n\r");
      myprintf ("%5d", ADC_Buf[i]);
    }*/
    
    unsigned int k, arg, f;
    signed long long ReX, ImX;
    
    //frecuency analisys, calculating SKZ relative to gain
    
    //from  400 Hz to 5125 Hz step 25Hz
    for (k=8, f=0; f<(Max_ValFreqCount)*Stripes; f++, k++)
    {
      arg = 0;
      ReX = 0;
      ImX = 0;
      for (i=0; i<ADC_Buf_Size; i++)
      {
        ReX += (ADC_Buf[i]*Cos[arg]);
        ImX += (ADC_Buf[i]*Sin[arg]);
        arg += (k * (CosSinCount/ADC_Buf_Size)); 
        arg &= CosSinArgMask;
      }
      ReX >>= ADC_Buf_Shift; //sin and cos scale
      ImX >>= ADC_Buf_Shift;
      
      sll_SKZ[f] = ReX * ReX + ImX * ImX;
      sll_SKZ[f] = (signed long long)sqrt ((double)sll_SKZ[f]);
      sll_SKZ[f] >>= (CosSinShift);
    }

/*    for (i=0; i<Max_ValFreqCount*Stripes; i++) 
    {
      if ((i%10)==0) myprintf ("\n\r");
      myprintf ("%7d", sll_SKZ[i]);
    }
    myprintf ("\n\r");
*/

    //auto attenuation
    signed char AutoAttn;
    AutoAttn = 1;
    for (k=0; k<Max_ValFreqCount*Stripes; k++) 
      if (sll_SKZ[k] > 32) AutoAttn = 0;
    for (k=0; k<Max_ValFreqCount*Stripes; k++) 
      if (sll_SKZ[k] > 128) AutoAttn = -1;

    for (k=0; k<Max_ValFreqCount*Stripes; k++)
    {
      sll_SKZ[k] <<= (8-Gain); // * gain
    }

    // calc SKZ by summing adjancent Stripes
    signed long long TempSKZ [Max_ValFreqCount];
    for (k=0; k<Max_ValFreqCount; k++)
    {
      TempSKZ[k] = 0;
      for (i=0; i<Stripes; i++) 
        TempSKZ[k] +=  (sll_SKZ[k*Stripes+i] * sll_SKZ[k*Stripes+i]);
    }
    Object_on=0;
    float AKoeff = 0.8;
    float BKoeff = 0.2;
    for (k=0; k<Max_ValFreqCount; k++)
    {
      f_SKZ[k] = AKoeff * f_SKZ[k] + BKoeff * sqrt((float)TempSKZ[k]);
      SKZ[k] = floorf(f_SKZ[k]);
      if(SKZ[k]<MinSKZ_val)SKZ[k]=0;//здесь у нас отбрасываются значения ниже минимального порога СКЗ
      if(SKZ[k]>(Object_on_level))Object_on=0xffff;//а здесь определяем, включена ли мельница//Object_on_level*sqrt(Config.ValFreqCount))
    }
    
/*    for (i=0; i<Max_ValFreqCount; i++) 
    {
      if ((i%5)==0) myprintf ("\n\r");
      myprintf ("%8d", SKZ[i]);
    }
    myprintf ("\n\r");*/

    // calc ValSKZ & RelValSKZ
    ValSKZ = CalcValSKZ (SKZ);
    if(Config.ValSKZ_2==Config.ValSKZ_1)Config.ValSKZ_2++;
    RelValSKZ = 100*(double)(ValSKZ-Config.ValSKZ_1)/(double)(Config.ValSKZ_2 - Config.ValSKZ_1);
    if(RelValSKZ>999)RelValSKZ=999;
    if(RelValSKZ<-999)RelValSKZ=-999;
    

    Gain+=AutoAttn;
    if (Gain > 8) Gain = 8;
    if (Gain < 0) Gain = 0;
    SetGain(Gain);
}


void SetSKZ_1 () 
{first_pass=0; 
  memcpy (&Config.SKZ_1[0], &SKZ[0], sizeof(SKZ));
}

void SetSKZ_2 () 
{first_pass=0; 
  memcpy (&Config.SKZ_2[0], &SKZ[0], sizeof(SKZ));
  
  // calc frequency change
  unsigned int i;
  
  double TotalSKZ_1;
  double TotalSKZ_2;
  
  double Change [Max_ValFreqCount];
  double WeightSKZ_1;
  double WeightSKZ_2;
  
/*  TotalSKZ_1 = 0;
  TotalSKZ_2 = 0;
  for (i = 0; i < Max_ValFreqCount; i++)
  {
    TotalSKZ_1 += ((double)Config.SKZ_1[i]) * ((double)Config.SKZ_1[i]);
    TotalSKZ_2 += ((double)Config.SKZ_2[i]) * ((double)Config.SKZ_2[i]);
  }

  for (i = 0; i < Max_ValFreqCount; i++)
  {
    WeightSKZ_1 = ((double)Config.SKZ_1[i]) * ((double)Config.SKZ_1[i]) / TotalSKZ_1;
    WeightSKZ_2 = ((double)Config.SKZ_2[i]) * ((double)Config.SKZ_2[i]) / TotalSKZ_2;
    Change[i] = (WeightSKZ_2 - WeightSKZ_1);*/
  for (i = 0; i < Max_ValFreqCount; i++)
  {if((double)Config.SKZ_1[i]==0){Change[i]=100*((double)Config.SKZ_2[i]/MinSKZ_val);}
  else {Change[i]=100*((double)Config.SKZ_2[i]/(double)Config.SKZ_1[i]);}
    //if(Change[i]<MinSKZ_val)Change[i]=0;
   // myprintf ("change[%d]=%f", i, Change[i]);
  }

  //  sorting frequency change
  unsigned int j;
  double Max;
  unsigned int MaxIndex;
  
  for (j=0; j<Max_ValFreqCount; j++)
  {
    MaxIndex = 0; 
    Max = Change[MaxIndex];
   // Config.Change[j]=Change[j];
    for (i=0; i<Max_ValFreqCount; i++)
      if (Change[i]>Max) {MaxIndex = i; Max = Change[MaxIndex];}
    Config.Change[MaxIndex]=Max;
    Config.FreqOrder[j] = MaxIndex;
    Change[MaxIndex] = -10e10;
  }

    // calculating skz using freq order  
  Config.ValSKZ_1 = CalcValSKZ (Config.SKZ_1);
  Config.ValSKZ_2 = CalcValSKZ (Config.SKZ_2);
  WriteConfig();

}

void SetSKZ_2_ () 
{first_pass=0; 
//  memcpy (&Config.SKZ_2[0], &SKZ[0], sizeof(SKZ));
  
  // calc frequency change
  unsigned int i;
  
  double TotalSKZ_1;
  double TotalSKZ_2;
  
  double Change [Max_ValFreqCount];
  double WeightSKZ_1;
  double WeightSKZ_2;
/*  
  TotalSKZ_1 = 0;
  TotalSKZ_2 = 0;
  for (i = 0; i < Max_ValFreqCount; i++)
  {
    TotalSKZ_1 += ((double)Config.SKZ_1[i]) * ((double)Config.SKZ_1[i]);
    TotalSKZ_2 += ((double)Config.SKZ_2[i]) * ((double)Config.SKZ_2[i]);
  }

  for (i = 0; i < Max_ValFreqCount; i++)
  {
    WeightSKZ_1 = ((double)Config.SKZ_1[i]) * ((double)Config.SKZ_1[i]) / TotalSKZ_1;
    WeightSKZ_2 = ((double)Config.SKZ_2[i]) * ((double)Config.SKZ_2[i]) / TotalSKZ_2;
    Change[i] = (WeightSKZ_2 - WeightSKZ_1);*/
    
   // if(Change[i]<MinSKZ_val)Change[i]=0;
   
 // }

  
  for (i = 0; i < Max_ValFreqCount; i++)
  {if((double)Config.SKZ_1[i]==0)Change[i]=100*((double)Config.SKZ_2[i]/MinSKZ_val);
  else {Change[i]=100*((double)Config.SKZ_2[i]/(double)Config.SKZ_1[i]);}
  //myprintf ("change[%d]=%f", i, Change[i]);
  }
  //  sorting frequency change
  unsigned int j;
  double Max;
  unsigned int MaxIndex;
  
  for (j=0; j<Max_ValFreqCount; j++)
  {
    MaxIndex = 0; 
    Max = Change[MaxIndex];
    //Config.Change[j]=Change[j];
    for (i=0; i<Max_ValFreqCount; i++)
      if (Change[i]>Max) {MaxIndex = i; Max = Change[MaxIndex];}
     Config.Change[MaxIndex]=Max;
    Config.FreqOrder[j] = MaxIndex;
    Change[MaxIndex] = -10e10;
  }

    // calculating skz using freq order  
  Config.ValSKZ_1 = CalcValSKZ (Config.SKZ_1);
  Config.ValSKZ_2 = CalcValSKZ (Config.SKZ_2);
  WriteConfig();

}
void myprintfSKZSpectr()
{
    unsigned int k;
    myprintf ("____________________________________\n\r");
    unsigned int row; 
    unsigned int exit;
    exit = 0;
    row = 0;
    while (!exit)
    {
      exit = 1;
      myprintf ("%4d | ",row);
      for (k=0; k<Max_ValFreqCount; k++)
      {
        if ( SKZ[k] >= (1<<row) ) 
        {
          myprintf ("*");
          exit = 0;
        }
        else myprintf(" ");
      }
      row++;
      myprintf("\n\r");
    }
}

void myprintfSKZ()
{
      for (int k=0; k<Max_ValFreqCount; k++)
        myprintf("SKZ[%2d] = %d\n\r", k, SKZ[k]);
}

void myprintfSKZ_1()
{
      for (int k=0; k<Max_ValFreqCount; k++)
        myprintf("SKZ1[%2d] = %d\n\r", k, Config.SKZ_1[k]);
}

void myprintfSKZ_2()
{
      for (int k=0; k<Max_ValFreqCount; k++)
        myprintf("SKZ2[%2d] = %d\n\r", k, Config.SKZ_2[k]);
}

void myprintfFreqOrder()
{
      for (int k=0; k<Max_ValFreqCount; k++)
        myprintf("FreqOrder[%2d] = %d  %3d\n\r", k, Config.FreqOrder[k],Config.Change[k]);
}
