//
//    FILE: main.ino
//  AUTHOR: Janaka
// VERSION: 0.01
// PURPOSE: read multiple analog inputs continuously
//          interrupt driven to catch all conversions.
//
#include "ADS1X15.h"
#include <stdio.h>
#include <string.h>
//#include <EEPROM.h>

typedef struct
{
  uint8_t mode;
  uint16_t reg;
  char data[30];
  bool dataAvailable;
}cmdDet_t;

typedef struct
{
  uint16_t reg;
  uint8_t mode;
  char (*cmd_fn)(uint8_t mode,char *str);
}cmd_t;

#define REG_ZERO_CAL_SEN0           0x0010
#define REG_ZERO_CAL_SEN1           0x0011
#define REG_ZERO_CAL_SEN2           0x0012
#define REG_ZERO_CAL_SEN3           0x0013
#define REG_CAP_CAL_SEN0            0x0014
#define REG_CAP_CAL_SEN1            0x0015
#define REG_CAP_CAL_SEN2            0x0016
#define REG_CAP_CAL_SEN3            0x0017
#define REG_SPAN_CAL_SEN0           0x0018
#define REG_SPAN_CAL_SEN1           0x0019
#define REG_SPAN_CAL_SEN2           0x001A
#define REG_SPAN_CAL_SEN3           0x001B
#define REG_ZERO_LIM                0x001C

#define REG_CAP_ALL                 0x0020
#define REG_ZERO_ALL                0x0021
#define REG_SPAN_ALL                0x0022


#define REG_READ_MINUTE_DATA        0x0030
#define REG_READ_SAMPLE_DATA        0x0031
#define REG_ALL_ADC_VALUE           0x0032
#define REG_GET_FINAL_VALUE         0x0033
#define REG_GET_FINAL_ALL           0x0034
#define REG_GET_PREV_LUX_SAMPLE     0x0035

#define REG_END                     0xFFFF

#define arrLen(arr) ((int)(sizeof(arr)/sizeof(arr)[0]))
#define lowerCaseToUpperCase(charactor) ((char)(((int)(charactor)) - 20))
#define upperCaseToLowerCase(charactor) ((char)(((int)(charactor)) + 20))

#define REG_EXECUTABLE      0x10
#define REG_READABLE        0x11
#define REG_WRITABLE        0x12

///////.............combined register mode...................//////
/* combined register modes created by adding individual register modes
 * ex: if 0x00FF reg is capable of read and wire the combined reg mode value (RRW_) = (REG_READABLE + REG_WRITABLE)
 *
 * (RXXX) is the short form of reg combined mode and XXX denote Read capability, write capability, and execute capability
 * "_" denote null capability
 */
#define RRWE  (REG_EXECUTABLE + REG_READABLE + REG_WRITABLE)
#define RRW_  (REG_READABLE + REG_WRITABLE)
#define RR_E  (REG_READABLE + REG_EXECUTABLE)
#define R_WE  (REG_WRITABLE + REG_EXECUTABLE)
#define RR__  (REG_READABLE)
#define R_W_  (REG_WRITABLE)
#define R__E  (REG_EXECUTABLE)

#define BUFFER_SIZE        256
#define MILI_TO_SECOND    1000
#define SECOND_TO_MINUTE  60
#define SAMPLE_RATE       10
#define MIN_CMD_LENGTH    7
#define MAX_ADC_VALUE     1024
#define MIN_ADC_VALUE     0
#define BASE_DECIMAL      10
#define GRADF_TO_GRAD     100000
#define MAX_SAMPLES       144

#define PROTOCOL_SEPERATOR ':'

typedef enum {REG_MODE_EXECUTE = 0, REG_MODE_READ, REG_MODE_WRITE, REG_MODE_MAX}regMode_t;

cmdDet_t cmdDet;

char message[BUFFER_SIZE];
char tempRegRes[35];
uint8_t buffer_count = 0;
uint16_t time_count = 0;
uint16_t second_count = 0;
uint16_t minute_count = 0;

///..............PROGRAM.............///
void initTimer(void);
void initCmdDet(void);
void idle(void);
bool handleConversion(void);

///...............COMMUNICATION...........///
void commsHandle(void);
bool checkForEnd(char inByte);
void commandHandle(char *msg);
bool isMsgValid(char *msg);
char findCommand( char *header, char *str);
void showResults(char res, char *str);
void updateHeader(char *hdr);
bool LookupRegMode(uint8_t regMode);
static char LookupAndRunCommand(uint16_t reg, char *param_ptr);
void ProccessCmd(void);
void ExicuteRinCmd(void);
void WriteRinRegister(void);
void ReplyRinCmd(void);
void MakeHeader(char *message);
long hex_to_long(char *string, unsigned short width);

char SampleFunction(uint8_t mode, char *str);
char NullTest(uint8_t mode, char *str);

static const cmd_t cmdSet[]=
{
  {REG_ZERO_CAL_SEN0,                   RR_E,   SampleFunction},
  {REG_ZERO_CAL_SEN1,                   RR_E,   SampleFunction},
  {REG_ZERO_CAL_SEN2,                   RR_E,   SampleFunction},
  {REG_ZERO_CAL_SEN3,                   RR_E,   SampleFunction},
  
  {REG_CAP_CAL_SEN0,                    RRW_,   SampleFunction},
  {REG_CAP_CAL_SEN1,                    RRW_,   SampleFunction},
  {REG_CAP_CAL_SEN2,                    RRW_,   SampleFunction},
  {REG_CAP_CAL_SEN3,                    RRW_,   SampleFunction},

  {REG_CAP_ALL,                         RR__,   SampleFunction},
  {REG_ZERO_ALL,                        RR__,   SampleFunction},
  {REG_SPAN_ALL,                        RR__,   SampleFunction},
 
  
  {REG_SPAN_CAL_SEN0,                   RR_E,   SampleFunction},
  {REG_SPAN_CAL_SEN1,                   RR_E,   SampleFunction},
  {REG_SPAN_CAL_SEN2,                   RR_E,   SampleFunction},
  {REG_SPAN_CAL_SEN3,                   RR_E,   SampleFunction},
  
  {REG_ZERO_LIM,                        RRW_,   SampleFunction},
  
  {REG_READ_MINUTE_DATA,                RR__,   SampleFunction},
  {REG_READ_SAMPLE_DATA,                RR__,   SampleFunction},
  {REG_ALL_ADC_VALUE,                   RR__,   SampleFunction},
  {REG_GET_FINAL_VALUE,                 RR__,   SampleFunction},
  {REG_GET_FINAL_ALL,                   RR__,   SampleFunction},
  {REG_GET_PREV_LUX_SAMPLE,             RR__,   SampleFunction},
  

  {REG_END,                             RRWE,   NullTest}
};


// adjust addresses if needed
ADS1115 ADS_1(0x49);
ADS1115 ADS_2(0x48);

//  two interrupt flags
volatile bool RDY_1 = false;
volatile bool RDY_2 = false;

uint8_t RDY_Pin_1 = 2;        // interrupt pin for device 1
uint8_t RDY_Pin_2 = 3;        // interrupt pin for device 2

uint8_t channel_1 = 0;         // channel from device 1
uint8_t channel_2 = 0;         // channel from device 2

//  array to hold the data.
int16_t val[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };


void setup()
{
  Serial.begin(115200);
  //Serial.println(__FILE__);
  //Serial.print("ADS1X15_LIB_VERSION: ");
  //Serial.println(ADS1X15_LIB_VERSION);

  // SETUP FIRST ADS1115
  ADS_1.begin();
  ADS_1.setGain(0);        // 6.144 volt
  ADS_1.setDataRate(7);

  // SET ALERT RDY PIN
  ADS_1.setComparatorThresholdHigh(0x8000);
  ADS_1.setComparatorThresholdLow(0x0000);
  ADS_1.setComparatorQueConvert(0);

  // SET INTERRUPT HANDLER TO CATCH CONVERSION READY
  pinMode(RDY_Pin_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RDY_Pin_1), adsReady_1, RISING);

  ADS_1.setMode(0);          // continuous mode
  ADS_1.readADC(channel_1);  // trigger first read


  // SETUP SECOND ADS1115
  ADS_2.begin();
  ADS_2.setGain(0);        // 6.144 volt
  ADS_2.setDataRate(7);

  // SET ALERT RDY PIN
  ADS_2.setComparatorThresholdHigh(0x8000);
  ADS_2.setComparatorThresholdLow(0x0000);
  ADS_2.setComparatorQueConvert(0);

  // SET INTERRUPT HANDLER TO CATCH CONVERSION READY
  pinMode(RDY_Pin_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RDY_Pin_2), adsReady_2, RISING);

  ADS_2.setMode(0);          // continuous mode
  ADS_2.readADC(channel_2);  // trigger first read
}


void loop()
{
  idle();
  commsHandle();

/*
  for (int i = 0; i < 8; i++)
  {
    Serial.print(val[i]);
    Serial.print('\t');
    handleConversion();
  }
  Serial.println();
  delay(100);
 */
}


// catch interrupt and set flag device 1
void adsReady_1()
{
  RDY_1 = true;
}

// catch interrupt and set flag device 1
void adsReady_2()
{
  RDY_2 = true;
}


// handle conversions that are ready
bool handleConversion(void)
{
  bool rv = false;
  if (RDY_1)
  {
    // save the last value
    val[channel_1] = ADS_1.getValue();
    // request next channel
    channel_1++;
    if (channel_1 >= 4) channel_1 = 0;
    ADS_1.readADC(channel_1);
    RDY_1 = false;
    rv = true;
  }
  if (RDY_2)
  {
    // save the last value
    val[4 + channel_2] = ADS_2.getValue();
    // request next channel
    channel_2++;
    if (channel_2 >= 4) channel_2 = 0;
    ADS_2.readADC(channel_2);
    RDY_2 = false;
    rv = true;
  }
  return rv;
}



void initTimer(void)
{
  TCCR0A=(1<<WGM01);    //Set the CTC mode   
  OCR0A=0xF9; //Value for ORC0A for 1ms 
  
  TIMSK0|=(1<<OCIE0A);   //Set the interrupt request
  sei(); //Enable interrupt
  
  TCCR0B|=(1<<CS01);    //Set the prescale 1/64 clock
  TCCR0B|=(1<<CS00);
}

void initCmdDet(void)
{
  cmdDet.mode = 0;
  cmdDet.reg = 0;
  memset(cmdDet.data,'\0', arrLen(cmdDet.data));
  cmdDet.dataAvailable = false;
}

void idle(void)
{
  handleConversion();
  
  if(time_count > (MILI_TO_SECOND - 1))
  {
    second_count++;
    time_count = 0;
  }
  
  if(second_count > (SECOND_TO_MINUTE - 1))
  {
    //updateReadings(minute_count);
    //printMinuteData(minute_count);
    minute_count++;
    second_count = 0;
  }
  
  if (minute_count > (SAMPLE_RATE - 1))
  {
    //updateSample();
    minute_count = 0;
  }
}

///...............COMMUNICATION...........///
void commsHandle(void)
{
  char incomingByte;
  bool msgAvailable = false;
  
  while (Serial.available() > 0)
  {
    incomingByte = Serial.read();
    message[buffer_count++] = incomingByte;

    if(checkForEnd(incomingByte))
    {
      msgAvailable = true;
      Serial.flush();
    }
        
    if (buffer_count > (BUFFER_SIZE - 1))
      break;
  }
  
  if (msgAvailable)
  {
   commandHandle(message);
   memset(message, '\0', BUFFER_SIZE);
   buffer_count = 0;
  }
}

void commandHandle(char *msg)
{
 
  char *cp, *str;
  uint8_t heade_array_lenght = 20;
  char header[heade_array_lenght], res;
  uint8_t headerLength = 0 , count = 0 ;

  if (isMsgValid(msg))
  {
    cp = strchr(msg,PROTOCOL_SEPERATOR);
    headerLength = (uint8_t)(cp - msg);
    memset(header, '\0', heade_array_lenght);
    strncpy( header , msg , headerLength );
    initCmdDet();
    updateHeader(header);
    //strcpy(cmdDet.header,header);
    *cp++;
    if(checkForEnd(*cp))
      cmdDet.data[0] = '0';
       
    while(!checkForEnd(*cp))
    {
      cmdDet.data[count++] = *cp++;
    }
    
    ProccessCmd();
    //showResults(res, str); 
  }
}

void updateHeader(char *hdr)
{
  uint8_t mode_length = 3,reg_length = 5;
  char mode[mode_length];
  char reg[reg_length];
  
  memset(reg, '\0', reg_length);
  memset(mode, '\0', mode_length);
  mode[0] = *hdr++;
  mode[1] = *hdr++;
  reg[0] = *hdr++;
  reg[1] = *hdr++;
  reg[2] = *hdr++;
  reg[3] = *hdr++;
  //strncpy( mode , hdr , 2);
  //*hdr++;
  //*hdr++;
  //strncpy( reg , hdr , 4);
  //hex_to_long
  cmdDet.mode = (uint8_t)strtol(mode,NULL,16);
  cmdDet.reg = (uint16_t)strtol(reg,NULL,16);
  //cmdDet.mode = (uint8_t)hex_to_long(mode,2);
  //cmdDet.reg = (uint16_t)hex_to_long(reg,4);
        //Serial.print(cmdDet.mode);
      //Serial.print(",");
      //Serial.println(cmdDet.reg);
  
}

/*...............................End of Register Related functions.................................... */

/* see response is a magic number
 *
 * magic numbers (defined by programmer)
 *
 *   20: Invalid register
 *  21: Authentication fails
 *  22: Write successful
 *  23: Write fail
 *
 * */

void GetRegValue(uint16_t addres ,char *data)
{
  uint8_t res;
  
  res = LookupAndRunCommand(addres,data);
  switch (res)    // see magic 20 or 21 (20 is responsible for no reg found, 21 is responsible for authentication fails)
  {
    case (char)23:
      memset(tempRegRes, '\0', arrLen(tempRegRes));
      sprintf(tempRegRes, "%s;", "Fail");
      break;
    case (char)22:
      memset(tempRegRes, '\0', arrLen(tempRegRes));
      sprintf(tempRegRes, "%s;", "Ok");
      break;
    case (char)21:
      memset(tempRegRes, '\0', arrLen(tempRegRes));
      sprintf(tempRegRes, "%s;", "Auth_Fail");
      break;
    case (char)20:
      memset(tempRegRes, '\0', arrLen(tempRegRes));
      sprintf(tempRegRes, "%s;", "Inv_Reg");
      break;
  }
}

/** Lookup Reg Mode and return true if the incoming mode from rinCommand allow.*/
bool LookupRegMode(uint8_t regMode)
{
  bool status = false;
  uint8_t incomingRegMode;

  incomingRegMode = cmdDet.mode;
  switch (incomingRegMode)
  {
  case REG_READABLE:
    status = ((regMode == RRWE) ||
              (regMode == RRW_) ||
              (regMode == RR_E) ||
              (regMode == RR__)   )? true: false;
    break;
  case REG_WRITABLE:
    status = ((regMode == RRWE) ||
              (regMode == R_WE) ||
              (regMode == RRW_) ||
              (regMode == R_W_)   )? true: false;
    break;
  case REG_EXECUTABLE:
    status = ((regMode == RRWE) ||
              (regMode == R_WE) ||
              (regMode == RR_E) ||
              (regMode == R__E)   )? true: false;
    break;
  }

  return status;
}

/** Lookup cmd in the list of tests, and run the command.
 * Returns magic 20 if not found.
 * Returns magic 21 if register is not accessible with the current mode
 * */
static char LookupAndRunCommand(uint16_t reg, char *param_ptr)
{
  int i = 0;
  char result = 0;
  uint8_t regMode = 0;

  /* Locate command */
  while(!(cmdSet[i].reg == reg))
  {
    if((cmdSet[i].reg == REG_END))
      break;
    i += 1;
  }

  if(cmdSet[i].reg == REG_END)
    result = cmdSet[i].cmd_fn(regMode, param_ptr);
  else
  {
    if(LookupRegMode(cmdSet[i].mode))
    {
      switch(cmdDet.mode)
      {
        case REG_EXECUTABLE:
          regMode = REG_MODE_EXECUTE;
          break;
        case REG_READABLE:
          regMode = REG_MODE_READ;
          break;
        case REG_WRITABLE:
          regMode = REG_MODE_WRITE;
          break;
      }
      result = cmdSet[i].cmd_fn(regMode, param_ptr);
    }
    else
      result = 21;
  }

  return result;
}

char findCommand( char *header, char *str)
{
//  uint8_t count = 0;
//  char res;

//  for (count = 0 ; (strcmp("END" ,cmdSet[count].command) != 0) ; count++)
//  {
//    if (strcmp(header,cmdSet[count].command) == 0)
//      break;
//  }
//  res = cmdSet[count].cmd_fn(str); 
//  return res; 
}

long hex_to_long(char *string, unsigned short width)
{
   char szTemp[sizeof(long)*2 + 1]="";

   if (width > sizeof(long)*2)
      width = sizeof(long)*2;
   strncat(szTemp, string, (size_t)width);

   return strtol(szTemp, NULL, 0x10);
}

void MakeHeader(char *message)
{
  uint8_t cmdMode = cmdDet.mode;
  uint16_t cmdReg = cmdDet.reg;
//  sprintf(message, "%02X%0*X:", (uint8_t)pHeader->mode,
 //          (uint8_t)(sizeof(pHeader->reg) * 2), (uint16_t)pHeader->reg);
   sprintf(message, "%02X%04X:", cmdMode, cmdReg);
}

void ReplyRinCmd(void)
{
  char szTemp[15];
  char szReply[35];

  memset(szReply, '\0', arrLen(szReply));
  memset(szTemp, '\0', arrLen(szTemp));
  MakeHeader(szTemp);
  GetRegValue(cmdDet.reg ,szReply);
  Serial.print(szTemp);
  Serial.println(tempRegRes);
}

void WriteRinRegister(void)
{
  char szReply[35];
  char szHead[15];
  
  memset(szReply, '\0', arrLen(szReply));
  memset(szHead, '\0', arrLen(szHead));
  GetRegValue(cmdDet.reg ,szReply);
  MakeHeader(szHead);
  Serial.print(szHead);
  Serial.println(tempRegRes);
}

void ExicuteRinCmd(void)
{
  char szReply[35];
  char szHead[15];

  memset(szReply, '\0', arrLen(szReply));
  memset(szHead, '\0', arrLen(szHead));
  GetRegValue(cmdDet.reg ,szReply);
  MakeHeader(szHead);
  Serial.print(szHead);
  Serial.println(tempRegRes);
}

void ProccessCmd(void)
{
  switch(cmdDet.mode)
  {
    case (uint8_t)REG_EXECUTABLE:
      ExicuteRinCmd();
      break;
    case (uint8_t)REG_READABLE:
      ReplyRinCmd();
      break;
    case (uint8_t)REG_WRITABLE:
      WriteRinRegister();
      break;
  }

}

void showResults(char res, char *str)
{
  switch(res)
  {
    case 19:
      break;
    case 20:
      Serial.print(cmdDet.mode);
      Serial.print(cmdDet.reg);
      Serial.print(":");
      Serial.println("FAIL");
      break;
    case 21:
      Serial.print(cmdDet.mode);
      Serial.print(cmdDet.reg);
      Serial.print(":");
      Serial.println("SUCCESS");
      break;
    case 22:
      Serial.print(cmdDet.mode);
      Serial.print(cmdDet.reg);
      Serial.print(":");
      Serial.println(str);
      break;
  }
}

bool isMsgValid(char *msg)
{
  char *cp;
  bool msgValid = false;
  
  cp = strchr(msg,PROTOCOL_SEPERATOR);
  msgValid = ((cp != NULL) && ((cp - message) > (MIN_CMD_LENGTH - 2)))? true : false;
    
  return msgValid;
}

bool checkForEnd(char inByte)
{
  bool msgEnd = false;
  switch(inByte)
  {
    case ';':
    case '\r':
    case '\n':
      msgEnd = true;
    break;
  }
  return msgEnd;
}

char SampleFunction(uint8_t mode, char *str)
{
  char status = 0; 

  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    // nothing to Exicute
    //status = 23;
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%d,%d,%d,%d;", 1, 2, 3, 4);
    status = 19;
    break;
  case REG_MODE_WRITE:
    // noting to write
    break;
  }
  return status;
}


char NullTest(uint8_t mode, char *str)
{
  return (char)20;
}

ISR(TIMER0_COMPA_vect)
{
  initTimer();
  time_count++;
  
}
