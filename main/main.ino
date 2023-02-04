//
//    FILE: main.ino
//  AUTHOR: Janaka
// VERSION: 0.01
// PURPOSE: read multiple analog inputs continuously interrupt 
//          driven to catch all conversions and send the value 
//          via the communication protocol.
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

#define REG_SOFTWARE_VER            0x0001
#define REG_HARDWARE_VER            0x0002
#define REG_SERIAL_NUMBER           0x0003

#define REG_INPUT_VOLTAGE           0x0020
#define REG_OUTPUT_VOLTAGE          0x0021

#define REG_ALL_ADC_VALUE           0x0030
#define REG_CH1_ADC_VALUE           0x0031
#define REG_CH2_ADC_VALUE           0x0032
#define REG_CH3_ADC_VALUE           0x0033
#define REG_CH4_ADC_VALUE           0x0034
#define REG_CH5_ADC_VALUE           0x0035
#define REG_CH6_ADC_VALUE           0x0036
#define REG_CH7_ADC_VALUE           0x0037
#define REG_CH8_ADC_VALUE           0x0038
#define REG_CH1_TO_CH6_ADC_VALUE    0x003A

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

#define BUFFER_SIZE       256
#define MILI_TO_SECOND    1000
#define SECOND_TO_MINUTE  60
#define SAMPLE_RATE       10
#define MIN_CMD_LENGTH    7
#define MAX_ADC_VALUE     1024
#define MIN_ADC_VALUE     0
#define BASE_DECIMAL      10
#define GRADF_TO_GRAD     100000
#define MAX_SAMPLES       144
#define SOFT_VER_MAIN     0
#define SOFT_VER_SUB      1
#define HARD_VER_MAIN     0
#define HARD_VER_SUB      1
#define SERIAL_NUMBER     1
#define EXT_ADC_1_ADD     0x49
#define EXT_ADC_2_ADD     0x48
#define CHNL_PER_EXT_ADC  4
#define EXT_BAUD_RATE     115200

#define PROTOCOL_SEPERATOR ':'

typedef enum {REG_MODE_EXECUTE = 0, REG_MODE_READ, REG_MODE_WRITE, REG_MODE_MAX}regMode_t;
typedef enum {EXT_ADC_1 = 0, EXT_ADC_2, MAX_ADC_COUNT}extAdc_t;
typedef enum {EXT_ADC_CH1 = 0, EXT_ADC_CH2, EXT_ADC_CH3, EXT_ADC_CH4, EXT_ADC_CH5, EXT_ADC_CH6, EXT_ADC_CH7, EXT_ADC_CH8, MAX_ADC_CHANNELS}extAdcChannels_t;

cmdDet_t cmdDet;

char message[BUFFER_SIZE];
char tempRegRes[35];
uint8_t buffer_count = 0;
uint16_t time_count = 0;
uint16_t second_count = 0;
uint16_t minute_count = 0;
uint8_t testLEDPin = 13;
uint8_t ExtAdcRedyPin[MAX_ADC_COUNT] = { 2, 3 };  // interrupt pins for device 1 and 2
bool extAdcStatus[MAX_ADC_COUNT];
int16_t extAdcVal[MAX_ADC_COUNT*CHNL_PER_EXT_ADC] = { 0, 0, 0, 0, 0, 0, 0, 0 };

///..............PROGRAM.............///
void initTimer(void);
void initSerial(void);
void initExtAdc(void);
void initVoltageRead(void);
void initCmdDet(void);
void idle(void);
bool updateExtAdcVal(void);
void setExtAdcReady(uint8_t adc,bool status_adc);
int16_t getExtAdcValue(uint8_t channel);
void setExtAdcValue(uint8_t channel,int16_t value);
uint8_t getExtAdcIntPin(uint8_t pin);
bool isExtAdcReady(uint8_t adc);
bool handleConversion(void);

void ext_adc_1_set(void);
void ext_adc_2_set(void);

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
void substr(char *str, uint8_t str_val, uint8_t end_val, char *ans);


char SoftwareVersion(uint8_t mode, char *str);
char HardwareVersion(uint8_t mode, char *str);
char SerialNumber(uint8_t mode, char *str);

char SampleFunction(uint8_t mode, char *str);
char AllExtAdcValue(uint8_t mode, char *str);
char Ch1ExtAdcValue(uint8_t mode, char *str);
char Ch2ExtAdcValue(uint8_t mode, char *str);
char Ch3ExtAdcValue(uint8_t mode, char *str);
char Ch4ExtAdcValue(uint8_t mode, char *str);
char Ch5ExtAdcValue(uint8_t mode, char *str);
char Ch6ExtAdcValue(uint8_t mode, char *str);
char Ch7ExtAdcValue(uint8_t mode, char *str);
char Ch8ExtAdcValue(uint8_t mode, char *str);
char Ch1ToCh6ExtAdcValue(uint8_t mode, char *str);

char NullTest(uint8_t mode, char *str);

static const cmd_t cmdSet[]=
{
  {REG_SOFTWARE_VER,                    RR__,   SoftwareVersion},
  {REG_HARDWARE_VER,                    RR__,   HardwareVersion},
  {REG_SERIAL_NUMBER,                   RR__,   SerialNumber},

  {REG_INPUT_VOLTAGE,                   RR__,   SampleFunction},
  {REG_OUTPUT_VOLTAGE,                  RR__,   SampleFunction},
  
  {REG_ALL_ADC_VALUE,                   RR__,   AllExtAdcValue},
  {REG_CH1_ADC_VALUE,                   RR__,   Ch1ExtAdcValue},
  {REG_CH2_ADC_VALUE,                   RR__,   Ch2ExtAdcValue},
  {REG_CH3_ADC_VALUE,                   RR__,   Ch3ExtAdcValue},
  {REG_CH4_ADC_VALUE,                   RR__,   Ch4ExtAdcValue},
  {REG_CH5_ADC_VALUE,                   RR__,   Ch5ExtAdcValue},
  {REG_CH6_ADC_VALUE,                   RR__,   Ch6ExtAdcValue},
  {REG_CH7_ADC_VALUE,                   RR__,   Ch7ExtAdcValue},
  {REG_CH8_ADC_VALUE,                   RR__,   Ch8ExtAdcValue},
  {REG_CH1_TO_CH6_ADC_VALUE,            RR__,   Ch1ToCh6ExtAdcValue},
  

  {REG_END,                             RRWE,   NullTest}
};


// adjust addresses if needed
ADS1115 extADC_1(EXT_ADC_1_ADD);
ADS1115 extADC_2(EXT_ADC_2_ADD);

void setup()
{
  pinMode(testLEDPin,OUTPUT);
  initTimer();
  initSerial();
  initExtAdc();
  initVoltageRead();
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

void ext_adc_1_set(void)
{
  setExtAdcReady(EXT_ADC_1,true);
}

void ext_adc_2_set(void)
{
  setExtAdcReady(EXT_ADC_2,true);
}

bool isExtAdcReady(uint8_t adc)
{
  if(adc < MAX_ADC_COUNT)
    return extAdcStatus[adc];
  else
    return false;
}

uint8_t getExtAdcIntPin(uint8_t pin)
{
  return ExtAdcRedyPin[pin]; 
}

void setExtAdcReady(uint8_t adc,bool status_adc)
{
  extAdcStatus[adc] = status_adc;
}

void setExtAdcValue(uint8_t channel,int16_t value)
{
  extAdcVal[channel] = value;
}

int16_t getExtAdcValue(uint8_t channel)
{
  return extAdcVal[channel];
}

bool updateExtAdcVal(void)
{
  bool update_st = false;
  static uint8_t ch_1 = 0, ch_2 = 0;
  if(isExtAdcReady(EXT_ADC_1))
  {
    setExtAdcValue(ch_1,extADC_1.getValue());
    //extAdcVal[ch_1] = extADC_1.getValue();
    ch_1++;
    if (ch_1 >= CHNL_PER_EXT_ADC) 
    {
      ch_1 = 0;
    }
    extADC_1.readADC(ch_1);
    setExtAdcReady(EXT_ADC_1,false);
    update_st = true;
  }

  if(isExtAdcReady(EXT_ADC_2))
  {
    setExtAdcValue(CHNL_PER_EXT_ADC + ch_2,extADC_2.getValue());
    //extAdcVal[CHNL_PER_EXT_ADC + ch_2] = extADC_2.getValue();
    ch_1++;
    if (ch_2 >= CHNL_PER_EXT_ADC) 
    {
      ch_2 = 0;
    }
    extADC_2.readADC(ch_2);
    setExtAdcReady(EXT_ADC_2,false);
    update_st = true;
  }
  return update_st;
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

void initSerial(void)
{
  Serial.begin(EXT_BAUD_RATE);
}

void initExtAdc(void)
{
  uint8_t adc; 
  // SETUP FIRST ADS1115
  adc = EXT_ADC_1;
  extADC_1.begin();
  extADC_1.setGain(0);        // 6.144 volt
  extADC_1.setDataRate(7);

  // SET ALERT RDY PIN
  extADC_1.setComparatorThresholdHigh(0x8000);
  extADC_1.setComparatorThresholdLow(0x0000);
  extADC_1.setComparatorQueConvert(0);

  // SET INTERRUPT HANDLER TO CATCH CONVERSION READY
  setExtAdcReady(adc,false);
  pinMode(getExtAdcIntPin(adc), INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(getExtAdcIntPin(adc)), ext_adc_1_set, RISING);

  extADC_1.setMode(0);          // continuous mode
  extADC_1.readADC(0);  // trigger first read


  // SETUP SECOND ADS1115
  adc = EXT_ADC_2;
  extADC_1.begin();
  extADC_1.setGain(0);        // 6.144 volt
  extADC_1.setDataRate(7);

  // SET ALERT RDY PIN
  extADC_1.setComparatorThresholdHigh(0x8000);
  extADC_1.setComparatorThresholdLow(0x0000);
  extADC_1.setComparatorQueConvert(0);

  // SET INTERRUPT HANDLER TO CATCH CONVERSION READY
  setExtAdcReady(adc,false);
  pinMode(getExtAdcIntPin(adc), INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(getExtAdcIntPin(adc)), ext_adc_2_set, RISING);

  extADC_1.setMode(0);          // continuous mode
  extADC_1.readADC(0);  // trigger first read  
}

void initVoltageRead(void)
{
  
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
  //static bool status_pin = true;
  updateExtAdcVal();
  if(time_count > (MILI_TO_SECOND - 1))
  {
    second_count++;
    time_count = 0;
//    if(status_pin)
//      digitalWrite(testLEDPin,HIGH);
//     else
//      digitalWrite(testLEDPin,LOW);
//    status_pin = !status_pin;
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
    /*
    if((uint8_t)(cp) > MIN_CMD_LENGTH)
    {
      memset(header, '\0', heade_array_lenght);
      substr(msg,(uint8_t)cp - MIN_CMD_LENGTH + 1,(uint8_t)cp,header);
      //strncpy( header , msg , headerLength );
    }
    else
    {
      headerLength = (uint8_t)(cp - msg);
      memset(header, '\0', heade_array_lenght);
      strncpy( header , msg , headerLength );
    }
    */
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

void substr(char *str, uint8_t str_val, uint8_t end_val, char *ans) 
{
    strncpy(ans, str+str_val, end_val);
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

char Ch1ExtAdcValue(uint8_t mode, char *str)
{
  char status = 0; 
  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%04X;", getExtAdcValue(EXT_ADC_CH1));
    status = 19;
    break;
  case REG_MODE_WRITE:
    break;
  }
  return status;
}

char Ch2ExtAdcValue(uint8_t mode, char *str)
{
  char status = 0; 
  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%04X;", getExtAdcValue(EXT_ADC_CH2));
    status = 19;
    break;
  case REG_MODE_WRITE:
    break;
  }
  return status;
}

char Ch3ExtAdcValue(uint8_t mode, char *str)
{
  char status = 0; 
  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%04X;", getExtAdcValue(EXT_ADC_CH3));
    status = 19;
    break;
  case REG_MODE_WRITE:
    break;
  }
  return status;
}

char Ch4ExtAdcValue(uint8_t mode, char *str)
{
  char status = 0; 
  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%04X;", getExtAdcValue(EXT_ADC_CH4));
    status = 19;
    break;
  case REG_MODE_WRITE:
    break;
  }
  return status;
}

char Ch5ExtAdcValue(uint8_t mode, char *str)
{
  char status = 0; 
  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%04X;", getExtAdcValue(EXT_ADC_CH5));
    status = 19;
    break;
  case REG_MODE_WRITE:
    break;
  }
  return status;
}

char Ch6ExtAdcValue(uint8_t mode, char *str)
{
  char status = 0; 
  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%04X;", getExtAdcValue(EXT_ADC_CH6));
    status = 19;
    break;
  case REG_MODE_WRITE:
    break;
  }
  return status;
}

char Ch7ExtAdcValue(uint8_t mode, char *str)
{
  char status = 0; 
  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%04X;", getExtAdcValue(EXT_ADC_CH7));
    status = 19;
    break;
  case REG_MODE_WRITE:
    break;
  }
  return status;
}

char Ch8ExtAdcValue(uint8_t mode, char *str)
{
  char status = 0; 
  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%04X;", getExtAdcValue(EXT_ADC_CH8));
    status = 19;
    break;
  case REG_MODE_WRITE:
    break;
  }
  return status;
}

char Ch1ToCh6ExtAdcValue(uint8_t mode, char *str)
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
    sprintf(tempRegRes, "%04X,%04X,%04X,%04X,%04X,%04X;", getExtAdcValue(EXT_ADC_CH1),getExtAdcValue(EXT_ADC_CH2),
                                                          getExtAdcValue(EXT_ADC_CH3),getExtAdcValue(EXT_ADC_CH4),
                                                          getExtAdcValue(EXT_ADC_CH5),getExtAdcValue(EXT_ADC_CH6));
    status = 19;
    break;
  case REG_MODE_WRITE:
    // noting to write
    break;
  }
  return status;
}

char AllExtAdcValue(uint8_t mode, char *str)
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
    sprintf(tempRegRes, "%04X,%04X,%04X,%04X,%04X,%04X,%04X,%04X;", getExtAdcValue(EXT_ADC_CH1),getExtAdcValue(EXT_ADC_CH2),
                                                                    getExtAdcValue(EXT_ADC_CH3),getExtAdcValue(EXT_ADC_CH4),
                                                                    getExtAdcValue(EXT_ADC_CH5),getExtAdcValue(EXT_ADC_CH6),
                                                                    getExtAdcValue(EXT_ADC_CH7),getExtAdcValue(EXT_ADC_CH8));
    status = 19;
    break;
  case REG_MODE_WRITE:
    // noting to write
    break;
  }
  return status;
}

char SoftwareVersion(uint8_t mode, char *str)
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
    sprintf(tempRegRes, "V%d.%02d;", SOFT_VER_MAIN, SOFT_VER_SUB);
    status = 19;
    break;
  case REG_MODE_WRITE:
    // noting to write
    break;
  }
  return status;
}

char HardwareVersion(uint8_t mode, char *str)
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
    sprintf(tempRegRes, "V%d.%02d;", HARD_VER_MAIN, HARD_VER_SUB);
    status = 19;
    break;
  case REG_MODE_WRITE:
    // noting to write
    break;
  }
  return status;
}

char SerialNumber(uint8_t mode, char *str)
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
    sprintf(tempRegRes, "%07d;", SERIAL_NUMBER);
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
