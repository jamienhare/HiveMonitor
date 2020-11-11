/*
 * Author - Deepen Solanki
 * Purpose - LoRa Transmitter Code with 
 * Module - REYAX RYLR896 
 */

#include<string.h>

#define NETWORKID   6
#define TXNODE      121
#define RXNODE      100

int sendAT();
int setNetworkID(unsigned int);
int setNodeID(unsigned int);
int setParams(void);
int sendData(unsigned int nodeID, char message[5]);          
 
void setup() 
{
  int ret;
  Serial.begin(115200);
  Serial.setTimeout(5000);
  
  ret = sendAT();
  if(ret == -1)
    Serial.print("Sending AT failed, please check wiring or module\n");   
  
  ret = setNetworkID(NETWORKID);
  if(ret == -1)
    Serial.print("Setting Network ID failed. Please check wiring or module\n");   

  ret = setNodeID(TXNODE);
  if(ret == -1)
    Serial.print("Setting Node ID failed. Please check wiring or module\n"); 

  ret = setParams();
  if(ret == -1)
    Serial.print("Setting params failed. Please check wiring or module\n");   
}

void loop() 
{
  int ret;
  int sent = 0;
  int sentTime;
  bool notSentFlag = 0;
  String rx = "";
  char mess[9] = {65,',',66,',',67,',',68,',',69};
  //char *mess = "Krabby Patty";  
  ret = sendData(RXNODE,mess);
  if(ret == -1)
  {
    Serial.println("Sending data failed. Please check wiring or module");
    //writeToSD(Message, problem with module)
  }
  else
  {
    delay(5000);      // This delay is waiting for the message to come from the Gateway. Can optimize it. 
    rx = Serial.readString();
    if(rx.indexOf("GK")==-1)
    {
      ret = sendData(RXNODE,mess);
      delay(5000);    // This delay is waiting for the message to come from the Gateway. Can optimize it. 
      rx = Serial.readString();
      if(rx.indexOf("GK")==-1)
        notSentFlag = 1;
    }
  } 
}

int sendAT()
{
  String rx = "";
  Serial.print("AT\r\n");
  delay(1000);
  rx = Serial.readString();
  if(rx.indexOf("OK")>=0)
    return 1;
  else
    return -1;
}

int setNetworkID(unsigned int ID)
{
  String rx = "";
  Serial.print("AT+NETWORKID="+String(ID)+"\r\n");
  delay(1000);
  rx = Serial.readString();
  if(rx.indexOf("OK")>=0)
    return 1;
  else
    return -1;
}

int setNodeID(unsigned int ID)
{
  String rx = "";
  Serial.print("AT+ADDRESS="+String(ID)+"\r\n");
  delay(1000);
  rx = Serial.readString();
  if(rx.indexOf("OK")>=0)
    return 1;
  else
    return -1;
}

int setParams(void)
{
  String rx = "";
  Serial.print("AT+PARAMETER=7,9,4,5\r\n");
  delay(1000);
  rx = Serial.readString();
  if(rx.indexOf("OK")>=0)
    return 1;
  else
    return -1;
}

int sendData(unsigned int ID, char message[9])
{
  String rx = "";
  //int l = (int)(strlen(message));
  int l = 9;
  Serial.print("AT+SEND="+String(ID)+","+String(l)+","+message+"\r\n");
  delay(1000);
  rx = Serial.readString();
  if(rx.indexOf("OK")>=0)
    return 1;
  else
    return -1;
}
