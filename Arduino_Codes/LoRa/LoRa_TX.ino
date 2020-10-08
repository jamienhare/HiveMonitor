/*
 * Author - Deepen Solanki
 * Purpose - LoRa Transmitter Code
 * Module - REYAX RYLR890 
 */

#include<string.h>

#define NETWORKID   6
#define TXNODE      120
#define RXNODE      100

int sendAT();
int setNetworkID(unsigned int);
int setNodeID(unsigned int);
int sendData(unsigned int nodeID, char* message);          
 
void setup() 
{
  int ret;
  Serial.begin(115200);
  Serial.setTimeout(2000);
  
  ret = sendAT();
  if(ret == -1)
    Serial.print("Sending AT failed, please check wiring or module\n");   
  
  ret = setNetworkID(NETWORKID);
  if(ret == -1)
    Serial.print("Setting Network ID failed. Please check wiring or module\n");   

  ret = setNodeID(TXNODE);
  if(ret == -1)
    Serial.print("Setting Node ID failed. Please check wiring or module\n");   
  
  delay(1000);
}

void loop() 
{
  int ret;
  ret = sendData(RXNODE,"Krabby Patty");
  if(ret == -1)
  {
    Serial.println("Sending data failed. Please check wiring or module");
    while(1);
  }
  delay(2000);
}

int sendAT()
{
  String rx = "";
  Serial.print("AT\r\n");
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
  rx = Serial.readString();
  if(rx.indexOf("OK")>=0)
    return 1;
  else
    return -1;
}

int sendData(unsigned int ID, char* message)
{
  String rx = "";
  int l = (int)(strlen(message));
  Serial.print("AT+SEND="+String(ID)+","+String(l)+","+message+"\r\n");
  rx = Serial.readString();
  if(rx.indexOf("OK")>=0)
    return 1;
  else
    return -1;
}
