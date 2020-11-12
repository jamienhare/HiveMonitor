import boto3
import sys
import json
import logging
import datetime
import time
from time import sleep
import serial
from struct import *
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient

MAX_MESSAGE_SIZE = 50 
myport = serial.Serial("/dev/ttyS0", 115200, timeout = 0.1)
myport.flush()

def confirmedAWSMsgReceipt(client, userData, message):
    """

    Callback confirming successful message receipt by AWS.

    """
    print("Received Message from AWS IoT Core")
    print("Topic: ", message.topic)
    print("Payload: ", message.payload) # payload is the message
 
def confirmedLoRaMsgReceipt(deviceId):
    """

    Callback confirming successful message receipt by LoRa.
    
    deviceId - id of device to send confirmation to.

    """
    print("Entered funct")
    msg = "GK"
    myport.write("AT+SEND="+str(deviceId)+","+str(len(msg))+","+msg+"\r\n")
    time.sleep(1)

def parseSerialString():
    """

    Parses a serial message received from the LoRa and returns
    a dictionary with information to upload to the database.

    """
    # Test message
    # serialMsg = '+RCV=120,4,0123456789tl2,34,-45'
    
    serialMsg = myport.read(MAX_MESSAGE_SIZE)
    print(serialMsg)
    parsedMsg = serialMsg.split(',')
    parsedMsg[0] = parsedMsg[0][5:] # remove +RCV=
    size = len(parsedMsg)
    print(size)
    if(size > 2):
        print(parsedMsg[2])
    if(size == 5):
        print(sys.getsizeof(parsedMsg[2]))
        newMsg = unpack('HHfff', parsedMsg[2])
        print(newMsg)
        messageAWS = {}
        messageAWS["id"] = str(parsedMsg[0])
        messageAWS["deviceId"] = str(parsedMsg[0])
        messageAWS["co2"] = newMsg[0]
        messageAWS["tvoc"] = newMsg[1]
        messageAWS["humidity"] = newMsg[2]
        messageAWS["temp"] = newMsg[3]
        messageAWS["freq"] = newMsg[4]
        messageAWS["timeStamp"] = str(datetime.datetime.now())
        messageAWS["createdAt"] = "2020-11-07T22:52:20.977Z" # static value for proper database upload
        messageAWS["updatedAt"] = "2020-11-07T22:52:20.977Z" # static value for proper database upload
        confirmedLoRaMsgReceipt(messageAWS["deviceId"])
        # Publish sensor readings to AWS IoT Core
        print("Completed message receipt")
        myMQTTClient.publish(
            topic="home/test",
            QoS=1,
            payload=json.dumps(messageAWS)
        )
        print("Got past publish")

def readAndSendData():
    """

    Configures AWS, waits for data on the serial port, parses data, and sends to AWS.
        
        
    """

    logging.basicConfig(filename='pythonIotDevicePublish.log', filemode='w', format='%(name)s - %(levelname)s - %(message)s',level=logging.DEBUG)
    logger = logging.getLogger('pythonIotDevice')
    logger.info("pythonIotDevice")

    # For certificate based connection
    myMQTTClient = AWSIoTMQTTClient("BeeMonitorClient") # random key
    
    # For TLS mutual authentication
    myMQTTClient.configureEndpoint("a1r5j3v9sjm0wn-ats.iot.us-east-2.amazonaws.com", 8883) #AWS IoT Core endpoint
    myMQTTClient.configureCredentials("/home/pi/Documents/eecs473/root-ca.pem", "/home/pi/Documents/eecs473/private.pem.key", "/home/pi/Documents/eecs473/certificate.pem.crt") #Set path for Root CA and unique device credentials (use the private key and certificate retrieved from the logs in Step 1)

    # Connect to AWS IoT Core
    myMQTTClient.configureOfflinePublishQueueing(-1) # Infinite offline publish queing
    myMQTTClient.configureDrainingFrequency(2) # Draining: 2 Hz
    myMQTTClient.configureConnectDisconnectTimeout(10) # 10 sec
    myMQTTClient.configureMQTTOperationTimeout(5) # 5 sec

    logger.info("Connecting...")
    print("Initiating AWS IoT Core Topic...")

    myMQTTClient.connect()
    myMQTTClient.subscribe("home/test",1,confirmedAWSMsgReceipt)

    print("Got past connect")
    
    # Publish to the same topic in a loop forever
    while True:
        if(myport.in_waiting > 0):
            parseSerialString()

def main():
    while True:
        try:
            readAndSendData()
        except:
            continue
        
if __name__ == '__main__':
    main()