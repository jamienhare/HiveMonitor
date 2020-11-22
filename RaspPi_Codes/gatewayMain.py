"""

Author: Gillian Minnehan
Date: 11/19/2020

"""

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

# Globals
MAX_MESSAGE_SIZE = 50 
myport = serial.Serial("/dev/ttyS0", 115200, timeout = 0.1)
myport.flush()

def confirmedAWSMsgReceipt(client, userData, message):
    """

    Callback confirming successful message receipt by AWS.

    client - AWS client
    userData - data returned by AWS on user (?)
    message - details of message published to AWS

    """
    print("Received Message from AWS IoT Core")
    print("Topic: ", message.topic)
    print("Payload: ", message.payload) # payload is the message
 
def confirmedLoRaMsgReceipt(deviceId):
    """

    Callback confirming successful message receipt by LoRa.
    
    deviceId - id of device to send confirmation to.

    """
    msg = "GK"
    myport.write(bytes("AT+SEND="+str(deviceId)+","+str(len(msg))+","+msg+"\r\n", 'utf-8'))
    print("Sent aknowledgement")
    time.sleep(1)

def parseSerialString():
    """

    Parses a serial message received from the LoRa and returns
    a payload to upload to the database.

    """
    # Test message
    # serialMsg = '+RCV=120,4,0123456789tl2,34,-45'
    
    # read serial port
    serialMsg = myport.read(MAX_MESSAGE_SIZE)
    print(serialMsg)
    
    # parse
    parsedMsg = serialMsg.split(b',') # parse received LoRa message
    parsedMsg[0] = parsedMsg[0][5:] # remove +RCV=
    size = len(parsedMsg) # packet size
    print(size)
    payload = {}
    if(size == 5): # expected packet size
        # parse bytes object
        sensorData = unpack('HHfff', parsedMsg[2])
        print(sensorData)

        # populate payload
        payload["id"] = str(parsedMsg[0], 'utf-8')
        payload["deviceId"] = str(parsedMsg[0], 'utf-8')
        payload["co2"] = sensorData[0]
        payload["tvoc"] = sensorData[1]
        payload["humidity"] = sensorData[2]
        payload["temp"] = sensorData[3]
        payload["freq"] = sensorData[4]
        payload["timeStamp"] = str(datetime.datetime.now())
        payload["createdAt"] = "2020-11-07T22:52:20.977Z" # static value for proper database upload
        payload["updatedAt"] = "2020-11-07T22:52:20.977Z" # static value for proper database upload
        
        # send AK to node
        confirmedLoRaMsgReceipt(payload["deviceId"])
    
    return payload
        

def configureAWS(topic):
    """

    Configures AWS client
        
    """

    logging.basicConfig(filename='pythonIotDevicePublish.log', filemode='w', format='%(name)s - %(levelname)s - %(message)s',level=logging.DEBUG)
    logger = logging.getLogger('pythonIotDevice')
    logger.info("pythonIotDevice")

    # For certificate based connection
    myMQTTClient = AWSIoTMQTTClient("BeeMonitorClient") # random key
    
    # For TLS mutual authentication
    myMQTTClient.configureEndpoint("a1r5j3v9sjm0wn-ats.iot.us-east-2.amazonaws.com", 8883) #AWS IoT Core endpoint
    myMQTTClient.configureCredentials("/home/pi/Documents/hiveMonitor/RaspPi_Codes/root-ca.pem", "/home/pi/Documents/hiveMonitor/RaspPi_Codes/private.pem.key", "/home/pi/Documents/hiveMonitor/RaspPi_Codes/certificate.pem.crt") #Set path for Root CA and unique device credentials (use the private key and certificate retrieved from the logs in Step 1)

    # Connect to AWS IoT Core
    myMQTTClient.configureOfflinePublishQueueing(-1) # Infinite offline publish queing
    myMQTTClient.configureDrainingFrequency(2) # Draining: 2 Hz
    myMQTTClient.configureConnectDisconnectTimeout(10) # 10 sec
    myMQTTClient.configureMQTTOperationTimeout(5) # 5 sec

    # connect to client and subscribe to AWS topic
    logger.info("Connecting...")
    print("Initiating AWS IoT Core Topic...")
    myMQTTClient.connect()
    myMQTTClient.subscribe(topic, 1, confirmedAWSMsgReceipt)
    print("AWS connection successful")
    return myMQTTClient

def publishPayload(payload, client):
    """

    Publish payload to the "home/test" topic

    """
    client.publish(
        topic="home/test",
        QoS=1,
        payload=json.dumps(payload)
    )

def readAndSentData(client):
    """

    Continually seeks serial data. Parses and publishes to AWS upon receipt.

    """
    while True:
        if(myport.in_waiting > 0): # wait for incoming data
            message = parseSerialString()
            if(len(message)):
                publishPayload(message, client)

def main():
    while True:
        try:
            myMQTTClient = configureAWS("home/test")
            readAndSentData(myMQTTClient)
        except KeyboardInterrupt:
            return
        except Exception as e:
            # TODO: log error
            print(e)
            continue
        
if __name__ == '__main__':
    main()