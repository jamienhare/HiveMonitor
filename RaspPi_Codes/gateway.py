import boto3
import json
import logging
import datetime
import time
from time import sleep
#import serial
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient

#myport = serial.Serial("/dev/ttyS0", 115200) 
ADDR = 0x456 # TODO: Address of LoRa module

# callback confirming successful message receipt by AWS
def confirmedAWSMsgReceipt(self, param, packet):
    print("Received Message from AWS IoT Core")
    print("Topic: ", packet.topic)
    print("Payload: ", packet.payload) # payload is the message

# callback confirming successful message receipt by 
def confirmedLoRaMsgReceipt():
    msg = "Received sensor readings"
    #myport.write("AT+SEND="+str(ADDR)+","+str(sys.getsizeof(msg))+","+msg+"\r\n")
    # TODO: need to check for OK?

# returns LoRa serial data
def readSerialString():
    # TODO: Deepen
    #msg = myport.read()
    # Check for SOP
    # Check for EOP
    return "01234"

def parseSerialString(msg):
    message = {}
    # TODO: Update based on how many bytes each reading takes up
    message['deviceId'] = msg[0]
    message['timeStamp'] = datetime.datetime.now().time()
    message['temp'] = msg[1]
    message['humidity'] = msg[2]
    message['c02'] = msg[3]
    message['audio'] = msg[4]
    return message

logging.basicConfig(filename='pythonIotDevicePublish.log', filemode='w', format='%(name)s - %(levelname)s - %(message)s',level=logging.DEBUG)
logger = logging.getLogger('pythonIotDevice')
logger.info("pythonIotDevice")

# For certificate based connection
myMQTTClient = AWSIoTMQTTClient("BeeMonitorClient") # random key
# For TLS mutual authentication
myMQTTClient.configureEndpoint("a1r5j3v9sjm0wn-ats.iot.us-east-2.amazonaws.com", 8883) #AWS IoT Core endpoint
myMQTTClient.configureCredentials("/home/pi/Documents/aws-iot-core/root-ca.pem", "/home/pi/Documents/aws-iot-core/private.pem.key", "/home/pi/Documents/aws-iot-core/certificate.pem.crt") #Set path for Root CA and unique device credentials (use the private key and certificate retrieved from the logs in Step 1)

# Connect to AWS IoT Core
myMQTTClient.configureOfflinePublishQueueing(-1) # Infinite offline publish queing
myMQTTClient.configureDrainingFrequency(2) # Draining: 2 Hz
myMQTTClient.configureConnectDisconnectTimeout(10) # 10 sec
myMQTTClient.configureMQTTOperationTimeout(5) # 5 sec

logger.info("Connecting...")
print("Initiating AWS IoT Core Topic...")
myMQTTClient.connect()
myMQTTClient.subscribe("home/test",1,confirmedAWSMsgReceipt)

# Publish to the same topic in a loop forever
#while True:
msgString = readSerialString()
    # do error checking to ensure what I read was correct
message = parseSerialString(msgString)
    # Publish sensor readings to AWS IoT Core
myMQTTClient.publish("home/test",str(message), 1)