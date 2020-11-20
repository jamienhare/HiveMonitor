"""

Author: Gillian Minnehan
Date: 11/19/2020

"""
import datetime
import sys
sys.path.insert(1, "/Users/gillianminnehan/Documents/umich/eecs473/aws-iot-core/RaspPi_Codes")
from gatewayMain import configureAWS, publishPayload

def main():
    myMQTTClient = configureAWS("home/boot")
    bootMsg = {}
    bootMsg["time"] = str(datetime.datetime.now())
    bootMsg["msg"] = "Confirmed boot"
    publishPayload(bootMsg, myMQTTClient)

if __name__ == '__main__':
    main()