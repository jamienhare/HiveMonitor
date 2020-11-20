"""

Author: Gillian Minnehan
Date: 11/19/2020

"""
import datetime
from gateway import configureAWS, publishPayload

def main():
    myMQTTClient = configureAWS()
    bootMsg = {}
    bootMsg["time"] = str(datetime.datetime.now())
    bootMsg["msg"] = "Confirmed boot"
    publishPayload(bootMsg, myMQTTClient)

if __name__ == '__main__':
    main()