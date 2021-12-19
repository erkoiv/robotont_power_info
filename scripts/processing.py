#!/usr/bin/env python3
import rospy
import pandas as pd
import os
import uuid
from robotont_msgs.msg import PowerSupply
from statistics import mean

class processor:
    def __init__(self):
        self.firstRun = True #Flag for first 10 measurements captured
        self.powerData = {
            "time": rospy.Time.now().to_sec(),
            #It is expected that the max capacity is measured from 16.8V to nominal voltage of 14.8
            "maxCapacity": 5.2 * 3.6 * pow(10,3),
            "startingCapacity": 0.0,
            "current": 0.0,
            "startingCapacity": 0.0,
            "current": 0.0,
            "batteryPercentage": 0.0,
            "remainingCapacity": 0.0
        }
        self.startingTime = self.powerData["time"]
        self.df = pd.DataFrame(self.powerData, index=[0])
        self.sub()
        self.batRange = 2
        self.batMax = 16.8
        self.batMin = 14.8
        self.Varray = []
        self.Aarray = []
        self.path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        rospy.logwarn(self.path)



    def callback(self, data):
        if len(self.Varray) < 9 or len(self.Aarray) < 9:
            self.Aarray.append(data.current)
            self.Varray.append(data.voltage)
        else:
            #append last time so amount of measurements is 10
            self.Aarray.append(data.current)
            self.Varray.append(data.voltage)

            V = mean(self.Varray)
            A = mean(self.Aarray)

            #calculate battery percentage
            if self.batMax - V >= 2:
                self.powerData["batteryPercentage"] = 0.0
            else:
                self.powerData["batteryPercentage"] =  round(((V - self.batMin)/self.batRange)*100)

            #if first run set some values
            if self.firstRun:
                self.powerData["startingCapacity"] = self.powerData["maxCapacity"] * (self.powerData["batteryPercentage"]/100)
                self.powerData["remainingCapacity"] = self.powerData["startingCapacity"]
                self.powerData["current"] = A
                self.firstRun = False

            previousCurrent = self.powerData["current"]
            self.powerData["current"] = A
            
            previousTime = self.powerData["time"]
            self.powerData["time"] = rospy.Time.now().to_sec()
            elapsed = self.powerData["time"] - previousTime
            consumed = mean((self.powerData["current"], previousCurrent))*elapsed
            
            self.powerData["remainingCapacity"] = self.powerData["remainingCapacity"] - consumed
            self.df = self.df.append(self.powerData, ignore_index=True)
            rospy.loginfo("Testing for {} more seconds".format(300 - (self.powerData["time"] - self.startingTime)))

            #reset the measurement arrays
            self.Aarray = []
            self.Varray = []

        if self.powerData["time"] - self.startingTime >= 300:
            self.shutDown()



    def sub(self):
        rospy.Subscriber("/power_supply", PowerSupply, self.callback)
    
    def shutDown(self):
        id = str(uuid.uuid4())
        self.df.to_csv(self.path+'/data/'+id+".csv")
        rospy.signal_shutdown("Reached 5 minutes")


if __name__ == '__main__':
    rospy.init_node('dataProcessing', anonymous=True, disable_signals=True)
    node = processor()
    rospy.spin()