#!/usr/bin/env python3
import rospy
import pandas as pd
import os
import uuid
from robotont_msgs.msg import PowerSupply
from statistics import mean


class PowerInfo:
    def __init__(self):
        self.firstRun = True  # Flag for first 10 measurements captured

        #Dict for Pandas DataFrame creation
        self.powerData = {
            "time": rospy.Time.now().to_sec(), #Get time of node start
            "maxCapacity": 5.2 * 3.6 * pow(10, 3), # It is expected that the max capacity 5.2 Ah is measured from 16.8V to nominal voltage of 14.8
            "startingCapacity": 0.0,
            "draw": 0.0,
            "startingCapacity": 0.0,
            "batteryPercentage": 0.0,
            "remainingCapacity": 0.0,
        }
        
        self.startingTime = self.powerData["time"] #Set node starting time used in shutdown flagging
        self.df = pd.DataFrame(self.powerData, index=[0]) #Initialize "empty row", this can be done in callback in first run conditional if node starting time not necessary
        self.sub() #Init subscriber to /power_supply
        self.batRange = 2 #Maximal voltage drop from battery maximum voltage
        self.batMax = 16.8 #Battery maximum voltage
        self.batMin = 14.8 #Battery minimum voltage
        self.Varray = [] #Buffer for vattery voltages to be averaged
        self.Aarray = [] #Buffer for current draw values to be averaged
        self.path = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) #Path to package folder

    def callback(self, data):
        if len(self.Varray) < 9 or len(self.Aarray) < 9: #For 9 measurements only listen
            self.Aarray.append(data.current)
            self.Varray.append(data.voltage)
        else:
            # Append the 10th measurement and take the average
            self.Aarray.append(data.current)
            self.Varray.append(data.voltage)
            V = mean(self.Varray)
            A = mean(self.Aarray)


            # Calculate battery percentage from V data. If battery V has dropped more than self.batRange, battery is at 0.
            if self.batMax - V >= self.batRange:
                self.powerData["batteryPercentage"] = 0.0
            else:
                self.powerData["batteryPercentage"] = round(
                    ((V - self.batMin) / self.batRange) * 100
                )

            # If first run set starting capacity based on battery percentage, remaining capacity and first current draw
            if self.firstRun:
                self.powerData["startingCapacity"] = self.powerData["maxCapacity"] * (
                    self.powerData["batteryPercentage"] / 100
                )
                self.powerData["remainingCapacity"] = self.powerData["startingCapacity"]
                self.powerData["draw"] = A
                self.firstRun = False #Clear flag

            # Calculate consumed capacity based on average draw over the last measurement window
            previousCurrent = self.powerData["draw"]
            self.powerData["draw"] = A

            previousTime = self.powerData["time"]
            self.powerData["time"] = rospy.Time.now().to_sec()
            elapsed = self.powerData["time"] - previousTime
            consumed = mean((self.powerData["draw"], previousCurrent)) * elapsed

            # Calculate remaining capacity
            self.powerData["remainingCapacity"] = (
                self.powerData["remainingCapacity"] - consumed
            )

            # Add row to DataFrame
            self.df = self.df.append(self.powerData, ignore_index=True)

            # Log time left in test
            rospy.loginfo(
                "Testing for {} more seconds".format(
                    300 - (self.powerData["time"] - self.startingTime)
                )
            )

            # Reset the measurement arrays
            self.Aarray = []
            self.Varray = []


        if self.powerData["time"] - self.startingTime >= 300: #Flag shutdown
            self.shutDown()

    def sub(self):
        rospy.Subscriber("/power_supply", PowerSupply, self.callback)

    def shutDown(self):
        id = str(uuid.uuid4()) #Get a unique ID for measurement file
        self.df.to_csv(self.path + "/data/" + id + ".csv") #Save the measurements as csv
        rospy.signal_shutdown("Reached 5 minutes") #Shut down node


if __name__ == "__main__":
    rospy.init_node("PowerInfo", anonymous=True, disable_signals=True)
    node = PowerInfo()
    rospy.spin()
