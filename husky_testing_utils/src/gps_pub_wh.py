#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
import os
import json
import sys
from std_msgs.msg import String, Float64 
from geographiclib.geodesic import Geodesic



class GPS_Publisher:
    def __init__(self):
        rospy.Subscriber('/sensors/gps_1/fix', NavSatFix, self.gps_left_callback, queue_size=10)
        rospy.Subscriber('/sensors/gps_2/fix', NavSatFix, self.gps_right_callback, queue_size=10)
        self.gps_mid_pub = rospy.Publisher('/rover_mid/fix', NavSatFix, queue_size=10)
        self.heading_pub = rospy.Publisher('/heading', Float64, queue_size=10)

        self.gps_left = None
        self.gps_right = None


    @staticmethod
    def findHeadingAngle(pointA, pointB):
        geod = Geodesic.WGS84  #type: ignore #Create a WGS84 Geodesic object (WGS84 is the global standard for GPS)
        # Compute geodesic inverse solution between pointA and pointB
        angle = geod.Inverse(pointB[0], pointB[1], pointA[0], pointA[1])

        # Extract azimuth angle (heading direction) from pointB to pointA
        headingAngle = angle['azi1'] % 360.0  # Ensure angle is within [0, 360] degrees

        # Extract distance in meters
        distance = angle['s12']

        return headingAngle, distance
    
    @staticmethod
    def calculateRoverMidLatLon(roverOneLat, roverOneLon, roverTwoLat, roverTwoLon):
        geod = Geodesic.WGS84 #type: ignore
        # Calculate the midpoint between rover1 and rover2
        line = geod.InverseLine(roverOneLat, roverOneLon, roverTwoLat, roverTwoLon)
        distance = line.s13
        # Get the position at the halfway point (0.5 = 50% of the geodesic)
        position = line.Position(0.5 * distance)
        
        midLat = position['lat2']
        midLon = position['lon2']
        
        return midLat, midLon 


    def gps_left_callback(self,msg):
        self.gps_left = msg
        # print(f"Front GPS Data: lat={msg.latitude}, lon={msg.longitude}")
        self.compute_and_publish()

    def gps_right_callback(self, msg):
        self.gps_right = msg
        # print(f"Back GPS Data: lat={msg.latitude}, lon={msg.longitude}")
        self.compute_and_publish()


    def compute_and_publish(self):
        gps_front_data = self.gps_left
        gps_back_data = self.gps_right

        if gps_front_data is not None and gps_back_data is not None:
            lat_front = gps_front_data.latitude
            lon_front = gps_front_data.longitude
            lat_back = gps_back_data.latitude
            lon_back = gps_back_data.longitude

            # Compute heading from back to front
            heading, distance = self.findHeadingAngle([lat_front, lon_front], [lat_back, lon_back])

            print(f"Heading={heading} degrees, Distance={distance} meters")

            latitude, longitude = self.calculateRoverMidLatLon(lat_front, lon_front, lat_back, lon_back)
            # Log the data to the database
            # print(f"Logging GPS Data: Latitude={latitude}, Longitude={longitude}, Heading={heading}")
            gps_data = {"gps_data" : json.dumps({"lat": latitude,"lon": longitude, "heading": heading + 110})}
        

            gps_mid = NavSatFix()
            gps_mid.header.stamp = rospy.Time.now()
            gps_mid.header.frame_id = "rover_mid"
            gps_mid.latitude = latitude
            gps_mid.longitude = longitude
            gps_mid.altitude = 0.0  # Assuming altitude is not provided, set
            self.gps_mid_pub.publish(gps_mid)
            self.heading_pub.publish(Float64(heading))
        


if __name__ == "__main__":

    try:
        rospy.init_node('gps_pub_wh', anonymous=True)
        gps_pub = GPS_Publisher()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
