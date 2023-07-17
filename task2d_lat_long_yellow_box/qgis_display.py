#!/usr/bin/env python3
import rospy
from sentinel_drone.msg import Geolocation

class Edrone():
    def __init__(self):
        rospy.init_node('waypoint')
        rospy.Subscriber('geolocation1', Geolocation ,self.coord1)
        self.block_lat1 = 0.0
        self.block_lon1 = 0.0
        self.flag1 = 0
        rospy.Subscriber('geolocation2', Geolocation ,self.coord2)
        self.block_lat2 = 0.0
        self.block_lon2 = 0.0
        self.flag2 = 0
        rospy.Subscriber('geolocation3', Geolocation ,self.coord3)
        self.block_lat3 = 0.0
        self.block_lon3 = 0.0
        self.flag3 = 0
    def coord1(self,msg):
        self.block_lat1 = msg.lat
        self.block_lon1 = msg.long
    def coord2(self,msg):
        self.block_lat2 = msg.lat
        self.block_lon2 = msg.long
    def coord3(self,msg):
        self.block_lat3 = msg.lat
        self.block_lon3 = msg.long
    def plotpoint(self):
        while(self.flag1<20):
            canvas = iface.mapCanvas()
            lat = self.block_lat1
            lon = self.block_lon1
            print(lat,lon)
            pnt = QgsPointXY(lat, lon)
            m = QgsVertexMarker(canvas)
            m.setCenter(pnt)
            m.setColor(QColor('Black'))
            m.setIconType(QgsVertexMarker.ICON_CIRCLE)
            m.setIconSize(12)
            m.setPenWidth(1)
            m.setFillColor(QColor(0, 200, 0))
            self.flag1 = self.flag1 + 1
        while(self.flag2<20):
            canvas = iface.mapCanvas()
            lat = self.block_lat2
            lon = self.block_lon2
            print(lat,lon)
            pnt = QgsPointXY(lat, lon)
            m = QgsVertexMarker(canvas)
            m.setCenter(pnt)
            m.setColor(QColor('Black'))
            m.setIconType(QgsVertexMarker.ICON_CIRCLE)
            m.setIconSize(12)
            m.setPenWidth(1)
            m.setFillColor(QColor(0, 200, 0))
            self.flag2 = self.flag2 + 1
        while(self.flag3<20):
            canvas = iface.mapCanvas()
            lat = self.block_lat3
            lon = self.block_lon3
            print(lat,lon)
            pnt = QgsPointXY(lat, lon)
            m = QgsVertexMarker(canvas)
            m.setCenter(pnt)
            m.setColor(QColor('Black'))
            m.setIconType(QgsVertexMarker.ICON_CIRCLE)
            m.setIconSize(12)
            m.setPenWidth(1)
            m.setFillColor(QColor(0, 200, 0))
            self.flag3 = self.flag3 + 1
        
e_drone = Edrone()
e_drone.plotpoint()