import cv2
import numpy as np
img = cv2.imread('yellow_detect.jpeg')
#cv2.imshow(' ',img)
#cv2.waitKey(0)

blurred = cv2.GaussianBlur(img, (13,13), 0)

img_HSV = cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)
#cv2.imshow(' ',img_HSV)
#cv2.waitKey(0)

lowest = np.array([22, 93, 0])
highest = np.array([45, 255, 255])

hiding = cv2.inRange(img_HSV, lowest, highest)
result=cv2.bitwise_and(img,img,mask=hiding)
#cv2.imshow(' ',result)
#cv2.waitKey(0)
result1 = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
contours,hierarchy=cv2.findContours(result1,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
#cv2.drawContours(img,contours,-1,(0,255,0),2)
#cv2.imshow(' ',img)
#cv2.waitKey(0)

for i in contours:
    M=cv2.moments(i)
    if M['m00']!=0:
        cX=int(M["m10"]/M["m00"])
        cY=int(M["m01"]/M["m00"])
        print(cX,cY)
        
        	'''contours= contours[0] if len(contours) == 2 else contours[1]
				x = [0] * len(contours)
				y = [0] * len(contours)
				w = [0] * len(contours)
				h = [0] * len(contours)
				i = 0

				for c in contours:
					x[i], y[i], w[i], h[i] = cv2.boundingRect(contours[i])
					cv2.rectangle(self.img, (x[i], y[i]), (x[i] + w[i], y[i] + h[i]), (36, 255, 12), 2)
					i = i + 1
				index = 0
				a = 0
				c=0
				for b in range(len(x)):
					if int(w[b]) > a:
						a = w[b]
						c = b
				t = cv2.minAreaRect(contours[c])
				self.he=t[0][0]
				self.we=t[0][1]
				self.rx=1280-self.he
				self.ry=1280-self.we
				self.x=-(self.rx - self.h)/25
				self.y=-(self.w - self.ry)/25


				if(self.count>20):
					self.setpoint=[self.x,self.y,24]'''
					
					def image_callback(self,data):
		try:
			while((self.drone_position[0]<=(self.setpoint[0]+0.2) and self.drone_position[0]>=(self.setpoint[0]-0.2)) and (self.drone_position[1]<=(self.setpoint[1]+0.2) and self.drone_position[1]>=(self.setpoint[1]-0.2)) and (self.drone_position[2]<=(self.setpoint[2]+0.2) and self.drone_position[2]>=(self.setpoint[2]-0.2)) ) :
				self.count=self.count+1				
				self.img=self.bridge.imgmsg_to_cv2(data,"bgr8")
				self.blurred=cv2.GaussianBlur(self.img,(13,13),0)
				self.img_HSV=cv2.cvtColor(self.blurred,cv2.COLOR_BGR2HSV)
				lowest=np.array([22,93,0])
            
				highest=np.array([45,255,255])
				self.hiding=cv2.inRange(self.img_HSV,lowest,highest)
				self.result=cv2.bitwise_and(self.img,self.img,mask=self.hiding)
				self.result1=cv2.cvtColor(self.result,cv2.COLOR_BGR2GRAY)
				contours,hierarchy=cv2.findContours(self.result1,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE) 
                cv2.drawContours(self.img,contours,-1,(0,255,0),2)
                cv2.imshow(' ',self.img)
                cv2.waitKey(0)         

               
		except CvBridgeError as e:
			print(e)
