#!/usr/bin/env python3
#You need to name this node "color_detector"

#The node subscribes to "/camera/rgb/image_raw" and analyses the colors in the video feed of the camera placed on the bot..
# The idea implemented is to check the color of various pixels using pixel_readers and if any of them detects a colour, then to publish the cleaning mode based on the color in "/cleaning_mode" rostopic
# blue = "sweep"  green = "sucktion"  none = "not cleaning"

import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

bridge = CvBridge()

pub=None
def detect_color(img):   #img because video is multiple frames

    cleaning_mode = String()

    new_img = bridge.imgmsg_to_cv2(img,desired_encoding="passthrough") #converting image to cv2 format
    cv_img = cv2.cvtColor(new_img,cv2.COLOR_RGB2BGR)    #image was in RGB but cv2 reads BGR


    height,width,_ = cv_img.shape

    pixel_reader1 = cv_img[int(height/2),int(width/2)]
    pixel_reader2 = cv_img[int(height/4),int(width/4)]
    pixel_reader3 = cv_img[int(height/4),int(3*width/4)]
    pixel_reader4 = cv_img[int(3*height/4),int(width/4)]
    pixel_reader5 = cv_img[int(3*height/4),int(3*width/4)]
    pixel_reader6 = cv_img[int(height/2),int(width/4)]
    pixel_reader7 = cv_img[int(height/4),int(width/2)]
    pixel_reader8 = cv_img[int(height/2),int(3*width/4)]
    pixel_reader9 = cv_img[int(3*height/4),int(width/2)]

    a = [pixel_reader1,pixel_reader2,pixel_reader3,pixel_reader4,pixel_reader5,pixel_reader6,pixel_reader7,pixel_reader8,pixel_reader9]

    cleaning_mode.data = "Not cleaning"
    for i in range(9):
        if (a[i]==[127,25,25]).all():
            cleaning_mode.data = "Sweep mode"
            break
        if (a[i]==[25,127,25]).all():
            cleaning_mode.data = "Suction mode"
            break

    pub.publish(cleaning_mode)
    cv2.imshow('image' , cv_img)
    cv2.waitKey(1) 

    ########   reader reads in [y,x] format!!!!!!!!
    

    # cv2.circle(cv_img,(int(width/2),int(height/2)),5,(255,0,0),3)
    # cv2.circle(cv_img,(int(width/4),int(height/4)),5,(255,0,0),3)
    # cv2.circle(cv_img,(int(3*width/4),int(height/4)),5,(255,0,0),3)
    # cv2.circle(cv_img,(int(width/4),int(3*height/4)),5,(255,0,0),3)
    # cv2.circle(cv_img,(int(3*width/4),int(3*height/4)),5,(255,0,0),3)
    # cv2.circle(cv_img,(int(width/4),int(height/2)),5,(255,0,0),3)
    # cv2.circle(cv_img,(int(width/2),int(height/4)),5,(255,0,0),3)
    # cv2.circle(cv_img,(int(3*width/4),int(height/2)),5,(255,0,0),3)
    # cv2.circle(cv_img,(int(width/2),int(3*height/4)),5,(255,0,0),3)

    # Tha above was used by me to check the location of the readers  
    
    

if __name__=='__main__':
    rospy.init_node("color_detector")

    sub = rospy.Subscriber('/camera/rgb/image_raw',Image,detect_color)

    pub = rospy.Publisher('cleaning_mode',String,queue_size=10)

    rospy.spin()