
import rospy
import matplotlib as mpl
import matplotlib.pyplot as plt
from cv_bridge import CvBridge

import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import Image
from classification.msg import bbox_2d

def callback(data):
    global pub_bbox, pub_det
    meta = data.info
    # print(meta)
    imgdata = np.array(data.data)
    img = imgdata.reshape((meta.height,meta.width))
    bin = img>1
    bin.dtype='uint8'
    kernel = np.ones((5,5),np.uint8)
    imgfiltered = cv2.dilate(bin,kernel,iterations = 1)

    im_floodfill = imgfiltered.copy()
    h, w = bin.shape[:2]
    mask = np.zeros((h+2, w+2), np.uint8)
    cv2.floodFill(im_floodfill, mask, (0,0), 255)
    im_floodfill_inv = cv2.bitwise_not(im_floodfill)
    im_out = bin | im_floodfill_inv
    im_out = cv2.erode(im_out,kernel,iterations = 1)

    contours, hierarchy = cv2.findContours(im_out,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    im2 = im_out.copy()
    # im2 = im_out.copy()
    backtorgb = cv2.cvtColor(im_out,cv2.COLOR_GRAY2RGB)

    cnts = []

    for data in contours:
        if cv2.contourArea(data) > 300:
            x,y,w,h = cv2.boundingRect(data)
            cv2.rectangle(backtorgb,(x,y),(x+w,y+h),(0,255,0),2)

            cnts.append(data)
            # cv2.drawContours(backtorgb,[data],-1,(0,255,0),5)
        # print ("The contours have this data" + str(data))

    b = CvBridge()
    image_msg = b.cv2_to_imgmsg(backtorgb, encoding="passthrough")
    pub_det.publish(image_msg)

    if len(cnts) > 0:
        a = bbox_2d()
        M = cv2.moments(cnts[2])
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        print (f"{cx}  {cy}")
        a.width.data = 15
        a.origin.position.x = meta.origin.position.x + cx*meta.resolution
        a.origin.position.z = meta.origin.position.y + cy*meta.resolution
        pub_bbox.publish(a)



def listener():
    global pub_bbox, pub_det
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/projected_map", OccupancyGrid, callback)

    pub_det = rospy.Publisher('/feature/2d_det', Image, queue_size=1)
    pub_bbox = rospy.Publisher('/feature/2d_bbox', bbox_2d, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()