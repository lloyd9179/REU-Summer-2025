#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Float32MultiArray, String
from std_msgs.msg import Float32, Bool
import time
import math
import sys
import os
import logging as log
import importlib
import platform
import numpy 
import onnxruntime as ort
#from scipy.special import softmax
from scipy import linalg as LA
from openvino.runtime import Core
import time 
from cv_bridge import CvBridge
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy


def f3(val):
    try:
        v = float(val)
        if v.is_integer():
            return '{:,}'.format(int(v))  
        if not isinstance(v, float):
            v = 0.0
        return '{:,.3f}'.format(v)
    except:
        return str(val)


class Perception(Node):

    def __init__(self, compiled_model_onnx, input_layer_onnx, output_layer_onnx):
        super().__init__('vision_perception')  # Initializes the node

    #def __init__(self,loaded_net,input_blob,out_blob):
    #def __init__(self,ort_session):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.enable_perception = True
        self.verbose = False
        #self.session = ort_session
        # self.exec_net = loaded_net
        # self.input_blob = input_blob
        # self.out_blob  = out_blob
        self.input_cam = 'front'

        self.compiled_model_onnx = compiled_model_onnx
        self.input_layer_onnx = input_layer_onnx
        self.output_layer_onnx = output_layer_onnx

        # Replacing rospy.get_time() with ROS2 equivalent
        self.last_print_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Parameters must be declared before being used in ROS2
        self.declare_parameter('output_heading', '/terrasentia/vision/heading')
        self.declare_parameter('output_distance', '/terrasentia/vision/distance')
        self.declare_parameter('in_row_method', 'vision')
        self.declare_parameter('latency_threshold_fps_print', 10)
        self.declare_parameter('mode', 'both')
        self.declare_parameter('rotate_180', True)
        self.declare_parameter('primary_cam_for_nav', 'front')
        self.declare_parameter('use_heading_from_inference', True)

        # print("It has:",self.get_parameter('gain_ctrack_error_x').get_parameter_value().double_value)
        # Fetching parameters using self.get_parameter()
        self.in_row_method = self.get_parameter('in_row_method').get_parameter_value().string_value
        self.latency_threshold_fps_print = self.get_parameter('latency_threshold_fps_print').get_parameter_value().integer_value
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.rotate_180 = self.get_parameter('rotate_180').get_parameter_value().bool_value
        self.primary_cam = self.get_parameter('primary_cam_for_nav').get_parameter_value().string_value
        self.use_heading_from_inference = self.get_parameter('use_heading_from_inference').get_parameter_value().bool_value


        #pks11
        #self.row_width = 1.25  ##testing at TB basement
        #self.row_width = 0.75   ##HighT row length
        #self.row_width = 1.5   #HIghT 2 row length
        self.row_width = 72*0.0254
        #intrinsic parameters
        self.pfx = 0
        self.pfy = 0
        self.flx = 0
        self.fly = 0

        self.heading = 0
        self.roll = 0


        # output_distance_topic = self.get_parameter('~output_distance', 'distance') 
        output_distance_topic = self.get_parameter_or('~output_distance','/terrasentia/vision/distance')
        self.output_distance = self.create_publisher(Float32MultiArray,output_distance_topic, qos_profile = 1) 

        output_heading_topic = self.get_parameter_or('~output_heading', '/terrasentia/vision/heading') 
        self.output_heading = self.create_publisher(Float32MultiArray,output_heading_topic, qos_profile = 1) 

        self.output_vp = self.create_publisher(Float32MultiArray,'/terrasentia/vision/vp', qos_profile = 1) 
        self.output_ll = self.create_publisher(Float32MultiArray,'/terrasentia/vision/ll', qos_profile = 1) 
        self.output_lr = self.create_publisher(Float32MultiArray,'/terrasentia/vision/lr', qos_profile = 1) 

        self.output_vp_conf = self.create_publisher(Float32MultiArray,'/terrasentia/vision/vp_confidence', qos_profile = 1) 
        self.output_ll_conf = self.create_publisher(Float32MultiArray,'/terrasentia/vision/ll_confidence', qos_profile = 1) 
        self.output_lr_conf = self.create_publisher(Float32MultiArray,'/terrasentia/vision/lr_confidence', qos_profile = 1) 

        output_vpl_img_topic = self.get_parameter_or('~output_vpl_img','/terrasentia/vision/vpl_img_argmax')
        self.output_vpl_argmax_img = self.create_publisher(Image,output_vpl_img_topic, qos_profile=1)
        self.output_vpl_img = self.create_publisher(Image,'/terrasentia/vision/vpl_img', qos_profile=1)

        self.debug_pub = self.create_publisher(Float32MultiArray,'debug_vision_row_follow', qos_profile = 1)

        self.subscribers()
            
        self.myloginfo('### mode ' + str(self.mode) + ' in_row_method is ' + self.in_row_method + ' rotate_180? ' + str(self.rotate_180))    
        
    def subscribers(self):
        '''
        input_topic = rospy.get_param('~input')
        #buff_size = rospy.get_param('~input_buff_size')
        self.img_subscriber = rospy.Subscriber(input_topic,
            CompressedImage, self.callback, queue_size = 1, buff_size=2**24)  

        input_calibration_topic = rospy.get_param('~input_calibration_topic')
        #buff_size = rospy.get_param('~input_buff_size')
        self.img_calibration_subscriber = rospy.Subscriber(input_calibration_topic,
            CameraInfo, self.callbackCameraInfo, queue_size = 1)  
        '''

        # input_topic = '/front_cam/image_raw'
        # input_calibration_topic = '/front_cam/camera_info'

        input_topic = '/front_cam_proc/image_color'
        input_calibration_topic = '/front_cam/camera_info'
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,  # or ReliabilityPolicy.BEST_EFFORT
                                    depth=1)

        self.img_subscriber = self.create_subscription(Image,input_topic, self.callback,qos_profile = qos_profile)  

        self.img_calibration_subscriber = self.create_subscription(CameraInfo,input_calibration_topic, self.callbackCameraInfo, qos_profile = 1)  

        input_back_cam_topic = self.get_parameter_or('~input_back_cam', 'none')
        if input_back_cam_topic != 'none':
            self.img_back_cam_subscriber = self.create_subscription(CompressedImage,input_back_cam_topic, self.callbackBackCam, qos_profile= 1)  
        
        self.sub = self.create_subscription(String,'cam2use',  self.callbackCam2use, qos_profile=1)

        self.sub_toggle = self.create_subscription(Bool,'toggle_perception_vision',  self.togglePerceptionCb, qos_profile=1)

        
    def myloginfo(self, msg=''):
        self.get_logger().info('[' + str(self.get_name()) + '] ' + str(msg))

    def mylogwarn(self, msg=''):
        self.get_logger().warn('[' + str(self.get_name()) + '] ' + str(msg))

    def mylogerr(self, msg=''):
        self.get_logger().error('[' + str(self.get_name()) + '] ' + str(msg))
    
    def pubDistance(self, res):

        vp_x,vp_y = res[:2]

        ll_x, ll_y = res[2:4]
        lr_x, lr_y = res[4:6]

        # heading = np.deg2rad(self.heading)  #prev case
        heading = self.heading    #self.heading is already in radians

        A = np.array([[0, 0, 1],
                    [0, 0, 0],
                    [-1, 0, 0 ]])
        

        assert self.h==224 and self.w == 320

        R = LA.expm(A*(heading))
        K = np.array([[self.flx, 0, self.pfx],
                    [0, self.fly, self.pfy-8],
                    [0, 0,   1]])

        

        # Final and overall transformation matrix
        H = K @ R @ LA.inv(K)


        lrow = np.array([[ll_x,ll_y,1],[vp_x,vp_y,1]])
        rrow = np.array([[lr_x,lr_y,1],[vp_x,vp_y,1]])

        lrow = (H@lrow.T).T 
        rrow = (H@rrow.T).T 

        lrow[0] = lrow[0]/lrow[0][2]; rrow[0] = rrow[0]/rrow[0][2]
        lrow[1] = lrow[1]/lrow[1][2]; rrow[1] = rrow[1]/rrow[1][2]

        #pks11 - checks
        if abs(lrow[1][0]-lrow[0][0]) < 0.001 or abs(rrow[1][0]-rrow[0][0])< 0.001:
            lrow_slope = 1
            rrow_slope = 1 #pks11 : Need testing of these checks
        else:
            lrow_slope = (lrow[1][1]-lrow[0][1])/(lrow[1][0]-lrow[0][0])
            rrow_slope = (rrow[1][1]-rrow[0][1])/(rrow[1][0]-rrow[0][0])
        lrow_intercept = lrow[0][0]+(self.h-lrow[0][1])/lrow_slope
        rrow_intercept = rrow[0][0]+(self.h-rrow[0][1])/rrow_slope

        dl = (self.pfx)-lrow_intercept
        dr = rrow_intercept-(self.pfx)
        distance_ratio = dl/(dl+dr)
        
        #pks11 - fixing distance ratio
        #distance_ratio = 0.5

        area = 0.5*(dr+dl)*(self.h-(self.pfy-8))

        # print('The area of triangle is {}'.format(area))

        # if area<19000 and area>10000:
        
        #     output_data = [distance_ratio*0.76,(1-distance_ratio)*0.76]
        #     output = Float32MultiArray(data=output_data)                
        #     #### Publish numpy topic  ####
        #     self.output_distance.publish(output)
        #     d_ratio_data = Float32MultiArray(data=[distance_ratio]) 
        

        #     if self.verbose:
        #         myloginfo('Distance ratio is {}'.format(distance_ratio))
        
           
        output_data = [distance_ratio*self.row_width,(1-distance_ratio)*self.row_width]
        output = Float32MultiArray(data=output_data)                
        #### Publish numpy topic  ####
        self.output_distance.publish(output)
        if self.verbose:
            self.myloginfo('Distance ratio is {}'.format(distance_ratio))

    def pubHeading(self, res):
        
        heading = math.atan((res[0]-self.pfx)/self.flx)
        #heading = -numpy.rad2deg(heading)
        #heading = -heading
        
        self.heading = heading
        # print("Heading:",np.rad2deg(heading))
        # self.myloginfo('Heading is {} degrees'.format(np.rad2deg(heading)))
        output = Float32MultiArray(data=[heading])
        #### Publish numpy topic  ####
        self.output_heading.publish(output)  
        if self.verbose:
            self.myloginfo('Heading is {} degrees'.format(heading))

         
    def processImage(self, ros_data, msg=''):
        if self.enable_perception == False:
            print("retuenedd false perception")
            return -1  
        if self.pfx == 0 or self.pfy == 0 or self.flx == 0 or self.fly == 0:
            self.mylogerr('calibration not available... skipping inference...')
            return -1
        #cur_time = rospy.get_time()
        #if cur_time - self.last_print_time > 5:
        #    self.myloginfo(rospy.get_name() + ' ' + msg)
        #    self.last_print_time = cur_time

        if self.in_row_method.find('vision') < 0 and self.in_row_method != 'full':
            self.in_row_method = self.get_parameter_or('in_row_method', 'vision')
            return -1        
        start = self.get_clock().now().seconds_nanoseconds()[0]
        '''Here images get converted and deep learning inference done'''
        if self.verbose:
            self.myloginfo('received image of type: "%s"' % ros_data.format)

        # if not isinstance(msg, Image):
        #     self.get_logger().error("Received message is not a sensor_msgs/Image")
        #     return


        # #### direct conversion to CV2 ####
        # np_arr = np.fromstring(ros_data.data, np.uint8)
        # image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) 


        try:
            msg = ros_data
            # print("MSG:::", msg)
            # Convert ROS Image to NumPy array
            np_arr = np.frombuffer(msg.data, np.uint8)
            
            # Reshape the array into the appropriate dimensions
            image = np_arr.reshape(msg.height, msg.width, -1)

            # Convert from RGB to BGR if necessary (OpenCV format)
            if msg.encoding == 'rgb8':
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        
        # Further processing...
        
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")

        #     # Convert the ROS Image message to a NumPy array
        # np_arr = np.frombuffer(ros_data.data, np.uint8)

        # # Reshape the array into the appropriate dimensions
        # # height, width, channels
        # image = np_arr.reshape(ros_data.height, ros_data.width, -1)

        # # If the image is in RGB format, convert it to BGR format for OpenCV
        # if ros_data.encoding == 'rgb8':
        #     image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        self.bridge = CvBridge()
        # image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # self.get_logger().info(f"Processing image:, Type: {type(msg)}")
        # try:
        #     # Convert ROS Image to OpenCV format
        #     image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #     # Further processing...
        # except Exception as e:
        #     self.get_logger().error(f"Could not convert image: {e}")

        # cv2.imshow('Image1',image)
        # cv2.waitKey(1)        
        #if self.rotate_180:
        
        image = image[8:232]
        image = cv2.flip(image,-1)
        image = cv2.rotate(image, cv2.ROTATE_180)
        # cv2.imshow('Image2',image)
        # cv2.waitKey(1)
        
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        #image = cv2.copyMakeBorder(image, 6, 6, 0, 0, cv2.BORDER_CONSTANT, None, value = 0)

        # cv2.destroyAllWindows()
        h,w = image.shape[:2]
        self.h = h
        self.w = w
        
        data = image/255
        data[:,:,0] = (data[:,:,0]-0.485)/0.229
        data[:,:,1] = (data[:,:,1]-0.456)/0.224
        data[:,:,2] = (data[:,:,2]-0.406)/0.225
        data = np.expand_dims(data,axis=0)
        data = np.transpose(data, axes=[0,3,1,2]).astype(np.float32)
        
        # Start sync inference
        # self.myloginfo("Starting inference in synchronous mode")
        #rospy.sleep(0.4)
        pred = self.compiled_model_onnx(inputs=[data])[self.output_layer_onnx][0]


        #pred = self.exec_net.infer(inputs={self.input_blob: data})

        #Processing output blob
        #log.info("Processing output blob")
        #pred = pred[self.out_blob][0][0]
    
        #pred = self.session.run(['keypoint_output'], {"input_image": data})[0][0]

        def softmax(keypoint_map):
            temp_factor = 1
            keypoint_map_exp = np.exp(keypoint_map/temp_factor)
            return keypoint_map_exp/(np.sum(keypoint_map_exp))

        pred[0] = softmax(pred[0])
        pred[1] = softmax(pred[1])
        pred[2] = softmax(pred[2])


        vp_y,vp_x = np.unravel_index(pred[0].argmax(), pred[0].shape)
        ll_y,ll_x = np.unravel_index(pred[1].argmax(), pred[1].shape)
        lr_y, lr_x = np.unravel_index(pred[2].argmax(), pred[2].shape)

        # def _gaussian(xL, yL, sigma, H, W):
        
        #     #Also change origin in column axis to account for image padding in width 

        #     channel = [math.exp(-((c - xL) ** 2 + (r - yL) ** 2) / (2 * sigma ** 2)) for r in range(H) for c in range(W)]
        #     channel = np.array(channel, dtype=np.float32)
        #     channel = np.reshape(channel, newshape=(H, W))

        #     return channel

        # vp_test = _gaussian(vp_x, vp_y, 1, 48, 80)
        # vp_test = softmax(vp_test)
        # vp_test_var = 1/np.sqrt(np.sum(np.square(vp_test-vp_test.max())))

        # ll_test = _gaussian(ll_x, ll_y, 1, 48, 80)
        # ll_test = softmax(ll_test)        
        # ll_test_var = 1/np.sqrt(np.sum(np.square(ll_test-ll_test.max())))
        
        # lr_test = _gaussian(lr_x, lr_y, 1, 48, 80)
        # lr_test = softmax(lr_test)        
        # lr_test_var = 1/np.sqrt(np.sum(np.square(lr_test-lr_test.max())))

        # vp_test = _gaussian(vp_x, vp_y, 1, 48, 80)
        # vp_test = softmax(vp_test)
        # vp_test_var = np.sqrt(np.var(vp_test))

        # ll_test = _gaussian(ll_x, ll_y, 1, 48, 80)
        # ll_test = softmax(ll_test)        
        # ll_test_var = np.sqrt(np.var(ll_test))
        
        # lr_test = _gaussian(lr_x, lr_y, 1, 48, 80)
        # lr_test = softmax(lr_test)        
        # lr_test_var = np.sqrt(np.var(lr_test))

        # print(vp_test_var)
        # print(ll_test_var)
        # print(lr_test_var)



        # vp_sig = 1/np.sqrt(np.sum(np.square(pred[0]-pred[0].max())))
        # ll_sig = 1/np.sqrt(np.sum(np.square(pred[1]-pred[1].max())))
        # lr_sig = 1/np.sqrt(np.sum(np.square(pred[2]-pred[2].max())))

        vp_sig = np.sqrt(np.var(pred[0]))
        ll_sig = np.sqrt(np.var(pred[1]))
        lr_sig = np.sqrt(np.var(pred[2]))

    
        # scale_factor = 30
        # vp_var = vp_var/150
        # ll_var = ll_var/400
        # lr_var = lr_var/400
       

        #check sigma value from dataloader of training network and divide that by 4 since our pred map is scaled down by 4
        #sigma = 40
        sigma = 1.0e-5

        def sigmoid(x):
            sig = 1 / (1 + np.exp(-x))
            return sig

        
        output_vp_conf = sigmoid(3*np.tanh(5*(1-((sigma)/(3*vp_sig)))))
        output_ll_conf = sigmoid(3*np.tanh(5*(1-((sigma)/(3*ll_sig)))))
        output_lr_conf = sigmoid(3*np.tanh(5*(1-((sigma)/(3*lr_sig)))))

   
        # ### Publish numpy topic  ####
        # self.output_vp_conf.publish(Float32MultiArray(data=[output_vp_conf]))  
        # self.output_ll_conf.publish(Float32MultiArray(data=[output_ll_conf]))  
        # self.output_lr_conf.publish(Float32MultiArray(data=[output_lr_conf]))


        # Normalize and resize the pred image
        pred_vis = cv2.resize(self.normalize(pred), image.shape[:2][::-1])

        # Ensure both image and pred_vis are in BGR format (since OpenCV uses BGR for encoding)
        image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)  # Convert image from RGB to BGR
        pred_vis_bgr = cv2.cvtColor(pred_vis, cv2.COLOR_RGB2BGR)  # Convert pred_vis from RGB to BGR

        # Blend the images
        blended_image = 0.3 * image_bgr + 0.7 * pred_vis_bgr * 255  # Blending the images

        # Ensure that blended_image is in the correct range [0, 255] for uint8 images
        blended_image = np.clip(blended_image, 0, 255).astype(np.uint8)

        # Create the Image message
        msg_vpl_img = Image()
        msg_vpl_img.header.stamp = self.get_clock().now().to_msg()  # Set timestamp
        msg_vpl_img.header.frame_id = "camera_frame"  # Optional: Set the frame_id if needed (e.g., "camera_frame")

        # Set the encoding to "bgr8" because we are using BGR image format with 8 bits per channel
        msg_vpl_img.encoding = "bgr8"

        # Set the image dimensions
        msg_vpl_img.height = blended_image.shape[0]  # Image height (rows)
        msg_vpl_img.width = blended_image.shape[1]   # Image width (columns)
        msg_vpl_img.step = blended_image.shape[1] * 3  # The number of bytes per row (3 for BGR)

        # Flatten the image into a 1D array (required by the `data` field of the Image message)
        msg_vpl_img.data = blended_image.ravel().tolist()  # Convert the image into a byte list

        # Publish the image message
        self.output_vpl_img.publish(msg_vpl_img)
        # self.get_logger().info(f"Published image with shape: {blended_image.shape}")

        # # Optionally, display the blended image for debugging
        # cv2.imshow("Blended Image", blended_image)
        # cv2.imshow("pred vis rgb", pred_vis_bgr)
        # cv2.waitKey(1)  # Allows OpenCV to update the window

 
        # image[vp_x,vp_y] = [255,0,0]
        image = cv2.circle(image, (vp_x*4,vp_y*4), 5, (255,0,0), -1)
        image = cv2.circle(image, (int(self.pfx),int(self.pfy)), 2, (255,255,255), -1)
        image = cv2.line(image,(vp_x*4,vp_y*4),(ll_x*4,ll_y*4),(0,255,0),2)
        image = cv2.line(image,(vp_x*4,vp_y*4),(lr_x*4,lr_y*4),(0,0,255),2)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # msg_vpl_argmax_img = CompressedImage()
        # msg_vpl_argmax_img.header.stamp = self.get_clock().now().to_msg()
        # msg_vpl_argmax_img.format = "jpeg"
        # msg_vpl_argmax_img.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        # self.output_vpl_argmax_img.publish(msg_vpl_argmax_img)

        # Create the Image message
        msg_vpl_argmax_img = Image()

        # Set the timestamp to the current time
        msg_vpl_argmax_img.header.stamp = self.get_clock().now().to_msg()

        # Optional: Set the frame_id (e.g., "camera_frame")
        msg_vpl_argmax_img.header.frame_id = "camera_frame"

        # Set the encoding for the image (assuming it's a BGR image)
        msg_vpl_argmax_img.encoding = "bgr8"

        # Set the image dimensions
        msg_vpl_argmax_img.height = image.shape[0]  # Number of rows
        msg_vpl_argmax_img.width = image.shape[1]   # Number of columns
        msg_vpl_argmax_img.step = image.shape[1] * 3  # Number of bytes per row (3 channels for BGR)

        # Flatten the image into a 1D array and set it as the data field
        msg_vpl_argmax_img.data = image.ravel().tolist()  # Flatten the 2D array to 1D

        # Publish the image message
        self.output_vpl_argmax_img.publish(msg_vpl_argmax_img)
        # self.get_logger().info(f"Published image with shape: {image.shape}")



        self.output_vp.publish(Float32MultiArray(data=[vp_x*4,vp_y*4]))
        self.output_ll.publish(Float32MultiArray(data=[ll_x*4,ll_y*4]))
        self.output_lr.publish(Float32MultiArray(data=[lr_x*4,lr_y*4]))

        self.output_vp_conf.publish(Float32MultiArray(data=[output_vp_conf]))  
        self.output_ll_conf.publish(Float32MultiArray(data=[output_ll_conf]))  
        self.output_lr_conf.publish(Float32MultiArray(data=[output_lr_conf]))  

        #pks11 changes : commented the code below and add lines from res... to self.pubdistance
        # if vp_y*4 < 150:

        #     self.image = image            

        #     res = [vp_x,vp_y,ll_x,ll_y,lr_x,lr_y]
        #     res = [x*4 for x in res]

        #     if output_vp_conf>0.5:
        #         self.pubHeading(res)
        #         #self.pubDistance(res)

        #         if output_ll_conf>0.5 and output_lr_conf>0.5:              
        #             self.pubDistance(res)
        
        self.image = image
        #pks11 - fixing the lr_x and lr_y
        #lr_x =  
        
        res = [vp_x,vp_y,ll_x,ll_y,lr_x,lr_y]
        res = [x*4 for x in res]
        # if output_vp_conf>0.3:
            
        self.pubHeading(res)
        
        self.pubDistance(res)
        
        end = self.get_clock().now().seconds_nanoseconds()[0]

        #time.sleep(0.3)
        
        if end-start > 0 and 1/(end-start) < self.latency_threshold_fps_print:
            self.myloginfo("latency is {} seconds".format(end-start))


    def callback(self, ros_data):
        self.myloginfo(self.primary_cam)
        
        if self.primary_cam == 'front':
            self.processImage(ros_data, 'using front cam to find rows')
    def callbackBackCam(self, ros_data):
        if self.primary_cam == 'back':
            self.processImage(ros_data, 'using back cam to find rows')
    def callbackCameraInfo(self, msg):
        # if msg.P[2] != self.pfx:
        #     self.myloginfo('setting intrinsic values from camera_info topic pfx ' + str(msg.P[2])+'\tpfy '+str(msg.P[6])+'\tflx '+str(msg.P[0])+'\tfly '+str(msg.K[5]))
        #     self.myloginfo(self.primary_cam)

        # self.pfx = msg.P[2] # PRINCIPAL FOCOUS ON X-axis WRT TO CAMERA
        # self.pfy = msg.P[6] # PRINCIPAL FOCOUS ON Y-axis WRT TO CAMERA
        # self.flx = msg.P[0] # FOCAL LENGHT ON X-axis WRT TO CAMERA
        # self.fly = msg.P[5] # FOCAL LENGHT ON Y-axis WRT TO CAMERA
        self.pfx = msg.k[2] # PRINCIPAL FOCOUS ON X-axis WRT TO CAMERA
        self.pfy = msg.k[5] # PRINCIPAL FOCOUS ON Y-axis WRT TO CAMERA
        self.flx = msg.k[0] # FOCAL LENGHT ON X-axis WRT TO CAMERA
        self.fly = msg.k[4] # FOCAL LENGHT ON Y-axis WRT TO CAMERA
        
        
    def callbackCam2use(self, data):
        self.primary_cam = data.data
        allowed_cameras = ['front', 'back']
        if self.primary_cam not in allowed_cameras:
            self.mylogerr('Failure to set primary cam. Received ' + str(self.primary_cam))
    def togglePerceptionCb(self, data):
        self.enable_perception = data.data

    def normalize(self,input):
        #input is c,h,w which we change to w,h,c
        input = input.transpose((1,2,0))

        output = input.copy()

        r =  input[:,:,0]
        g =  input[:,:,1]
        b = input[:,:,2]

        r_min = r.min()
        g_min = g.min()
        b_min = b.min()

        r_max = r.max()
        g_max = g.max()
        b_max = b.max()

        output[:,:,0] = (input[:,:,2]-b_min)/(b_max-b_min) 
        output[:,:,1] = (input[:,:,1]-g_min)/(g_max-g_min)
        output[:,:,2] = (input[:,:,0]-r_min)/(r_max-r_min)
        
        output = np.where(output > 0.5, output, np.zeros_like(output))

        return output

def main(args=None):
    rclpy.init(args=args)

    # Instantiate the node
    # machine = platform.machine()
    # if machine != 'x86_64':   
    #     rclpy.logging.get_logger('vision_perception').error('exited because machine is not x86_64')
    #     sys.exit()

    # Loading ONNX model with OpenVINO
    ie = Core()
    current_file_path = os.path.dirname(os.path.abspath(__file__))
    
    dir_path = os.path.abspath(os.path.join(current_file_path, "../../../..","src/neural-network-models/models_in_row/icra_experiments/kp_TB.onnx"))
    print("Currentfilepath:",dir_path)

    model_onnx = ie.read_model(model=dir_path)
    compiled_model_onnx = ie.compile_model(model=model_onnx, device_name="CPU")

    input_layer_onnx = next(iter(compiled_model_onnx.inputs))
    output_layer_onnx = next(iter(compiled_model_onnx.outputs))

    # Start the Perception node
    node = Perception(compiled_model_onnx, input_layer_onnx, output_layer_onnx)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().error("Shutting down ROS2 Image neural network prediction module")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main() or 0)
