#!/usr/bin/python3

import rospy

import argparse

import torch
import numpy as np
from cv_bridge import CvBridge


from models.common import DetectMultiBackend

from utils.general import (check_requirements, cv2,
                           non_max_suppression, print_args)

from utils.torch_utils import select_device, smart_inference_mode
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String


class signNODE():
    def __init__(self):

        self.bridge = CvBridge()

        rospy.init_node('signNode', anonymous=True)
        self.signSub = rospy.Subscriber("/automobile/image_raw", Image, self.callback)
        self.signPubWindow = rospy.Publisher("/yolo/window", String, queue_size=5)
        self.signPubLoc = rospy.Publisher("/yolo/loc", String, queue_size=5)
        # doraditi kada dodje do decision node-a
        # self.signPubTrigger = rospy.Publisher("/yolo/trigger", String, queue_size = 10) 

        # opt = self.parse_opt()
        # check_requirements(exclude=('tensorboard', 'thop'))
        # # self.yoloRun(sl, **vars(opt))

        # self.weights = 'best.onnx'
        # self.dnn = True
        # self.data = 'data/coco128.yaml'
        # self.half = True
        # self.device = select_device('cpu')
        # print('dosao')
        # self.model = DetectMultiBackend(weights = self.weights, device=self.device, dnn=self.dnn, data=self.data, fp16=self.half)
        # self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
        

    def run(self):
        rospy.loginfo("Starting signDetection node")
        # opt = self.parse_opt()
        # check_requirements(exclude=('tensorboard', 'thop'))
        # self.yoloRun(**vars(opt))
        rospy.spin()

    # only 4 epochs are done, so the aspect ratio is a lil prob
    def region_of_interest(self,img_data):
        height = img_data.shape[0]
        width = img_data.shape[1]
        mask = np.zeros_like(img_data)
        triangle = np.array([[
        (540,height),
        (200, int(height * 5/8)),
        (width, int(height * 5/8)),
        (width, height)]], np.int32)
        cv2.fillPoly(mask, triangle, 255)
        masked_image = cv2.bitwise_and(img_data, mask)
        return masked_image

    def parse_opt(self):
        parser = argparse.ArgumentParser()
        parser.add_argument('--weights', nargs='+', type=str, default='best.onnx', help='model path or triton URL')
        # promenjen model
        # img.jpg
        parser.add_argument('--source', type=str, default='img.jpg', help='file/dir/URL/glob/screen/0(webcam)')
        # promenjen source na kameru
        parser.add_argument('--data', type=str, default='data/coco128.yaml', help='(optional) dataset.yaml path')
        parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
        parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
        parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
        parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
        parser.add_argument('--device', default='cpu', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
        parser.add_argument('--view-img', action='store_true', help='show results')
        parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
        parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
        parser.add_argument('--save-crop', action='store_true', help='save cropped prediction boxes')
        parser.add_argument('--nosave', action='store_false', help='do not save images/videos')
        parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
        parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
        parser.add_argument('--augment', action='store_true', help='augmented inference')
        parser.add_argument('--visualize', action='store_true', help='visualize features')
        parser.add_argument('--update', action='store_true', help='update all models')
        parser.add_argument('--project', default='runs/detect', help='save results to project/name')
        parser.add_argument('--name', default='exp', help='save results to project/name')
        parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
        parser.add_argument('--line-thickness', default=3, type=int, help='bounding box thickness (pixels)')
        parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
        parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
        parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
        parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
        parser.add_argument('--vid-stride', type=int, default=1, help='video frame-rate stride')

        opt = parser.parse_args()


        return opt

    def callback(self, data):

        sl = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # imageResized = cv2.resize(sl, (640, 640))

        opt = self.parse_opt()
        check_requirements(exclude=('tensorboard', 'thop'))
        self.yoloRun(sl,**vars(opt))
        # **vars(opt)
        # cv2.imshow("yoloV5", sl)
        


    #yoloRun
    #izbrisan je imageFromCam kao drugi argument radi testiranja - > imageFromCam < -
    def yoloRun(
        self,
        img_data,
        weights='yolov5s.pt',  # model path or triton URL
        source='0',  # file/dir/URL/glob/screen/0(webcam) ########### data/images bilo ,promena2
        data='data/coco128.yaml',  # dataset.yaml path
        imgsz=(640, 640),  # inference size (height, width)
        conf_thres=0.25,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image
        device='cpu',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        view_img=False,  # show results
        save_txt=False,  # save results to *.txt
        save_conf=False,  # save confidences in --save-txt labels
        save_crop=False,  # save cropped prediction boxes
        nosave=True,  # do not save images/videos ######### promena1
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        augment=False,  # augmented inference
        visualize=False,  # visualize features
        update=False,  # update all models
        project='runs/detect',  # save results to project/name
        name='exp',  # save results to project/name
        exist_ok=False,  # existing project/name ok, do not increment
        line_thickness=3,  # bounding box thickness (pixels)
        hide_labels=False,  # hide labels
        hide_conf=False,  # hide confidences
        half=False,  # use FP16 half-precision inference
        dnn=False,  # use OpenCV DNN for ONNX inference
        vid_stride=1):
       
        # print('tezine')
        # print(type(weights))
        # print(weights)
    
        # print('data')
        # print(type(data))
        # print(data)



        # Load model
        device = select_device(device)
        model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
        stride, names, pt = model.stride, model.names, model.pt

        # Run inference

        seen, windows = 0, []

        # img = cv2.imread(img_data)
        im = self.region_of_interest(img_data)
        im = cv2.dnn.blobFromImage(img_data, 1 / 255, (640, 640), [0, 0, 0], 1, crop=False)

        im = torch.from_numpy(im).to(model.device)

        pred = model(im, augment=augment, visualize=visualize)

        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

        for i, det in enumerate(pred):  # per image

            seen += 1

            if len(pred):

                signs = [False, False, False, False, False, False,
                         False, False, False, False, False, False, False]
                
                # Print results
                s = " "
                
                for c in det[:, 5].unique():
                    if c == 12:
                        signs[12] = True
                    if c == 1:
                        signs[1] = True
                    if c == 6:
                        signs[6] = True
                    if c == 8:
                        signs[8] = True
                    if c == 11:
                        signs[11] = True
                
                    n = (det[:, 5] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # if signs[12]: print("traffic light")

            
                boundingBoxTL = []
                boundingBoxCROSS = []
                boundingBoxPARK = []
                boundingBoxPRIO = []
                boundingBoxSTOP = []
                if signs[12] or signs[1] or signs[6] or signs[8] or signs[11]:
                    for box in pred[0]:
                        if box.numpy()[5] == 12:
                            boundingBoxTL = box.numpy()[0:4]
                            break
                        if box.numpy()[5] == 1:
                            boundingBoxCROSS = box.numpy()[0:4]
                            break
                        if box.numpy()[5] == 6:
                            boundingBoxPARK = box.numpy()[0:4]
                            break
                        if box.numpy()[5] == 8:
                            boundingBoxPRIO = box.numpy()[0:4]
                            break
                        if box.numpy()[5] == 11:
                            boundingBoxSTOP = box.numpy()[0:4]
                            break

                ###################################
                # string output
                print(s)
                print(boundingBoxTL)
                ###################################
                # prikaz slike
                pom = str(boundingBoxTL)
                # print(float(pom[1:len(pom)-1].split("     ")[3]))
                self.signPubWindow.publish(pom[1:len(pom)-1])
                lis = []
                boundingBoxCROSS = str(boundingBoxCROSS)
                boundingBoxCROSS = boundingBoxCROSS[1:len(boundingBoxCROSS)-1].split("     ")
                boundingBoxPARK = str(boundingBoxPARK)
                boundingBoxPARK = boundingBoxPARK[1:len(boundingBoxPARK)-1].split("     ")
                boundingBoxPRIO = str(boundingBoxPRIO)
                boundingBoxPRIO = boundingBoxPRIO[1:len(boundingBoxPRIO)-1].split("     ")
                boundingBoxSTOP = str(boundingBoxSTOP)
                boundingBoxSTOP = boundingBoxSTOP[1:len(boundingBoxSTOP)-1].split("     ")
                
                boundingBoxCROSS = [float(i) for i in boundingBoxCROSS if i!='' ]
                boundingBoxPARK = [float(i) for i in boundingBoxPARK if i!='' ]
                boundingBoxPRIO = [float(i) for i in boundingBoxPRIO if i!='' ]
                boundingBoxSTOP = [float(i) for i in boundingBoxSTOP if i!='' ]
                
                print(boundingBoxPARK, len(boundingBoxPARK))
                if len(boundingBoxCROSS)!=0:
                    lis.append((boundingBoxCROSS[0]+boundingBoxCROSS[2])/2)
                    lis.append((boundingBoxCROSS[1]+boundingBoxCROSS[3])/2)
                else:
                    lis.append(-1)
                    lis.append(-1)
                
                if len(boundingBoxPARK)!=0:
                    lis.append((boundingBoxPARK[0]+boundingBoxPARK[2])/2)
                    lis.append((boundingBoxPARK[1]+boundingBoxPARK[3])/2)
                else:
                    lis.append(-1)
                    lis.append(-1)

                if len(boundingBoxPRIO)!=0:
                    lis.append((boundingBoxPRIO[0]+boundingBoxPRIO[2])/2)
                    lis.append((boundingBoxPRIO[1]+boundingBoxPRIO[3])/2)
                else:
                    lis.append(-1)
                    lis.append(-1)

                if len(boundingBoxSTOP)!=0:
                    lis.append((boundingBoxSTOP[0]+boundingBoxSTOP[2])/2)
                    lis.append((boundingBoxSTOP[1]+boundingBoxSTOP[3])/2)
                else:
                    lis.append(-1)
                    lis.append(-1)
                
                
                
                # boxes = str(boundingBoxCROSS) + "|" + str(boundingBoxPARK) + "|" \
                # + str(boundingBoxPRIO) + "|" + str(boundingBoxSTOP)
                pom = str(lis)
                # print(float(pom[1:len(pom)-1].split("     ")[3]))
                self.signPubLoc.publish(pom[1:len(pom)-1])
                ###################################




if __name__ == "__main__":
    signDet = signNODE()
    signDet.run()