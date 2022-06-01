import numpy as np
import cv2
from torch import true_divide
import PathTracking.utils as utils
from PathTracking.controller import Controller
import PathTracking.configs as configs


class ControllerPIDBasic(Controller):
    def __init__(self, kp=0.4, ki=0.0001, kd=0.5):
        self.path = None
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.acc_ep = 0
        self.last_ep = 0
    
    def set_path(self, path):
        super().set_path(path)
        self.acc_ep = 0
        self.last_ep = 0
    
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State
        x, y, dt = info["x"], info["y"], info["dt"]

        # Search Nesrest Target
        min_idx, min_dist = utils.search_nearest(self.path, (x,y))
        target = self.path[min_idx]
        
        # TODO: PID Control for Basic Kinematic Model 
        min_idx, min_dist = utils.search_nearest(self.path, (x, y))
        target = self.path[min_idx]
        ang = np.arctan2(self.path[min_idx, 1]-y, self.path[min_idx, 0]-x)
        ep = min_dist * np.sin(ang)
        self.acc_ep = self.acc_ep + dt * ep
        diff_ep = (ep - self.last_ep) / dt
        next_w = self.kp * ep + self.ki * self.acc_ep + self.kd * diff_ep
        self.last_ep = ep
        return next_w, 


def find_road_center(frame):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    horizon = configs.ROAD_DETECT_HORIZON
    isLeft, isRight = False, False
    pixelLeft, pixelRight = -1, -1
    for i in range(configs.FRAME_WIDTH):
        # Find out the most left road
        if isLeft == False and isRight == False:
            if (frame[horizon, i, :] == [128, 0, 0]).all():
                isLeft = True
                pixelLeft = i
                continue
        # Find out the most right road
        if isLeft == True and isRight == False:
            if (frame[horizon, i, :] == [0, 0, 0]).all() or (frame[horizon, i, :] == [0, 128, 0]).all() or (frame[horizon, i, :] == [128, 128, 0]).all() or (frame[horizon, i, :] == [0, 0, 128]).all():
                isRight = True
                pixelRight = i
                continue


    pixelCenter = (pixelLeft + pixelRight) / 2


    result_frame = np.zeros([256,256,3], dtype=np.float32)
    result_frame[horizon, pixelLeft:pixelRight,:] = [128, 0, 0]
    frame = cv2.cvtColor(result_frame, cv2.COLOR_BGR2RGB)

    cv2.imshow("Road", result_frame)

    return pixelCenter



def abs(number):
    if number < 0:
        return -1 * number
    else:
        return number

def cal_road_distance_offset(frame_center, road_center):

    dis = road_center - frame_center
    
    offset = "CENTER"
    if dis > 0:
        offset = "RIGHT"
    if dis < 0:
        offset = "LEFT"
    
    return (abs(dis), offset)


def find_obstancle(frame):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    horizon = configs.OBSTANCLE_DETECT_HORIZON
    
    isLeft, isRight = False, False
    countLeft, countRight = 0, 0

    result_frame = np.zeros([256,256,3], dtype=np.float32)

    # 從左掃到右，比較占比來確定障礙物在左邊 or 右邊
    for i in range(0, configs.FRAME_CENTER):
        if (frame[horizon, i, :] == [128, 128, 0]).all():
            result_frame[horizon, i, :] = [128, 128, 0]
            countLeft = countLeft + 1

    for i in range(configs.FRAME_CENTER, configs.FRAME_WIDTH):
        if (frame[horizon, i, :] == [128, 128, 0]).all():
            result_frame[horizon, i, :] = [128, 128, 0]
            countRight = countRight + 1

    # isObstancle : 是否有障礙物 / offset : 車車在障礙物的哪一邊
    isObstancle = False
    offset = ""

    if countLeft != 0 or countRight != 0:
        isObstancle = True
        if countLeft > countRight:
            offset = "RIGHT"
        if countLeft < countRight:
            offset = "LEFT"

    frame = cv2.cvtColor(result_frame, cv2.COLOR_BGR2RGB)
    cv2.imshow("Obstancle", frame)

    return (isObstancle, offset)

def find_finish(frame):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    horizon = configs.FINISHLINE_DETECT_HORIZON

    result_frame = np.zeros([256,256,3], dtype=np.float32)

    countFinish = 0
    for i in range(0, configs.FRAME_WIDTH):
        if (frame[horizon, i, :] == [0, 0, 128]).all():
            result_frame[horizon, i, :] = [0, 0, 128]
            countFinish = countFinish + 1

    isFinish = False

    if countFinish > 20:
        print(countFinish)
        isFinish = True

    frame = cv2.cvtColor(result_frame, cv2.COLOR_BGR2RGB)
    cv2.imshow("Finish", frame)

    return isFinish


def find_black_line(frame, offsetObstancle):
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
    horizon = configs.BOUNDARY_DETECT_HORIZON
    vertical = configs.BOUNDARY_DETECT_VERTICAL
    # # 車車在障礙物右邊
    if offsetObstancle == "RIGHT":
        vertical = configs.BOUNDARY_DETECT_VERTICAL_RIGHT
    if offsetObstancle == "LEFT":
        vertical = configs.BOUNDARY_DETECT_VERTICAL_LEFT


    # print("  \n=========== "   + str(vertical))

    # vertical = configs.BOUNDARY_DETECT_VERTICAL

    isBoundary = False
    result_frame = np.zeros([256,256,3], dtype=np.float32)
    if (frame[horizon, vertical, :] == [0, 128, 0]).all():
        isBoundary = True
        print(vertical)
        result_frame[horizon, vertical, :] = [0, 128, 0]

    frame = cv2.cvtColor(result_frame, cv2.COLOR_BGR2RGB)
    cv2.imshow("Boundary", frame)

    return isBoundary


def find_red_line(frame, offsetObstancle):
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
    horizon = configs.ROAD_DETECT_HORIZON
    vertical = configs.ROAD_DETECT_VERTICAL
    # # 車車在障礙物右邊
    if offsetObstancle == "RIGHT":
        vertical = configs.ROAD_DETECT_VERTICAL_LEFT
    if offsetObstancle == "LEFT":
        vertical = configs.ROAD_DETECT_VERTICAL_RIGHT


    isRoad = False
    result_frame = np.zeros([256,256,3], dtype=np.float32)
    if (frame[horizon, vertical, :] == [128, 0, 0]).all():
        isRoad = True
        result_frame[horizon, vertical, :] = [128, 0, 0]

    result_frame[:, vertical, :] = [255, 255, 255]
    result_frame[horizon, :, :] = [255, 255, 255]


    frame = cv2.cvtColor(result_frame, cv2.COLOR_BGR2RGB)
    cv2.imshow("Fix_Stage", frame)

    return isRoad