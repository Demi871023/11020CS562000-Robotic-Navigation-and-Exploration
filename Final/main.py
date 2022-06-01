from jetbotSim import Robot, Camera
import numpy as np
import cv2
import segmentation.interface as interface
import PathTracking.pcontrol as pcontrol


FRAME_CENTER = 128


isFix = False
frameFix = 0
frames = 0
def execute(change):
    global robot, frames, isFix, frameFix
    print("\rFrames", frames, end="")
    frames += 1



    # Control Example
    # if frames == 1:
    #     robot.forward(0.2)
    # if frames == 80:
    #     robot.left(0.05)
    # if frames == 88:
    #     robot.stop()
    # if frames == 90:
    #     robot.set_motor(0.2,0.2)
    # if frames == 200:
    #     robot.set_left_motor(0)
    # if frames == 201:
    #     robot.set_right_motor(0)
    # if frames == 202:
    #     robot.right(0.05)
    # if frames == 210:
    #     robot.backward(0.2)
    # if frames == 320:
    #     robot.add_motor(0,0.02)
    # if frames == 400:
    #     robot.reset()

    
    # (1) get new frame via websocket
    img = cv2.resize(change["new"],(256,256))
    # (2) segmentation
    seg_img = interface.segmentation(img)


    isFinish = pcontrol.find_finish(seg_img)

    if isFinish:
        robot.set_motor(0.0,0.0)
        robot.stop()
    else:
    #     # (3) find road center via segmentation result
        pixel_road_center = pcontrol.find_road_center(seg_img)


    #     # check if have obstancle
        isObstancle, offsetObstancle = pcontrol.find_obstancle(seg_img)
    #     # get dis between frame center and road center, offset : 車車在紅色中間的哪邊
        dis, offset = pcontrol.cal_road_distance_offset(FRAME_CENTER, pixel_road_center)


        # 水平線上有 obstancle
        if isObstancle:
            # 發現障礙物之後要修正
            isFix = True
            # 車車在障礙物右邊
            if offsetObstancle == "RIGHT":
                robot.right(0.005)
            if offsetObstancle == "LEFT":
                robot.left(0.005)

        else:
            if isFix:
                robot.forward(0.03)
                if offsetObstancle == "RIGHT":
                    robot.left(0.005)
                if offsetObstancle == "LEFT":
                    robot.right(0.005)
                robot.forward(0.01)
                frameFix = frameFix + 1
                if frameFix == 10:
                    isFix = False
                    frameFix = 0
            else:
                if dis < 3:
                    robot.forward(0.1)
                else:
                    if offset == "RIGHT":
                        robot.right(0.005)
                    if offset == "LEFT":
                        robot.left(0.005)


    cv2.imshow("camera", seg_img)


# declare robot and camera
robot = Robot()
camera = Camera()


camera.observe(execute)


