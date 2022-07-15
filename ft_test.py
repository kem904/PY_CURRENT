import cv2
import mediapipe as mp
from mathfuction import *
import Armserial
import Ranging

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

cap = cv2.VideoCapture(0)
width = cap.get(3)  # 获取摄像头画面像素宽度

with mp_hands.Hands(
        model_complexity=0,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as hands:
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            # If loading a video, use 'break' instead of 'continue'.
            continue

        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image)

        # Draw the hand annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                print("食指根坐标：")
                print(hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP])
                a = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].x
                b = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].y
                c = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].z
                Handpoint1 = HandList(a, b, c)
                print("小拇指根坐标：")
                print(hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP])
                a = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].x
                b = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].y
                c = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].z
                Handpoint2 = HandList(a, b, c)
                print("手腕坐标：")
                print(hand_landmarks.landmark[mp_hands.HandLandmark.WRIST])
                a = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x
                b = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].y
                c = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].z

                Handpoint3 = HandList(a, b, c)
                Handpoint1 = coordinateTrans(Handpoint1)
                Handpoint2 = coordinateTrans(Handpoint2)
                Handpoint3 = coordinateTrans(Handpoint3)
                temp = [Handpoint1, Handpoint2, Handpoint3]
                palm_incenter = trangleIncenterCalculate(temp)
                palm_incenter[2] = palm_incenter[2] * -1


                # 受单目测距启发
                # 计算食指根到小指根的像素距离 因为这个长度是近大远小的 可以用来表征空间上手部距离摄像头的位置
                hands_reletive_dis = distance_two_points(hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].x,
                                                        hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].y,
                                                        hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].x,
                                                        hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP].y,)
                reletive_dis=hands_reletive_dis*width
                reletive_z=reletive_dis/300.0  #这个比例暂定为像素长度/300 应该将
                print(abs(reletive_z),palm_incenter[0],palm_incenter[1])
                buffer = inverseKinematics(abs(reletive_z), palm_incenter[0], palm_incenter[1],
                                           180)  # 机械臂逆运动学求解 这里进行了坐标的变换
                if buffer[0] == True:  # 捕捉x=机械臂y 捕捉y等于机械臂z 捕捉z等于机械臂x
                    Armserial.wirte_angle(buffer[1], buffer[2], buffer[3], buffer[4])  # 发送舵机控制指令
                else:
                    Armserial.angle_reset()
                mp_drawing.draw_landmarks(
                    image,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style())
        # Flip the image horizontally for a selfie-view display.
        cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))
        if cv2.waitKey(5) & 0xFF == 27:
            break
cap.release()
