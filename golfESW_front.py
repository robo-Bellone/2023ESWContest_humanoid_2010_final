import time
import cv2
import numpy as np

from Drive_motors import *
from config import *
from vis_utils import *
from utils import *


golf_lower, golf_upper, flag_lower, flag_upper = get_color_from_dat((1,2))
i = 0
j = 0
print(get_color_from_dat((1,2)))
print(f'{golf_lower}, {golf_upper}, {flag_lower}, {flag_upper}')

neck_05_2sec = Utilization('Rx_neck', 0.2)
neck_02sec = Utilization('D_neck', 0.2)
neck_3sec = Utilization('Turn', 3)
flag_1sec = Utilization('stabil', 1)
walk_1_5sec = Utilization('Walk', 1.0)

flag_dist = 300
is_aligned = False

flag = 0


def golfESW():
    global i
    global j
    global is_aligned
    global flag_dist
    
    camera, shape = set_camera(cam_name, cam_ratio, cam_reso)
    flag = 0
    tag = 0
    tol = 0
    
    target_angle = 90
    move_motor(32) # body 초기화
    time.sleep(1)
    move_motor(init_ALL) #neck joint initialize
    
    #cv2.imshow('ball', ball_channel)
    #cv2.imshow('flag', flag_channel)
    
    time.sleep(1)
    tmp_time = time.time()
    
    while True:
        if neck_05_2sec.is_ready():
            neck_05_2sec.call()
            neck_UD_angle = RX_angle(28)
            neck_RL_angle = RX_angle(29)
        
        #neck_RL_angle, neck_UD_angle = move_motor(-99)
        print(f'{flag} is me, \n{neck_RL_angle} is my neck_rl, {neck_UD_angle} is my neck pitch,\n left turned {tol} times')
        
        frame, ret = grab_frame(camera)
        
        ball_channel, ball_mask = filter_hsv(frame, golf_lower, golf_upper)
        flag_channel, flag_mask = filter_hsv(frame, flag_lower, flag_upper)
        
        
        if flag ==0: # 목 각도 조정
            if masked_channel_perc(ball_mask) * 2000 > 5.0:
                cX, _, _ = weighted_sum_moment(get_contours(ball_mask))
                print((X_size/2) - cX)
                if (X_size/2) - cX < -80: #hype
                    move_motor(neck_R_1) # 목 관절 오른쪽
                    tag = 0

                elif (X_size/2) - cX > 80: #hype
                    move_motor(neck_L_1) # 목 관절 왼
                    #목이 움직일 수 없는 각도라면 몸통 조정
                    if neck_RL_angle <= 5:
                        flag = 1
                        tol = tol + 1
                    tag = 0
                else:
                    tag = tag + 1
                    if tag > 5: #hype
                        flag = 1
                        fig = False
            elif masked_channel_perc(ball_mask) * 100 < 5.0 or fig:
                print(f'{i}, {j}')
                if i == 0 and move_motor(LR_0):
                    time.sleep(0.4)
                    i = i + 1

                elif i == 20 and move_motor(neck_D): #hype
                    i = 0
                    j = j + 1
                    
                elif j == 10 and move_motor(init_UD): #hype
                    i = 0
                    j = 0
                    time.sleep(0.05)
                    move_motor(big_ccw)
                    time.sleep(1)
                elif move_motor(neck_R):
                    i = i + 1
        
        elif flag == 1: # 몸통 각도 조정
            neck_angle_error = target_angle - neck_RL_angle
            if neck_angle_error < -10 : 
                if time.time() - tmp_time > 1 and move_motor(turn_cw):
                    tmp_time = time.time()
                    flag = 0
            elif neck_angle_error > 10:
                if time.time() - tmp_time > 1 and move_motor(turn_ccw):
                    tmp_time = time.time()
                    flag = 0
            else:
                flag = 2    
                fig = True

        elif flag == 2 and fig: # 골프공이 충분히 가까워질 때 까지 접근
            cX, cY, _ = weighted_sum_moment(get_contours(ball_mask))
            print(f'cX: {cX}, cY: {cY}')
            if neck_RL_angle < 65 or neck_RL_angle > 115:
                flag = 1
            
            if neck_02sec.is_ready(): #hype
                if cY > 200:
                    move_motor(neck_D)
                    neck_02sec.call()
            elif abs((X_size / 2) - cX) > 100: #hype
                flag = 0
                tag = 0
                tol = 0
                #move_motor(init_UD)
                #j = 0
                if neck_UD_angle < 40 and neck_UD_angle >= 20:
                    flag = 3
            
            if neck_UD_angle < 20: #hype
                if time.time() - tmp_time > 1 and move_motor(walk_bw):
                    tmp_time = time.time()
                    flag = 1
                
            elif neck_UD_angle < 40 and neck_UD_angle >= 20:
                flag = 3
                step_bw = 0
                step_left = 0
                tmp_time = time.time()
            elif move_motor(walk_fw):
                pass
            
            
        elif flag == 3: # 공과 거리 및 방향 조정
            ret, min_contour = get_largest_moment_contour(get_contours(ball_mask))
            print('tlqkf')
            if ret == False:
                flag = 0
                fig = True
            else:
                ball_min_x, ball_min_y = get_lowest_point(min_contour)
                if ((X_size / 2) - ball_min_x) < -10:
                    move_motor(neck_R_1)
                elif ((X_size / 2) - ball_min_x) > 10:
                    move_motor(neck_L_1)
                
                if ((Y_size / 2) - ball_min_y) < -10:
                    move_motor(neck_D_1)
                elif ((Y_size / 2) - ball_min_x) > 10:
                    move_motor(neck_U_1)
                    
                else:
                    if move_motor(init_UD):
                        time.sleep(1)
                        i = 0
                        j = 0
                        flag = 3.1
                        tag = 0
                fig = True
            
        elif flag == 3.1:
            if masked_channel_perc(flag_mask) * 500 > 5.0:
                fX, fY, _ = get_lowest_and_largest_contour_point(get_contours(flag_mask))
                print((X_size/2) - fX)
                if (X_size/2) - fX < -80: #hype
                    move_motor(neck_R_1) # 목 관절 오른쪽
                    tag = 0
                    

                elif (X_size/2) - fX > 80: #hype
                    move_motor(neck_L_1) # 목 관절 왼
                    #목이 움직일 수 없는 각도라면 몸통 조정
                    if neck_RL_angle <= 5:
                        flag = 3.2
                        tol = tol + 1
                    tag = 0
                elif (Y_size/2) - fY < -50:
                    move_motor(neck_D_1) # 목 관절 down
                    tag = 0
                elif (Y_size/2) - fY > 50:
                    move_motor(neck_U_1) # 목 관절 up
                    tag = 0
                else:
                    tag = tag + 1
                    if tag > 5: #hype
                        flag = 3.2
                        tag = 0
                        fig = False
            elif masked_channel_perc(ball_mask) * 100 < 5.0 or fig:
                print(f'{i}, {j}')
                if i == 0 and move_motor(LR_0):
                    time.sleep(0.4)
                    i = i + 1

                elif i == 20 and move_motor(neck_D): #hype
                    i = 0
                    j = j + 1
                    
                elif j == 10 and move_motor(init_UD): #hype
                    i = 0
                    j = 0
                    time.sleep(0.05)
                    move_motor(big_ccw)
                    time.sleep(1)
                elif move_motor(neck_R):
                    i = i + 1
        
        elif flag == 3.2:
            if neck_RL_angle > 20 : 
                if time.time() - tmp_time > 3 and move_motor(ganggang):
                    tmp_time = time.time()
                    flag = 3.1
            elif neck_RL_angle < 20 and move_motor(LR_0):
                flag = 4
            

        elif flag == 4: #깃발과 정렬
            if not ret:
                continue
            flagX, _, _ = get_lowest_and_largest_contour_point(get_contours(flag_mask))
            if flagX is None:
                continue
                grab_frame(camera)
            if (X_size/2) - flagX < -50 and neck_3sec.is_ready(): #hype
                if time.time() - tmp_time > 1 and move_motor(turn_cw):
                    tmp_time = time.time()
                    neck_3sec.call()
            elif (X_size/2) - flagX > 50 and neck_3sec.is_ready(): #hype
                if time.time() - tmp_time > 1 and move_motor(turn_ccw):
                    tmp_time = time.time()
                    neck_3sec.call()
            elif flag_1sec.is_ready() and masked_channel_perc(flag_mask) *20 > 5:
                tag = tag + 1
                flag_1sec.call()
                if tag > 5:
                    time.sleep(1)
                    move_motor(init_LR)
                    time.sleep(1)
                    move_motor(init_90)
                    time.sleep(2)
                    flag = 5
            elif neck_3sec.is_ready():
                neck_3sec.call()
                move_motor(turn_cw)
        elif flag == 5:
            cX, _, _ = weighted_sum_moment(get_contours(ball_mask))
            if (X_size/2) - cX < -10 and walk_1_5sec.is_ready(): #hype
                move_motor(walk_right)
                walk_1_5sec.call()
                tag = 0
            elif (X_size/2) - cX > 10 and walk_1_5sec.is_ready(): #hype
                move_motor(walk_left)
                walk_1_5sec.call()
                tag = 0
            elif walk_1_5sec.is_ready():
                walk_1_5sec.call()
                tag = tag + 1
                if tag > 2:
                    flag = 6
                    tag = 0
        elif flag == 6:
            _, cY, _ = weighted_sum_moment(get_contours(ball_mask))
            if (Y_size/3) - cY < -30 and walk_1_5sec.is_ready(): #hype
                move_motor(walk_bw)
                walk_1_5sec.call()
                tag = 0
                tmp_time = time.time()
            elif (Y_size/3) - cY > 30 and walk_1_5sec.is_ready(): #hype
                move_motor(walk_fw)
                walk_1_5sec.call()
                tag = 0
                tmp_time = time.time()
            elif walk_1_5sec.is_ready() and is_aligned:
                tag = tag + 1
                walk_1_5sec.call()
                move_motor(init_UD)
                time.sleep(1.5)
                move_motor(LR_0)
                time.sleep(1.5)
                flag = 4
                if tag > 0:
                    flag = 7
            elif masked_channel_perc(ball_mask) * 100 < 5 and walk_1_5sec.is_ready():
                move_motor(walk_fw)
                walk_1_5sec.call()
                tmp_time = time.time()
                tag = 0
            elif walk_1_5sec.is_ready() and not is_aligned:
                if move_motor(init_UD):
                    time.sleep(0.5)
                    neck_align_ang = np.arcsin(14.5/flag_dist)
                    print(neck_align_ang)
                    neck_align_ang = np.rad2deg(neck_align_ang)
                    move_RL_angle(neck_align_ang)
                    is_aligned = True
                    flag = 4
                
            
        elif flag == 7:
            time.sleep(1)
            move_motor(swing)
            time.sleep(1)
            tag = 0
            valid = 0
            flag = 7.1
        elif flag == 7.1:
            if masked_channel_perc(ball_mask) * 500 > 5.0:
                cX, _, _ = weighted_sum_moment(get_contours(ball_mask))
                print((X_size/2) - cX)
                if (X_size/2) - cX < -80: #hype
                    move_motor(neck_R_1) # 목 관절 오른쪽
                    tag = 0
                    

                elif (X_size/2) - cX > 80: #hype
                    move_motor(neck_L_1) # 목 관절 왼
                    #목이 움직일 수 없는 각도라면 몸통 조정
                    if neck_RL_angle <= 5:
                        flag = 7.2
                        tol = tol + 1
                    tag = 0
                else:
                    tag = tag + 1
                    if tag > 5: #hype
                        flag = 7.2
                        tag = 0
                        fig = False
            elif masked_channel_perc(ball_mask) * 100 < 5.0 or fig:
                print(f'{i}, {j}')
                if i == 0 and move_motor(LR_0):
                    time.sleep(0.4)
                    i = i + 1

                elif i == 20 and move_motor(neck_D): #hype
                    i = 0
                    j = j + 1
                    
                elif j == 10 and move_motor(init_UD): #hype
                    i = 0
                    j = 0
                    time.sleep(0.05)
                    move_motor(big_ccw)
                    time.sleep(1)
                elif move_motor(neck_R):
                    i = i + 1
        elif flag == 7.2:
            ball_min_x, ball_min_y = get_lowest_point(min_contour)
            if ((X_size / 2) - ball_min_x) < -10:
                move_motor(neck_R_1)
            elif ((X_size / 2) - ball_min_x) > 10:
                move_motor(neck_L_1)
            
            elif ((Y_size / 2) - ball_min_y) < -10:
                move_motor(neck_D_1)
            elif ((Y_size / 2) - ball_min_x) > 10:
                move_motor(neck_U_1)
                
            else:
                flag = 8
                tag = 50
        elif flag == 8:
            try:
                fX, fY, _ = get_lowest_and_largest_contour_point(get_contours(flag_mask))
                cX, cY, _ = weighted_sum_moment(get_contours(ball_mask))
                print(f'tag: {tag}')
                if abs(fX-cX) < 100:
                    if cY - fY < 0:
                        tag = tag + 1
                else:
                    tag =  tag - 1
                if tag < 0:
                    flag = 0
                elif tag > 100:
                    if valid > 1 :
                        flag = 9
                        time.sleep(1)
                        move_motor(ceremony)
                    elif move_motor(walk_bw):
                        valid += 1
                        flag = 7.1
            except:
                flag = 7.1
        elif flag == 9:
            print('Finished')
            
        cv2.imshow('tlqkf', frame)
        cv2.imshow('ball', ball_mask)
        cv2.imshow('flag', flag_mask)
        
        if cv2.waitKey(1) != -1:
            break
