#!/usr/bin/python3
# -*- coding: utf8 -*- 
"""Joystick Agent."""
import os
os.environ['SDL_AUDIODRIVER'] = 'dummy' # 오디오 드라이버를 더미로 설정
import pygame, sys
import numpy as np
import time
import rospy
from std_msgs.msg import Float64MultiArray

#####################################
# Change these to match your joystick
RIGHT_UP_AXIS = 4
RIGHT_SIDE_AXIS = 3
LEFT_UP_AXIS = 1
LEFT_SIDE_AXIS = 0
#####################################

class InputActor(object):
    """Joystick Controller for Lunar Lander."""

    def __init__(self, fps=100, input_type='joystick'):
        """Init."""
        self.input_type = input_type
        self.human_agent_action = np.zeros(6)
        self.button = np.zeros(1)
        
        if self.input_type == 'joystick':
            pygame.joystick.init()
            joysticks = [pygame.joystick.Joystick(x)
                        for x in range(pygame.joystick.get_count())]
            if len(joysticks) > 1:
                self.joy = joysticks[1]
                self.joy.init()
                # raise ValueError("There must be exactly 1 joystick connected.",
                #                 "Found ", len(joysticks))
            elif len(joysticks) == 0:
                raise ValueError("There is no joystick connected.")
            elif len(joysticks) == 1:   
                self.joy = joysticks[0]
                self.joy.init()
        elif self.input_type == 'keyboard':
            width, height = 640, 480
            screen = pygame.display.set_mode((width, height))
            
        pygame.init()
        self.t = None
        self.fps = fps
    
    def _get_joystick_action(self):
        for event in pygame.event.get():
            # Joystick input
            if event.type == pygame.JOYAXISMOTION:
                if event.axis == LEFT_SIDE_AXIS:
                    self.human_agent_action[1] = event.value
                elif event.axis == LEFT_UP_AXIS:
                    self.human_agent_action[0] = -1.0 * event.value
                if event.axis == RIGHT_SIDE_AXIS:
                    self.human_agent_action[5] = event.value
                elif event.axis == RIGHT_UP_AXIS:
                    self.human_agent_action[2] = -1.0 * event.value
            if event.type == pygame.JOYBUTTONDOWN:
                self.button[0] = event.button
                if self.button[0] == 1:
                    self.human_agent_action[3] = 1
                elif self.button[0] == 2:
                    self.human_agent_action[3] = -1
                if self.button[0] == 0:
                    self.human_agent_action[4] = 1
                elif self.button[0] == 3:
                    self.human_agent_action[4] = -1
            else: # button clear
                self.button[0] = -1
                self.human_agent_action[3] = self.human_agent_action[4] = 0
            
        return self.human_agent_action, self.button
    
    def _get_keyboard_action(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    sys.exit()

        keys = pygame.key.get_pressed()
        if keys[pygame.K_w]:
            self.human_agent_action[0] = 1.0
        elif keys[pygame.K_s]:
            self.human_agent_action[0] = -1.0
        else:
            self.human_agent_action[0] = 0.0
            
        if keys[pygame.K_a]:
            self.human_agent_action[1] = -1.0
        elif keys[pygame.K_d]:
            self.human_agent_action[1] = 1.0
        else:
            self.human_agent_action[1] = 0.0
            
        if keys[pygame.K_e]:
            self.human_agent_action[2] = 1.0
        elif keys[pygame.K_c]:
            self.human_agent_action[2] = -1.0
        else:
            self.human_agent_action[2] = 0.0
            
        if keys[pygame.K_f]:
            self.human_agent_action[3] = 1.0
        elif keys[pygame.K_h]:
            self.human_agent_action[3] = -1.0
        else:
            self.human_agent_action[3] = 0.0
            
        if keys[pygame.K_t]:
            self.human_agent_action[4] = 1.0
        elif keys[pygame.K_g]:
            self.human_agent_action[4] = -1.0
        else:
            self.human_agent_action[4] = 0.0
            
        if keys[pygame.K_r]:
            self.human_agent_action[5] = 1.0
        elif keys[pygame.K_y]:
            self.human_agent_action[5] = -1.0
        else:
            self.human_agent_action[5] = 0.0
            
        if keys[pygame.K_1]:
            self.button[0] = 6.0 # init mode
        elif keys[pygame.K_2]:
            self.button[0] = 7.0 # teleop mode
        else:
            self.button[0] = -1
        
        return self.human_agent_action, self.button  

    def __call__(self):
        """Act."""
        if self.input_type == 'joystick':
            action, button = self._get_joystick_action()
        else:
           action, button = self._get_keyboard_action()  
        if self.t and (time.time() - self.t) < 1. / self.fps:
            st = 1. / self.fps - (time.time() - self.t)
            if st > 0.:
                time.sleep(st)
        self.t = time.time()
        return action, button

    def reset(self):
        self.human_agent_action[:] = 0.
        

if __name__ == '__main__':
    args = rospy.myargv()
    if len(args) > 1: 
        input_type = args[1]
    else:
        input_type = 'joystick'
    fps = 50
    actor = InputActor(fps=fps, input_type=input_type)
    pub = rospy.Publisher(input_type+'_command', Float64MultiArray, queue_size=10)
    rospy.init_node(input_type, anonymous=True)
    rate = rospy.Rate(fps)
    while not rospy.is_shutdown():
        # get input from joystick
        input, button = actor()
        command = Float64MultiArray()
        command.data.append(input[0]) # x
        command.data.append(input[1]) # y
        command.data.append(input[2]) # z
        command.data.append(input[3]) # roll
        command.data.append(input[4]) # pitch
        command.data.append(input[5]) # yaw
        command.data.append(button) # button
        pub.publish(command)

        rate.sleep()