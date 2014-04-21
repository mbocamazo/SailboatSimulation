# -*- coding: utf-8 -*-
"""
Created on Mon Apr 21 00:43:25 2014

@author: atproofer
"""

import pygame
from pygame.locals import *
from random import *
import math
from math import atan2, degrees, pi, sin, cos, radians
import time
import numpy as np

class WorldModel:
    """encodes simulator world state"""
    def __init__(self,windspeed,windheading):
        self.boat1 = Boat(10,200,200) #later include boat list for support of multiple boats
        self.windspeed = windspeed
        self.windheading = windheading
    
    def update_model(self):
        self.boat1.update(1) #later include clock module to determine dt
    
class Boat:
    """encodes information about the boat"""
    def __init__(self,length,xpos,ypos,color):
        self.length = length
        self.xpos = xpos
        self.ypos = ypos
        self.MainPos = 0
        self.JibPos = 0
        self.MainSuggestion = 0
        self.JibSuggestion = 0
        self.RudderPos = 0
        self.RudderSuggestion = 0
        self.vx = 0
        self.vy = 0
        self.heading = 0 #note: not redundant with vx, vy; in simple model could be
        self.angularVelocity = 0
        self.color = color #should be a three-tuple
        
    def update(self,dt):
        self.trim()
        self.calculate_physics()
        self.move(dt)
    
    def move(self,dt):
        self.xpos += self.vx*dt
        self.ypos += self.vy*dt
        self.heading += self.angularVelocity*dt
        
    def calculate_physics(self):
        """updates kinematics"""
        pass
    
    def trim(self):
        """readjust sails and rudder to suggestions if possible"""
        pass
    
class PyGameWindowView:
    """encodes view of simulation"""
    def __init__(self,model,screen):
        self.model = model
        self.screen = screen
    
    def draw(self):
        self.screen.fill(pygame.Color(255,255,255))
        
        pygame.display.update()
    
    def draw_boat(self,boat):
        pygame.draw.rect(self.screen,pygame.Color())

class PyGameController:
    """handles user inputs and communicates with model"""
    def __init__(self,model,view): 
        """initialize the class"""
        self.model = model
        self.view = view
        
    def handle_keystroke_event(self,event): 
        """builds and upgrades towers"""
        if event.type == KEYDOWN:
            if event.key == pygame.K_r:
                self.model.gold -= 10                

            if event.key == pygame.K_f:
                self.model.gold -= 5                

if __name__ == '__main__':
    pygame.init()
    size = (520,500)
    screen = pygame.display.set_mode(size)
    model = WorldModel()
    view = PyGameWindowView(model,screen)
    controller = PyGameController(model,view)
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.mouse.set_cursor(*pygame.cursors.arrow)
                running = False
            controller.handle_keystroke_event(event)
        model.update_model()
        view.draw()
        time.sleep(.001)
    pygame.quit()
