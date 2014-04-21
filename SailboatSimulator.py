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
        self.boat1 = Boat(25,200,200,(100,100,100)) #later include boat list for support of multiple boats
        self.windspeed = windspeed
        self.windheading = windheading
        self.clock = pygame.time.Clock()

    def update_model(self):
        dt = self.clock.tick()
        self.boat1.update(dt/1000.0)
    
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
        self.forward_speed = 0
        self.color = color #should be a three-tuple
        
    def update(self,dt):
        self.trim()
        self.kinematics(dt)
    
    def trim(self):
        """readjust sails and rudder to suggestions if possible"""
        pass
    
    def kinematics(self,dt):
        """updates kinematics"""
        self.heading += self.angularVelocity*dt
        self.vx = self.forward_speed*cos(self.heading)
        self.vy = self.forward_speed*sin(self.heading)
        self.xpos += self.vx*dt
        self.ypos += self.vy*dt
        
    
class PyGameWindowView:
    """encodes view of simulation"""
    def __init__(self,model,screen):
        self.model = model
        self.screen = screen
    
    def draw(self):
        self.screen.fill(pygame.Color(255,255,255))
        #later include for loop of boats
        self.draw_boat(self.model.boat1)
        pygame.display.update()
    
    def draw_boat(self,boat):
        bow = (boat.xpos+boat.length/2.0*cos(boat.heading),boat.ypos+boat.length/2.0*sin(boat.heading))
        starboard_stern = (boat.xpos+boat.length/2.0*cos(boat.heading+pi*5.0/6),boat.ypos+boat.length/2.0*sin(boat.heading+pi*5.0/6))
        port_stern = (boat.xpos+boat.length/2.0*cos(boat.heading+pi*7.0/6),boat.ypos+boat.length/2.0*sin(boat.heading+pi*7.0/6))
        pygame.draw.line(self.screen, boat.color, bow, starboard_stern, 3)
        pygame.draw.line(self.screen, boat.color, bow, port_stern, 2)
        pygame.draw.line(self.screen, boat.color, starboard_stern, port_stern, 1)

#        pygame.draw.rect(self.screen,pygame.Color(boat.color[0],boat.color[1],boat.color[2]),pygame.Rect(boat.xpos,boat.ypos,boat.length*2,boat.length))

class PyGameController:
    """handles user inputs and communicates with model"""
    def __init__(self,model,view): 
        """initialize the class"""
        self.model = model
        self.view = view
        
    def handle_keystroke_event(self,event): 
        """builds and upgrades towers"""
        if event.type == KEYDOWN:
            #remember that the directions are reversed in view
            if event.key == pygame.K_LEFT:
                self.model.boat1.vx += -10      

            if event.key == pygame.K_RIGHT:
                self.model.boat1.vx += 10          
                
            if event.key == pygame.K_UP:
                self.model.boat1.vy += -10              

            if event.key == pygame.K_DOWN:
                self.model.boat1.vy += 10  
                
            if event.key == pygame.K_w:
                self.model.boat1.forward_speed += 10
                
            if event.key == pygame.K_s:
                self.model.boat1.forward_speed += -10  
                
            if event.key == pygame.K_d:
                self.model.boat1.angularVelocity += 1  
                
            if event.key == pygame.K_a:
                self.model.boat1.angularVelocity += -1
                
            if event.key == pygame.K_b:
                self.model.boat1.xpos = 200
                self.model.boat1.ypos = 200
                self.model.boat1.vx = 0
                self.model.boat1.vy = 0
                self.model.boat1.heading = 0
                self.model.boat1.angularVelocity = 0
                self.model.boat1.forward_speed = 0
                

if __name__ == '__main__':
    pygame.init()
    size = (520,500)
    screen = pygame.display.set_mode(size)
    model = WorldModel(0,0)
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
        time.sleep(.01)
    pygame.quit()
