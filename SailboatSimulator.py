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
        self.boat1 = Boat(40,200,200,(100,100,100)) #later include boat list for support of multiple boats
        self.wind = Wind(windspeed,windheading)
        self.clock = pygame.time.Clock()
        self.relwind = abs(self.wind.windheading-self.boat1.heading)%(2.0*pi)
        if self.relwind > pi:
            self.relwind = 2.0*pi-self.relwind
        self.relwindcomp = pi-self.relwind
            
    def update_model(self):
        dt = self.clock.tick()/1000.0
        self.relwind = abs(self.wind.windheading-self.boat1.heading)%(2.0*pi)
        if self.relwind > pi:
            self.relwind = 2.0*pi-self.relwind
        self.relwindcomp = pi-self.relwind
        self.boat1.update(dt,model)
        self.wind.update(dt)
        
class Wind:
    """encodes information about the wind"""
    def __init__(self,windspeed,windheading):
        self.windspeed = windspeed
        self.windheading = windheading        
        
    def update(self,dt):
        """updates the wind behavior based on some pattern, placeholder for later"""
        self.windheading = self.windheading%(2.0*pi) #sanitization
        if self.windspeed <= 0:
            self.windspeed = 0
    
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
        self.wind_over_port = True 
        
    def update(self,dt,model):
        self.heading = self.heading % (2.0*pi) #sanitization
        self.trim(model)
        self.kinematics(dt,model)
    
    def trim(self,model):
        """readjust sails and rudder to suggestions if possible"""
        if self.MainSuggestion <= 0:
            self.MainSuggestion = 0
        if self.MainSuggestion >= 1:
            self.MainSuggestion = 1
        if self.JibSuggestion <= 0:
            self.JibSuggestion = 0
        if self.JibSuggestion >= 1:
            self.JibSuggestion = 1
        self.MainPos = min([self.MainSuggestion,model.relwindcomp*2.0/pi,1]) #in foolish ratio units, [0 1], 1 being full out, 90 degrees
        self.JibPos = min([self.JibSuggestion,model.relwindcomp*2.0/pi,1])
        self.wind_over_port = ((model.wind.windheading-self.heading)%(2.0*pi))<pi #a boolean
    
    def kinematics(self,dt,model):
        """updates kinematics"""
        self.heading += self.angularVelocity*dt
        self.vx = self.forward_speed*cos(self.heading)
        self.vy = self.forward_speed*sin(self.heading)
        self.vx += model.wind.windspeed*cos(model.wind.windheading)
        self.vy += model.wind.windspeed*sin(model.wind.windheading)

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
        self.draw_wind_vane(self.model.wind)
        self.disp_HUD_info()

        pygame.display.update()
    
    def draw_boat(self,boat):
        bow = (boat.xpos+boat.length/2.0*cos(boat.heading),boat.ypos+boat.length/2.0*sin(boat.heading))
        starboard_stern = (boat.xpos+boat.length/2.0*cos(boat.heading+pi*5.0/6),boat.ypos+boat.length/2.0*sin(boat.heading+pi*5.0/6))
        port_stern = (boat.xpos+boat.length/2.0*cos(boat.heading+pi*7.0/6),boat.ypos+boat.length/2.0*sin(boat.heading+pi*7.0/6))
        pygame.draw.line(self.screen, boat.color, bow, starboard_stern, 3)
        pygame.draw.line(self.screen, boat.color, bow, port_stern, 2)
        pygame.draw.line(self.screen, boat.color, starboard_stern, port_stern, 1)
        if boat.wind_over_port:
            switch = -1
        else:
            switch = 1
        main_end = (boat.xpos-boat.length/2.0*cos(boat.heading+boat.MainPos*pi/2.0*switch), boat.ypos-boat.length/2.0*sin(boat.heading+boat.MainPos*pi/2.0*switch))
        pygame.draw.line(self.screen, (0,255,0), (boat.xpos,boat.ypos), main_end, 2)
        
#        pygame.draw.rect(self.screen,pygame.Color(boat.color[0],boat.color[1],boat.color[2]),pygame.Rect(boat.xpos,boat.ypos,boat.length*2,boat.length))

    def draw_wind_vane(self,wind):
        origin = (30,30)
        dest = (30+(4+wind.windspeed)*cos(wind.windheading),30+(4+wind.windspeed)*sin(wind.windheading))
        pygame.draw.line(self.screen, (255,0,0), origin, dest, 2)
        pygame.draw.circle(self.screen, (100,100,100),origin, 3)
        
    def disp_HUD_info(self):
        """displays the relwind next to the wind vane"""
        myfont = pygame.font.SysFont("monospace", 12, bold = True)
        text = myfont.render("Relative Wind: "+str(self.model.relwind), 1, (0,0,0))
        text_2 = myfont.render("Relative Wind Comp: "+str(self.model.relwindcomp), 1, (0,0,0))
        text_3 = myfont.render("Main Pos: "+str(self.model.boat1.MainPos), 1, (0,0,0))
        self.screen.blit(text, (100,20))
        self.screen.blit(text_2, (100, 40))
        self.screen.blit(text_3, (100, 60))
        
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
            #direct control, deprecated
            if event.key == pygame.K_LEFT:
                self.model.boat1.vx += -10      
            if event.key == pygame.K_RIGHT:
                self.model.boat1.vx += 10                          
            if event.key == pygame.K_UP:
                self.model.boat1.vy += -10              
            if event.key == pygame.K_DOWN:
                self.model.boat1.vy += 10  
                
            #ijkl to modify forward and angular velocity
            if event.key == pygame.K_i:
                self.model.boat1.forward_speed += 10
            if event.key == pygame.K_k:
                self.model.boat1.forward_speed += -10  
            if event.key == pygame.K_l:
                self.model.boat1.angularVelocity += 1  
            if event.key == pygame.K_j:
                self.model.boat1.angularVelocity += -1
                
            if event.key == pygame.K_b:
                self.model.boat1.xpos = 200
                self.model.boat1.ypos = 200
                self.model.boat1.vx = 0
                self.model.boat1.vy = 0
                self.model.boat1.heading = 0
                self.model.boat1.angularVelocity = 0
                self.model.boat1.forward_speed = 0
                
            #tg, rf modify wind
            if event.key == pygame.K_t:
                self.model.wind.windspeed += 1
            if event.key == pygame.K_g:
                self.model.wind.windspeed += -1
            if event.key == pygame.K_r:
                self.model.wind.windheading += pi/16.0                
            if event.key == pygame.K_f:
                self.model.wind.windheading += -pi/16.0
                
            #ws, ed, modify main and jib
            if event.key == pygame.K_w:
                self.model.boat1.MainSuggestion += 0.1
            if event.key == pygame.K_s:
                self.model.boat1.MainSuggestion += -0.1
            if event.key == pygame.K_e:
                self.model.boat1.JibSuggestion += 0.1
            if event.key == pygame.K_d:
                self.model.boat1.JibSuggestion += -0.1
                
if __name__ == '__main__':
    pygame.init()
    size = (520,500)
    screen = pygame.display.set_mode(size)
    model = WorldModel(0,0) #initial windspeed, windheading
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
