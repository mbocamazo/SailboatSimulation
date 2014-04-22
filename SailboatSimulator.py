# -*- coding: utf-8 -*-
"""
Created on Mon Apr 21 00:43:25 2014

@author: atproofer
"""

import pygame
#from pygame.locals import *
#from random import *
import math
from math import atan2, degrees, pi, sin, cos, radians
import time
#import numpy as np

class WorldModel:
    """encodes simulator world state"""
    def __init__(self,windspeed,windheading):
        self.boat1 = Boat(40,400,400,(100,100,100)) #later include boat list for support of multiple boats
        self.wind = Wind(windspeed,windheading)
        self.clock = pygame.time.Clock()
        self.relwind = abs(self.wind.windheading-self.boat1.heading)%(2.0*pi)
        if self.relwind > pi:
            self.relwind = 2.0*pi-self.relwind
        self.relwindcomp = pi-self.relwind #the complement
            
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
        self.log_coefficient = 0.1
        self.lambda_1 = 0.1 #can't be named lambda, reserved
        self.lambda_2 = 0.4 #decay rate for angular velocity, to zero, way of encoding drag
        self.strength_Main = 0.65
        self.strength_Jib = 1-self.strength_Main
        self.debug_list = (0,0)
        self.main_angle = 0
        self.jib_angle = 0
        
        self.k = 2 #velocity scaling for the wind
        self.kw = 0.3 #angular velocity scaling for the torque from the rudder
        self.q = 1 #ang vel scaling for torque from the scales
        
        self.disp_k =1 #scaling for the output
        
    def update(self,dt,model):
        self.heading = self.heading % (2.0*pi) #sanitization
        self.trim(model)
        self.kinematics(dt,model)
        if self.xpos > 820:
            self.xpos = 0
        if self.xpos < 0:
            self.xpos = 820
        if self.ypos > 800:
            self.ypos = 0
        if self.ypos < 0:
            self.ypos = 800
    
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
        if self.RudderSuggestion >=1:
            self.RudderSuggestion = 1
        elif self.RudderSuggestion <= -1:
            self.RudderSuggestion = -1
        self.RudderPos = self.RudderSuggestion
        self.MainPos = min([self.MainSuggestion,model.relwindcomp*2.0/pi,1]) #in foolish ratio units, [0 1], 1 being full out, 90 degrees
        self.JibPos = min([self.JibSuggestion,model.relwindcomp*2.0/pi,1])
        self.wind_over_port = ((model.wind.windheading-self.heading)%(2.0*pi))<pi #a boolean
    
    def kinematics(self,dt,model):
        """updates kinematics"""
        #calc the forward speed
        direction = atan2(self.vy,self.vx)
        speed = norm([self.vx,self.vy])
        self.forward_speed = cos(direction-self.heading)*speed #as it accelerates, log aspect diminishes
        
        #rudder torque aspect
        Tr = -self.kw*log(abs(self.forward_speed)+1)*self.RudderPos #all of these ratios are made up 
        
        #sail torque aspect
        Ts = -self.q*self.strength_Main*sin(self.main_angle-model.wind.windheading)*sqrt(model.wind.windspeed)
        Ts = self.q*self.strength_Jib*sin(self.jib_angle-model.wind.windheading)*sqrt(model.wind.windspeed)

        #log torque aspect?
        
        angular_drag = -sign(self.angularVelocity)*self.angularVelocity**2*self.lambda_2 #effectively drag
        self.angularVelocity += angular_drag*dt
        self.angularVelocity += Tr*dt
        self.angularVelocity += Ts*dt
        
        
        #forward sail aspect
        Sr = shadow_ratio(model.relwindcomp)
        PrMain = power_ratio(self.MainPos,model.relwindcomp)*self.strength_Main
        PrJib = power_ratio(self.JibPos,model.relwindcomp)*self.strength_Jib
        Dr = drag_ratio(self.RudderPos)
        Vmax = Vtmax(model.relwindcomp,self.k)*(PrMain+(1-Sr)*PrJib)/(2.0-Sr)*Dr
        self.forward_speed += self.lambda_1*(Vmax-self.forward_speed)*dt #decay to the max, acceleration term
        
        self.debug_list = (Ts, angular_drag, Vtmax(model.relwindcomp,self.k), Vmax)

        #conversion to cartesian
        self.vx = self.forward_speed*cos(self.heading)
        self.vy = self.forward_speed*sin(self.heading)
        
#        floating log aspect (in cartesian, rather than doing vector addition, which is also possible)    
        self.vx += model.wind.windspeed*cos(model.wind.windheading)*self.log_coefficient
        self.vy += model.wind.windspeed*sin(model.wind.windheading)*self.log_coefficient
        
        #finally, updates
        self.heading += self.angularVelocity*dt
        self.xpos += self.vx*dt*self.disp_k
        self.ypos += self.vy*dt*self.disp_k
        
def Vtmax(theta,k):
    """theoretical max for a relative wind angle.  Doesn't belong to the boat class, but could!"""
    a = -0.2
    b = 0.8
    c = -0.2
    Vtmax = a*theta+b*theta**2+c*theta**3
    return Vtmax*k
    
def power_ratio(SailP,relwindcomp):
    """power given sail position, assuming relwindcomp/pi is best"""
    return sin(pi**2.0/(4*relwindcomp)*SailP)  
    
def shadow_ratio(relwindcomp):
    """shadow that falls on the jib given the wind"""
    if relwindcomp >= pi/2.0:
        return 0
    else:
        return (2.0/pi*relwindcomp)-1
        
def drag_ratio(RudderPos):
    """coefficient on velocity from the drag given the rudder pos"""
#    return 1 - 0.9*sin(abs(RudderPos)) #given current state, this means that at pi/4, effectively 1/4 as fast (1-0.9*sin(1))
    return 1 - 0.9*sin(abs(RudderPos*pi/2.0))
    
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
#        try:
        bow = (boat.xpos+boat.length/2.0*cos(boat.heading),boat.ypos+boat.length/2.0*sin(boat.heading))
        starboard_stern = (boat.xpos+boat.length/2.0*cos(boat.heading+pi*5.0/6),boat.ypos+boat.length/2.0*sin(boat.heading+pi*5.0/6))
        port_stern = (boat.xpos+boat.length/2.0*cos(boat.heading+pi*7.0/6),boat.ypos+boat.length/2.0*sin(boat.heading+pi*7.0/6))
        pygame.draw.line(self.screen, boat.color, bow, starboard_stern, 3)
        pygame.draw.line(self.screen, boat.color, bow, port_stern, 2)
        pygame.draw.line(self.screen, boat.color, starboard_stern, port_stern, 1)
#        except:
#            print boat.heading
#            print boat.angularVelocity
        if boat.wind_over_port:  #non-intuitive reversal needs to be explained
            switch = -1
        else:
            switch = 1
        main_end = (boat.xpos-boat.length/2.0*cos(boat.heading+boat.MainPos*pi/2.0*switch), boat.ypos-boat.length/2.0*sin(boat.heading+boat.MainPos*pi/2.0*switch))
        pygame.draw.line(self.screen, (0,255,0), (boat.xpos,boat.ypos), main_end, 2)
        jib_end = (bow[0]-boat.length/3.0*cos(boat.heading+boat.JibPos*pi/2.0*switch),bow[1]-boat.length/3.0*sin(boat.heading+boat.JibPos*pi/2.0*switch))
        pygame.draw.line(self.screen, (0,255,0), bow, jib_end, 2)
        rudder_origin = (mean((starboard_stern[0],port_stern[0])),mean((starboard_stern[1],port_stern[1])))
        rudder_end = (rudder_origin[0]-boat.length/3.0*cos(boat.heading+boat.RudderPos*pi/4.0),rudder_origin[1]-boat.length/3.0*sin(boat.heading+boat.RudderPos*pi/4.0))
        pygame.draw.line(self.screen, (0,0,0), rudder_origin, rudder_end, 3)
        boat.main_angle = atan2(main_end[1]-boat.ypos,main_end[0]-boat.xpos)
        boat.jib_angle = atan2(jib_end[1]-bow[1],jib_end[0]-bow[0])

    def draw_wind_vane(self,wind):
        origin = (30,30)
        dest = (30+(4+wind.windspeed)*cos(wind.windheading),30+(4+wind.windspeed)*sin(wind.windheading))
        pygame.draw.line(self.screen, (255,0,0), origin, dest, 2)
        pygame.draw.circle(self.screen, (100,100,100),origin, 3)
        
    def disp_HUD_info(self):
        """displays the relwind next to the wind vane"""
        myfont = pygame.font.SysFont("monospace", 12, bold = True)
        text = myfont.render("Relative Wind: "+str(self.model.relwind), 1, (0,0,0))
        text_2 = myfont.render("Boat angularVelocity: "+str(self.model.boat1.angularVelocity), 1, (0,0,0))
        text_3 = myfont.render("Boat debug: "+str(self.model.boat1.debug_list), 1, (0,0,0))
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
                
            #qa control rudder, zxc throw it
            if event.key == pygame.K_q:
                self.model.boat1.RudderSuggestion += 0.1
            if event.key == pygame.K_a:
                self.model.boat1.RudderSuggestion += -0.1
            if event.key == pygame.K_z:
                self.model.boat1.RudderSuggestion = -1
            if event.key == pygame.K_x:
                self.model.boat1.RudderSuggestion = 0
            if event.key == pygame.K_c:
                self.model.boat1.RudderSuggestion = 1
                
if __name__ == '__main__':
    pygame.init()
    size = (820,800)
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
