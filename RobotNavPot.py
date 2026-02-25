# -*- coding: utf-8 -*-
"""
Way Point navigtion

(c) S. Bertrand
"""

import math
import Robot as rob
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import Timer as tmr
import Potential

# robot
V =2.3  # On ralentit pour que le robot puisse prendre les virages
a = 25   
n = 3    # On commence par 4 pétales pour tester la stabilité
theta_c = 0.0 # Notre angle de progression
State = 0
F=0
seuil=200

x0 =- 20
y0 = -20
theta0 = 0
robot = rob.Robot(x0, y0, theta0)

# potential
pot = Potential.Potential(difficulty=1, random=True)

# position control loop: gain and timer
kpPos = 0.8
positionCtrlPeriod = 0.2
timerPositionCtrl = tmr.Timer(positionCtrlPeriod)

# orientation control loop: gain and timer
kpOrient = 2.5
orientationCtrlPeriod = 0.05
timerOrientationCtrl = tmr.Timer(orientationCtrlPeriod)

# list of way points list of [x coord, y coord]
WPlist = [ [x0,y0] ]
epsilonWP = 0.2
WPManager = rob.WPManager(WPlist, epsilonWP)

# duration of scenario and time step for numerical integration
t0 = 0.0
tf = 200.0
dt = 0.01
simu = rob.RobotSimulation(robot, t0, tf, dt)

# initialize control inputs
Vr = 0.0
thetar = 0.0
omegar = 0.0
theta_c = 0.0  # NOUVEAU: On stocke l'angle de la courbe et on le fait avancer manuellement

firstIter = True
m=0
# loop on simulation time
for t in simu.t: 

    if pot.value([robot.x, robot.y]) >seuil and State == 0:
        State = 1
        pn = pot.value([robot.x, robot.y]) # Amorce
        omegar = 0.5 # On commence par un petit virage pour "chercher"

    # position control loop
    if timerPositionCtrl.isEllapsed(t):
        potentialValue = pot.value([robot.x, robot.y])
        
        # reference orientation
        thetar = 0   
        if math.fabs(robot.theta-thetar)>math.pi:
            thetar = thetar + math.copysign(2*math.pi,robot.theta)        

    # orientation control loop
    if timerOrientationCtrl.isEllapsed(t):
        if State == 0 :
            # 1. Calculs basés sur le SINUS (pour commencer pile au centre 0,0)
            sin_n = np.sin(n * theta_c)
            cos_n = np.cos(n * theta_c)
        
            # 2. Dénominateur au carré (attention, cos et sin sont inversés par rapport à avant !)
            denom_sq = sin_n**2 + (n**2) * cos_n**2
        
            # 3. La vitesse linéaire est fixée
            Vr = V
        
            # 4. Calcul de la courbure mathématique pour le sinus
            numerateur = (1 + n**2) * sin_n**2 + 2 * (n**2) * cos_n**2
            kappa = numerateur / (a * (denom_sq**1.5))
        
            # 5. Vitesse angulaire envoyée au robot
            omegar = Vr * kappa
        
            # 6. On fait avancer l'angle sur la courbe
            dtheta_dt = V / (a * np.sqrt(denom_sq))
            theta_c += dtheta_dt * orientationCtrlPeriod
            
        if State == 1:
             pn1, pn = pn, pot.value([robot.x, robot.y])
             
           
             if pn >=   pn1:
               
                 Vr = 2.5
                 omegar = 0.0 
             else:
              
                 Vr = Vr/4
               
                 sens = np.sign(omegar) if omegar != 0 else 1.0
                 omegar =np.pi*3 * sens+omegar
                 
  
             if pn > 313:
                Vr, omegar = 0, 0
                m=m+1
                if m==1 :
                    print("source trouvée !",t)

    # integrate motion
    robot.integrateMotion(dt)
    # assign control inputs to robot
    robot.setV(Vr)
    robot.setOmega(omegar)  
    # store data to be plotted   
    simu.addData(robot, WPManager, Vr, thetar, omegar, pot.value([robot.x,robot.y]))
    

# end of loop on simulation time


# close all figures
plt.close("all")

# generate plots
fig,ax = simu.plotXY(1)
pot.plot(noFigure=None, fig=fig, ax=ax)  # plot potential for verification of solution

#simu.plotXYTheta(2)
#simu.plotVOmega(3)

#simu.plotPotential(4)

#simu.plotPotential3D(5)

# show plots
#plt.show()


# # Animation *********************************
# fig = plt.figure()
# ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-25, 25), ylim=(-25, 25))
# ax.grid()
# ax.set_xlabel('x (m)')
# ax.set_ylabel('y (m)')

# robotBody, = ax.plot([], [], 'o-', lw=2)
# robotDirection, = ax.plot([], [], '-', lw=1, color='k')
# wayPoint, = ax.plot([], [], 'o-', lw=2, color='b')
# time_template = 'time = %.1fs'
# time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
# potential_template = 'potential = %.1f'
# potential_text = ax.text(0.05, 0.1, '', transform=ax.transAxes)
# WPArea, = ax.plot([], [], ':', lw=1, color='b')

# thetaWPArea = np.arange(0.0,2.0*math.pi+2*math.pi/30.0, 2.0*math.pi/30.0)
# xWPArea = WPManager.epsilonWP*np.cos(thetaWPArea)
# yWPArea = WPManager.epsilonWP*np.sin(thetaWPArea)

# def initAnimation():
#     robotDirection.set_data([], [])
#     robotBody.set_data([], [])
#     wayPoint.set_data([], [])
#     WPArea.set_data([], [])
#     robotBody.set_color('r')
#     robotBody.set_markersize(20)    
#     time_text.set_text('')
#     potential_text.set_text('')
#     return robotBody,robotDirection, wayPoint, time_text, potential_text, WPArea  
    
# def animate(i):  
#     robotBody.set_data(simu.x[i], simu.y[i])          
#     wayPoint.set_data(simu.xr[i], simu.yr[i])
#     WPArea.set_data(simu.xr[i]+xWPArea.transpose(), simu.yr[i]+yWPArea.transpose())    
#     thisx = [simu.x[i], simu.x[i] + 0.5*math.cos(simu.theta[i])]
#     thisy = [simu.y[i], simu.y[i] + 0.5*math.sin(simu.theta[i])]
#     robotDirection.set_data(thisx, thisy)
#     time_text.set_text(time_template%(i*simu.dt))
#     potential_text.set_text(potential_template%(pot.value([simu.x[i],simu.y[i]])))
#     return robotBody,robotDirection, wayPoint, time_text, potential_text, WPArea

# ani = animation.FuncAnimation(fig, animate, np.arange(1, len(simu.t)),
#     interval=4, blit=True, init_func=initAnimation, repeat=False)
# #interval=25

# #ani.save('robot.mp4', fps=15)
