#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32


class Robot_CDR():
    def __init__(self):
        rospy.init_node("robot_move")
        rospy.loginfo("Press CTRL + C to terminate")
        
	
	self.encodeur_left = 0
	self.encodeur_right = 0
	
	self.last_encodeur_right = 0
	self.last_encodeur_left = 0

	self.moteur_left = 0
	self.moteur_right = 0
        
	self.error_right = 0
	self.error_left = 0
	
	self.last_error_right = 0
	self.last_error_left = 0
	
	self.sum_error_right = 0
	self.sum_error_left = 0

	self.speed_right = 0
	self.speed_left = 0

        self.encodeur_left_sub = rospy.Subscriber("robot/base/wheel/left/state", Float32, self.encodeur_left_callback)
	self.encodeur_right_sub = rospy.Subscriber("robot/base/wheel/right/state", Float32, self.encodeur_right_callback)
	

        self.moteur_left_pub = rospy.Publisher("robot/base/wheel/left/vel", Int16, queue_size=10)
	self.moteur_right_pub = rospy.Publisher("robot/base/wheel/right/vel",Int16, queue_size=10)

	self.speed_wanted_pub = rospy.Publisher("/Setpoint", Int16, queue_size=10)

        self.rate = rospy.Rate(10)
	self.timer = 0
	self.speed_left_list = list()
	self.speed_right_list = list()
	self.time = list()

        try:
            self.run()
        except rospy.ROSInterruptException:
            pass
        finally:

            self.visualization()

	

    def run(self):
	self.speed_wanted = 100
	self.Kp = 0   #A CHANGER
	self.Kd = 0   #A CHANGER
	self.Ki = 0   #A CHANGER

	self.speed_wanted_pub.publish(self.speed_wanted)

	while not (rospy.is_shutdown()):
		
		#ENCODEUR DROIT
		self.speed_right = self.encodeur_right - self.last_encodeur_right #On obtient la vitesse actuelle en tic encodeur		
		if (self.speed_right < 0):					  
			self.speed_right += 1024		
		self.last_encodeur_right = self.encodeur_right			#On conserve la valeur l'encodeur au temps t pour avoir la vitesse quand on sera au temps t + 1
		
		self.error_right = self.speed_wanted - self.speed_right 	#Erreur pour le PID 
	
		#ENCODEUR GAUCHE
		self.speed_left = self.encodeur_left - self.last_encodeur_left		
		if (self.speed_left < 0):
			self.speed_left += 1024
		self.last_encodeur_left = self.encodeur_left
		
		self.error_left = self.speed_wanted - self.speed_left
		
    		self.moteur_right += int((self.Kp * self.error_right) + (self.Kd*(self.error_right - self.last_error_right)) + (self.Ki*self.sum_error_right)) #Calcul de l'instruction moteur à envoyer
		self.moteur_left += int((self.Kp * self.error_left) + (self.Kd*(self.error_left - self.last_error_left)) + (self.Ki*self.sum_error_left))
		
		#ATTENTION, A TESTER : Je ne sais pas si l'intruction que peut donner les formules ci dessus peut être hors de l'intervalle [-255,255]
		#Le cas échéant, juste rajouter if > 255, on affecte 255 et si < -255, on affecte -255

		self.last_error_right = self.error_right
		self.last_error_left = self.error_left
		
		self.sum_error_right += self.error_right
		self.sum_error_left += self.error_left

		self.moteur_left_pub.publish(self.moteur_left)   #Publication aux moteurs
		self.moteur_right_pub.publish(self.moteur_right)
		
		self.timer += 0.1
		self.time.append(self.timer) 
		self.speed_left_list.append(self.moteur_left)
		self.speed_right_list.append(self.moteur_right)		


    def visualization(self):
        # plot speed
        
        plt.plot(self.timer,self.speed_left_list)
        plt.plot(self.timer,self.speed_right_list)


        plt.show()
	

    

    def encodeur_right_callback(self, msg):
        self.encodeur_right = msg.data

    def encodeur_left_callback(self, msg):
        self.encodeur_left = msg.data
            


if __name__ == '__main__':
    try:
        whatever = Robot_CDR()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminée.")
