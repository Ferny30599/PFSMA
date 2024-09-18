#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range


hz=100
k1=3#0.5#0.3
delta=1/hz
#Parametros de robot 
h=0.025
L=0.1244
r=0.015

#Variables para cada agente
x1c=0
y1c=0
th1=0
x1h=0
y1h=0

#Condiciones iniciales 
V=0.0
w=0.0
#Punto deseado
xd=1
yd=1

t=0
xd1=0
dxd=0
dyd=0
xini=0
yini=0
xfin=2
yfin=2
tini=5
tfin=40
dis=0

def ultrasonic_callback(data):
    global dis
    dis=data.range
    rospy.loginfo("Distancia detectada por el sensor ultrasonico: {:.2f} metros".format(dis))


# Funcion para obtener posicion y angulo de agentes
def callback1(data):
    global x1c,y1c,y1h,x1h,th1
    x1c = data.pose.pose.position.x
    y1c = data.pose.pose.position.y
    #Calculo de orientacion en cuaterniones
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w  = data.pose.pose.orientation.w
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    th1 = yaw_z + math.pi/2
    #Calculo de punto de operacion h
    x1h = x1c - h*math.cos(th1) 
    y1h = y1c


# Control clasico h
def control():
    global V,w
    r1=-k1*(x1h-xd)
    r2=-k1*(y1h-yd)
    V = r1*math.cos(th1) + r2*math.sin(th1)
    w = -r1*math.sin(th1)/h + r2*math.cos(th1)/h
    #Saturacion de salidas en funcion del robot real
    #if (V>0.5):
     #   V=0.5
    #if (V<-0.5):
     #   V=-0.5
    #if (w>8.69):
     #   w=8.68
    #if (w<-8.69):
     #   w=-8.68
    print("Pos(xh,yh)",(x1h,y1h))  
    print("Vel(Lin,Ang)= ",(V,w))
    print("Ori(rad)=",th1)

# Funcion de trayectoria deseada
def trayectoria():
    global t,xd,yd,dxd,dyd,xd1
    time=t*delta
    tau= (time - tini)/(tfin-tini)
    poli=pow(tau,5)*(126 - 420*tau + 540*pow(tau,2) - 315*pow(tau,3) + 70*pow(tau,4))
    if time < tini:
        xd=xini
        xd1=xini
    if time > tfin:
        xd=xfin
    if (time >= tini)&(time <= tfin):
        xd= xini + poli*(xfin-xini) 
        
    dxd=(xd-xd1)/delta
    yd = (yfin-yini)/(xfin-xini)*(xd - xini) + yini
    dyd= (yfin-yini)/(xfin-xini)*dxd 
    xd1=xd
    print("#####################################")
    print("time= ", time)
    print("error x= ", xd-x1h)
    print("error y= ", yd-y1h)
    t=t+1
    
if __name__=="__main__":
    #inicializacion de nodo
    rospy.init_node('Control1')
    #Publicar en topico
    pub=rospy.Publisher('/1/cmd_vel1',Twist,queue_size=5)
    #Suscripciones a topicos
    #rospy.Subscriber('/1/odom1',Odometry,callback1)
    rospy.Subscriber('/ultrasonic', Range, ultrasonic_callback)
    rate=rospy.Rate(hz)

    try:
        while not rospy.is_shutdown():
            #trayectoria()
            #control()
            twist = Twist()
            twist.linear.x = V; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w
            pub.publish(twist)
            rate.sleep()
       
        
    except rospy.ROSInterruptException:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        print("Error")
        pass

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
