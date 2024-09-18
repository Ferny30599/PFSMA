#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

hz=10
delta=1/hz
#Parametros de robot 
h=0.085
L=0.1244
r=0.015

#Variables para cada agente
x7c=0
y7c=0
th7=0
x7h=0
y7h=0
x7=0
y7=0
z7=0
w7=0

x6c=0
y6c=0
th6=0
x6h=0
y6h=0
x6=0
y6=0
z6=0
w6=0

x5c=0
y5c=0
th5=0
x5h=0
y5h=0
x5=0
y5=0
z5=0
w5=0

x1c=0
y1c=0
th1=0
x1h=0
y1h=0
x1=0
y1=0
z1=0
w1=0

#Condiciones iniciales 
V=0.0
w=0.0

x1h, x2h, x3h, x4h, x5h,x6h,x7h=0,0,0,0,0,0,0
y1h, y2h, y3h, y4h, y5h,y6h,y7h=0,0,0,0,0,0,0
k1=1#1#0.8#0.5#0.3
global xhpos,yhpos,Laplaciana, lx, ly
# Laplaciana=np.array([[ 4, -1,  0, -1, -1,  0, -1],
#                      [-1,  2, -1,  0,  0,  0,  0],
#                      [ 0, -1,  2, -1,  0,  0,  0],
#                      [-1,  0, -1,  3, -1,  0,  0],
#                      [-1,  0,  0, -1,  3, -1,  0],
#                      [ 0,  0,  0,  0, -1,  2, -1],
#                      [-1,  0,  0,  0,  0, -1,  2]])
Laplaciana=np.array([[ 2, -1,  0, -1, -1,  0, -1],
                     [-1,  3, -1, -1,  0,  0,  0],
                     [ 0, -1,  1,  0,  0,  0,  0],
                     [ 0, -1,  0,  1,  0,  0,  0],
                     [ 0,  0,  0,  0,  1,  0, -1],
                     [ 0,  0,  0,  0,  0,  1, -1],
                     [-1,  0,  0,  0, -1, -1,  3]])

lx=np.array([0, -0.5, -1, -1.5, -1.5, -1, -0.5])
ly=np.array([0,  0.5,  1,  1.5, -1.5, -1, -0.5])
#ly=np.array([0, 0.75, 1.5, 2.25, -2.25, -1.5, -0.75])



xhpos=np.array([x1h, x2h, x3h, x4h, x5h,x6h, x7h])
yhpos=np.array([y1h, y2h, y3h, y4h, y5h,y6h, y7h])    
agente=7

#Variables Campos Potenciales
Katrac=0.3#1
Krep=0.003#0.5#0.08#0.16

Frepx=0
Frepy=0
dmin=0.1#0.18
dmax=0.61#0.61

s1=0
s2=0
s3=0
s4=0
s5=0
s6=0
dist=[0,0,0,0,0,0]
sensPos=[-1.5708,1.5708,3.1416,0,-0.8726,0.8726]

def ultrasonic1_callback(data):
    global s1
    # Filtra los valores finitos
    dist = [x for x in data.ranges if math.isfinite(x)]
    # Encuentra el valor minimo
    if not dist:
        s1=float('inf')
    else:
        s1=min(dist)
    #print(f"Sensor 1:{s1}")   

def ultrasonic2_callback(data):
    global s2
    # Filtra los valores finitos
    dist = [x for x in data.ranges if math.isfinite(x)]
    # Encuentra el valor minimo
    if not dist:
        s2=float('inf')
    else:
        s2=min(dist)
    #print(f"Sensor 2:{s2}")

def ultrasonic3_callback(data):
    global s3
    # Filtra los valores finitos
    dist = [x for x in data.ranges if math.isfinite(x)]
    # Encuentra el valor minimo
    if not dist:
        s3=float('inf')
    else:
        s3=min(dist)
    #print(f"Sensor 3:{s3}")
    
def ultrasonic4_callback(data):
    global s4
    # Filtra los valores finitos
    dist = [x for x in data.ranges if math.isfinite(x)]
    # Encuentra el valor minimo
    if not dist:
        s4=float('inf')
    else:
        s4=min(dist)
    #print(f"Sensor 4:{s4}")
    
def ultrasonic5_callback(data):
    global s5
    # Filtra los valores finitos
    dist = [x for x in data.ranges if math.isfinite(x)]
    # Encuentra el valor minimo
    if not dist:
        s5=float('inf')
    else:
        s5=min(dist)
    #print(f"Sensor 5:{s5}")  

def ultrasonic6_callback(data):
    global s6
    # Filtra los valores finitos
    dist = [x for x in data.ranges if math.isfinite(x)]
    # Encuentra el valor minimo
    if not dist:
        s6=float('inf')
    else:
        s6=min(dist)
    #print(f"Sensor 6:{s6}")  

def callback7(data):
    global x7c,y7c,y7h,x7h,th7,x7,y7,z7,w7
    x7c = data.pose.pose.position.x
    y7c = data.pose.pose.position.y
    #Calculo de orientacion en cuaterniones
    x7 = data.pose.pose.orientation.x
    y7 = data.pose.pose.orientation.y
    z7 = data.pose.pose.orientation.z
    w7 = data.pose.pose.orientation.w
    t37 = 2.0 * (w7 * z7 + x7 * y7)
    t47 = 1.0 - 2.0 * (y7**2 + z7**2)
    yaw_z7 = math.atan2(t37, t47)
    th7 = yaw_z7 + math.pi/2
    #Calculo de punto de operacion h
    x7h = x7c + h*math.cos(th7) 
    y7h = y7c + h*math.sin(th7)
    with open('/home/ferny/smacp/src/smacp/scripts/graficas/robot7.csv', 'a') as file:
        file.write(f"{x7h}, {y7h}\n")
    CamposPotenciales()


def callback6(data):
    global x6c,y6c,y6h,x6h,th6,x6,y6,z6,w6
    x6c = data.pose.pose.position.x
    y6c = data.pose.pose.position.y
    #Calculo de orientacion en cuaterniones
    x6 = data.pose.pose.orientation.x
    y6 = data.pose.pose.orientation.y
    z6 = data.pose.pose.orientation.z
    w6 = data.pose.pose.orientation.w
    t36 = 2.0 * (w6 * z6 + x6 * y6)
    t46 = 1.0 - 2.0 * (y6**2 + z6**2)
    yaw_z6 = math.atan2(t36, t46)
    th6 = yaw_z6 + math.pi/2
    #Calculo de punto de operacion h
    x6h = x6c + h*math.cos(th6) 
    y6h = y6c + h*math.sin(th6)


def callback1(data):
    global x1c,y1c,y1h,x1h,th1,x1,y1,z1,w1
    x1c = data.pose.pose.position.x
    y1c = data.pose.pose.position.y
    #Calculo de orientacion en cuaterniones
    x1 = data.pose.pose.orientation.x
    y1 = data.pose.pose.orientation.y
    z1 = data.pose.pose.orientation.z
    w1 = data.pose.pose.orientation.w
    t31 = 2.0 * (w1 * z1 + x1 * y1)
    t41 = 1.0 - 2.0 * (y1**2 + z1**2)
    yaw_z1 = math.atan2(t31, t41)
    th1 = yaw_z1 + math.pi/2
    #Calculo de punto de operacion h
    x1h = x1c + h*math.cos(th1) 
    y1h = y1c + h*math.sin(th1)

def callback5(data):
    global x5c,y5c,y5h,x5h,th5,x5,y5,z5,w5
    x5c = data.pose.pose.position.x
    y5c = data.pose.pose.position.y
    #Calculo de orientacion en cuaterniones
    x5 = data.pose.pose.orientation.x
    y5 = data.pose.pose.orientation.y
    z5 = data.pose.pose.orientation.z
    w5  = data.pose.pose.orientation.w
    t35 = 2.0 * (w5 * z5 + x5 * y5)
    t45 = 1.0 - 2.0 * (y5**2 + z5**2)
    yaw_z5 = math.atan2(t35, t45)
    th5 = yaw_z5 + math.pi/2
    #Calculo de punto de operacion h
    x5h = x5c + h*math.cos(th5) 
    y5h = y5c + h*math.sin(th5)

    
def CamposPotenciales():
    global V,w
    Fa=F_atractiva()
    Fr=F_repulsiva()

    ux=Fa[0]+1.0*Fr[0]
    uy=Fa[1]+1.0*Fr[1]
    
    V = ux*math.cos(th7) + uy*math.sin(th7)
    w = -ux*math.sin(th7)/h + uy*math.cos(th7)/h
    #print("Pos(xh,yh)",(x1h,y1h)) 

def F_atractiva():
    #--::Campos de atraccion::--
    xhpos=np.array([x1h, x2h, x3h, x4h, x5h,x6h, x7h])-lx
    yhpos=np.array([y1h, y2h, y3h, y4h, y5h,y6h, y7h])-ly
    Fatracx=-Katrac*Laplaciana[agente-1].dot(xhpos.T)
    Fatracy=-Katrac*Laplaciana[agente-1].dot(yhpos.T)

    return [Fatracx,Fatracy]

def F_repulsiva():
    global Frepx,Frepy
    dist=[s1,s2,s3,s4,s5,s6]
    Frepx=0
    Frepy=0
    Frepxi=0
    Frepyi=0
    for i in range(6):
            xs = dist[i] * math.cos(sensPos[i])
            ys = dist[i] * math.sin(sensPos[i])
            di = math.sqrt(xs ** 2 + ys ** 2)
            if dmin < di and di < dmax:
                Frepxi = -Krep * xs * (1 / di - 1 / dmax) / (di ** 3)
                Frepyi = -Krep * ys * (1 / di - 1 / dmax) / (di ** 3)
            Frepx += Frepxi
            Frepy += Frepyi
    #print(f"Frepx={Frepx},Frepy={Frepy}")
    return [Frepx, Frepy]


if __name__=="__main__":
    #inicializacion de nodo
    rospy.init_node('Control7')
    #Publicar en topico
    pub=rospy.Publisher('/7/cmd_vel7',Twist,queue_size=5)
    #Suscripciones a topicos    
    rospy.Subscriber('/7/odom7',Odometry,callback7)
    rospy.Subscriber("/6/odom6",Odometry,callback6)
    rospy.Subscriber('/5/odom5',Odometry,callback5)
    rospy.Subscriber('/1/odom1',Odometry,callback1)
    rospy.Subscriber('/r7/s1', LaserScan, ultrasonic1_callback)
    rospy.Subscriber('/r7/s2', LaserScan, ultrasonic2_callback)
    rospy.Subscriber('/r7/s3', LaserScan, ultrasonic3_callback)
    rospy.Subscriber('/r7/s4', LaserScan, ultrasonic4_callback)
    rospy.Subscriber('/r7/s5', LaserScan, ultrasonic5_callback)
    rospy.Subscriber('/r7/s6', LaserScan, ultrasonic6_callback)
    
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
