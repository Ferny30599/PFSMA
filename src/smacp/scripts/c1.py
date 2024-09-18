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
x1c=0
y1c=0
th1=0
x1h=0
y1h=0
dxh1=0
dyh1=0
x1=0
y1=0
z1=0
w1=0
#Condiciones iniciales 
V=0.0
w=0.0
#Punto deseado
#xd=12
#yd=0
xini=0
yini=0
xfin=8
yfin=0
tini=15#15
tfin=195#255#:4min #225#:3min30seg#195#:3min #165#:2min30seg #135#:2min #105#:1min30seg

#Variables Campos Potenciales
Katrac=0.3#1
Katrac1=0.3
Krep=0.001#0.008
Frepx=0
Frepy=0
dmin=0.001#0.18
dmax=0.65#0.61
s1=0
s2=0
s3=0
s4=0
s5=0
s6=0
dist=[0,0,0,0,0,0]
sensPos=[-1.5708,1.5708,3.1416,0,-0.8726,0.8726]

itera=1
xd1=0
yd1=0
dxd=0
dyd=0
dis=0

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

def callback1(data):
    global x1c,y1c,y1h,x1h,th1,x1,y1,z1,w1,dxh1,dyh1
    x1c = data.pose.pose.position.x
    y1c = data.pose.pose.position.y
    #Calculo de orientacion en cuaterniones
    x1 = data.pose.pose.orientation.x
    y1 = data.pose.pose.orientation.y
    z1 = data.pose.pose.orientation.z
    w1  = data.pose.pose.orientation.w
    t31 = 2.0 * (w1 * z1 + x1 * y1)
    t41 = 1.0 - 2.0 * (y1**2 + z1**2)
    yaw_z1 = math.atan2(t31, t41)
    th1 = yaw_z1 + math.pi/2
    #Calculo de punto de operacion h
    x1hant=x1h
    y1hant=y1h
    x1h = x1c + h*math.cos(th1) 
    y1h = y1c + h*math.sin(th1)
    dxh1=(x1h-x1hant)/delta
    dyh1=(y1h-y1hant)/delta
    #print(f"Posición anterior xh1ant:{x1hant},yh1ant:{y1hant}")
    print(f"Posición xh1:{x1h},yh1:{y1h}")
    #print(f"Velocidad dxh1:{dxh1},dyh1:{dyh1}")
    time = rospy.get_time()
    with open('/home/ferny/smacp/src/smacp/scripts/graficas/robot1.csv', 'a') as file:
        file.write(f"{x1h}, {y1h}, {time}\n")
    trayectoria()
    CamposPotenciales()

# Funcion de trayectoria deseada
def trayectoria():
    global t,xd,yd,dxd,dyd,xd1,yd1,itera
    #::::::::Lemniscata::::::::
    # time= 0.075*rospy.get_time()-math.pi/2
    # ax=1#2.5#3
    # ay=1.5#3.25

    # xd=ax*math.cos(time)/(1+math.sin(time)**2)
    # yd=ay*math.cos(time)*math.sin(time)/(1+math.sin(time)**2)
    #:::::::::::Recta::::::::::::::::
    time=rospy.get_time()
    npuntos=8
    a=(xfin-xini)/npuntos
    #tau= (time - tini)/(tfin-tini)
    #poli=pow(tau,5)*(126 - 420*tau + 540*pow(tau,2) - 315*pow(tau,3) + 70*pow(tau,4))
    if time < tini:
        xd=xini
        xd1=xini
    else:
        xd=a*itera
        if xd-x1h<0.1 and yd-y1h<0.1:
            itera=itera+1
        if itera>npuntos:
            itera=npuntos
    dxd=(xd-xd1)/delta
    
    yd = (yfin-yini)/(xfin-xini)*(xd - xini) + yini
    #dyd= (yfin-yini)/(xfin-xini)*dxd 
    dyd=(yd-yd1)/delta
    xd1=xd
    yd1=yd

    #xd=0
    #yd=0
    print(f"Trayectoria xd:{xd},yd:{yd},time:{time}")
    with open('/home/ferny/smacp/src/smacp/scripts/graficas/trayectoria.csv', 'a') as file:
        file.write(f"{xd}, {yd}, {time}\n")

def CamposPotenciales():
    global V,w
    Fa=F_atractiva()
    Fr=F_repulsiva()
    ux=Fa[0]+Fr[0]
    uy=Fa[1]+Fr[1]
    
    V= ux*math.cos(th1)+uy*math.sin(th1)
    w= -ux*math.sin(th1)/h+uy*math.cos(th1)/h
    #print("Pos(xh,yh)",(x1h,y1h)) 

def F_atractiva():
    #--::Campos de atraccion::--
    global xd,yd
    Fatracx=Katrac1*(-dxd)-Katrac*(x1h-xd)
    Fatracy=Katrac1*(-dyd)-Katrac*(y1h-yd)
    #print(f"Fatrac={(Fatracx,Fatracy)}")
    return [Fatracx,Fatracy]

def F_repulsiva():
    global Frepx,Frepy
    dist=[s1,s2,s3,s4,s5,s6]
    Frepx=0
    Frepy=0
    for i in range(6):
        xs = dist[i] * math.cos(sensPos[i])
        ys = dist[i] * math.sin(sensPos[i])
        di = math.sqrt(xs ** 2 + ys ** 2)
        if dmin <= di and di < dmax:
            Frepxi = -Krep * xs * (1 / di - 1 / dmax) / (di ** 3)
            Frepyi = -Krep * ys * (1 / di - 1 / dmax) / (di ** 3)
        else:
            Frepxi=0
            Frepyi=0           
        Frepx += Frepxi
        Frepy += Frepyi
    #print(f"Frep=({Frepx},{Frepy})")
    return [Frepx, Frepy]

if __name__=="__main__":
    #inicializacion de nodo
    rospy.init_node('Control1')
    #Publicar en topico
    pub=rospy.Publisher('/1/cmd_vel1',Twist,queue_size=5)
    #Suscripciones a topicos
    rospy.Subscriber('/1/odom1',Odometry,callback1)
    rospy.Subscriber('/r1/s1', LaserScan, ultrasonic1_callback)
    rospy.Subscriber('/r1/s2', LaserScan, ultrasonic2_callback)
    rospy.Subscriber('/r1/s3', LaserScan, ultrasonic3_callback)
    rospy.Subscriber('/r1/s4', LaserScan, ultrasonic4_callback)
    rospy.Subscriber('/r1/s5', LaserScan, ultrasonic5_callback)
    rospy.Subscriber('/r1/s6', LaserScan, ultrasonic6_callback)
    rate=rospy.Rate(hz)

    try:
        while not rospy.is_shutdown():
            #trayectoria()
            #CamposPotenciales()
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
        print("Cerrando C1...")
        pass

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
