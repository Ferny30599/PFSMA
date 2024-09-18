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
x1=0
y1=0
z1=0
w1=0

x2c=0
y2c=0
th2=0
x2h=0
y2h=0
x2=0
y2=0
z2=0
w2=0

x4c=0
y4c=0
th4=0
x4h=0
y4h=0
x4=0
y4=0
z4=0
w4=0

x5c=0
y5c=0
th5=0
x5h=0
y5h=0
x5=0
y5=0
z5=0
w5=0

x7c=0
y7c=0
th7=0
x7h=0
y7h=0
x7=0
y7=0
z7=0
w7=0

#Control del SMA
x1h, x2h, x3h, x4h, x5h,x6h,x7h=0,0,0,0,0,0,0
y1h, y2h, y3h, y4h, y5h,y6h,y7h=0,0,0,0,0,0,0
k1=1#1#0.8#0.5#0.3
global xhpos,yhpos,Laplaciana, lx, ly
Laplaciana=np.array([[ 4, -1,  0, -1, -1,  0, -1],
                     [-1,  2, -1,  0,  0,  0,  0],
                     [ 0, -1,  2, -1,  0,  0,  0],
                     [-1,  0, -1,  3, -1,  0,  0],
                     [-1,  0,  0, -1,  3, -1,  0],
                     [ 0,  0,  0,  0, -1,  2, -1],
                     [-1,  0,  0,  0,  0, -1,  2]])

lx=np.array([0, 1.0, 0.5, -0.5, -1, -0.5, 0.5])
ly=np.array([0,  0,  1.0,  1.0, 0, -1.0, -1.0])
#lx=np.array([0, -0.5, -1, -1.5, -1.5, -1, -0.5])
#ly=np.array([0,  0.5,  1,  1.5, -1.5, -1, -0.5])
#ly=np.array([0, 0.75, 1.5, 2.25, -2.25, -1.5, -0.75]) #Funcional


xhpos=np.array([x1h, x2h, x3h, x4h, x5h,x6h, x7h])
yhpos=np.array([y1h, y2h, y3h, y4h, y5h,y6h, y7h])    
agente=1


#Condiciones iniciales 
V=0.0
w=0.0
#Punto deseado
#xd=12
#yd=0
xini=0
yini=0
xfin=6#8
yfin=5#0
tini=10#15
tfin=195#255#:4min #225#:3min30seg#195#:3min #165#:2min30seg #135#:2min #105#:1min30seg
npuntos=7

#Variables Campos Potenciales
Katrac=0.3#1
Krep=0.005#0.008
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

def callback2(data):
    global x2c,y2c,y2h,x2h,th2,x2,y2,z2,w2
    x2c = data.pose.pose.position.x
    y2c = data.pose.pose.position.y
    #Calculo de orientacion en cuaterniones
    x2 = data.pose.pose.orientation.x
    y2 = data.pose.pose.orientation.y
    z2 = data.pose.pose.orientation.z
    w2  = data.pose.pose.orientation.w
    t32 = 2.0 * (w2 * z2 + x2 * y2)
    t42 = 1.0 - 2.0 * (y2**2 + z2**2)
    yaw_z2 = math.atan2(t32, t42)
    th2 = yaw_z2 + math.pi/2
    #Calculo de punto de operacion h
    x2h = x2c + h*math.cos(th2) 
    y2h = y2c + h*math.sin(th2)

def callback1(data):
    global x1c,y1c,y1h,x1h,th1,x1,y1,z1,w1
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
    x1h = x1c + h*math.cos(th1) 
    y1h = y1c + h*math.sin(th1)
    #print(f"Posición:{x1h},{y1h}")
    #print(f"[{x1h},{x1h-0.5}, {x1h-1}, {x1h-1.5}, {x1h-1.5}, {x1h-1}, {x1h-0.5}]")
    #print(f"[{y1h},{y1h-0.5}, {y1h-1}, {y1h-1.5}, {y1h-1.5}, {y1h-1}, {y1h-0.5}]")
    time = rospy.get_time()
    with open('/home/ferny/smacp/src/smacp/scripts/graficas/robot1.csv', 'a') as file:
        file.write(f"{x1h}, {y1h}, {time}\n")
    trayectoria()
    CamposPotenciales()

def callback4(data):
    global x4c,y4c,y4h,x4h,th4,x4,y4,z4,w4
    x4c = data.pose.pose.position.x
    y4c = data.pose.pose.position.y
    #Calculo de orientacion en cuaterniones
    x4 = data.pose.pose.orientation.x
    y4 = data.pose.pose.orientation.y
    z4 = data.pose.pose.orientation.z
    w4  = data.pose.pose.orientation.w
    t34 = 2.0 * (w4 * z4 + x4 * y4)
    t44 = 1.0 - 2.0 * (y4**2 + z4**2)
    yaw_z4 = math.atan2(t34, t44)
    th4 = yaw_z4 + math.pi/2
    #Calculo de punto de operacion h
    x4h = x4c + h*math.cos(th4) 
    y4h = y4c + h*math.sin(th4)

def callback5(data):
    global x5c,y5c,y5h,x5h,th5,x5,y5,z5,w5
    x5c = data.pose.pose.position.x
    y5c = data.pose.pose.position.y
    #Calculo de orientacion en cuaterniones
    x5 = data.pose.pose.orientation.x
    y5 = data.pose.pose.orientation.y
    z5 = data.pose.pose.orientation.z
    w5 = data.pose.pose.orientation.w
    t35 = 2.0 * (w5 * z5 + x5 * y5)
    t45 = 1.0 - 2.0 * (y5**2 + z5**2)
    yaw_z5 = math.atan2(t35, t45)
    th5 = yaw_z5 + math.pi/2
    #Calculo de punto de operacion h
    x5h = x5c + h*math.cos(th5) 
    y5h = y5c + h*math.sin(th5)

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

# Control lider seguidor
def control():
    global V,w
    xhpos=np.array([x1h, x2h, x3h, x4h, x5h,x6h, x7h])-lx
    yhpos=np.array([y1h, y2h, y3h, y4h, y5h,y6h, y7h])-ly
    ux=-k1*Laplaciana[agente-1].dot(xhpos.T)#+2*k1*(xd-x1h)
    uy=-k1*Laplaciana[agente-1].dot(yhpos.T)#+2*k1*(yd-y1h)
    V = ux*math.cos(th1) + uy*math.sin(th1)
    w = -ux*math.sin(th1)/h + uy*math.cos(th1)/h


# Funcion de trayectoria deseada
def trayectoria():
    global t,xd,yd,dxd,dyd,xd1,yd1,itera,npuntos
    #::::::::Lemniscata::::::::
    # time= 0.075*rospy.get_time()-math.pi/2
    # ax=1#2.5#3
    # ay=1.5#3.25

    # xd=ax*math.cos(time)/(1+math.sin(time)**2)
    # yd=ay*math.cos(time)*math.sin(time)/(1+math.sin(time)**2)
    #:::::::::::Recta::::::::::::::::
    time=rospy.get_time()
    a=(xfin-xini)/npuntos
    #tau= (time - tini)/(tfin-tini)
    #poli=pow(tau,5)*(126 - 420*tau + 540*pow(tau,2) - 315*pow(tau,3) + 70*pow(tau,4))
    if time < tini:
        xd=xini
        xd1=xini
    else:
        xd=a*itera
        if xd-x1h<0.3 and yd-y1h<0.3:
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
    #print(f"Trayectoria xd:{xd},yd:{yd}")
    with open('/home/ferny/smacp/src/smacp/scripts/graficas/trayectoria.csv', 'a') as file:
        file.write(f"{xd}, {yd}, {time}\n")

def trayectoria1():
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

    

def CamposPotenciales():
    global V,w
    Fa=F_atractiva()
    Fr=F_repulsiva()

    ux=Fa[0]+1.0*Fr[0]
    uy=Fa[1]+1.0*Fr[1]
    
    V= ux*math.cos(th1)+uy*math.sin(th1)
    w= -ux*math.sin(th1)/h+uy*math.cos(th1)/h
    #print("Pos(xh,yh)",(x1h,y1h)) 

def F_atractiva():
    #--::Campos de atraccion::--
    xhpos=np.array([x1h, x2h, x3h, x4h, x5h,x6h, x7h])-lx
    yhpos=np.array([y1h, y2h, y3h, y4h, y5h,y6h, y7h])-ly
    Fatracx=-Katrac*(-dxd -2.5*(xd-x1h)+Laplaciana[agente-1].dot(xhpos.T))
    Fatracy=-Katrac*(-dyd -2.5*(yd-y1h)+Laplaciana[agente-1].dot(yhpos.T))

    return [Fatracx,Fatracy]

def F_repulsiva():
    global Frepx,Frepy
    dist=[s1,s2,s3,s4,s5,s6]
    Frepx=0
    Frepy=0
    Frepxi = 0
    Frepyi = 0
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
    rospy.init_node('Control1')
    #Publicar en topico
    pub=rospy.Publisher('/1/cmd_vel1',Twist,queue_size=5)
    #Suscripciones a topicos
    rospy.Subscriber('/1/odom1',Odometry,callback1)
    rospy.Subscriber("/2/odom2",Odometry,callback2)
    rospy.Subscriber("/4/odom4",Odometry,callback4)
    rospy.Subscriber("/5/odom5",Odometry,callback5)
    rospy.Subscriber("/7/odom7",Odometry,callback7)
    rospy.Subscriber('/r1/s1', LaserScan, ultrasonic1_callback)
    rospy.Subscriber('/r1/s2', LaserScan, ultrasonic2_callback)
    rospy.Subscriber('/r1/s3', LaserScan, ultrasonic3_callback)
    rospy.Subscriber('/r1/s4', LaserScan, ultrasonic4_callback)
    rospy.Subscriber('/r1/s5', LaserScan, ultrasonic5_callback)
    rospy.Subscriber('/r1/s6', LaserScan, ultrasonic6_callback)
    
    rate=rospy.Rate(hz)

    try:
        while not rospy.is_shutdown():
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
