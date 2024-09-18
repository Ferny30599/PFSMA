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
x5c=0
y5c=0
th5=0
x5h=0
y5h=0
x5=0
y5=0
z5=0
w5=0

x6c=0
y6c=0
th6=0
x6h=0
y6h=0
x6=0
y6=0
z6=0
w6=0

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

#Control del SMA
x1h, x2h, x3h, x4h, x5h,x6h=0,0,0,0,0,0
y1h, y2h, y3h, y4h, y5h,y6h=0,0,0,0,0,0
k1=1#1#2#0.8#0.5#0.3
global xhpos,yhpos,Laplaciana
Laplaciana=np.array([[ 3, -1,  0, -1,  0, -1],
                     [-1,  2, -1,  0,  0,  0],
                     [ 0, -1,  2, -1,  0,  0],
                     [-1,  0, -1,  3, -1,  0],
                     [ 0,  0,  0, -1,  2, -1],
                     [-1,  0,  0,  0, -1,  2]])
lx=np.array([1, 0.5, -0.5, 0.5, -0.5, -1])
ly=np.array([0, 1, 1, -1, -1, 0])

xhpos=np.array([x1h, x2h, x3h, x4h, x5h,x6h])
yhpos=np.array([y1h, y2h, y3h, y4h, y5h,y6h])    
agente=6

#Variables Campos Potenciales
Katrac=0.5#1
Krep=0.08#0.008
Qd=0.5
Frepx=0
Frepy=0
dmin=0.1#0.18
dmax=0.61#0.11#0.3
noDetectionDist=0.25#1
s1=0
s2=0
s3=0
s4=0
s5=0
s6=0
dist=[0,0,0,0,0,0]
sensPos=[-1.5708,1.5708,3.1416,0,-0.8726,0.8726]

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
    time = rospy.get_time()
    with open('/home/ferny/smacp/src/smacp/scripts/plot/SMA/evasion/robot6.csv', 'a') as file:
        file.write(f"{x6h}, {y6h}, {time}\n")

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


# Control clasico h
def control():
    global V,w
    xhpos=np.array([x1h, x2h, x3h, x4h, x5h,x6h])-lx
    yhpos=np.array([y1h, y2h, y3h, y4h, y5h,y6h])-ly
    ux=-k1*Laplaciana[agente-1].dot(xhpos.T)
    uy=-k1*Laplaciana[agente-1].dot(yhpos.T)
    V = ux*math.cos(th6) + uy*math.sin(th6)
    w = -ux*math.sin(th6)/h + uy*math.cos(th6)/h
    #ux=-k1*(x6h-x5h)-k1*(x6h-x1h)
    #uy=-k1*(y6h-y5h)-k1*(y6h-y1h)
       
    #V = ux*math.cos(th6) + uy*math.sin(th6)
    #w = -ux*math.sin(th6)/h + uy*math.cos(th6)/h


    #Saturacion de salidas en funcion del robot real
    #if (V>0.5):
     #   V=0.5
    #if (V<-0.5):
     #   V=-0.5
    #if (w>8.69):
     #   w=8.68
    #if (w<-8.69):
     #   w=-8.68
    #print("Pos(xh,yh)",(x1h,y1h))  
    #print("Vel(Lin,Ang)= ",(V,w))
    #print("Ori(rad)=",th1)

# Funcion de trayectoria deseada
def trayectoria():
    global t,xd,yd,dxd,dyd,xd1
    time= 0.05*rospy.get_time()-math.pi/2
    ax=2.5#3
    ay=3.25

    xd=ax*math.cos(time)/(1+math.sin(time)**2)
    yd=ay*math.cos(time)*math.sin(time)/(1+math.sin(time)**2)

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

    ux=Fa[0]+Fr[0]
    uy=Fa[1]+Fr[1]
    
    V = ux*math.cos(th6) + uy*math.sin(th6)
    w = -ux*math.sin(th6)/h + uy*math.cos(th6)/h
    #print("Pos(xh,yh)",(x1h,y1h)) 

def F_atractiva():
    #--::Campos de atraccion::--
    xhpos=np.array([x1h, x2h, x3h, x4h, x5h,x6h])-lx
    yhpos=np.array([y1h, y2h, y3h, y4h, y5h,y6h])-ly
    d=math.sqrt((x6h-x5h)**2+(y6h-y5h)**2)
    #if d<Qd:
    Fatracx=-Katrac*Laplaciana[agente-1].dot(xhpos.T)
    Fatracy=-Katrac*Laplaciana[agente-1].dot(yhpos.T)
    #else:
    #    Fatracx=-Qd*Katrac*Laplaciana[agente-1].dot(xhpos.T)/d
    #    Fatracy=-Qd*Katrac*Laplaciana[agente-1].dot(yhpos.T)/d
    #print(f"Fatrac={(Fatracx,Fatracy)}")
    return [Fatracx,Fatracy]

def F_repulsiva():
    global Frepx,Frepy
    dist=[s1,s2,s3,s4,s5,s6]
    #Katrac=1.5
    #Krep=Katrac/6
    #Qd=0.5
    #Frepx=0
    #Frepy=0
    #dmin=0.1
    #dmax=0.3
    #noDetectionDist=0.8
    #maxDetectionDist=0.15
    Frepx=0
    Frepy=0
    for i in range(6):
        #if dist[i] < noDetectionDist:
        #    if dist[i] > maxDetectionDist:
        #        dist[i] = maxDetectionDist
        #    j=i+1
            #print(f'Distancias del sensor {j}: {dist[i]}')
            xs = dist[i] * math.cos(sensPos[i])
            ys = dist[i] * math.sin(sensPos[i])
            di = math.sqrt(xs ** 2 + ys ** 2)

            if dmin < di and di < dmax:
                Frepxi = -Krep * xs * (1 / di - 1 / dmax) / (di ** 3)
                Frepyi = -Krep * ys * (1 / di - 1 / dmax) / (di ** 3)
            else:
                Frepxi = 0
                Frepyi = 0
            Frepx += Frepxi
            Frepy += Frepyi
    #print(f"Frepx={Frepx},Frepy={Frepy}")
    return [Frepx, Frepy]


if __name__=="__main__":
    #inicializacion de nodo
    rospy.init_node('Control6')
    #Publicar en topico
    pub=rospy.Publisher('/6/cmd_vel6',Twist,queue_size=5)
    #Suscripciones a topicos    
    rospy.Subscriber('/5/odom5',Odometry,callback5)
    rospy.Subscriber("/6/odom6",Odometry,callback6)
    rospy.Subscriber('/1/odom1',Odometry,callback1)
    rospy.Subscriber('/r6/s1', LaserScan, ultrasonic1_callback)
    rospy.Subscriber('/r6/s2', LaserScan, ultrasonic2_callback)
    rospy.Subscriber('/r6/s3', LaserScan, ultrasonic3_callback)
    rospy.Subscriber('/r6/s4', LaserScan, ultrasonic4_callback)
    rospy.Subscriber('/r6/s5', LaserScan, ultrasonic5_callback)
    rospy.Subscriber('/r6/s6', LaserScan, ultrasonic6_callback)
    
    rate=rospy.Rate(hz)

    try:
        while not rospy.is_shutdown():
            #trayectoria()
            #control()
            CamposPotenciales()
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
