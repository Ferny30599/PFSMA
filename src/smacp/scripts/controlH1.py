#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

hz=10
k1=0.5#0.5#0.3
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

#Condiciones iniciales 
V=0.0
w=0.0
#Punto deseado
xd=3
yd=3
#Variables Campos Potenciales
Katrac=1#1
Krep=0.008#0.5#0.08#0.16
Qd=0.5
Frepx=0
Frepy=0
dmin=0.1#0.18
dmax=0.61#0.11#0.3
noDetectionDist=0.25#1
#maxDetectionDist=0.5
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
xfin=3
yfin=3
tini=2
tfin=10
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
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y**2 + z**2)
    yaw_z = math.atan2(t3, t4)
    th1 = yaw_z + math.pi/2
    #Calculo de punto de operacion h
    x1h = x1c + h*math.cos(th1) 
    y1h = y1c + h*math.sin(th1)
    time = rospy.get_time()
    with open('plot/robot1_PFSegWO.csv', 'a') as file:
        file.write(f"{x1h}, {y1h}, {xd},{yd}, {time}\n")





# Control clasico h
def control():
    global V,w
    ux=-k1*(x1h-xd)
    uy=-k1*(y1h-yd)
    
    V = ux*math.cos(th1) + uy*math.sin(th1)
    w = -ux*math.sin(th1)/h + uy*math.cos(th1)/h

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

def trayectoria():
    global t,xd,yd,dxd,dyd,xd1
    time= 0.05*rospy.get_time()-math.pi/2
    ax=2.5#3
    ay=3.25

    xd=ax*math.cos(time)/(1+math.sin(time)**2)
    yd=ay*math.cos(time)*math.sin(time)/(1+math.sin(time)**2)
    # xd=3
    # yd=3
    
    # print(f"Trayectoria xd:{xd},yd:{yd},tiempo:{time}")

# Funcion de trayectoria deseada polinomio 
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
    
    V= ux*math.cos(th1)+uy*math.sin(th1)
    w= -ux*math.sin(th1)/h+uy*math.cos(th1)/h
    #print("Pos(xh,yh)",(x1h,y1h)) 

def F_atractiva():
    #--::Campos de atraccion::--
    d=math.sqrt((x1h-xd)**2+(y1h-yd)**2)
    Fatracx=0
    Fatracy=0
    if d<Qd:
        Fatracx=-Katrac*dxd-Katrac*(x1h-xd)
        Fatracy=-Katrac*dyd-Katrac*(y1h-yd)
    else:
        Fatracx=-Qd*Katrac*dxd/d-Qd*Katrac*(x1h-xd)/d
        Fatracy=-Qd*Katrac*dyd/d-Qd*Katrac*(y1h-yd)/d
    print(f"Fatrac={(Fatracx,Fatracy)}")
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
         #   if dist[i] > maxDetectionDist:
          #      dist[i] = maxDetectionDist
           # j=i+1
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
    print(f"Frepx={Frepx},Frepy={Frepy}")
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
            trayectoria()
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
