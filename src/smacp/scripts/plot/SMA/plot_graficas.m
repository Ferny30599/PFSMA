ruta1=csvread('robot1_cons.csv');
posx1=ruta1(:,1);
posy1=ruta1(:,2);
tiempo1=ruta1(:,3);
ruta2=csvread('robot2_cons.csv');
posx2=ruta2(:,1);
posy2=ruta2(:,2);
tiempo2=ruta2(:,3);
ruta3=csvread('robot3_cons.csv');
posx3=ruta3(:,1);
posy3=ruta3(:,2);
ruta4=csvread('robot4_cons.csv');
posx4=ruta4(:,1);
posy4=ruta4(:,2);
tiempo4=ruta4(:,3);
ruta5=csvread('robot5_cons.csv');
posx5=ruta5(:,1);
posy5=ruta5(:,2);
tiempo5=ruta5(:,3);
ruta6=csvread('robot6_cons.csv');
posx6=ruta6(:,1);
posy6=ruta6(:,2);
tiempo6=ruta6(:,3);
figure(1);
plot(posx1,tiempo1,'LineWidth',4,'Color', 'b');
hold on;
grid on;
plot(posx2,tiempo2,'LineWidth',4,'Color', 'r');
plot(posx3,tiempo2,'LineWidth',4,'Color', 'g');
plot(posx4,tiempo4,'LineWidth',4,'Color', [1, 0.5, 0]);
plot(posx5,tiempo5,'LineWidth',4,'Color', [0.5, 0, 0.5]);
plot(posx6,tiempo6,'LineWidth',4,'Color', [0.5, 0, 0.5]);
xlabel('Eje X (m)','FontSize', 18);
ylabel('Tiempo (s)','FontSize', 18);
set(gca, 'FontSize', 18);  % Tamaño de los números en los ejes
set(gca, 'XScale', 'linear', 'YScale', 'linear', 'ZScale', 'linear');  % Escala lineal en los ejes X, Y, Z
legend('RMD1','RMD2','RMD3','RMD4','RMD5','RMD6','Location', 'northwest')

figure(2);
plot(posy1,tiempo1,'LineWidth',4,'Color', 'b');
hold on;
grid on;
plot(posy2,tiempo2,'LineWidth',4,'Color', 'r');
plot(posy3,tiempo2,'LineWidth',4,'Color', 'g');
plot(posy4,tiempo4,'LineWidth',4,'Color', [1, 0.5, 0]);
plot(posy5,tiempo5,'LineWidth',4,'Color', [0.5, 0, 0.5]);
plot(posy6,tiempo6,'LineWidth',4,'Color', [0.5, 0, 0.5]);
xlabel('Eje y (m)','FontSize', 18);
ylabel('Tiempo (s)','FontSize', 18);
set(gca, 'FontSize', 18);  % Tamaño de los números en los ejes
set(gca, 'XScale', 'linear', 'YScale', 'linear', 'ZScale', 'linear');  % Escala lineal en los ejes X, Y, Z
legend('RMD1','RMD2','RMD3','RMD4','RMD5','RMD6','Location', 'northwest')

figure(3);
plot(posx1,posy1,'LineWidth',4,'Color', 'b');
hold on;
grid on;
plot(posx2,posy2,'LineWidth',4,'Color', 'r');
plot(posx3,posy3,'LineWidth',4,'Color', 'g');
plot(posx4,posy4,'LineWidth',4,'Color', [1, 0.5, 0]);
plot(posx5,posy5,'LineWidth',4,'Color', [0.5, 0, 0.5]);
plot(posx6,posy6,'LineWidth',4,'Color', [0.5, 0, 0.5]);
xlabel('Eje X (m)','FontSize', 18);
ylabel('Eje Y (m)','FontSize', 18);
set(gca, 'FontSize', 18);  % Tamaño de los números en los ejes
set(gca, 'XScale', 'linear', 'YScale', 'linear', 'ZScale', 'linear');  % Escala lineal en los ejes X, Y, Z
legend('RMD1','RMD2','RMD3','RMD4','RMD5','RMD6','Location', 'northwest')

figure(4);
plot3(posx1,posy1,tiempo1,'LineWidth',4,'Color', 'b');
hold on;
grid on;
plot3(posx2,posy2,tiempo1,'LineWidth',4,'Color', 'r');
plot3(posx3,posy3,tiempo1,'LineWidth',4,'Color', 'g');
plot3(posx4,posy4,tiempo1,'LineWidth',4,'Color', [1, 0.5, 0]);
plot3(posx5,posy5,tiempo1,'LineWidth',4,'Color', [0.5, 0, 0.5]);
plot3(posx6,posy6,tiempo1,'LineWidth',4,'Color', [0.5, 0, 0.5]);
xlabel('Eje X (m)','FontSize', 18);
ylabel('Eje Y (m)','FontSize', 18);
zlabel('Tiempo (s)','FontSize', 18);
set(gca, 'FontSize', 18);  % Tamaño de los números en los ejes
set(gca, 'XScale', 'linear', 'YScale', 'linear', 'ZScale', 'linear');  % Escala lineal en los ejes X, Y, Z
legend('RMD1','RMD2','RMD3','RMD4','RMD5','RMD6','Location', 'northwest')
