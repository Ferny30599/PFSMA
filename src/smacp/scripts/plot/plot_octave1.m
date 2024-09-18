ruta=csvread('ruta_robotz.csv');
posx=ruta(:,1);
posy=ruta(:,2);
tiempo=ruta(:,3);
ruta2=csvread('ruta_robot_so.csv');
posx2=ruta2(:,1);
posy2=ruta2(:,2);
tiempo2=ruta2(:,3);
figure(1);
plot(posx,posy,'LineWidth',4,'Color', 'b');
hold on;
plot(posx2,posy2,'LineWidth',4,'LineStyle', '--', 'Color', 'r');
grid on;
xlabel('Eje X (m)','FontSize', 18);
ylabel('Eje Y (m)','FontSize', 18);
set(gca, 'FontSize', 18);  % Tamaño de los números en los ejes
set(gca, 'XScale', 'linear', 'YScale', 'linear', 'ZScale', 'linear');  % Escala lineal en los ejes X, Y, Z
legend('Con obstáculos','Sin obstáculos','Location', 'northwest')
figure(2);
plot3(posx,posy,tiempo,'LineWidth',4,'Color', 'b');
grid on;
hold on;
plot3(posx2,posy2,tiempo2,'LineWidth',4,'LineStyle', '--', 'Color', 'r');
xlabel('Eje X (m)','FontSize', 18);
ylabel('Eje Y (m)','FontSize', 18);
zlabel('Tiempo (s)','FontSize', 18);
legend('Con obstáculos','Sin obstáculos','Location', 'northwest')
set(h, 'FontSize', 18); % Ajusta el tamaño de la fuente de la leyenda
set(gca, 'FontSize', 18);  % Tamaño de los números en los ejes
set(gca, 'XScale', 'linear', 'YScale', 'linear', 'ZScale', 'linear');  % Escala lineal en los ejes X, Y, Z
