ruta=csvread('ruta_robot_so2.csv');
posx=ruta(:,1);
posy=ruta(:,2);
posdx=ruta(:,3);
posdy=ruta(:,4);
tiempo=ruta(:,5);

figure(1);
%plot3(posx,posy,tiempo,'LineWidth',4,'Color', 'b');
plot(posx,posy,'LineWidth',4,'Color', 'b');
hold on;
%plot3(posdx,posdy,tiempo,'LineWidth',4,'LineStyle', '--', 'Color', 'r');
plot(posdx,posdy,'LineWidth',4,'LineStyle', '--', 'Color', 'r');
grid on;
xlabel('Eje X (m)','FontSize', 18);
ylabel('Eje Y (m)','FontSize', 18);
%zlabel('Tiempo (s)','FontSize', 18);
set(gca, 'FontSize', 18);  % Tamaño de los números en los ejes
set(gca, 'XScale', 'linear', 'YScale', 'linear', 'ZScale', 'linear');  % Escala lineal en los ejes X, Y, Z
%legend('Trayectoria realizada','Trayectoria deseada','Location', 'northwest')