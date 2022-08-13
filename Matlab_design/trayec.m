
%************************** trayec.m ************************************

% En este programa calculo las velocidades correspondientes al eje X
% y al eje Y (Vx, Vy) a partir de ingresar la velocidad maxima resultante
%
%************************************************************************

function trayectoria = trayec()

% Ingreso de valores de velocidad
% -------------------------------
disp('   '); disp('   '); disp('   '); 

%V    =  input('Ingrese velocidad resultante   (m/s) : ');
%ramp =  input('Ingrese el porcentaje de rampa   (%) : ');
%dt   = input('Ingrese diferencial de tiempo    (ms) : ');
disp('   '); disp('   '); 

V = 0.001;  %V = 0.004;       %rango de velocidades recomendadas
dt = 0.02;
ramp =0.2;

% Coordenadas de la trayectoria
% -----------------------------
a = [0,0]; 
b = [2,0];
c = [3,2];
d = [1,2];
e = [1,1];
f = [0,0];

punto_ini= a;              % punto de partida de la trayectoria
R =0.01*[a;b;c;d;e;f];     % coordenadas en metros





% Calculo de las distancias a rrecorrer en cada tramo
% ---------------------------------------------------
d1=abs(R(2,1)-R(1,1));                           %longitud tramo a-b
d2 =sqrt((R(3,1)-R(2,1)).^2+(R(3,2)-R(2,2)).^2); %longitud tramo b-c
d3=abs(R(4,1)-R(3,1));						    		 %longitud tramo c-d	
d4=abs(R(5,2)-R(4,2));							      %longitud tramo d-e
d5=sqrt((R(6,1)-R(5,1)).^2+(R(6,2)-R(5,2)).^2);  %longitud tramo e-f

ceros = zeros(2,2);
sizeceros = size(ceros,2);

% Calculo de las componentes de las velocidades Vx, Vy
% -----------------------------------------------------

% Evaluacion de la velocidad Vx y Vy en el tramo a-b
V1 = Velocidad(d1, V, ramp,dt);  %Velocidad resultante en tramo a-b
m=0;								    %pendiente en tramo a-b	
Vab(1,:) = V1;                   %Velocidad Vx en tramo a-b
Vab(2,:) = m.*Vab(1,:);          %Velocidad Vy en tramo a-b

% Evaluacion de la velocidad Vx y Vy en el tramo b-c
V2 = Velocidad(d2, V, ramp,dt); %Velocidad resultante en tramo b-c
m=2;     						  	   %pendiente en tramo b-c
Vbc(1,:) = V2./sqrt(1+m.^2);    %Velocidad Vx en tramo b-c
Vbc(2,:) =m.*Vbc(1,:);          %Velocidad Vx en tramo b-c

% Evaluacion de la velocidad Vx y Vy en el tramo c-d
V3 = Velocidad(d3, -V,ramp, dt); %Velocidad resultante en tramo c-d
m=0;                             %pendiente en tramo c-d 
Vcd(1,:) = V3;                   %Velocidad Vx en tramo c-d
Vcd(2,:) = m.*Vcd(1,:);          %Velocidad Vy en tramo c-d

% Evaluacion de la velocidad Vx y Vy en el tramo d-e
V4 = Velocidad(d4, -V, ramp,dt); %Velocidad resultante en tramo d-e
m=0;                             %pendiente en tramo d-e
Vde(2,:) = V4;                   %Velocidad Vx en tramo d-e
Vde(1,:) = m.*Vde(2,:);          %Velocidad Vy en tramo d-e

% Evaluacion de la velocidad Vx y Vy en el tramo e-f

V5 = Velocidad(d5, -V, ramp,dt);  %Velocidad resultante en tramo e-f
m=1;                              %pendiente en tramo e-f
Vef(1,:) = V5./sqrt(1+m.^2);      %Velocidad Vx en tramo e-f
Vef(2,:) =m.* Vef(1,:);           %Velocidad Vy en tramo e-f

Vel_total = [Vab Vbc Vcd Vde Vef];  %velocidad total

% Reconstruyo la trayectoria a partir de las velocidades
% ------------------------------------------------------
ruta(:,1)=punto_ini';                   %inicio punto de partida
for i=1:length(Vel_total)-1
   ruta(:,i+1)=ruta(:,i)+dt.*Vel_total(:,i+1);  %integro la velocidad
end

Vel_total = [Vel_total ceros];
ruta = [ruta ceros];

% Determinacion del vector de tiempos 
% ------------------------------------
t = 0:dt:(length(Vel_total)-1)*dt;
sizet = length(t);


% Ploteo de Graficos
% -----------------
figure(1);
plot(t,Vel_total(1,:));         %ploteo Vx
xlabel('t[seg]');
ylabel('V_x [mt/seg]');
title('Velocidad en el EJE X'); grid;

figure(2);
plot(t,Vel_total(2,:));          %ploteo Vy
xlabel('t[seg]');
ylabel('V_y [mt/seg]');
title('Velocidad en el EJE Y'); grid;

figure(3);
plot(t,ruta(1,:),'b');				%ploteo X reconstruido
xlabel('t[seg]');
ylabel('[mt]');
title('Posicion en el EJE X');
legend('Referencia en X para mi Control');grid;

figure(4);
plot(t,ruta(2,:));					%ploteo Y reconstruido
xlabel('t[seg]');
ylabel('[mt]');
title('Posicion en el EJE Y');
legend('Referencia  en Y para mi control');grid;

figure(5);
plot(ruta(1,:),ruta(2,:));     %ploteo curva reconstruida X-Y
xlabel('t[seg]');
ylabel('[mt]');
title('CURVA RECONSTRUIDA A PARTIR DE LAS VELOCIDADES');grid;

save curvas Vel_total ruta R t dt;
