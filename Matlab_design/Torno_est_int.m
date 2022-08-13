%******************************** Torno.m *****************************

% Este programa realiza el control del sistema de control numerico

clc, clear all; close all;

% Parametros tenicos del motor que realiza trayectoria en X 
% ---------------------------------------------------------
R_X    = 2;		   % resistencia interna (ohm)
Kb_X   = 0.025;   	% coeficiente de Fuerza Electromotriz (volt-sec/rad)
Kt_X   = 0.0205;  	% coeficiente torque corriente (N-m/Amp)
r_X    = 0.02/2;		% radio deltornillo sinfin (m)
p_X    = 0.004;		% paso del tornillo sinfin (m)
Ks_X   = 0.9;       % constante de fuerzas del tornillo sin fin
m_X    = 0.125;	   	% masa x (kg)
M_X    = 0.22;	 	% masa del motor x y tornillo sinfin (kg)
I_X    = 0.0000435;  % inercia total del motor x y tornillo sinfin (kg-m2)
Fric_X = 0.2;        % friccion viscosa (N-seg/m)

% -----------------------------------------------------------------------
Pmax_X = 400;	    % potencia maxima del motor x (W)
Vmax_X = 40;	    	% maximo voltaje aplicable (V)
 
% Parametros tenicos del motor que realiza trayectoria en Y 
% -----------------------------------------------------------------------
R_Y    = 30;			% resistencia interna (ohm)
Kb_Y   = 0.021;		% coeficiente de Fuerza Electromotriz (volt-sec/rad)
Kt_Y   = 0.032;		% coeficiente torque corriente (N-m/Amp)
r_Y    = 0.04/2;    	% radio del tornillo sinfin (m)
p_Y    = 0.004;		% paso del tornillo sinfin (m)
Ks_Y   = 0.95;      % constante de fuerzas del tornillo sin fin
m_Y    = 0.35;	   % masa y (kg)
M_Y    = 0.55;		% masa del motor y y tornillo sinfin (kg)
I_Y    = 0.0000855;	% inercia total del motor y y tornillo sinfin (kg-m2)
Fric_Y = 0.25;      % friccion viscosa (N-seg/m)
% -------------------------------------------------------------------------
Pmax_Y = 600;	   	% potencia maxima del motor y (W)
Vmax_Y = 60;	       % maximo voltaje aplicable (V)

%*********** Ecuaciones de estado del sistema del motor X  ************** 
% -------------------------------------------------------------------------
% Para que las matrices A y B sean mas comprensible, se reasignan valores
% a las variables estandar :

R=R_X; Kb=Kb_X; Kt=Kt_X; r=r_X; p=p_X; Ks=Ks_X; m=m_X; I=I_X; Fric=Fric_X;

% aqui solo se considera la masa m_X del motor

a22 = -2*pi*Kt*Kb/(p*R*(2*pi*I/p + m*r*Ks));
b21 =   Kt/(R*(2*pi*I/p + m*r*Ks)); 

w21 =  -Ks*r/(2*pi*I/p + m*r*Ks); 

A_X = [ 0      1;    0    a22 ];

B_X = [  0 ;  b21];

Wf_X =[ 0 ;   w21 ];

C_X = [ 1 0 ]; 
D_X = 0;

% Ecuacion para el calculo de la corriente del motor X
a_x = -2*pi*Kb/(p*R);
b_x = 1/R;

% Matrices del nuevo sistema con accion integral

Ai_X = [ 0      1   0 ;  0    a22   0;   1      0   0 ];

Bi_X = [ 0
         b21        
         0 ];

% **********  Ecuaciones de estado del sistema del motor Y **************
% -----------------------------------------------------------------------

% Para que las matrices A y B sean mas comprensible, se reasignan valores
% a las variables estandar :

R=R_Y; Kb=Kb_Y; Kt=Kt_Y; r=r_Y; p=p_Y; Ks=Ks_Y; m=m_Y+m_X+M_X; I=I_Y; Fric=Fric_Y;

a22 = -2*pi*Kt*Kb/(p*R*(2*pi*I/p + m*r*Ks));
b21 =   Kt/(R*(2*pi*I/p + m*r*Ks)); 
w21 =  -Ks*r/(2*pi*I/p + m*r*Ks) ;

A_Y = [ 0     1;   0     a22];

B_Y = [ 0 ;  b21];

Wf_Y = [ 0;  w21];

C_Y = [ 1  0 ];

D_Y = 0;

% Ecuacion para el calculo de la corriente del motor Y
a_y = -2*pi*Kb/(p*R);
b_y = 1/R;

% Matrices del nuevo sistema con accion integral

Ai_Y = [ 0      1   0  ;   0    a22   0;    1      0   0 ];

Bi_Y = [ 0
         b21        
         0 ];
 %*************** Aqui comienza el control **************************

trayec;					%llamada a la funcion que me genera mis 
							%trayectorias a partir de la velocidad dada

load curvas;				%cargo los valores: Vel_total ruta R t dt;
							%obtenidos en la funcion 'trayec'



                            
% Discretización de la matriz aumentada con acción integral y la matriz normal 
% ----------------------------------------------------------------------------

%Matrices aumentadas discreta
A_X_I=[A_X(1,:) 0;A_X(2,:) 0; 1 0 0];
B_X_I=[B_X; 0];

A_Y_I=[A_Y(1,:) 0;A_Y(2,:) 0; 1 0 0];
B_Y_I=[B_Y; 0];

[Ax_d_I,Bx_d_I]=c2d(A_X_I,B_X_I,dt);
[Ay_d_I,By_d_I]=c2d(A_Y_I,B_Y_I,dt);

[Ax_d,Bx_d]=c2d(A_X,B_X,dt);
[Ax_d,Wfx_d]=c2d(A_X,Wf_X,dt);
Cx_d=[1 0];

[Ay_d,By_d]=c2d(A_Y,B_Y,dt);
[Ay_d,Wfy_d]=c2d(A_Y,Wf_Y,dt);
Cy_d=[1 0];

% Determinación de las ganacias para la matriz aumentada 
% ------------------------------------------------------

%Recomendaciones
%wn*T=0.2-0.6
%T=dt;->wn=[10-30]
%phi=[0.5-0.866]
phi=0.5; %esfuerzo de control deseado max 15*10^-5
wn=5;
M=exp(-(phi*pi/((1-phi^2)^0.5)))*100;
Ts=4/(wn*phi);
s1=-1*phi*wn+wn*sqrt(1-phi^2)*(0+j);
s2=-1*phi*wn-wn*sqrt(1-phi^2)*(0+j);
s3=-10*phi*wn;
z1=exp(s1*dt);
z2=exp(s2*dt);
z3=exp(s3*dt);

s1_o=2*s1; %Poner polos reales y que sean iguales
s2_o=2*s1;
z1_o=exp(s1_o*dt);
z2_o=exp(s2_o*dt);

Kix= acker(Ax_d_I,Bx_d_I,[z1 z2 z3]); 
Kix_d=Kix(1,1:2); % Ganancia delos estados del motor x
Kix_I=Kix(1,3);   %Ganancia de la integral del error del motor x

Lx=acker(Ax_d',Cx_d',[z1_o z2_o])';%Ganancia del obsevador del motor x

Kiy = acker(Ay_d_I,By_d_I,[z1 z2 z3]);
Kiy_d=Kiy(1,1:2);    % Ganancia delos estados del motor y
Kiy_I=Kiy(1,3);   %Ganancia de la integral del error del motor y

Ly=acker(Ay_d',Cy_d',[z1_o z2_o])';  %Ganancia del obsevador del motor y


% Condiciones Iniciales para simulacion
% -------------------------------------
x(:,1) = [0 0]';    %condiciones iniciales  para la velocidad y posicion en X
y(:,1) = [0 0]';    %condiciones iniciales  para la velocidad y posicion en Y
errint_x=0;		    %condiciones iniciales para el error
errint_y=0;		    %condiciones iniciales para el error


% En la matriz ruta() se encuentran las referencias constantes por tramos
% ----------------------------------------------------------------------
%DECLARACION DE VARIABLES (referencia para el codigo en C)

%Calculo de señal de control para motor x
Kix11=Kix(1,1); %841.7093
Kix12=Kix(1,2); %130.6615
Kix13=Kix(1,3); %3.3561e+03
errint_x=0;
ux=0;
ref_x=0;

%Datos sensados del motor x
xx_actual=0; 
xv_actual=0;

%Calculo de señal de control para motor y
Kiy11=Kiy(1,1); %1.6271e+04
Kiy12=Kiy(1,2); %3.2524e+03
Kiy13=Kiy(1,3); %6.4826e+04
errint_y=0;
uy=0;
ref_y=0;

%Datos sensados del motor y
yx_actual=0;
yv_actual=0;

%Simulacion/Dinamica motor x
Fric_X=Fric_X+0; %0.2
Ffx = 0;
xx_futuro=0; 
xv_futuro=0;
Ax_d11=Ax_d(1,1); %1
Ax_d21=Ax_d(2,1); %0
Ax_d12=Ax_d(1,2); %0.0189
Ax_d22=Ax_d(2,2); %0.8906
Bx_d11=Bx_d(1,1); %2.8408e-05
Bx_d21=Bx_d(2,1); %0.0028
Wfx_d11=Wfx_d(1,1); %-2.4943e-05
Wfx_d21=Wfx_d(2,1); %-0.0024

%Simulacion/Dinamica motor y
Fric_Y=Fric_Y+0; %0.25
Ffy = 0; 
yx_futuro=0; 
yv_futuro=0;
Ay_d11=Ay_d(1,1); %1
Ay_d21=Ay_d(2,1); %0
Ay_d12=Ay_d(1,2); %0.02
Ay_d22=Ay_d(2,2); %0.9952
By_d11=By_d(1,1); %1.4440e-06
By_d21=By_d(2,1); %1.4428e-04
Wfy_d11=Wfy_d(1,1); %-2.5720e-05
Wfy_d21=Wfy_d(2,1); %-0.0026

for i=1:length(t)
     %CONTROL X
     xx_actual=x(1,i); %Lectura de sensor posicion
     xv_actual=x(2,i); %Lectura de sensor velocidad
     ref_x=ruta(1,i); %Punto de referencia deseada (se actualiza cada dt)
     errint_x = errint_x + (xx_actual - ref_x)*dt;	 %error de posicion X
     ux = -Kix11*xx_actual-Kix12*xv_actual-Kix13*errint_x; %Calculo de señal de control
     
     %CONTROL Y
     yx_actual=y(1,i); %Lectura de sensor
     yv_actual=y(2,i); %Lectura de sensor
     ref_y=ruta(2,i);
     errint_y = errint_y + (yx_actual - ref_y)*dt;	 %error de posicion Y
     uy = -Kiy11*yx_actual-Kiy12*yv_actual-Kiy13*errint_y; %Calculo de señal de control
   
   volt_x(1,i)=ux;
   Ffx = Fric_X*xv_actual;
   
   volt_y(1,i)=uy;
   Ffy = Fric_Y*yv_actual;	
   
   if i==length(t)
   break;
   end
   
   %Simulador/Dinámica x
   
   volt_x(1,i)=ux;        	  			%almaceno señal de control (voltaje)
   %Ffx = Fric_X*x(2,i);					%constante friccion por la velocidad
   Ffx = Fric_X*xv_actual;  %fricción actual
   %x(:,i+1) = Ax_d*x(:,i) + Bx_d*ux + Wfx_d*Ffx; %se almacena los valores de:  posicion y velocidad en X
   xx_futuro=Ax_d11*xx_actual+Ax_d12*xv_actual+Bx_d11*ux+Wfx_d11*Ffx;
   xv_futuro=Ax_d21*xx_actual+Ax_d22*xv_actual+Bx_d21*ux+Wfx_d21*Ffx;
   x(1,i+1)=xx_futuro;
   x(2,i+1)=xv_futuro;
   
   %Simulador/Dinámica y
   volt_y(1,i)=uy;   %almaceno señal de control (voltaje)
   %Ffy = Fric_Y*y(2,i); 					%constante friccion por la velocidad
   Ffy = Fric_Y*yv_actual;	
   %y(:,i+1) = Ay_d*y(:,i) + By_d*uy + Wfy_d*Ffy; %se almacena los valores de:  posicion y velocidad en Y
   yx_futuro=Ay_d11*yx_actual+Ay_d12*yv_actual+By_d11*uy+Wfy_d11*Ffy;
   yv_futuro=Ay_d21*yx_actual+Ay_d22*yv_actual+By_d21*uy+Wfy_d21*Ffy;
   y(1,i+1)=yx_futuro;
   y(2,i+1)=yv_futuro;
   
end

% Calculo de la Corriente de los motores
ix = a_x*x(2,:) + b_x*volt_x;
iy = a_y*y(2,:) + b_y*volt_y;

% Calculo de la Potencia consumida en los motores
Potx = ix.*volt_x;
Poty = iy.*volt_y;
figure;
plot(t,x(1,:),t,ruta(1,:),':r');        %trayectoria real vs  trayectoria de referencia
xlabel('t [seg]'); ylabel('x [mt]');
title('Posicion x');grid;
legend('Posicion obtenida','Referencia'); 

figure;
plot(t,x(2,:));							  %velocidad X
xlabel('t [seg]'); ylabel('dx/dt [mt/seg]'); 
title('Velocidad x'); grid;

%-----------------------------------------
figure;
plot(t,y(1,:),t,ruta(2,:),':r');         %trayectoria real vs trayectoria de refernecia 
xlabel('t [seg]'); ylabel('y [mt]');
title('Posicion y'); grid;
legend('Posicion obtenida','Referencia'); 

figure;
plot(t,y(2,:));								%velocidad Y
xlabel('t [seg]');  ylabel('dy/dt [mt/seg]');
title('Velocidad y'); grid;

figure;
plot(t,volt_x,t,volt_y,':b');
xlabel('t [seg]'); ylabel('u [voltios]');
title('Esfuerzo de control'); grid; 
legend('Voltaje en X','Voltaje en Y'); 

figure;
plot(x(1,:),y(1,:));
title('Forma generada con el control'); 
xlabel('x [mt]'); ylabel('y [mt]');grid;

figure;
plot(t,Potx,':b');
hold on;
plot(t,Poty);
xlabel('t [seg]'); ylabel('potencia]');
title('Potencias instantaneas'); 
legend('Potencia Px','Potecia Py'); 
grid;
hold off

var=ruta(1,i);
fileID = fopen('rutaxy2.txt','w');
%fprintf(fileID,'%6s %12s\n','x','exp(x)');
for c=1:length(var)
    fprintf(fileID,'%4.6f \n',var(c));
end
fclose(fileID);

