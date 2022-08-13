
%************************** velocidad.m ***************************************

% Esta funcion calcula las velocidades resultantes para cada tramo de la figura 
% a formar. Nosotros hemos asumido que la forma de la velocidad sera en forma 
% trapezoidal por cada tramo; es decir como tenemos 5 tramos entonces tendremos 
% cinco velocidades de forma trapezoidal en todo el recorrido de la figura
%                             *******************
%									*						*
%								   *	 					 *
%	   							  *							  *
%

function V_tramo = Velocidad(d, Velomax,P_rampa, dt)
% Esta funcion recibe como parametros:
% d        : distancia del tramo a recorrer  
% Velomax  : Velocidad maxima que alcanza en cada tramo y el cual permanece constante por cierto tiempo
% P_rampa  : Duracion de la rampa de subida y bajada expresado en '%'del tiempo total en cada tramo 
% dt       : Diferencial de tiempo 

t_fin   = d/((1-P_rampa)*abs(Velomax));
t_rampa = P_rampa*t_fin;      
V_subida    =  (Velomax/t_rampa)*[0:dt:+t_rampa-dt];   
V_constante =  Velomax*ones(1,length([t_rampa:dt:t_fin-t_rampa-dt]));
V_bajada  = -(Velomax/t_rampa)*[t_fin-t_rampa:dt:t_fin]+(Velomax*t_fin/t_rampa); 
V_tramo     =  [ V_subida   V_constante    V_bajada ];
