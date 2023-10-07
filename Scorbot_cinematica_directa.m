clc 
clear all

%Definir constantes del robot (Largo de eslabones)
E1 = 3.64;
E2 = 2.2;
E3 = 2.2;
E4 = 0.7;
E5 = 0.7;

L(1)=Link([1 E1 0 pi/2]) ;   %Eslabón 1 en base a los parámetros de DH 
L(2)=Link([1 0 E2 0]);       %Eslabón 2 en base a los parámetros de DH  
L(3)=Link([1 0 E3 0]);       %Eslabón 3 en base a los parámetros de DH 
L(4)=Link([1 0 0 pi/2]);     %Eslabón 4 en base a los parámetros de DH 
L(5)=Link([1 E4+E5 0 0]);    %Eslabón 5 en base a los parámetros de DH 

%Crear el robot en base a los eslabones definidos
RR=SerialLink(L,'name','Scorbot ER-V+');
 
%Definir coordenadas del espacio de trabajo para la simulación virtual del robot
ws = ([-5 5 -5 5 -1 10]);

%Definir posición home del SCORBOT -ER V+
home = [0, 120*pi/180, -90*pi/180, -6*pi/180, 0];

%Abrir la simulación con el modo para modificar los angulos, además el robot inicia en la posición home 
RR.teach(home, 'workspace', ws, 'noname');

