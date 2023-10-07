clc 
clear all

% Longitudes de los eslabones
L1 = 3.64;
L2 = 2.2;
L3 = 2.2;
L4 = 0.7;
L5 = 0.7;

L(1)=Link([1 L1 0 pi/2]) ;   %Eslabón 1 en base a los parámetros de DH 
L(2)=Link([1 0 L2 0]);       %Eslabón 2 en base a los parámetros de DH  
L(3)=Link([1 0 L3 0]);       %Eslabón 3 en base a los parámetros de DH 
L(4)=Link([1 0 0 pi/2]);     %Eslabón 4 en base a los parámetros de DH 
L(5)=Link([1 L4+L5 0 0]);    %Eslabón 5 en base a los parámetros de DH 

%Crear el robot en base a los eslabones definidos
RR=SerialLink(L,'name','Scorbot ER-V+');

%Definir coordenadas del espacio de trabajo para la simulación virtual del robot
ws = ([-5 5 -5 5 -1 10]);

% Posición deseada en el espacio cartesiano (coordenadas x, y, z)
xd = [2.2; 2.2; 2.0];

% Ángulos iniciales de las articulaciones (q1, q2, q3, q4)
q = [pi/2; 0; 0; -pi/2];

% Tolerancia para la condición de finalización
epsilon = 1e-3;

% Número máximo de iteraciones
max_iter = 1000;

% Tamaño de paso (tasa de aprendizaje) para el descenso de gradiente
alpha = 0.3;

% Iteraciones: Descenso de gradiente
for i = 1:max_iter
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4);
    J = zeros(3, 4);

    % Derivadas parciales de X con respecto a q1, q2, q3 y q4
    J(1, 1) = (L3 * cos(q1) * cos(q2) * sin(q3)) - (cos(q1) * cos(q2) * cos(q3) * (L4 + L5)) - (sin(q1) * sin(q2) * sin(q4) * (L4 + L5)) + (cos(q4) * (cos(q1) * sin(q2) - cos(q2) * cos(q3) * sin(q1)) * (L4 + L5)) - (sin(q3) * (sin(q1) - cos(q1) * sin(q2)) * sin(q4) * (L4 + L5));
    J(1, 2) = -(L3 * sin(q2) * sin(q3) * sin(q1)) - (cos(q2) * cos(q3) * sin(q1) * (L4 + L5)) - (cos(q4) * (sin(q1) + cos(q3) * (sin(q1) - cos(q1) * sin(q2)) - cos(q1) * cos(q2) * sin(q3)) * (L4 + L5)) - (sin(q4) * (sin(q3) * (sin(q1) - cos(q1) * sin(q2)) + cos(q1) * cos(q2) * cos(q3)) * (L4 + L5));
    J(1, 3) = -(L3 * cos(q1) * sin(q2) * cos(q3)) - (cos(q1) * cos(q2) * sin(q3) * (L4 + L5)) - (sin(q3) * sin(q1) * sin(q4) * (L4 + L5)) + (cos(q4) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * (sin(q1) - cos(q1) * sin(q2))) * (L4 + L5)) + (cos(q1) * sin(q2) * sin(q3) * sin(q4) * (L4 + L5));
    J(1, 4) = -(cos(q4) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * (sin(q1) - cos(q1) * sin(q2))) * (L4 + L5)) - (sin(q4) * (cos(q1) * sin(q2) - cos(q2) * cos(q3) * sin(q1)) * (L4 + L5));

    % Derivadas parciales de Y con respecto a q1, q2, q3 y q4
    J(2, 1) = (L3 * cos(q1) * sin(q2) * sin(q3)) - (cos(q1) * cos(q2) * sin(q3) * (L4 + L5)) + (sin(q3) * sin(q1) * sin(q4) * (L4 + L5)) - (cos(q4) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * (sin(q1) - cos(q1) * sin(q2))) * (L4 + L5)) - (cos(q1) * sin(q2) * sin(q3) * sin(q4) * (L4 + L5));
    J(2, 2) = (L3 * cos(q2) * sin(q1) * sin(q3)) + (sin(q2) * sin(q4) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * (sin(q1) - cos(q1) * sin(q2))) * (L4 + L5)) + (cos(q4) * (sin(q1) + cos(q3) * (sin(q1) - cos(q1) * sin(q2)) - cos(q1) * cos(q2) * sin(q3)) * (L4 + L5));
    J(2, 3) = -(L3 * cos(q2) * cos(q3) * sin(q1)) - (cos(q2) * sin(q1) * sin(q4) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * (sin(q1) - cos(q1) * sin(q2))) * (L4 + L5)) + (cos(q4) * (cos(q1) * cos(q2) * cos(q3) * (sin(q1) - cos(q1) * sin(q2)) - sin(q1) * sin(q3)) * (L4 + L5));
    J(2, 4) = (cos(q4) * (cos(q1) * cos(q2) * cos(q3) * (sin(q1) - cos(q1) * sin(q2)) - sin(q1) * sin(q3)) * (L4 + L5)) - (sin(q4) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * (sin(q1) - cos(q1) * sin(q2))) * (L4 + L5));

    % Derivadas parciales de Z con respecto a q1, q2, q3 y q4
    J(3, 1) = 0; 
    J(3, 2) = -(L3 * cos(q2) * cos(q3)) + (sin(q2) * sin(q4) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * (sin(q1) - cos(q1) * sin(q2))) * (L4 + L5)) + (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) * (L4 + L5));
    J(3, 3) = -(cos(q2) * sin(q3) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * (sin(q1) - cos(q1) * sin(q2))) * (L4 + L5)) - (sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) * (L4 + L5));
    J(3, 4) = (sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2)) * (L4 + L5)) - (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) * (L4 + L5));
    
    %Coordenadas X, Y, Z del efector según matricez DH
    X = (sin(q4) * (sin(q3) * (sin(q1) - cos(q1) * sin(q2)) + cos(q1) * cos(q2) * cos(q3)) - cos(q4) * (sin(q1) + cos(q3) * (sin(q1) - cos(q1) * sin(q2)) - cos(q1) * cos(q2) * sin(q3))) * (L4 + L5) + L2 * cos(q1) * cos(q2) + L3 * sin(q3) * (sin(q1) - cos(q1) * sin(q2)) + L3 * cos(q1) * cos(q2) * cos(q3);
    Y = L2 * cos(q2) * sin(q1) - (sin(q4) * (sin(q3) * (cos(q1) + sin(q1) * sin(q2)) - cos(q2) * cos(q3) * sin(q1)) - cos(q4) * (cos(q1) + cos(q3) * (cos(q1) + sin(q1) * sin(q2)) + cos(q2) * sin(q1) * sin(q3))) * (L4 + L5) - L3 * sin(q3) * (cos(q1) + sin(q1) * sin(q2)) + L3 * cos(q2) * cos(q3) * sin(q1);
    Z = L1 - (cos(q4) * (cos(q2) * cos(q3) - sin(q2) * sin(q3)) - sin(q4) * (cos(q2) * sin(q3) + cos(q3) * sin(q2))) * (L4 + L5) + L2 * sin(q2) + L3 * cos(q2) * sin(q3) + L3 * cos(q3) * sin(q2);

    f = [X; Y; Z];
    e = xd - f;
    q = q + alpha * (J' * e);
    
    % Condición de finalización
    if (norm(e) < epsilon)
        break;
    end
end

% Imprimir los ángulos
disp('Ángulos:');
disp(q);

disp('Comprobación (f(q)):');
disp(f);

%Graficamos la coordenada deseada como una esfera amarilla
plot_sphere(xd, 0.25, 'y')  

%Evaluamos los angulos obtenidos y graficamos
RR.teach([q(1)-pi/4,q(2),q(3),q(4),0], 'workspace', ws, 'noname');


