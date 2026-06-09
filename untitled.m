clear all
clc

%model liniarizat
% definirea variabilelor
x1 = sym('x1','real'); % pozitie
x2 = sym('x2','real'); % viteza 
x3 = sym('x3','real'); % acceleratie
x4 = sym('x4','real'); % presiunea diferentiala Pd = Pa - Pb
uv = sym('uv','real'); % comanda valva

% parametri
M = 10.8; 
Bv = 13.1; 
L0 = 0.1635; 
b = 1.04; 
Kg = 1.035;
gamma = 1.4; 
R = 287; 
T = 293; 
RT = R*T;
Patm = 101325; 
Ps = 600000; 
cvA = 0.024; 
Kv = 1e-6; 

% P de echilibru
Pa_eq = 300000;
Pb_eq = 300000;

% geometrie si volumul
La = L0 + x1; 
Lb = L0 - x1;
Va = (La * (b^2 - La^2)) / Kg;
Vb = (Lb * (b^2 - Lb^2)) / Kg;
dVadx = (b^2 - 3*La^2) / Kg;
dVbdx = -(b^2 - 3*Lb^2) / Kg;

% ecuatiile prin orficiu
qA = cvA * uv * sqrt((Ps - Pa_eq)/1e5); % ca sa transform pascal in bari
qB = -cvA * uv * sqrt((Pb_eq - Patm)/1e5);

% dinamica presiunilor 
dPa = (gamma * RT * qA) / Va - (gamma * Pa_eq / Va) * dVadx * x2;
dPb = (gamma * RT * qB) / Vb - (gamma * Pb_eq / Vb) * dVbdx * x2;

% dinamica starii
dPd = dPa - dPb;

% dinamica miscarii - derivata acceleratiei
dacc_mecanic = (1/M) * (x4 * dVadx - Bv * x3); 

% vectorul de stare
dx1 = x2;
dx2 = x3;
dx3 = dacc_mecanic;
dx4 = dPd;
x = [x1; x2; x3; x4];
F = [dx1; dx2; dx3; dx4];

% liniarizare automata -pt derivate partiale
A_sym = jacobian(F, x);
B_sym = jacobian(F, uv);

% evaluare in punctul de echilibru
x1 = 0; 
x2 = 0; 
x3 = 0; 
x4 = 0; 
uv = 0;      
A = double(subs(A_sym));
B = double(subs(B_sym));

A = real(A);
B = real(B);

disp('matricea A liniarizata:');
disp(vpa(A, 5));
disp('matricea B liniarizata:');
disp(vpa(B, 5));

%  controlabilitate
Co = ctrb(A, B);
fprintf('rangul matricei de co: %d\n', rank(Co));

% controlere
C = [1 0 0 0];
D = 0;

poli = [-10 -15 -20 -25]; 
k = place(A, B, poli);

disp(' K:');
disp(k);

N = inv(C * inv(-A + B*k) * B);

% filtrare finala
k = real(k);
N = real(N);

fprintf('factorul de scalare N: %f\n', N);