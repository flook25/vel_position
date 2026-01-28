clear all
close all
clc

% ==========================================
% SECTION 1: PARAMETERS & INPUT
% ==========================================
L1 = 0.210; % Ground (Pink)
L2_Loop1 = 0.180; % Green
L3_Loop1 = 0.180; % Yellow
L4_Shared = 0.118; % Grey

L2_Loop2 = 0.118; % Cyan
L3_Loop2 = 0.210; % Red

L2_Loop3 = 0.118; % Cyan (Extended)
L3_Loop3 = 0.210; % Blue
L4_Loop3 = 0.118; % Brown

d = L1;
offset_deg = 0.81;
offset = deg2rad(offset_deg);

q3d_global = 19.94; 
q3 = deg2rad(q3d_global) - offset;

w_Yellow = -2.2; % Input Angular Velocity
alpha_Yellow = -0.8; % Input Angular Acceleration

% ==========================================
% SECTION 2: POSITION ANALYSIS
% ==========================================

% --- LOOP 1 ---
a = L2_Loop1; b = L3_Loop1; c = L4_Shared;
K1=d/b; K2=d/c; K3=(b^2-a^2+c^2+d^2)/(2*b*c); K4=d/a; K5=(c^2-d^2-b^2-a^2)/(2*b*a);
A=cos(q3)-K1-K2*cos(q3)+K3; B=-2*sin(q3); C=K1-(K2+1)*cos(q3)+K3;
D=cos(q3)-K1+K4*cos(q3)+K5; E=-2*sin(q3); F=K1+(K4-1)*cos(q3)+K5;

q4_L1_open = 2*atan((-B - sqrt(B^2 - 4*A*C))/(2*A));
q2_L1_open = 2*atan((-E - sqrt(E^2 - 4*D*F))/(2*D));
q4_L1_cross = 2*atan((-B + sqrt(B^2 - 4*A*C))/(2*A));
q2_L1_cross = 2*atan((-E + sqrt(E^2 - 4*D*F))/(2*D));

% --- LOOP 2 ---
a = L2_Loop2; b = L3_Loop2; c = L4_Shared;
K1_L2=d/c; K2_L2=d/a; K3_L2=(c^2-b^2+a^2+d^2)/(2*c*a); K4_L2=d/b; K5_L2=(a^2-d^2-c^2-b^2)/(2*c*b);

% Case 1 (Open)
q_in_1 = q4_L1_open;
A2_1=cos(q_in_1)-K1_L2-K2_L2*cos(q_in_1)+K3_L2; B2_1=-2*sin(q_in_1); C2_1=K1_L2-(K2_L2+1)*cos(q_in_1)+K3_L2;
D2_1=cos(q_in_1)-K1_L2+K4_L2*cos(q_in_1)+K5_L2; E2_1=-2*sin(q_in_1); F2_1=K1_L2+(K4_L2-1)*cos(q_in_1)+K5_L2;
q2_Cyan_1 = 2*atan((-B2_1 + sqrt(B2_1^2 - 4*A2_1*C2_1))/(2*A2_1)); 
q3_Red_1 = 2*atan((-E2_1 + sqrt(E2_1^2 - 4*D2_1*F2_1))/(2*D2_1));

% Case 2 (Crossed)
q_in_2 = q4_L1_cross;
A2_2=cos(q_in_2)-K1_L2-K2_L2*cos(q_in_2)+K3_L2; B2_2=-2*sin(q_in_2); C2_2=K1_L2-(K2_L2+1)*cos(q_in_2)+K3_L2;
D2_2=cos(q_in_2)-K1_L2+K4_L2*cos(q_in_2)+K5_L2; E2_2=-2*sin(q_in_2); F2_2=K1_L2+(K4_L2-1)*cos(q_in_2)+K5_L2;
q2_Cyan_2 = 2*atan((-B2_2 + sqrt(B2_2^2 - 4*A2_2*C2_2))/(2*A2_2)); 
q3_Red_2 = 2*atan((-E2_2 + sqrt(E2_2^2 - 4*D2_2*F2_2))/(2*D2_2));

% --- LOOP 3 ---
a = L2_Loop3; b = L3_Loop3; c = L4_Loop3;
K1_L3=d/a; K2_L3=d/c; K3_L3=(a^2-b^2+c^2+d^2)/(2*a*c); K4_L3=d/b; K5_L3=(c^2-d^2-a^2-b^2)/(2*a*b);

% Case 1 (Open)
q_in_3_1 = q2_Cyan_1 + pi;
A3_1=cos(q_in_3_1)-K1_L3-K2_L3*cos(q_in_3_1)+K3_L3; B3_1=-2*sin(q_in_3_1); C3_1=K1_L3-(K2_L3+1)*cos(q_in_3_1)+K3_L3;
D3_1=cos(q_in_3_1)-K1_L3+K4_L3*cos(q_in_3_1)+K5_L3; E3_1=-2*sin(q_in_3_1); F3_1=K1_L3+(K4_L3-1)*cos(q_in_3_1)+K5_L3;
q4_Brown_1 = 2*atan((-B3_1 + sqrt(B3_1^2 - 4*A3_1*C3_1))/(2*A3_1));
q3_Blue_1 = 2*atan((-E3_1 + sqrt(E3_1^2 - 4*D3_1*F3_1))/(2*D3_1));

% Case 2 (Crossed)
q_in_3_2 = q2_Cyan_2 + pi;
A3_2=cos(q_in_3_2)-K1_L3-K2_L3*cos(q_in_3_2)+K3_L3; B3_2=-2*sin(q_in_3_2); C3_2=K1_L3-(K2_L3+1)*cos(q_in_3_2)+K3_L3;
D3_2=cos(q_in_3_2)-K1_L3+K4_L3*cos(q_in_3_2)+K5_L3; E3_2=-2*sin(q_in_3_2); F3_2=K1_L3+(K4_L3-1)*cos(q_in_3_2)+K5_L3;
q4_Brown_2 = 2*atan((-B3_2 + sqrt(B3_2^2 - 4*A3_2*C3_2))/(2*A3_2));
q3_Blue_2 = 2*atan((-E3_2 + sqrt(E3_2^2 - 4*D3_2*F3_2))/(2*D3_2));

% --- DISPLAY ANGLES ---
disp('======================================');
disp('       POSITION RESULTS (Degrees)');
disp('======================================');
% (Display Code Skipped for brevity, same as before)

% ==========================================
% SECTION 3: VELOCITY ANALYSIS
% ==========================================

% --- Loop 1 ---
a=L2_Loop1; b=L3_Loop1; c=L4_Shared;
% Case 1
w_Grey_1 = (b * w_Yellow * sin(q3 - q2_L1_open)) / (c * sin(q4_L1_open - q2_L1_open));
w_Green_1 = (b * w_Yellow * sin(q4_L1_open - q3)) / (a * sin(q4_L1_open - q2_L1_open));
% Case 2
w_Grey_2 = (b * w_Yellow * sin(q3 - q2_L1_cross)) / (c * sin(q4_L1_cross - q2_L1_cross));
w_Green_2 = (b * w_Yellow * sin(q4_L1_cross - q3)) / (a * sin(q4_L1_cross - q2_L1_cross));

% --- Loop 2 ---
a=L2_Loop2; b=L3_Loop2; c=L4_Shared;
% Case 1
w_Cyan_1 = (c * w_Grey_1 * sin(q4_L1_open - q3_Red_1)) / (a * sin(q2_Cyan_1 - q3_Red_1));
w_Red_1 = (c * w_Grey_1 * sin(q4_L1_open - q2_Cyan_1)) / (b * sin(q3_Red_1 - q2_Cyan_1));
% Case 2
w_Cyan_2 = (c * w_Grey_2 * sin(q4_L1_cross - q3_Red_2)) / (a * sin(q2_Cyan_2 - q3_Red_2));
w_Red_2 = (c * w_Grey_2 * sin(q4_L1_cross - q2_Cyan_2)) / (b * sin(q3_Red_2 - q2_Cyan_2));

% --- Loop 3 ---
a=L2_Loop3; b=L3_Loop3; c=L4_Loop3;
% Case 1
w_Brown_1 = (a * w_Cyan_1 * sin(q_in_3_1 - q3_Blue_1)) / (c * sin(q4_Brown_1 - q3_Blue_1));
w_Blue_1 = (a * w_Cyan_1 * sin(q4_Brown_1 - q_in_3_1)) / (b * sin(q4_Brown_1 - q3_Blue_1));
% Case 2
w_Brown_2 = (a * w_Cyan_2 * sin(q_in_3_2 - q3_Blue_2)) / (c * sin(q4_Brown_2 - q3_Blue_2));
w_Blue_2 = (a * w_Cyan_2 * sin(q4_Brown_2 - q_in_3_2)) / (b * sin(q4_Brown_2 - q3_Blue_2));

% --- DISPLAY VELOCITIES ---
disp('======================================');
disp('       VELOCITY RESULTS (rad/s)');
disp('======================================');
% (Display Code Skipped for brevity)

% ==========================================
% SECTION 5: ACCELERATION ANALYSIS
% ==========================================
% Formula derived from slides: alpha = (C*E - B*F) / (A*E - B*D)
% System: A*alpha1 + B*alpha2 = C
%         D*alpha1 + E*alpha2 = F

% --- LOOP 1: Green(2) + Yellow(3) - Grey(4) = 0 ---
% Unknowns: Green(alpha2), Grey(alpha4)
% Input: Yellow(alpha3)
a = L2_Loop1; b = L3_Loop1; c = L4_Shared;

% Case 1
% Coefficients for Green (Unknown 1) and Grey (Unknown 2)
% Real part eq: -a*sin(th2)*alp2 + c*sin(th4)*alp4 = RHS_Real
% Imag part eq:  a*cos(th2)*alp2 - c*cos(th4)*alp4 = RHS_Imag
A = -a*sin(q2_L1_open); 
B = c*sin(q4_L1_open);
D = a*cos(q2_L1_open); 
E = -c*cos(q4_L1_open);

% RHS terms (C and F in Cramer's rule context)
% Move all w^2 and input alpha terms to RHS
RHS_Complex = a*w_Green_1^2*exp(1i*q2_L1_open) ...
            - (1i*b*alpha_Yellow - b*w_Yellow^2)*exp(1i*q3) ...
            - c*w_Grey_1^2*exp(1i*q4_L1_open);
C_val = real(RHS_Complex);
F_val = imag(RHS_Complex);

alpha_Green_1 = (C_val*E - B*F_val) / (A*E - B*D);
alpha_Grey_1  = (A*F_val - C_val*D) / (A*E - B*D);

% Case 2
A = -a*sin(q2_L1_cross); 
B = c*sin(q4_L1_cross);
D = a*cos(q2_L1_cross); 
E = -c*cos(q4_L1_cross);

RHS_Complex = a*w_Green_2^2*exp(1i*q2_L1_cross) ...
            - (1i*b*alpha_Yellow - b*w_Yellow^2)*exp(1i*q3) ...
            - c*w_Grey_2^2*exp(1i*q4_L1_cross);
C_val = real(RHS_Complex);
F_val = imag(RHS_Complex);

alpha_Green_2 = (C_val*E - B*F_val) / (A*E - B*D);
alpha_Grey_2  = (A*F_val - C_val*D) / (A*E - B*D);


% --- LOOP 2: Cyan(2) + Red(3) - Grey(4) = 0 ---
% Unknowns: Cyan(alpha2), Red(alpha3)
% Input: Grey(alpha4 - Known from Loop 1)
a = L2_Loop2; b = L3_Loop2; c = L4_Shared;

% Case 1
A = -a*sin(q2_Cyan_1); 
B = -b*sin(q3_Red_1);
D = a*cos(q2_Cyan_1);  
E = b*cos(q3_Red_1);

% RHS: Move Grey terms (Known) to RHS
% Eq: A_Cyan + A_Red = A_Grey
% A_Grey = (j*c*alp_gr - c*w_gr^2)*e^jth_gr
A_Grey_Vec = (1i*c*alpha_Grey_1 - c*w_Grey_1^2)*exp(1i*q4_L1_open);
RHS_Complex = A_Grey_Vec ...
            + a*w_Cyan_1^2*exp(1i*q2_Cyan_1) ...
            + b*w_Red_1^2*exp(1i*q3_Red_1);
C_val = real(RHS_Complex);
F_val = imag(RHS_Complex);

alpha_Cyan_1 = (C_val*E - B*F_val) / (A*E - B*D);
alpha_Red_1  = (A*F_val - C_val*D) / (A*E - B*D);

% Case 2
A = -a*sin(q2_Cyan_2); 
B = -b*sin(q3_Red_2);
D = a*cos(q2_Cyan_2);  
E = b*cos(q3_Red_2);

A_Grey_Vec = (1i*c*alpha_Grey_2 - c*w_Grey_2^2)*exp(1i*q4_L1_cross);
RHS_Complex = A_Grey_Vec ...
            + a*w_Cyan_2^2*exp(1i*q2_Cyan_2) ...
            + b*w_Red_2^2*exp(1i*q3_Red_2);
C_val = real(RHS_Complex);
F_val = imag(RHS_Complex);

alpha_Cyan_2 = (C_val*E - B*F_val) / (A*E - B*D);
alpha_Red_2  = (A*F_val - C_val*D) / (A*E - B*D);


% --- LOOP 3: Cyan(2) + Blue(3) - Brown(4) = 0 ---
% Unknowns: Blue(alpha3), Brown(alpha4)
% Input: Cyan(alpha2 - Known from Loop 2)
a = L2_Loop3; b = L3_Loop3; c = L4_Loop3;

% Case 1
% Eq: A_Cyan + A_Blue - A_Brown = 0
% Unknowns: Blue (b), Brown (c)
% Coeffs: Blue(+), Brown(-)
A = -b*sin(q3_Blue_1); 
B = c*sin(q4_Brown_1); % minus minus = plus
D = b*cos(q3_Blue_1);  
E = -c*cos(q4_Brown_1);

A_Cyan_Vec = (1i*a*alpha_Cyan_1 - a*w_Cyan_1^2)*exp(1i*q_in_3_1);
RHS_Complex = -A_Cyan_Vec ...
            + b*w_Blue_1^2*exp(1i*q3_Blue_1) ...
            - c*w_Brown_1^2*exp(1i*q4_Brown_1);
C_val = real(RHS_Complex);
F_val = imag(RHS_Complex);

alpha_Blue_1  = (C_val*E - B*F_val) / (A*E - B*D);
alpha_Brown_1 = (A*F_val - C_val*D) / (A*E - B*D);

% Case 2
A = -b*sin(q3_Blue_2); 
B = c*sin(q4_Brown_2);
D = b*cos(q3_Blue_2);  
E = -c*cos(q4_Brown_2);

A_Cyan_Vec = (1i*a*alpha_Cyan_2 - a*w_Cyan_2^2)*exp(1i*q_in_3_2);
RHS_Complex = -A_Cyan_Vec ...
            + b*w_Blue_2^2*exp(1i*q3_Blue_2) ...
            - c*w_Brown_2^2*exp(1i*q4_Brown_2);
C_val = real(RHS_Complex);
F_val = imag(RHS_Complex);

alpha_Blue_2  = (C_val*E - B*F_val) / (A*E - B*D);
alpha_Brown_2 = (A*F_val - C_val*D) / (A*E - B*D);


% ==========================================
% SECTION 6: DISPLAY RESULTS
% ==========================================
disp('======================================');
disp('    ANGULAR ACCELERATION (rad/s^2)');
disp('======================================');
disp('--- CASE 1 (OPEN) ---');
disp(['  alpha_Green:  ', num2str(alpha_Green_1)]);
disp(['  alpha_Yellow: ', num2str(alpha_Yellow)]);
disp(['  alpha_Grey:   ', num2str(alpha_Grey_1)]);
disp(['  alpha_Cyan:   ', num2str(alpha_Cyan_1)]);
disp(['  alpha_Red:    ', num2str(alpha_Red_1)]);
disp(['  alpha_Blue:   ', num2str(alpha_Blue_1)]);
disp(['  alpha_Brown:  ', num2str(alpha_Brown_1)]);
disp(' ');
disp('--- CASE 2 (CROSSED) ---');
disp(['  alpha_Green:  ', num2str(alpha_Green_2)]);
disp(['  alpha_Yellow: ', num2str(alpha_Yellow)]);
disp(['  alpha_Grey:   ', num2str(alpha_Grey_2)]);
disp(['  alpha_Cyan:   ', num2str(alpha_Cyan_2)]);
disp(['  alpha_Red:    ', num2str(alpha_Red_2)]);
disp(['  alpha_Blue:   ', num2str(alpha_Blue_2)]);
disp(['  alpha_Brown:  ', num2str(alpha_Brown_2)]);

% ==========================================
% SECTION 7: VECTORS & PLOTTING
% ==========================================
% Formula: A = (j*L*alpha - L*w^2) * exp(j*theta)

% --- Case 1 Vectors ---
A_Green_1 = (1i*L2_Loop1*alpha_Green_1 - L2_Loop1*w_Green_1^2)*exp(1i*(q2_L1_open+offset));
A_Yellow_1 = (1i*L3_Loop1*alpha_Yellow - L3_Loop1*w_Yellow^2)*exp(1i*(q3+offset));
A_Grey_1 = (1i*L4_Shared*alpha_Grey_1 - L4_Shared*w_Grey_1^2)*exp(1i*(q4_L1_open+offset));
A_Cyan_1 = (1i*L2_Loop2*alpha_Cyan_1 - L2_Loop2*w_Cyan_1^2)*exp(1i*(q2_Cyan_1+offset));
A_Red_1 = (1i*L3_Loop2*alpha_Red_1 - L3_Loop2*w_Red_1^2)*exp(1i*(q3_Red_1+offset));
A_Cyan_Up_1 = (1i*L2_Loop3*alpha_Cyan_1 - L2_Loop3*w_Cyan_1^2)*exp(1i*(q_in_3_1+offset));
A_Blue_1 = (1i*L3_Loop3*alpha_Blue_1 - L3_Loop3*w_Blue_1^2)*exp(1i*(q3_Blue_1+offset));
A_Brown_1 = (1i*L4_Loop3*alpha_Brown_1 - L4_Loop3*w_Brown_1^2)*exp(1i*(q4_Brown_1+offset));

% --- Case 2 Vectors ---
A_Green_2 = (1i*L2_Loop1*alpha_Green_2 - L2_Loop1*w_Green_2^2)*exp(1i*(q2_L1_cross+offset));
A_Yellow_2 = (1i*L3_Loop1*alpha_Yellow - L3_Loop1*w_Yellow^2)*exp(1i*(q3+offset));
A_Grey_2 = (1i*L4_Shared*alpha_Grey_2 - L4_Shared*w_Grey_2^2)*exp(1i*(q4_L1_cross+offset));
A_Cyan_2 = (1i*L2_Loop2*alpha_Cyan_2 - L2_Loop2*w_Cyan_2^2)*exp(1i*(q2_Cyan_2+offset));
A_Red_2 = (1i*L3_Loop2*alpha_Red_2 - L3_Loop2*w_Red_2^2)*exp(1i*(q3_Red_2+offset));
A_Cyan_Up_2 = (1i*L2_Loop3*alpha_Cyan_2 - L2_Loop3*w_Cyan_2^2)*exp(1i*(q_in_3_2+offset));
A_Blue_2 = (1i*L3_Loop3*alpha_Blue_2 - L3_Loop3*w_Blue_2^2)*exp(1i*(q3_Blue_2+offset));
A_Brown_2 = (1i*L4_Loop3*alpha_Brown_2 - L4_Loop3*w_Brown_2^2)*exp(1i*(q4_Brown_2+offset));

RO4O2 = L1*exp(1i*offset);
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);
AccScale = 0.02; 

% --- Plot Case 1 ---
figure(1);
% Position
R_Green = L2_Loop1*exp(1i*(q2_L1_open+offset));
R_Cyan = L2_Loop2*exp(1i*(q2_Cyan_1+offset));
R_Cyan_Up = L2_Loop3*exp(1i*(q_in_3_1+offset));
% Acceleration Vectors
quiver(real(R_Green), imag(R_Green), real(A_Green_1)*AccScale, imag(A_Green_1)*AccScale, 0, 'Color', [0 0.5 0], 'LineWidth', 2);
quiver(real(R_Green), imag(R_Green), real(A_Yellow_1)*AccScale, imag(A_Yellow_1)*AccScale, 0, 'Color', [0.8 0.8 0], 'LineWidth', 2);
quiver(RO4O2x, RO4O2y, real(A_Grey_1)*AccScale, imag(A_Grey_1)*AccScale, 0, 'k', 'LineWidth', 2);
quiver(real(R_Cyan), imag(R_Cyan), real(A_Cyan_1)*AccScale, imag(A_Cyan_1)*AccScale, 0, 'c', 'LineWidth', 2);
quiver(real(R_Cyan), imag(R_Cyan), real(A_Red_1)*AccScale, imag(A_Red_1)*AccScale, 0, 'r', 'LineWidth', 2);
quiver(real(R_Cyan_Up), imag(R_Cyan_Up), real(A_Cyan_Up_1)*AccScale, imag(A_Cyan_Up_1)*AccScale, 0, 'c', 'LineWidth', 2);
quiver(real(R_Cyan_Up), imag(R_Cyan_Up), real(A_Blue_1)*AccScale, imag(A_Blue_1)*AccScale, 0, 'b', 'LineWidth', 2);
quiver(RO4O2x, RO4O2y, real(A_Brown_1)*AccScale, imag(A_Brown_1)*AccScale, 0, 'Color', [0.6 0.3 0], 'LineWidth', 2);

% --- Plot Case 2 ---
figure(2);
R_Green_2 = L2_Loop1*exp(1i*(q2_L1_cross+offset));
R_Cyan_2 = L2_Loop2*exp(1i*(q2_Cyan_2+offset));
R_Cyan_Up_2 = L2_Loop3*exp(1i*(q_in_3_2+offset));

quiver(real(R_Green_2), imag(R_Green_2), real(A_Green_2)*AccScale, imag(A_Green_2)*AccScale, 0, 'Color', [0 0.5 0], 'LineWidth', 2);
quiver(real(R_Green_2), imag(R_Green_2), real(A_Yellow_2)*AccScale, imag(A_Yellow_2)*AccScale, 0, 'Color', [0.8 0.8 0], 'LineWidth', 2);
quiver(RO4O2x, RO4O2y, real(A_Grey_2)*AccScale, imag(A_Grey_2)*AccScale, 0, 'k', 'LineWidth', 2);
quiver(real(R_Cyan_2), imag(R_Cyan_2), real(A_Cyan_2)*AccScale, imag(A_Cyan_2)*AccScale, 0, 'c', 'LineWidth', 2);
quiver(real(R_Cyan_2), imag(R_Cyan_2), real(A_Red_2)*AccScale, imag(A_Red_2)*AccScale, 0, 'r', 'LineWidth', 2);
quiver(real(R_Cyan_Up_2), imag(R_Cyan_Up_2), real(A_Cyan_Up_2)*AccScale, imag(A_Cyan_Up_2)*AccScale, 0, 'c', 'LineWidth', 2);
quiver(real(R_Cyan_Up_2), imag(R_Cyan_Up_2), real(A_Blue_2)*AccScale, imag(A_Blue_2)*AccScale, 0, 'b', 'LineWidth', 2);
quiver(RO4O2x, RO4O2y, real(A_Brown_2)*AccScale, imag(A_Brown_2)*AccScale, 0, 'Color', [0.6 0.3 0], 'LineWidth', 2);
