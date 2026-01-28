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
alpha_Yellow = -0.8; % Input Angular Acceleration (rad/s^2)

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
disp('--- CASE 1 (OPEN) ---');
disp(['  Green:  ', num2str(rad2deg(q2_L1_open)+offset_deg)]);
disp(['  Yellow: ', num2str(q3d_global)]);
disp(['  Grey:   ', num2str(rad2deg(q4_L1_open)+offset_deg)]);
disp(' ');

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

disp('======================================');
disp('       VELOCITY RESULTS (rad/s)');
disp('======================================');
disp('--- CASE 1 (OPEN) ---');
disp(['  w_Green: ', num2str(w_Green_1)]);
disp(['  w_Yellow (Input): ', num2str(w_Yellow)]);
disp(['  w_Grey:  ', num2str(w_Grey_1)]);
disp(' ');

% ==========================================
% SECTION 5: ACCELERATION ANALYSIS
% ==========================================
% Solving linear equations A*x + B*y = C, D*x + E*y = F
% alpha_x = (C*E - B*F) / (A*E - B*D)
% alpha_y = (A*F - C*D) / (A*E - B*D)

% --- Loop 1: Green(2) + Yellow(3) - Grey(4) = 0 ---
% Unknowns: alpha_Green (2) and alpha_Grey (4)
% Known Input: alpha_Yellow (3) = -0.8
a = L2_Loop1; b = L3_Loop1; c = L4_Shared;

% Case 1
A = -a*sin(q2_L1_open); B = c*sin(q4_L1_open);
D = a*cos(q2_L1_open);  E = -c*cos(q4_L1_open);
% C and F are RHS (Known terms)
C = a*w_Green_1^2*cos(q2_L1_open) + b*w_Yellow^2*cos(q3) + b*alpha_Yellow*sin(q3) - c*w_Grey_1^2*cos(q4_L1_open);
F = a*w_Green_1^2*sin(q2_L1_open) + b*w_Yellow^2*sin(q3) - b*alpha_Yellow*cos(q3) - c*w_Grey_1^2*sin(q4_L1_open);

alpha_Green_1 = (C*E - B*F) / (A*E - B*D);
alpha_Grey_1  = (A*F - C*D) / (A*E - B*D);

% Case 2
A = -a*sin(q2_L1_cross); B = c*sin(q4_L1_cross);
D = a*cos(q2_L1_cross);  E = -c*cos(q4_L1_cross);
C = a*w_Green_2^2*cos(q2_L1_cross) + b*w_Yellow^2*cos(q3) + b*alpha_Yellow*sin(q3) - c*w_Grey_2^2*cos(q4_L1_cross);
F = a*w_Green_2^2*sin(q2_L1_cross) + b*w_Yellow^2*sin(q3) - b*alpha_Yellow*cos(q3) - c*w_Grey_2^2*sin(q4_L1_cross);

alpha_Green_2 = (C*E - B*F) / (A*E - B*D);
alpha_Grey_2  = (A*F - C*D) / (A*E - B*D);


% --- Loop 2: Cyan(2) + Red(3) - Grey(4) = 0 ---
% Unknowns: alpha_Cyan (2) and alpha_Red (3)
% Known Input: alpha_Grey (4) from Loop 1
a = L2_Loop2; b = L3_Loop2; c = L4_Shared;

% Case 1
A = -a*sin(q2_Cyan_1); B = -b*sin(q3_Red_1);
D = a*cos(q2_Cyan_1);  E = b*cos(q3_Red_1);
% RHS includes Grey terms (Known)
C = a*w_Cyan_1^2*cos(q2_Cyan_1) + b*w_Red_1^2*cos(q3_Red_1) - c*w_Grey_1^2*cos(q4_L1_open) - c*alpha_Grey_1*sin(q4_L1_open);
F = a*w_Cyan_1^2*sin(q2_Cyan_1) + b*w_Red_1^2*sin(q3_Red_1) - c*w_Grey_1^2*sin(q4_L1_open) + c*alpha_Grey_1*cos(q4_L1_open);

alpha_Cyan_1 = (C*E - B*F) / (A*E - B*D);
alpha_Red_1  = (A*F - C*D) / (A*E - B*D);

% Case 2
A = -a*sin(q2_Cyan_2); B = -b*sin(q3_Red_2);
D = a*cos(q2_Cyan_2);  E = b*cos(q3_Red_2);
C = a*w_Cyan_2^2*cos(q2_Cyan_2) + b*w_Red_2^2*cos(q3_Red_2) - c*w_Grey_2^2*cos(q4_L1_cross) - c*alpha_Grey_2*sin(q4_L1_cross);
F = a*w_Cyan_2^2*sin(q2_Cyan_2) + b*w_Red_2^2*sin(q3_Red_2) - c*w_Grey_2^2*sin(q4_L1_cross) + c*alpha_Grey_2*cos(q4_L1_cross);

alpha_Cyan_2 = (C*E - B*F) / (A*E - B*D);
alpha_Red_2  = (A*F - C*D) / (A*E - B*D);


% --- Loop 3: Cyan(2) + Blue(3) - Brown(4) = 0 ---
% Unknowns: alpha_Blue (3) and alpha_Brown (4)
% Known Input: alpha_Cyan (2) from Loop 2
a = L2_Loop3; b = L3_Loop3; c = L4_Loop3;

% Case 1
A = -b*sin(q3_Blue_1); B = c*sin(q4_Brown_1);
D = b*cos(q3_Blue_1);  E = -c*cos(q4_Brown_1);
% RHS includes Cyan terms (Known)
C = a*w_Cyan_1^2*cos(q_in_3_1) + a*alpha_Cyan_1*sin(q_in_3_1) + b*w_Blue_1^2*cos(q3_Blue_1) - c*w_Brown_1^2*cos(q4_Brown_1);
F = a*w_Cyan_1^2*sin(q_in_3_1) - a*alpha_Cyan_1*cos(q_in_3_1) + b*w_Blue_1^2*sin(q3_Blue_1) - c*w_Brown_1^2*sin(q4_Brown_1);

alpha_Blue_1  = (C*E - B*F) / (A*E - B*D);
alpha_Brown_1 = (A*F - C*D) / (A*E - B*D);

% Case 2
A = -b*sin(q3_Blue_2); B = c*sin(q4_Brown_2);
D = b*cos(q3_Blue_2);  E = -c*cos(q4_Brown_2);
C = a*w_Cyan_2^2*cos(q_in_3_2) + a*alpha_Cyan_2*sin(q_in_3_2) + b*w_Blue_2^2*cos(q3_Blue_2) - c*w_Brown_2^2*cos(q4_Brown_2);
F = a*w_Cyan_2^2*sin(q_in_3_2) - a*alpha_Cyan_2*cos(q_in_3_2) + b*w_Blue_2^2*sin(q3_Blue_2) - c*w_Brown_2^2*sin(q4_Brown_2);

alpha_Blue_2  = (C*E - B*F) / (A*E - B*D);
alpha_Brown_2 = (A*F - C*D) / (A*E - B*D);


% --- DISPLAY ACCELERATION RESULTS ---
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
% SECTION 6: VECTORS & PLOTTING
% ==========================================
% Vector A = (j*alpha - w^2) * R_vector
RO4O2 = L1*exp(1i*offset);
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);
Scale = 0.05;

% --- Case 1 Calculation ---
% Tangential = j*alpha*R, Normal = -w^2*R
A_Green_1 = (1i*alpha_Green_1 - w_Green_1^2) * (L2_Loop1*exp(1i*(q2_L1_open+offset)));
A_Yellow_1 = (1i*alpha_Yellow - w_Yellow^2) * (L3_Loop1*exp(1i*(q3+offset)));
A_Grey_1 = (1i*alpha_Grey_1 - w_Grey_1^2) * (L4_Shared*exp(1i*(q4_L1_open+offset)));
A_Cyan_1 = (1i*alpha_Cyan_1 - w_Cyan_1^2) * (L2_Loop2*exp(1i*(q2_Cyan_1+offset)));
A_Red_1 = (1i*alpha_Red_1 - w_Red_1^2) * (L3_Loop2*exp(1i*(q3_Red_1+offset)));
A_Cyan_Up_1 = (1i*alpha_Cyan_1 - w_Cyan_1^2) * (L2_Loop3*exp(1i*(q_in_3_1+offset)));
A_Blue_1 = (1i*alpha_Blue_1 - w_Blue_1^2) * (L3_Loop3*exp(1i*(q3_Blue_1+offset)));
A_Brown_1 = (1i*alpha_Brown_1 - w_Brown_1^2) * (L4_Loop3*exp(1i*(q4_Brown_1+offset)));

% --- Plot Case 1 ---
figure(1);
% Re-plot Position for reference
R_Green_Vec = L2_Loop1*exp(1i*(q2_L1_open+offset));
R_Cyan_Vec = L2_Loop2*exp(1i*(q2_Cyan_1+offset));
R_Cyan_Up_Vec = L2_Loop3*exp(1i*(q_in_3_1+offset));

quiver(real(R_Green_Vec), imag(R_Green_Vec), real(A_Green_1)*Scale, imag(A_Green_1)*Scale, 0, 'Color', [0 0.5 0], 'LineWidth', 2);
quiver(real(R_Green_Vec), imag(R_Green_Vec), real(A_Yellow_1)*Scale, imag(A_Yellow_1)*Scale, 0, 'Color', [0.8 0.8 0], 'LineWidth', 2);
quiver(RO4O2x, RO4O2y, real(A_Grey_1)*Scale, imag(A_Grey_1)*Scale, 0, 'k', 'LineWidth', 2);
quiver(real(R_Cyan_Vec), imag(R_Cyan_Vec), real(A_Cyan_1)*Scale, imag(A_Cyan_1)*Scale, 0, 'c', 'LineWidth', 2);
quiver(real(R_Cyan_Vec), imag(R_Cyan_Vec), real(A_Red_1)*Scale, imag(A_Red_1)*Scale, 0, 'r', 'LineWidth', 2);
quiver(real(R_Cyan_Up_Vec), imag(R_Cyan_Up_Vec), real(A_Cyan_Up_1)*Scale, imag(A_Cyan_Up_1)*Scale, 0, 'c', 'LineWidth', 2);
quiver(real(R_Cyan_Up_Vec), imag(R_Cyan_Up_Vec), real(A_Blue_1)*Scale, imag(A_Blue_1)*Scale, 0, 'b', 'LineWidth', 2);
quiver(RO4O2x, RO4O2y, real(A_Brown_1)*Scale, imag(A_Brown_1)*Scale, 0, 'Color', [0.6 0.3 0], 'LineWidth', 2);


% --- Case 2 Calculation ---
A_Green_2 = (1i*alpha_Green_2 - w_Green_2^2) * (L2_Loop1*exp(1i*(q2_L1_cross+offset)));
A_Yellow_2 = (1i*alpha_Yellow - w_Yellow^2) * (L3_Loop1*exp(1i*(q3+offset)));
A_Grey_2 = (1i*alpha_Grey_2 - w_Grey_2^2) * (L4_Shared*exp(1i*(q4_L1_cross+offset)));
A_Cyan_2 = (1i*alpha_Cyan_2 - w_Cyan_2^2) * (L2_Loop2*exp(1i*(q2_Cyan_2+offset)));
A_Red_2 = (1i*alpha_Red_2 - w_Red_2^2) * (L3_Loop2*exp(1i*(q3_Red_2+offset)));
A_Cyan_Up_2 = (1i*alpha_Cyan_2 - w_Cyan_2^2) * (L2_Loop3*exp(1i*(q_in_3_2+offset)));
A_Blue_2 = (1i*alpha_Blue_2 - w_Blue_2^2) * (L3_Loop3*exp(1i*(q3_Blue_2+offset)));
A_Brown_2 = (1i*alpha_Brown_2 - w_Brown_2^2) * (L4_Loop3*exp(1i*(q4_Brown_2+offset)));

% --- Plot Case 2 ---
figure(2);
R_Green_Vec2 = L2_Loop1*exp(1i*(q2_L1_cross+offset));
R_Cyan_Vec2 = L2_Loop2*exp(1i*(q2_Cyan_2+offset));
R_Cyan_Up_Vec2 = L2_Loop3*exp(1i*(q_in_3_2+offset));

quiver(real(R_Green_Vec2), imag(R_Green_Vec2), real(A_Green_2)*Scale, imag(A_Green_2)*Scale, 0, 'Color', [0 0.5 0], 'LineWidth', 2);
quiver(real(R_Green_Vec2), imag(R_Green_Vec2), real(A_Yellow_2)*Scale, imag(A_Yellow_2)*Scale, 0, 'Color', [0.8 0.8 0], 'LineWidth', 2);
quiver(RO4O2x, RO4O2y, real(A_Grey_2)*Scale, imag(A_Grey_2)*Scale, 0, 'k', 'LineWidth', 2);
quiver(real(R_Cyan_Vec2), imag(R_Cyan_Vec2), real(A_Cyan_2)*Scale, imag(A_Cyan_2)*Scale, 0, 'c', 'LineWidth', 2);
quiver(real(R_Cyan_Vec2), imag(R_Cyan_Vec2), real(A_Red_2)*Scale, imag(A_Red_2)*Scale, 0, 'r', 'LineWidth', 2);
quiver(real(R_Cyan_Up_Vec2), imag(R_Cyan_Up_Vec2), real(A_Cyan_Up_2)*Scale, imag(A_Cyan_Up_2)*Scale, 0, 'c', 'LineWidth', 2);
quiver(real(R_Cyan_Up_Vec2), imag(R_Cyan_Up_Vec2), real(A_Blue_2)*Scale, imag(A_Blue_2)*Scale, 0, 'b', 'LineWidth', 2);
quiver(RO4O2x, RO4O2y, real(A_Brown_2)*Scale, imag(A_Brown_2)*Scale, 0, 'Color', [0.6 0.3 0], 'LineWidth', 2);
