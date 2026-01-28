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

w_Yellow = -2.2;      % Input Angular Velocity
alpha_Yellow = -0.8;  % Input Angular Acceleration

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
% Formula: A*x + B*y = C, D*x + E*y = F (Cramer's Rule)

% --- Loop 1: Green(2) + Yellow(3) - Grey(4) = 0 ---
a = L2_Loop1; b = L3_Loop1; c = L4_Shared;

% Case 1
A = -a*sin(q2_L1_open); B = c*sin(q4_L1_open);
D = a*cos(q2_L1_open);  E = -c*cos(q4_L1_open);
C_val = a*w_Green_1^2*cos(q2_L1_open) + b*w_Yellow^2*cos(q3) + b*alpha_Yellow*sin(q3) - c*w_Grey_1^2*cos(q4_L1_open);
F_val = a*w_Green_1^2*sin(q2_L1_open) + b*w_Yellow^2*sin(q3) - b*alpha_Yellow*cos(q3) - c*w_Grey_1^2*sin(q4_L1_open);

alpha_Green_1 = (C_val*E - B*F_val) / (A*E - B*D);
alpha_Grey_1  = (A*F_val - C_val*D) / (A*E - B*D);

% Case 2
A = -a*sin(q2_L1_cross); B = c*sin(q4_L1_cross);
D = a*cos(q2_L1_cross);  E = -c*cos(q4_L1_cross);
C_val = a*w_Green_2^2*cos(q2_L1_cross) + b*w_Yellow^2*cos(q3) + b*alpha_Yellow*sin(q3) - c*w_Grey_2^2*cos(q4_L1_cross);
F_val = a*w_Green_2^2*sin(q2_L1_cross) + b*w_Yellow^2*sin(q3) - b*alpha_Yellow*cos(q3) - c*w_Grey_2^2*sin(q4_L1_cross);

alpha_Green_2 = (C_val*E - B*F_val) / (A*E - B*D);
alpha_Grey_2  = (A*F_val - C_val*D) / (A*E - B*D);


% --- Loop 2: Cyan(2) + Red(3) - Grey(4) = 0 ---
a = L2_Loop2; b = L3_Loop2; c = L4_Shared;

% Case 1
A = -a*sin(q2_Cyan_1); B = -b*sin(q3_Red_1);
D = a*cos(q2_Cyan_1);  E = b*cos(q3_Red_1);
C_val = a*w_Cyan_1^2*cos(q2_Cyan_1) + b*w_Red_1^2*cos(q3_Red_1) - c*w_Grey_1^2*cos(q4_L1_open) - c*alpha_Grey_1*sin(q4_L1_open);
F_val = a*w_Cyan_1^2*sin(q2_Cyan_1) + b*w_Red_1^2*sin(q3_Red_1) - c*w_Grey_1^2*sin(q4_L1_open) + c*alpha_Grey_1*cos(q4_L1_open);

alpha_Cyan_1 = (C_val*E - B*F_val) / (A*E - B*D);
alpha_Red_1  = (A*F_val - C_val*D) / (A*E - B*D);

% Case 2
A = -a*sin(q2_Cyan_2); B = -b*sin(q3_Red_2);
D = a*cos(q2_Cyan_2);  E = b*cos(q3_Red_2);
C_val = a*w_Cyan_2^2*cos(q2_Cyan_2) + b*w_Red_2^2*cos(q3_Red_2) - c*w_Grey_2^2*cos(q4_L1_cross) - c*alpha_Grey_2*sin(q4_L1_cross);
F_val = a*w_Cyan_2^2*sin(q2_Cyan_2) + b*w_Red_2^2*sin(q3_Red_2) - c*w_Grey_2^2*sin(q4_L1_cross) + c*alpha_Grey_2*cos(q4_L1_cross);

alpha_Cyan_2 = (C_val*E - B*F_val) / (A*E - B*D);
alpha_Red_2  = (A*F_val - C_val*D) / (A*E - B*D);


% --- Loop 3: Cyan(2) + Blue(3) - Brown(4) = 0 ---
a = L2_Loop3; b = L3_Loop3; c = L4_Loop3;

% Case 1
A = -b*sin(q3_Blue_1); B = c*sin(q4_Brown_1);
D = b*cos(q3_Blue_1);  E = -c*cos(q4_Brown_1);
C_val = a*w_Cyan_1^2*cos(q_in_3_1) + a*alpha_Cyan_1*sin(q_in_3_1) + b*w_Blue_1^2*cos(q3_Blue_1) - c*w_Brown_1^2*cos(q4_Brown_1);
F_val = a*w_Cyan_1^2*sin(q_in_3_1) - a*alpha_Cyan_1*cos(q_in_3_1) + b*w_Blue_1^2*sin(q3_Blue_1) - c*w_Brown_1^2*sin(q4_Brown_1);

alpha_Blue_1  = (C_val*E - B*F_val) / (A*E - B*D);
alpha_Brown_1 = (A*F_val - C_val*D) / (A*E - B*D);

% Case 2
A = -b*sin(q3_Blue_2); B = c*sin(q4_Brown_2);
D = b*cos(q3_Blue_2);  E = -c*cos(q4_Brown_2);
C_val = a*w_Cyan_2^2*cos(q_in_3_2) + a*alpha_Cyan_2*sin(q_in_3_2) + b*w_Blue_2^2*cos(q3_Blue_2) - c*w_Brown_2^2*cos(q4_Brown_2);
F_val = a*w_Cyan_2^2*sin(q_in_3_2) - a*alpha_Cyan_2*cos(q_in_3_2) + b*w_Blue_2^2*sin(q3_Blue_2) - c*w_Brown_2^2*sin(q4_Brown_2);

alpha_Blue_2  = (C_val*E - B*F_val) / (A*E - B*D);
alpha_Brown_2 = (A*F_val - C_val*D) / (A*E - B*D);


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
RO4O2 = L1*exp(1i*offset);
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);
AccScale = 0.05; % Scale for better visibility

% --- Calculate Vectors ---
% Case 1
A_Green_1 = (1i*alpha_Green_1*L2_Loop1 - L2_Loop1*w_Green_1^2) * exp(1i*(q2_L1_open+offset));
A_Yellow_Rel_1 = (1i*alpha_Yellow*L3_Loop1 - L3_Loop1*w_Yellow^2) * exp(1i*(q3+offset));
A_Grey_1 = (1i*alpha_Grey_1*L4_Shared - L4_Shared*w_Grey_1^2) * exp(1i*(q4_L1_open+offset));
A_Cyan_1 = (1i*alpha_Cyan_1*L2_Loop2 - L2_Loop2*w_Cyan_1^2) * exp(1i*(q2_Cyan_1+offset));
A_Red_Rel_1 = (1i*alpha_Red_1*L3_Loop2 - L3_Loop2*w_Red_1^2) * exp(1i*(q3_Red_1+offset));
A_Cyan_Up_1 = (1i*alpha_Cyan_1*L2_Loop3 - L2_Loop3*w_Cyan_1^2) * exp(1i*(q_in_3_1+offset));
A_Blue_Rel_1 = (1i*alpha_Blue_1*L3_Loop3 - L3_Loop3*w_Blue_1^2) * exp(1i*(q3_Blue_1+offset));
A_Brown_1 = (1i*alpha_Brown_1*L4_Loop3 - L4_Loop3*w_Brown_1^2) * exp(1i*(q4_Brown_1+offset));

% Case 2
A_Green_2 = (1i*alpha_Green_2*L2_Loop1 - L2_Loop1*w_Green_2^2) * exp(1i*(q2_L1_cross+offset));
A_Yellow_Rel_2 = (1i*alpha_Yellow*L3_Loop1 - L3_Loop1*w_Yellow^2) * exp(1i*(q3+offset));
A_Grey_2 = (1i*alpha_Grey_2*L4_Shared - L4_Shared*w_Grey_2^2) * exp(1i*(q4_L1_cross+offset));
A_Cyan_2 = (1i*alpha_Cyan_2*L2_Loop2 - L2_Loop2*w_Cyan_2^2) * exp(1i*(q2_Cyan_2+offset));
A_Red_Rel_2 = (1i*alpha_Red_2*L3_Loop2 - L3_Loop2*w_Red_2^2) * exp(1i*(q3_Red_2+offset));
A_Cyan_Up_2 = (1i*alpha_Cyan_2*L2_Loop3 - L2_Loop3*w_Cyan_2^2) * exp(1i*(q_in_3_2+offset));
A_Blue_Rel_2 = (1i*alpha_Blue_2*L3_Loop3 - L3_Loop3*w_Blue_2^2) * exp(1i*(q3_Blue_2+offset));
A_Brown_2 = (1i*alpha_Brown_2*L4_Loop3 - L4_Loop3*w_Brown_2^2) * exp(1i*(q4_Brown_2+offset));

% ==========================================
% SECTION 8: ACCELERATION AT POINT P
% ==========================================
% Point P is on Brown Link, 65mm from O4
L_P = 0.065; 

% Case 1
R_P_1 = RO4O2 + L_P * exp(1i * (q4_Brown_1 + offset));
A_P_1 = (1i * alpha_Brown_1 * L_P - L_P * w_Brown_1^2) * exp(1i * (q4_Brown_1 + offset));

% Case 2
R_P_2 = RO4O2 + L_P * exp(1i * (q4_Brown_2 + offset));
A_P_2 = (1i * alpha_Brown_2 * L_P - L_P * w_Brown_2^2) * exp(1i * (q4_Brown_2 + offset));

disp('======================================');
disp('   ACCELERATION AT POINT P (m/s^2)');
disp('======================================');
disp('--- CASE 1 (OPEN) ---');
disp(['  Ax_P: ', num2str(real(A_P_1))]);
disp(['  Ay_P: ', num2str(imag(A_P_1))]);
disp(['  Mag_P: ', num2str(abs(A_P_1))]);
disp(' ');
disp('--- CASE 2 (CROSSED) ---');
disp(['  Ax_P: ', num2str(real(A_P_2))]);
disp(['  Ay_P: ', num2str(imag(A_P_2))]);
disp(['  Mag_P: ', num2str(abs(A_P_2))]);


% --- PLOT CASE 1 ---
figure(1); hold on; title('Case 1: Open Circuit (Position & Acceleration)');
% 1. Plot Mechanism (Position) - Explicitly drawing lines
R_Green = L2_Loop1*exp(1i*(q2_L1_open+offset));
R_Grey = L4_Shared*exp(1i*(q4_L1_open+offset));
R_Cyan = L2_Loop2*exp(1i*(q2_Cyan_1+offset));
R_Cyan_Up = L2_Loop3*exp(1i*(q_in_3_1+offset));
R_Brown = L4_Loop3*exp(1i*(q4_Brown_1+offset));

% Loop 1
plot([0 RO4O2x], [0 RO4O2y], 'm-', 'LineWidth', 2); % Ground
plot([0 real(R_Green)], [0 imag(R_Green)], 'g-', 'LineWidth', 2); % Green
plot([real(R_Green) real(R_Grey)+RO4O2x], [imag(R_Green) imag(R_Grey)+RO4O2y], 'y-', 'LineWidth', 2); % Yellow
plot([RO4O2x real(R_Grey)+RO4O2x], [RO4O2y imag(R_Grey)+RO4O2y], 'Color', [0.5 0.5 0.5], 'LineWidth', 2); % Grey
% Loop 2
plot([0 real(R_Cyan)], [0 imag(R_Cyan)], 'c-', 'LineWidth', 2); % Cyan
plot([real(R_Cyan) real(R_Grey)+RO4O2x], [imag(R_Cyan) imag(R_Grey)+RO4O2y], 'r-', 'LineWidth', 2); % Red
% Loop 3
plot([0 real(R_Cyan_Up)], [0 imag(R_Cyan_Up)], 'c:', 'LineWidth', 2); % Cyan Up
plot([real(R_Cyan_Up) real(R_Brown)+RO4O2x], [imag(R_Cyan_Up) imag(R_Brown)+RO4O2y], 'b-', 'LineWidth', 2); % Blue
plot([RO4O2x real(R_Brown)+RO4O2x], [RO4O2y imag(R_Brown)+RO4O2y], 'Color', [0.6 0.3 0], 'LineWidth', 2); % Brown

% 2. Plot Acceleration Vectors
quiver(real(R_Green), imag(R_Green), real(A_Green_1)*AccScale, imag(A_Green_1)*AccScale, 0, 'Color', 'g', 'LineWidth', 2);
quiver(real(R_Green), imag(R_Green), real(A_Yellow_Rel_1)*AccScale, imag(A_Yellow_Rel_1)*AccScale, 0, 'Color', 'y', 'LineWidth', 2);
quiver(real(R_Grey)+RO4O2x, imag(R_Grey)+RO4O2y, real(A_Grey_1)*AccScale, imag(A_Grey_1)*AccScale, 0, 'Color',[0.5 0.5 0.5], 'LineWidth', 2);
quiver(real(R_Cyan), imag(R_Cyan), real(A_Cyan_1)*AccScale, imag(A_Cyan_1)*AccScale, 0, 'c', 'LineWidth', 2);
quiver(real(R_Cyan), imag(R_Cyan), real(A_Red_Rel_1)*AccScale, imag(A_Red_Rel_1)*AccScale, 0, 'r', 'LineWidth', 2);
quiver(real(R_Cyan_Up), imag(R_Cyan_Up), real(A_Cyan_Up_1)*AccScale, imag(A_Cyan_Up_1)*AccScale, 0, 'c', 'LineWidth', 2);
quiver(real(R_Cyan_Up), imag(R_Cyan_Up), real(A_Blue_Rel_1)*AccScale, imag(A_Blue_Rel_1)*AccScale, 0, 'b', 'LineWidth', 2);
quiver(real(R_Brown)+RO4O2x, imag(R_Brown)+RO4O2y, real(A_Brown_1)*AccScale, imag(A_Brown_1)*AccScale, 0, 'Color', [0.6 0.3 0], 'LineWidth', 2);

% Plot Point P
quiver(real(R_P_1), imag(R_P_1), real(A_P_1)*AccScale, imag(A_P_1)*AccScale, 0, 'm', 'LineWidth', 2);
plot(real(R_P_1), imag(R_P_1), 'mo', 'MarkerFaceColor', 'm');
axis equal; grid on;

% --- PLOT CASE 2 ---
figure(2); hold on; title('Case 2: Crossed Circuit (Position & Acceleration)');
R_Green_2 = L2_Loop1*exp(1i*(q2_L1_cross+offset));
R_Grey_2 = L4_Shared*exp(1i*(q4_L1_cross+offset));
R_Cyan_2 = L2_Loop2*exp(1i*(q2_Cyan_2+offset));
R_Cyan_Up_2 = L2_Loop3*exp(1i*(q_in_3_2+offset));
R_Brown_2 = L4_Loop3*exp(1i*(q4_Brown_2+offset));

% Loop 1
plot([0 RO4O2x], [0 RO4O2y], 'm-', 'LineWidth', 2);
plot([0 real(R_Green_2)], [0 imag(R_Green_2)], 'g-', 'LineWidth', 2);
plot([real(R_Green_2) real(R_Grey_2)+RO4O2x], [imag(R_Green_2) imag(R_Grey_2)+RO4O2y], 'y-', 'LineWidth', 2);
plot([RO4O2x real(R_Grey_2)+RO4O2x], [RO4O2y imag(R_Grey_2)+RO4O2y], 'Color', [0.5 0.5 0.5], 'LineWidth', 2);
% Loop 2
plot([0 real(R_Cyan_2)], [0 imag(R_Cyan_2)], 'c-', 'LineWidth', 2);
plot([real(R_Cyan_2) real(R_Grey_2)+RO4O2x], [imag(R_Cyan_2) imag(R_Grey_2)+RO4O2y], 'r-', 'LineWidth', 2);
% Loop 3
plot([0 real(R_Cyan_Up_2)], [0 imag(R_Cyan_Up_2)], 'c:', 'LineWidth', 2);
plot([real(R_Cyan_Up_2) real(R_Brown_2)+RO4O2x], [imag(R_Cyan_Up_2) imag(R_Brown_2)+RO4O2y], 'b-', 'LineWidth', 2);
plot([RO4O2x real(R_Brown_2)+RO4O2x], [RO4O2y imag(R_Brown_2)+RO4O2y], 'Color', [0.6 0.3 0], 'LineWidth', 2);

% Acceleration Vectors
quiver(real(R_Green_2), imag(R_Green_2), real(A_Green_2)*AccScale, imag(A_Green_2)*AccScale, 0, 'g', 'LineWidth', 2);
quiver(real(R_Green_2), imag(R_Green_2), real(A_Yellow_Rel_2)*AccScale, imag(A_Yellow_Rel_2)*AccScale, 0, 'y', 'LineWidth', 2);
quiver(real(R_Grey_2)+RO4O2x, imag(R_Grey_2)+RO4O2y, real(A_Grey_2)*AccScale, imag(A_Grey_2)*AccScale, 0, 'Color',[0.5 0.5 0.5], 'LineWidth', 2);
quiver(real(R_Cyan_2), imag(R_Cyan_2), real(A_Cyan_2)*AccScale, imag(A_Cyan_2)*AccScale, 0, 'c', 'LineWidth', 2);
quiver(real(R_Cyan_2), imag(R_Cyan_2), real(A_Red_Rel_2)*AccScale, imag(A_Red_Rel_2)*AccScale, 0, 'r', 'LineWidth', 2);
quiver(real(R_Cyan_Up_2), imag(R_Cyan_Up_2), real(A_Cyan_Up_2)*AccScale, imag(A_Cyan_Up_2)*AccScale, 0, 'c', 'LineWidth', 2);
quiver(real(R_Cyan_Up_2), imag(R_Cyan_Up_2), real(A_Blue_Rel_2)*AccScale, imag(A_Blue_Rel_2)*AccScale, 0, 'b', 'LineWidth', 2);
quiver(real(R_Brown_2)+RO4O2x, imag(R_Brown_2)+RO4O2y, real(A_Brown_2)*AccScale, imag(A_Brown_2)*AccScale, 0, 'Color', [0.6 0.3 0], 'LineWidth', 2);

% Plot Point P
quiver(real(R_P_2), imag(R_P_2), real(A_P_2)*AccScale, imag(A_P_2)*AccScale, 0, 'm', 'LineWidth', 2);
plot(real(R_P_2), imag(R_P_2), 'mo', 'MarkerFaceColor', 'm');
axis equal; grid on;
