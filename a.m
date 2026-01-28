% ... [วางโค้ดส่วนที่ 1-4 เดิมของคุณไว้ด้านบนตรงนี้ ห้ามแก้] ...

% ==========================================
% SECTION 5: ACCELERATION ANALYSIS
% ==========================================
alpha_Yellow = 0; % Input Angular Acceleration (Assume constant speed)

% Function to solve 2 unknowns (alpha_out1, alpha_out2)
% Equation form: Vec_Link1 + Vec_Link2 = Vec_Link3 + Vec_Link4
solve_alpha = @(r1, t1, w1, r2, t2, w2, r3, t3, w3, a3, r4, t4, w4, a4) ...
    [(imag(1i*r1*exp(1i*t1)*(-r1*w1^2*exp(1i*t1) + 1i*r2*a3*exp(1i*t2) - r2*w2^2*exp(1i*t2) - 1i*r3*a3*exp(1i*t3) + r3*w3^2*exp(1i*t3) + 1i*r4*a4*exp(1i*t4) - r4*w4^2*exp(1i*t4)))/imag(1i*r1*exp(1i*t1))); ...
     0]; 
% หมายเหตุ: ฟังก์ชันด้านบนซับซ้อนเกินไป ขอใช้วิธี Cramer's Rule แบบ Explicit ด้านล่างจะแม่นยำกว่าตาม Format อาจารย์

% --- Loop 1: Green(2) + Yellow(3) - Grey(4) - Ground = 0 ---
% Unknowns: alpha_Green (2), alpha_Grey (4)
% Known: alpha_Yellow (3)

calculate_loop1_alpha = @(a, th2, w2, b, th3, w3, a3, c, th4, w4) ...
    inv([ -a*sin(th2), c*sin(th4); a*cos(th2), -c*cos(th4) ]) * ...
    [ b*a3*sin(th3) + b*w3^2*cos(th3) + a*w2^2*cos(th2) - c*w4^2*cos(th4); ...
     -b*a3*cos(th3) + b*w3^2*sin(th3) + a*w2^2*sin(th2) - c*w4^2*sin(th4) ];

% Case 1
sol_L1_1 = calculate_loop1_alpha(L2_Loop1, q2_L1_open, w_Green_1, L3_Loop1, q3, w_Yellow, alpha_Yellow, L4_Shared, q4_L1_open, w_Grey_1);
alpha_Green_1 = sol_L1_1(1);
alpha_Grey_1 = sol_L1_1(2);

% Case 2
sol_L1_2 = calculate_loop1_alpha(L2_Loop1, q2_L1_cross, w_Green_2, L3_Loop1, q3, w_Yellow, alpha_Yellow, L4_Shared, q4_L1_cross, w_Grey_2);
alpha_Green_2 = sol_L1_2(1);
alpha_Grey_2 = sol_L1_2(2);

% --- Loop 2: Cyan(2) + Red(3) - Grey(4) - Ground = 0 ---
% Unknowns: alpha_Cyan (2), alpha_Red (3)
% Known: alpha_Grey (4) (from Loop 1) -> Here Grey is on RHS
% Eq: A_Cyan + A_Red = A_Grey

calculate_loop2_alpha = @(a, th_cy, w_cy, b, th_rd, w_rd, c, th_gr, w_gr, a_gr) ...
    inv([ -a*sin(th_cy), -b*sin(th_rd); a*cos(th_cy), b*cos(th_rd) ]) * ...
    [ -c*a_gr*sin(th_gr) - c*w_gr^2*cos(th_gr) + a*w_cy^2*cos(th_cy) + b*w_rd^2*cos(th_rd); ...
       c*a_gr*cos(th_gr) - c*w_gr^2*sin(th_gr) + a*w_cy^2*sin(th_cy) + b*w_rd^2*sin(th_rd) ];

% Case 1
sol_L2_1 = calculate_loop2_alpha(L2_Loop2, q2_Cyan_1, w_Cyan_1, L3_Loop2, q3_Red_1, w_Red_1, L4_Shared, q4_L1_open, w_Grey_1, alpha_Grey_1);
alpha_Cyan_1 = sol_L2_1(1);
alpha_Red_1 = sol_L2_1(2);

% Case 2
sol_L2_2 = calculate_loop2_alpha(L2_Loop2, q2_Cyan_2, w_Cyan_2, L3_Loop2, q3_Red_2, w_Red_2, L4_Shared, q4_L1_cross, w_Grey_2, alpha_Grey_2);
alpha_Cyan_2 = sol_L2_2(1);
alpha_Red_2 = sol_L2_2(2);

% --- Loop 3: Cyan_Up(2) + Blue(3) - Brown(4) - Ground = 0 ---
% Unknowns: alpha_Blue (3), alpha_Brown (4)
% Known: alpha_Cyan (2) (from Loop 2)

calculate_loop3_alpha = @(a, th_cy, w_cy, a_cy, b, th_bl, w_bl, c, th_br, w_br) ...
    inv([ -b*sin(th_bl), c*sin(th_br); b*cos(th_bl), -c*cos(th_br) ]) * ...
    [ a*a_cy*sin(th_cy) + a*w_cy^2*cos(th_cy) + b*w_bl^2*cos(th_bl) - c*w_br^2*cos(th_br); ...
     -a*a_cy*cos(th_cy) + a*w_cy^2*sin(th_cy) + b*w_bl^2*sin(th_bl) - c*w_br^2*sin(th_br) ];

% Case 1
sol_L3_1 = calculate_loop3_alpha(L2_Loop3, q_in_3_1, w_Cyan_1, alpha_Cyan_1, L3_Loop3, q3_Blue_1, w_Blue_1, L4_Loop3, q4_Brown_1, w_Brown_1);
alpha_Blue_1 = sol_L3_1(1);
alpha_Brown_1 = sol_L3_1(2);

% Case 2
sol_L3_2 = calculate_loop3_alpha(L2_Loop3, q_in_3_2, w_Cyan_2, alpha_Cyan_2, L3_Loop3, q3_Blue_2, w_Blue_2, L4_Loop3, q4_Brown_2, w_Brown_2);
alpha_Blue_2 = sol_L3_2(1);
alpha_Brown_2 = sol_L3_2(2);

% ==========================================
% SECTION 6: DISPLAY ACCELERATION RESULTS
% ==========================================
disp('======================================');
disp('    ACCELERATION RESULTS (rad/s^2)');
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
% SECTION 7: ACCELERATION VECTORS & PLOTTING
% ==========================================
% Formula: A_vec = (j*L*alpha - L*w^2) * exp(j*theta)
% Note: This is Tangential + Normal Acceleration

% --- Case 1 Calculation ---
A_Green_1 = (1i*L2_Loop1*alpha_Green_1 - L2_Loop1*w_Green_1^2) * exp(1i*(q2_L1_open+offset));
A_Yellow_Rel_1 = (1i*L3_Loop1*alpha_Yellow - L3_Loop1*w_Yellow^2) * exp(1i*(q3+offset));
A_Grey_1 = (1i*L4_Shared*alpha_Grey_1 - L4_Shared*w_Grey_1^2) * exp(1i*(q4_L1_open+offset));
A_Cyan_1 = (1i*L2_Loop2*alpha_Cyan_1 - L2_Loop2*w_Cyan_1^2) * exp(1i*(q2_Cyan_1+offset));
A_Red_Rel_1 = (1i*L3_Loop2*alpha_Red_1 - L3_Loop2*w_Red_1^2) * exp(1i*(q3_Red_1+offset));
A_Cyan_Up_1 = (1i*L2_Loop3*alpha_Cyan_1 - L2_Loop3*w_Cyan_1^2) * exp(1i*(q_in_3_1+offset));
A_Blue_Rel_1 = (1i*L3_Loop3*alpha_Blue_1 - L3_Loop3*w_Blue_1^2) * exp(1i*(q3_Blue_1+offset));
A_Brown_1 = (1i*L4_Loop3*alpha_Brown_1 - L4_Loop3*w_Brown_1^2) * exp(1i*(q4_Brown_1+offset));

% --- Case 2 Calculation ---
A_Green_2 = (1i*L2_Loop1*alpha_Green_2 - L2_Loop1*w_Green_2^2) * exp(1i*(q2_L1_cross+offset));
A_Yellow_Rel_2 = (1i*L3_Loop1*alpha_Yellow - L3_Loop1*w_Yellow^2) * exp(1i*(q3+offset));
A_Grey_2 = (1i*L4_Shared*alpha_Grey_2 - L4_Shared*w_Grey_2^2) * exp(1i*(q4_L1_cross+offset));
A_Cyan_2 = (1i*L2_Loop2*alpha_Cyan_2 - L2_Loop2*w_Cyan_2^2) * exp(1i*(q2_Cyan_2+offset));
A_Red_Rel_2 = (1i*L3_Loop2*alpha_Red_2 - L3_Loop2*w_Red_2^2) * exp(1i*(q3_Red_2+offset));
A_Cyan_Up_2 = (1i*L2_Loop3*alpha_Cyan_2 - L2_Loop3*w_Cyan_2^2) * exp(1i*(q_in_3_2+offset));
A_Blue_Rel_2 = (1i*L3_Loop3*alpha_Blue_2 - L3_Loop3*w_Blue_2^2) * exp(1i*(q3_Blue_2+offset));
A_Brown_2 = (1i*L4_Loop3*alpha_Brown_2 - L4_Loop3*w_Brown_2^2) * exp(1i*(q4_Brown_2+offset));

% --- Plotting Acceleration Quivers ---
AccScale = 0.05; % Scale factor for visualization

% Case 1
figure(1);
quiver(real(R_Green), imag(R_Green), real(A_Green_1)*AccScale, imag(A_Green_1)*AccScale, 0, 'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(R_Green), imag(R_Green), real(A_Yellow_Rel_1)*AccScale, imag(A_Yellow_Rel_1)*AccScale, 0, 'Color', 'y', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(RO4O2x, RO4O2y, real(A_Grey_1)*AccScale, imag(A_Grey_1)*AccScale, 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(R_Cyan), imag(R_Cyan), real(A_Cyan_1)*AccScale, imag(A_Cyan_1)*AccScale, 0, 'c', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(R_Cyan), imag(R_Cyan), real(A_Red_Rel_1)*AccScale, imag(A_Red_Rel_1)*AccScale, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(R_Cyan_Up), imag(R_Cyan_Up), real(A_Cyan_Up_1)*AccScale, imag(A_Cyan_Up_1)*AccScale, 0, 'c', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(R_Cyan_Up), imag(R_Cyan_Up), real(A_Blue_Rel_1)*AccScale, imag(A_Blue_Rel_1)*AccScale, 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(RO4O2x, RO4O2y, real(A_Brown_1)*AccScale, imag(A_Brown_1)*AccScale, 0, 'Color', [0.6 0.3 0], 'LineWidth', 2, 'MaxHeadSize', 0.5);

% Case 2
figure(2);
quiver(real(R_Green_2), imag(R_Green_2), real(A_Green_2)*AccScale, imag(A_Green_2)*AccScale, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(R_Green_2), imag(R_Green_2), real(A_Yellow_Rel_2)*AccScale, imag(A_Yellow_Rel_2)*AccScale, 0, 'y', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(RO4O2x, RO4O2y, real(A_Grey_2)*AccScale, imag(A_Grey_2)*AccScale, 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(R_Cyan_2), imag(R_Cyan_2), real(A_Cyan_2)*AccScale, imag(A_Cyan_2)*AccScale, 0, 'c', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(R_Cyan_2), imag(R_Cyan_2), real(A_Red_Rel_2)*AccScale, imag(A_Red_Rel_2)*AccScale, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(R_Cyan_Up_2), imag(R_Cyan_Up_2), real(A_Cyan_Up_2)*AccScale, imag(A_Cyan_Up_2)*AccScale, 0, 'c', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(R_Cyan_Up_2), imag(R_Cyan_Up_2), real(A_Blue_Rel_2)*AccScale, imag(A_Blue_Rel_2)*AccScale, 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(RO4O2x, RO4O2y, real(A_Brown_2)*AccScale, imag(A_Brown_2)*AccScale, 0, 'Color', [0.6 0.3 0], 'LineWidth', 2, 'MaxHeadSize', 0.5);
