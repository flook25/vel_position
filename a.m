% ... [CODE PART 1-4 ของคุณ ห้ามแก้ไข] ...

% ==========================================
% SECTION 5: ACCELERATION ANALYSIS
% ==========================================
alpha_Yellow = -0.8; % Input Angular Acceleration (rad/s^2)

% Function to solve for Alpha (Angular Acceleration)
% Equation: A_Green + A_Yellow - A_Grey = 0
% Rearranged: [Coeff_Green, Coeff_Grey] * [alpha_Green; alpha_Grey] = [Known_Terms]
% Vector A = (j*L*alpha - L*w^2) * exp(j*theta)

solve_acc = @(a, th2, w2, b, th3, w3, a3, c, th4, w4) ...
    inv([ real(1i*a*exp(1i*th2)), real(-1i*c*exp(1i*th4)); ...
          imag(1i*a*exp(1i*th2)), imag(-1i*c*exp(1i*th4)) ]) * ...
    [ real( a*w2^2*exp(1i*th2) + b*w3^2*exp(1i*th3) - 1i*b*a3*exp(1i*th3) - c*w4^2*exp(1i*th4) ); ...
      imag( a*w2^2*exp(1i*th2) + b*w3^2*exp(1i*th3) - 1i*b*a3*exp(1i*th3) - c*w4^2*exp(1i*th4) ) ];

% --- Loop 1 Analysis (Green-Yellow-Grey) ---
% Unknowns: alpha_Green, alpha_Grey

% Case 1 (Open)
sol_L1_1 = solve_acc(L2_Loop1, q2_L1_open, w_Green_1, L3_Loop1, q3, w_Yellow, alpha_Yellow, L4_Shared, q4_L1_open, w_Grey_1);
alpha_Green_1 = sol_L1_1(1);
alpha_Grey_1 = sol_L1_1(2);

% Case 2 (Crossed)
sol_L1_2 = solve_acc(L2_Loop1, q2_L1_cross, w_Green_2, L3_Loop1, q3, w_Yellow, alpha_Yellow, L4_Shared, q4_L1_cross, w_Grey_2);
alpha_Green_2 = sol_L1_2(1);
alpha_Grey_2 = sol_L1_2(2);

% --- Loop 2 Analysis (Cyan-Red-Grey) ---
% Unknowns: alpha_Cyan, alpha_Red
% Known Input: alpha_Grey (from Loop 1)
% Equation: A_Cyan + A_Red - A_Grey = 0

solve_acc_L2 = @(a, th_cy, w_cy, b, th_rd, w_rd, c, th_gr, w_gr, a_gr) ...
    inv([ real(1i*a*exp(1i*th_cy)), real(1i*b*exp(1i*th_rd)); ...
          imag(1i*a*exp(1i*th_cy)), imag(1i*b*exp(1i*th_rd)) ]) * ...
    [ real( a*w_cy^2*exp(1i*th_cy) + b*w_rd^2*exp(1i*th_rd) - c*w_gr^2*exp(1i*th_gr) + 1i*c*a_gr*exp(1i*th_gr) ); ...
      imag( a*w_cy^2*exp(1i*th_cy) + b*w_rd^2*exp(1i*th_rd) - c*w_gr^2*exp(1i*th_gr) + 1i*c*a_gr*exp(1i*th_gr) ) ];

% Case 1
sol_L2_1 = solve_acc_L2(L2_Loop2, q2_Cyan_1, w_Cyan_1, L3_Loop2, q3_Red_1, w_Red_1, L4_Shared, q4_L1_open, w_Grey_1, alpha_Grey_1);
alpha_Cyan_1 = sol_L2_1(1);
alpha_Red_1 = sol_L2_1(2);

% Case 2
sol_L2_2 = solve_acc_L2(L2_Loop2, q2_Cyan_2, w_Cyan_2, L3_Loop2, q3_Red_2, w_Red_2, L4_Shared, q4_L1_cross, w_Grey_2, alpha_Grey_2);
alpha_Cyan_2 = sol_L2_2(1);
alpha_Red_2 = sol_L2_2(2);

% --- Loop 3 Analysis (Cyan-Blue-Brown) ---
% Unknowns: alpha_Brown, alpha_Blue
% Known Input: alpha_Cyan (from Loop 2)
% Equation: A_Cyan + A_Blue - A_Brown = 0

solve_acc_L3 = @(a, th_cy, w_cy, a_cy, b, th_bl, w_bl, c, th_br, w_br) ...
    inv([ real(-1i*c*exp(1i*th_br)), real(1i*b*exp(1i*th_bl)); ...
          imag(-1i*c*exp(1i*th_br)), imag(1i*b*exp(1i*th_bl)) ]) * ...
    [ real( -a*a_cy*1i*exp(1i*th_cy) + a*w_cy^2*exp(1i*th_cy) - c*w_br^2*exp(1i*th_br) + b*w_bl^2*exp(1i*th_bl) ); ...
      imag( -a*a_cy*1i*exp(1i*th_cy) + a*w_cy^2*exp(1i*th_cy) - c*w_br^2*exp(1i*th_br) + b*w_bl^2*exp(1i*th_bl) ) ];
      
% หมายเหตุ: การย้ายข้างสมการใน Loop 3 อาจซับซ้อน ผมใช้ Solver แบบ Matrix เพื่อความชัวร์ที่สุด
% Eq: A_Brown * (vec) + A_Blue * (vec) = RHS terms

sol_L3_1 = solve_acc_L3(L2_Loop3, q_in_3_1, w_Cyan_1, alpha_Cyan_1, L3_Loop3, q3_Blue_1, w_Blue_1, L4_Loop3, q4_Brown_1, w_Brown_1);
alpha_Brown_1 = sol_L3_1(1);
alpha_Blue_1 = sol_L3_1(2);

sol_L3_2 = solve_acc_L3(L2_Loop3, q_in_3_2, w_Cyan_2, alpha_Cyan_2, L3_Loop3, q3_Blue_2, w_Blue_2, L4_Loop3, q4_Brown_2, w_Brown_2);
alpha_Brown_2 = sol_L3_2(1);
alpha_Blue_2 = sol_L3_2(2);

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
% SECTION 7: VECTORS & PLOTTING
% ==========================================
% Vector A = (j*L*alpha - L*w^2) * exp(j*theta)

% --- Calculate Vectors (Case 1) ---
Ag_1 = (1i*L2_Loop1*alpha_Green_1 - L2_Loop1*w_Green_1^2)*exp(1i*(q2_L1_open+offset));
Ay_1 = (1i*L3_Loop1*alpha_Yellow - L3_Loop1*w_Yellow^2)*exp(1i*(q3+offset));
Agy_1 = (1i*L4_Shared*alpha_Grey_1 - L4_Shared*w_Grey_1^2)*exp(1i*(q4_L1_open+offset));
Ac_1 = (1i*L2_Loop2*alpha_Cyan_1 - L2_Loop2*w_Cyan_1^2)*exp(1i*(q2_Cyan_1+offset));
Ar_1 = (1i*L3_Loop2*alpha_Red_1 - L3_Loop2*w_Red_1^2)*exp(1i*(q3_Red_1+offset));
Acup_1 = (1i*L2_Loop3*alpha_Cyan_1 - L2_Loop3*w_Cyan_1^2)*exp(1i*(q_in_3_1+offset));
Abl_1 = (1i*L3_Loop3*alpha_Blue_1 - L3_Loop3*w_Blue_1^2)*exp(1i*(q3_Blue_1+offset));
Abr_1 = (1i*L4_Loop3*alpha_Brown_1 - L4_Loop3*w_Brown_1^2)*exp(1i*(q4_Brown_1+offset));

% --- Calculate Vectors (Case 2) ---
Ag_2 = (1i*L2_Loop1*alpha_Green_2 - L2_Loop1*w_Green_2^2)*exp(1i*(q2_L1_cross+offset));
Ay_2 = (1i*L3_Loop1*alpha_Yellow - L3_Loop1*w_Yellow^2)*exp(1i*(q3+offset));
Agy_2 = (1i*L4_Shared*alpha_Grey_2 - L4_Shared*w_Grey_2^2)*exp(1i*(q4_L1_cross+offset));
Ac_2 = (1i*L2_Loop2*alpha_Cyan_2 - L2_Loop2*w_Cyan_2^2)*exp(1i*(q2_Cyan_2+offset));
Ar_2 = (1i*L3_Loop2*alpha_Red_2 - L3_Loop2*w_Red_2^2)*exp(1i*(q3_Red_2+offset));
Acup_2 = (1i*L2_Loop3*alpha_Cyan_2 - L2_Loop3*w_Cyan_2^2)*exp(1i*(q_in_3_2+offset));
Abl_2 = (1i*L3_Loop3*alpha_Blue_2 - L3_Loop3*w_Blue_2^2)*exp(1i*(q3_Blue_2+offset));
Abr_2 = (1i*L4_Loop3*alpha_Brown_2 - L4_Loop3*w_Brown_2^2)*exp(1i*(q4_Brown_2+offset));

% --- Plotting ---
S = 0.05; % Scale factor for acceleration vectors

% Case 1
figure(1);
quiver(real(R_Green), imag(R_Green), real(Ag_1)*S, imag(Ag_1)*S, 0, 'Color', 'g', 'LineWidth', 2);
quiver(real(R_Green), imag(R_Green), real(Ay_1)*S, imag(Ay_1)*S, 0, 'Color', 'y', 'LineWidth', 2);
quiver(RO4O2x, RO4O2y, real(Agy_1)*S, imag(Agy_1)*S, 0, 'k', 'LineWidth', 2);
quiver(real(R_Cyan), imag(R_Cyan), real(Ac_1)*S, imag(Ac_1)*S, 0, 'c', 'LineWidth', 2);
quiver(real(R_Cyan), imag(R_Cyan), real(Ar_1)*S, imag(Ar_1)*S, 0, 'r', 'LineWidth', 2);
quiver(RO4O2x, RO4O2y, real(Abr_1)*S, imag(Abr_1)*S, 0, 'Color', [0.6 0.3 0], 'LineWidth', 2);

% Case 2
figure(2);
quiver(real(R_Green_2), imag(R_Green_2), real(Ag_2)*S, imag(Ag_2)*S, 0, 'g', 'LineWidth', 2);
quiver(real(R_Green_2), imag(R_Green_2), real(Ay_2)*S, imag(Ay_2)*S, 0, 'y', 'LineWidth', 2);
quiver(RO4O2x, RO4O2y, real(Agy_2)*S, imag(Agy_2)*S, 0, 'k', 'LineWidth', 2);
quiver(real(R_Cyan_2), imag(R_Cyan_2), real(Ac_2)*S, imag(Ac_2)*S, 0, 'c', 'LineWidth', 2);
quiver(real(R_Cyan_2), imag(R_Cyan_2), real(Ar_2)*S, imag(Ar_2)*S, 0, 'r', 'LineWidth', 2);
quiver(RO4O2x, RO4O2y, real(Abr_2)*S, imag(Abr_2)*S, 0, 'Color', [0.6 0.3 0], 'LineWidth', 2);
