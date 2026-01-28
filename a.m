% ... (โค้ดส่วนที่ 1-4 ของคุณ วางไว้ด้านบน) ...

% ==========================================
% SECTION 5: ACCELERATION ANALYSIS
% ==========================================
alpha_Yellow = -0.8; % Input Angular Acceleration (rad/s^2)

% Function to solve Linear System for Alpha [A][x] = [B]
% Based on Loop Equation: A_vec1 + A_vec2 + ... = 0
% We separate terms with Alpha (Unknowns) to LHS, others to RHS.
% General form: Coeff1*alpha1 + Coeff2*alpha2 = RHS_Terms

solve_alpha_matrix = @(r1, t1, w1, r2, t2, w2, rhs_vec) ...
    [real(1i*r1*exp(1i*t1)), real(1i*r2*exp(1i*t2)); ...
     imag(1i*r1*exp(1i*t1)), imag(1i*r2*exp(1i*t2))] \ ...
    [real(rhs_vec); imag(rhs_vec)];

% --- LOOP 1: Green(2) + Yellow(3) - Grey(4) - Ground = 0 ---
% Acceleration Eq: A_Green + A_Yellow - A_Grey = 0
% Unknowns: alpha_Green, alpha_Grey
% Rearranged: A_Green - A_Grey = -A_Yellow
% Terms involving w^2 are moved to RHS.

% RHS Term for Loop 1 (Group all knowns: w terms and alpha_Yellow terms)
% Equation: (j*a*alpha_Green - a*w_Green^2)*e^(j*t_Green) + ...
%           (j*b*alpha_Yellow - b*w_Yellow^2)*e^(j*t_Yellow) - ...
%           (j*c*alpha_Grey - c*w_Grey^2)*e^(j*t_Grey) = 0

% Case 1 (Open)
RHS_L1_1 = - (1i*L3_Loop1*alpha_Yellow - L3_Loop1*w_Yellow^2)*exp(1i*(q3)) ...
           + L2_Loop1*w_Green_1^2*exp(1i*q2_L1_open) ...
           - L4_Shared*w_Grey_1^2*exp(1i*q4_L1_open);
           
alphas_L1_1 = solve_alpha_matrix(L2_Loop1, q2_L1_open, 0, -L4_Shared, q4_L1_open, 0, RHS_L1_1);
alpha_Green_1 = alphas_L1_1(1);
alpha_Grey_1 = alphas_L1_1(2);

% Case 2 (Crossed)
RHS_L1_2 = - (1i*L3_Loop1*alpha_Yellow - L3_Loop1*w_Yellow^2)*exp(1i*(q3)) ...
           + L2_Loop1*w_Green_2^2*exp(1i*q2_L1_cross) ...
           - L4_Shared*w_Grey_2^2*exp(1i*q4_L1_cross);

alphas_L1_2 = solve_alpha_matrix(L2_Loop1, q2_L1_cross, 0, -L4_Shared, q4_L1_cross, 0, RHS_L1_2);
alpha_Green_2 = alphas_L1_2(1);
alpha_Grey_2 = alphas_L1_2(2);


% --- LOOP 2: Cyan(2) + Red(3) - Grey(4) = 0 ---
% Acceleration Eq: A_Cyan + A_Red - A_Grey = 0
% Unknowns: alpha_Cyan, alpha_Red
% Known: A_Grey (calculated from Loop 1)

% Case 1
A_Grey_Vec_1 = (1i*L4_Shared*alpha_Grey_1 - L4_Shared*w_Grey_1^2)*exp(1i*q4_L1_open);
RHS_L2_1 = A_Grey_Vec_1 ...
           + L2_Loop2*w_Cyan_1^2*exp(1i*q2_Cyan_1) ...
           + L3_Loop2*w_Red_1^2*exp(1i*q3_Red_1);

alphas_L2_1 = solve_alpha_matrix(L2_Loop2, q2_Cyan_1, 0, L3_Loop2, q3_Red_1, 0, RHS_L2_1);
alpha_Cyan_1 = alphas_L2_1(1);
alpha_Red_1 = alphas_L2_1(2);

% Case 2
A_Grey_Vec_2 = (1i*L4_Shared*alpha_Grey_2 - L4_Shared*w_Grey_2^2)*exp(1i*q4_L1_cross);
RHS_L2_2 = A_Grey_Vec_2 ...
           + L2_Loop2*w_Cyan_2^2*exp(1i*q2_Cyan_2) ...
           + L3_Loop2*w_Red_2^2*exp(1i*q3_Red_2);

alphas_L2_2 = solve_alpha_matrix(L2_Loop2, q2_Cyan_2, 0, L3_Loop2, q3_Red_2, 0, RHS_L2_2);
alpha_Cyan_2 = alphas_L2_2(1);
alpha_Red_2 = alphas_L2_2(2);


% --- LOOP 3: Cyan(2) + Blue(3) - Brown(4) = 0 ---
% Acceleration Eq: A_Cyan_Up + A_Blue - A_Brown = 0
% Unknowns: alpha_Blue, alpha_Brown
% Known: A_Cyan (calculated from Loop 2)

% Case 1
A_Cyan_Up_Vec_1 = (1i*L2_Loop3*alpha_Cyan_1 - L2_Loop3*w_Cyan_1^2)*exp(1i*q_in_3_1);
RHS_L3_1 = - A_Cyan_Up_Vec_1 ...
           + L3_Loop3*w_Blue_1^2*exp(1i*q3_Blue_1) ...
           - L4_Loop3*w_Brown_1^2*exp(1i*q4_Brown_1);

alphas_L3_1 = solve_alpha_matrix(L3_Loop3, q3_Blue_1, 0, -L4_Loop3, q4_Brown_1, 0, RHS_L3_1);
alpha_Blue_1 = alphas_L3_1(1);
alpha_Brown_1 = alphas_L3_1(2);

% Case 2
A_Cyan_Up_Vec_2 = (1i*L2_Loop3*alpha_Cyan_2 - L2_Loop3*w_Cyan_2^2)*exp(1i*q_in_3_2);
RHS_L3_2 = - A_Cyan_Up_Vec_2 ...
           + L3_Loop3*w_Blue_2^2*exp(1i*q3_Blue_2) ...
           - L4_Loop3*w_Brown_2^2*exp(1i*q4_Brown_2);

alphas_L3_2 = solve_alpha_matrix(L3_Loop3, q3_Blue_2, 0, -L4_Loop3, q4_Brown_2, 0, RHS_L3_2);
alpha_Blue_2 = alphas_L3_2(1);
alpha_Brown_2 = alphas_L3_2(2);


% ==========================================
% SECTION 6: DISPLAY ACCELERATION RESULTS
% ==========================================
disp('======================================');
disp('    ACCELERATION RESULTS (rad/s^2)');
disp('======================================');
disp('--- CASE 1 (OPEN) ---');
disp(['  alpha_Green:  ', num2str(alpha_Green_1)]);
disp(['  alpha_Yellow (Input): ', num2str(alpha_Yellow)]);
disp(['  alpha_Grey:   ', num2str(alpha_Grey_1)]);
disp(['  alpha_Cyan:   ', num2str(alpha_Cyan_1)]);
disp(['  alpha_Red:    ', num2str(alpha_Red_1)]);
disp(['  alpha_Blue:   ', num2str(alpha_Blue_1)]);
disp(['  alpha_Brown:  ', num2str(alpha_Brown_1)]);
disp(' ');
disp('--- CASE 2 (CROSSED) ---');
disp(['  alpha_Green:  ', num2str(alpha_Green_2)]);
disp(['  alpha_Yellow (Input): ', num2str(alpha_Yellow)]);
disp(['  alpha_Grey:   ', num2str(alpha_Grey_2)]);
disp(['  alpha_Cyan:   ', num2str(alpha_Cyan_2)]);
disp(['  alpha_Red:    ', num2str(alpha_Red_2)]);
disp(['  alpha_Blue:   ', num2str(alpha_Blue_2)]);
disp(['  alpha_Brown:  ', num2str(alpha_Brown_2)]);


% ==========================================
% SECTION 7: ACCELERATION VECTORS & PLOTTING
% ==========================================
% Formula: A = (j*alpha - w^2) * R_vector
% Note: Using offset for plotting direction

% --- Case 1 Vectors ---
A_Green_Vec_1 = (1i*alpha_Green_1 - w_Green_1^2) * R_Green;
A_Yellow_Rel_Vec_1 = (1i*alpha_Yellow - w_Yellow^2) * R_Yellow;
A_Grey_Vec_1 = (1i*alpha_Grey_1 - w_Grey_1^2) * R_Grey;
A_Cyan_Vec_1 = (1i*alpha_Cyan_1 - w_Cyan_1^2) * R_Cyan;
A_Red_Rel_Vec_1 = (1i*alpha_Red_1 - w_Red_1^2) * R_Red;
A_Cyan_Up_Vec_1 = (1i*alpha_Cyan_1 - w_Cyan_1^2) * R_Cyan_Up;
A_Blue_Rel_Vec_1 = (1i*alpha_Blue_1 - w_Blue_1^2) * R_Blue;
A_Brown_Vec_1 = (1i*alpha_Brown_1 - w_Brown_1^2) * R_Brown;

% --- Case 2 Vectors ---
A_Green_Vec_2 = (1i*alpha_Green_2 - w_Green_2^2) * R_Green_2;
A_Yellow_Rel_Vec_2 = (1i*alpha_Yellow - w_Yellow^2) * R_Yellow_2;
A_Grey_Vec_2 = (1i*alpha_Grey_2 - w_Grey_2^2) * R_Grey_2;
A_Cyan_Vec_2 = (1i*alpha_Cyan_2 - w_Cyan_2^2) * R_Cyan_2;
A_Red_Rel_Vec_2 = (1i*alpha_Red_2 - w_Red_2^2) * R_Red_2;
A_Cyan_Up_Vec_2 = (1i*alpha_Cyan_2 - w_Cyan_2^2) * R_Cyan_Up_2;
A_Blue_Rel_Vec_2 = (1i*alpha_Blue_2 - w_Blue_2^2) * R_Blue_2;
A_Brown_Vec_2 = (1i*alpha_Brown_2 - w_Brown_2^2) * R_Brown_2;

% Scale factor for better visibility
AccScale = 0.02; 

% --- Plot Case 1 ---
figure(1);
quiver(real(R_Green), imag(R_Green), real(A_Green_Vec_1)*AccScale, imag(A_Green_Vec_1)*AccScale, 0, 'Color', [0 0.5 0], 'LineWidth', 2, 'MaxHeadSize', 0.5); % Dark Green
quiver(real(R_Green), imag(R_Green), real(A_Yellow_Rel_Vec_1)*AccScale, imag(A_Yellow_Rel_Vec_1)*AccScale, 0, 'Color', [0.8 0.8 0], 'LineWidth', 2, 'MaxHeadSize', 0.5); % Dark Yellow
quiver(RO4O2x, RO4O2y, real(A_Grey_Vec_1)*AccScale, imag(A_Grey_Vec_1)*AccScale, 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(R_Cyan), imag(R_Cyan), real(A_Cyan_Vec_1)*AccScale, imag(A_Cyan_Vec_1)*AccScale, 0, 'c', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(R_Cyan), imag(R_Cyan), real(A_Red_Rel_Vec_1)*AccScale, imag(A_Red_Rel_Vec_1)*AccScale, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(RO4O2x, RO4O2y, real(A_Brown_Vec_1)*AccScale, imag(A_Brown_Vec_1)*AccScale, 0, 'Color', [0.6 0.3 0], 'LineWidth', 2, 'MaxHeadSize', 0.5);

% --- Plot Case 2 ---
figure(2);
quiver(real(R_Green_2), imag(R_Green_2), real(A_Green_Vec_2)*AccScale, imag(A_Green_Vec_2)*AccScale, 0, 'Color', [0 0.5 0], 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(R_Green_2), imag(R_Green_2), real(A_Yellow_Rel_Vec_2)*AccScale, imag(A_Yellow_Rel_Vec_2)*AccScale, 0, 'Color', [0.8 0.8 0], 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(RO4O2x, RO4O2y, real(A_Grey_Vec_2)*AccScale, imag(A_Grey_Vec_2)*AccScale, 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(R_Cyan_2), imag(R_Cyan_2), real(A_Cyan_Vec_2)*AccScale, imag(A_Cyan_Vec_2)*AccScale, 0, 'c', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(R_Cyan_2), imag(R_Cyan_2), real(A_Red_Rel_Vec_2)*AccScale, imag(A_Red_Rel_Vec_2)*AccScale, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(RO4O2x, RO4O2y, real(A_Brown_Vec_2)*AccScale, imag(A_Brown_Vec_2)*AccScale, 0, 'Color', [0.6 0.3 0], 'LineWidth', 2, 'MaxHeadSize', 0.5);
