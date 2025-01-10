%-----------------Define underlying parameters---------------------%
                %---Mass properties - Deducted from CAD model---%u
m_1 = 120*10^-3;                             %---Mass of Active Link---%
m_2 = 150*10^-3;                               %---Mass of Simplified Passive Link---%
m_0 = 80*10^-3;                                %---Mass of End-Effector---%

                %---Theoretical Friction coefficients---%
f_v1 = 0.055;                            %---Viscous friction of Active Link 1---%
f_v2 = 0.045;                            %---Viscous friction of Active Link 2---%
f_v3 = 0.050;                            %---Viscous friction of Active Link 3---%

f_c1 = 0.025;                            %---Coulomb friction of Active Link 1---%
f_c2 = 0.024;                            %---Coulomb friction of Active Link 2---%
f_c3 = 0.021;                            %---Coulomb friction of Active Link 3---%

g    = 9.8;                              %---Gravitational acceleration---%

                %---Size properties - Deducted from CAD model---%
l_1c = 85*10^-3;                               %---COM of Active Link---%
l_1  = 176*10^-3;                              %---Length of Active Link (in other word: rf)---%
l_2  = 330*10^-3;                              %---Length of Passive Link (in other word: re)---%

f = 60*10^-3;
e = 45*10^-3;

alpha1 = deg2rad(-30);
alpha2 = deg2rad(90);
alpha3 = deg2rad(210);


I_1 = 256400.64*10^-9 + m_1*l_1^2;                         %---yy Inertia Moment of Active Link w.r.t the connecting joint---%
                                                           %0.00657170688%
%----------------------------Load data-------------------------------%
angle1 = deg2rad(load("simulated_angle1.mat","angle1").("angle1"));
angle2 = deg2rad(load("simulated_angle2.mat","angle2").("angle2"));
angle3 = deg2rad(load("simulated_angle3.mat","angle3").("angle3"));

angle1_dot = deg2rad(load("simulated_angle1_dot.mat","angle1_dot").("angle1_dot"));
angle2_dot = deg2rad(load("simulated_angle2_dot.mat","angle2_dot").("angle2_dot"));
angle3_dot = deg2rad(load("simulated_angle3_dot.mat","angle3_dot").("angle3_dot"));

angle1_Ddot = deg2rad(load("simulated_angle1_Ddot.mat","angle1_Ddot").("angle1_Ddot"));
angle2_Ddot = deg2rad(load("simulated_angle2_Ddot.mat","angle2_Ddot").("angle2_Ddot"));
angle3_Ddot = deg2rad(load("simulated_angle3_Ddot.mat","angle3_Ddot").("angle3_Ddot"));

data_x = load("simulated_x.mat","data_x").("data_x");
data_y = load("simulated_y.mat","data_y").("data_y");
data_z = load("simulated_z.mat","data_z").("data_z");

data_x_Ddot = load("simulated_x_Ddot.mat","data_x_Ddot").("data_x_Ddot");
data_y_Ddot = load("simulated_y_Ddot.mat","data_y_Ddot").("data_y_Ddot");
data_z_Ddot = load("simulated_z_Ddot.mat","data_z_Ddot").("data_z_Ddot");

%------------------------------------------------------------------------------------%
                %------------Torque data Generation------------%
                                                                                   
tau = zeros(3, 1, 50000);
tau_first_row = zeros(1, 50000);
tau_second_row = zeros(1, 50000);
tau_third_row = zeros(1, 50000);


for i = 3:49998

    %------------Data aquisition-----------%
    %---Angular position, speed and acceleration---%
    angle_1 = angle1(2, i); 
    angle_2 = angle2(2, i);
    angle_3 = angle3(2, i); 
    angle   = [angle_1; angle_2; angle_3];

    angle_1_dot = angle1_dot(2, i);
    angle_2_dot = angle2_dot(2, i);
    angle_3_dot = angle3_dot(2, i);
    angle_dot   = [angle_1_dot; angle_2_dot; angle_3_dot];


    angle_1_Ddot = angle1_Ddot(2, i);
    angle_2_Ddot = angle2_Ddot(2, i);
    angle_3_Ddot = angle3_Ddot(2, i);
    angle_Ddot   = [angle_1_Ddot; angle_2_Ddot; angle_3_Ddot];

    %---End-effector XYZ coordinates and accelerations---%
    x = data_x(2, i)*10^-3;
    y = data_y(2, i)*10^-3;
    z = data_z(2, i)*10^-3;

    x_Ddot = data_x_Ddot(2, i)*10^-3;
    y_Ddot = data_y_Ddot(2, i)*10^-3;
    z_Ddot = data_z_Ddot(2, i)*10^-3;

    %---Define related matrices---%
    I = (2 * I_1 + m_2 * l_1 ^ 2)*eye(3);                                               %---Mass matrix---%
    G = [cos(angle_1); cos(angle_2); cos(angle_3)] * (m_1 * l_1c + m_2 * l_1) * g;      %---G(q) matrix---%
    K = [((x * cos(alpha1) + y * sin(alpha1) + f - e) * sin(angle_1) - z * cos(angle_1)), 0, 0;
         0, ((x * cos(alpha2) + y * sin(alpha2) + f - e) * sin(angle_2) - z * cos(angle_2)), 0;
         0, 0, ((x * cos(alpha3) + y * sin(alpha3) + f - e) * sin(angle_3) - z * cosd(angle_3))];   
                                                                                        %---K(q) matrix---%
    
    %---Define elements of A matrix---%                                                                                    
    a_11 = x + e * cos(alpha1) - f * cos(alpha1) - l_1 * cos(alpha1) * cos(angle_1);
    a_12 = x + e * cos(alpha2) - f * cos(alpha2) - l_1 * cos(alpha2) * cos(angle_2);
    a_13 = x + e * cos(alpha3) - f * cos(alpha3) - l_1 * cos(alpha3) * cos(angle_3);
    
    a_21 = y + e * sin(alpha1) - f * sin(alpha1) - l_1 * sin(alpha1) * cos(angle_1);
    a_22 = y + e * sin(alpha2) - f * sin(alpha2) - l_1 * sin(alpha2) * cos(angle_2);
    a_23 = y + e * sin(alpha3) - f * sin(alpha3) - l_1 * sin(alpha3) * cos(angle_3);
    
    a_31 = z - l_1 * cos(angle_1);
    a_32 = z - l_1 * cos(angle_2);
    a_33 = z - l_1 * cos(angle_3);
    
    A = [a_11 a_12 a_13; a_21 a_22 a_23; a_31 a_32 a_33];                               %---A matrix---%
    
    B = [(m_0 + 3 * m_2) * x_Ddot; (m_0 + 3 * m_2) * y_Ddot; (m_0 + 3 * m_2) * (z_Ddot - g)];
                                                                                        %---B matrix---%
    F_v = [f_v1 0 0; 0 f_v2 0; 0 0 f_v3];                                               %---Viscous friction matrix---%
    F_c = [f_c1 0 0; 0 f_c2 0; 0 0 f_c3];                                               %---Coulomb friction matrix---%

    tau(:, :, i) = I * angle_Ddot - K * inv(A) * B + G + F_v * angle_dot + F_c * sign(angle_dot);
    tau_first_row(i) = tau(1, 1, i);
    tau_second_row(i) = tau(2, 1, i);
    tau_third_row(i) = tau(3, 1, i);
end

figure;
plot(1:50000, tau_first_row, 'r');
hold on;
plot(1:50000, tau_second_row, 'g');
plot(1:50000, tau_third_row, 'b');
title('First Row of Every Layer of tau');
xlabel('Index');
ylabel('Torque');
legend('First Row of tau');

save("tau_2.mat", "tau_first_row", "tau_second_row", "tau_third_row")