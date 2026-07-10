function [p4, R04] = qarmForwardKinematics(phi)
%QARMFORWARDKINEMATICS Compute the QArm end-effector pose.
%   phi is the 4-by-1 vector of physical QArm joint angles. p4 is the
%   end-effector position in frame {0}, and R04 is its orientation.

% Manipulator dimensions (m)
L1 = 0.1400;
L2 = 0.3500;
L3 = 0.0500;
L4 = 0.2500;
L5 = 0.1500;

% Equivalent geometric parameters
l1 = L1;
l2 = sqrt(L2^2 + L3^2);
l3 = L4 + L5;
beta = atan(L3/L2);

% Convert the physical joint coordinates phi to DH coordinates theta.
theta = phi;
theta(1) = phi(1);
theta(2) = phi(2) + beta - pi/2;
theta(3) = phi(3) - beta;
theta(4) = phi(4);

% Standard DH table:
% i       a_i       alpha_i       d_i       theta_i
% 1        0          -pi/2        l1        theta(1)
% 2        l2          0            0        theta(2)
% 3        0          -pi/2         0        theta(3)
% 4        0           0            l3       theta(4)
T01 = quanser_arm_DH(0,  -pi/2, l1, theta(1));
T12 = quanser_arm_DH(l2,  0,     0, theta(2));
T23 = quanser_arm_DH(0,  -pi/2,  0, theta(3));
T34 = quanser_arm_DH(0,   0,    l3, theta(4));

T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;

p4 = T04(1:3, 4);
R04 = T04(1:3, 1:3);
end

function T = quanser_arm_DH(a, alpha, d, theta)
%QUANSER_ARM_DH Standard DH homogeneous transform.
T_R_z = [cos(theta), -sin(theta), 0, 0;
         sin(theta),  cos(theta), 0, 0;
         0,           0,          1, 0;
         0,           0,          0, 1];

T_T_z = [1, 0, 0, 0;
         0, 1, 0, 0;
         0, 0, 1, d;
         0, 0, 0, 1];

T_T_x = [1, 0, 0, a;
         0, 1, 0, 0;
         0, 0, 1, 0;
         0, 0, 0, 1];

T_R_x = [1, 0,          0,           0;
         0, cos(alpha), -sin(alpha), 0;
         0, sin(alpha),  cos(alpha), 0;
         0, 0,           0,           1];

T = T_R_z*T_T_z*T_T_x*T_R_x;
end
