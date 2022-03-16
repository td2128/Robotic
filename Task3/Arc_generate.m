clear all
centre = [-8,-20,8];
phi = 0;
tf = 6;
t = 0:1:tf;
r = 4;
full_traj = [];
angle_step = 10;
angle = 180;
start_pos = [centre(1)+r*cosd(0), centre(2)+r*sind(0), centre(3)]';
for i = 1:angle_step:angle
desired_pos = [centre(1)+r*cosd(i), centre(2)+r*sind(i), centre(3)]';
[~, start_ang] = IK(start_pos, 0);
[~, desired_ang] = IK(desired_pos, 0);
theta1 = [start_ang(1), desired_ang(1)];
theta2 = [start_ang(2), desired_ang(2)];
theta3 = [start_ang(3), desired_ang(3)];
theta4 = [start_ang(4), desired_ang(4)];
theta1_traj = trajectory(theta1, t, tf);
theta2_traj = trajectory(theta2, t, tf);
theta3_traj = trajectory(theta3, t, tf);
theta4_traj = trajectory(theta4, t, tf);
full_traj = [full_traj, [theta1_traj; theta2_traj; theta3_traj; theta4_traj]];
start_pos = desired_pos;
end

theta11 = (full_traj(1, :) + 180)./0.088;
theta21 = (full_traj(2, :) + 180)./0.088;
theta31 = (-full_traj(3, :) + 180)./0.088;
theta41 = (-full_traj(4, :) + 180)./0.088;

function [centers, res] = IK(target_end_pos, phi)
a2 = 13;
a3 = 12.4;
a4 = 12.6;
beta = atand(0.024/0.128);
r3 = sqrt(target_end_pos(1)^2 + target_end_pos(2)^2);
z3 = target_end_pos(3) - 7.7;
r2 = r3 - a4*cosd(phi);
z2 = z3 - a4*sind(phi);
cos_theta3 = (r2^2+z2^2-a2^2-a3^2) / (2*a2*a3);
% no solution case
if cos_theta3 < -1 || cos_theta3 > 1
res = 'no solution found';
end
theta3_temp = acosd(cos_theta3);
theta3_temp_ = -acosd(cos_theta3);
theta3 = theta3_temp - beta + 90;
theta3_ = theta3_temp_ - beta + 90;
k1 = a2 + a3 * cosd(theta3_temp) ;
k2 = a3 * sind(theta3_temp);
k2_ = a3 * sind(theta3_temp_);
theta2_temp = atand(z2/r2) - atand(k2/k1);
theta2_temp_ = atand(z2/r2) - atand(k2_/k1);
theta2 = 90 - theta2_temp - beta;
theta2_ = 90 - theta2_temp_ - beta;
theta4 = phi - theta2_temp - theta3_temp;
theta4_ = phi - theta2_temp_ - theta3_temp_;
theta1 = atand(target_end_pos(2)/target_end_pos(1));
theta1_ = -atand(target_end_pos(2)/target_end_pos(1));
if theta1 == 0
theta1 = 0;
theta1_ = 180;
end
res1 = [theta1, theta2, theta3, theta4];
res2 = [theta1, theta2_, theta3_, theta4_];
res3 = [theta1_, theta2, theta3, theta4];
res4 = [theta1_, theta2_, theta3_, theta4_];
% verfity end_pos
[T_mat1, center1_1, center2_1, center3_1, center4_1, center5_1, center6_1] = FK(theta1, theta2, theta3, theta4, beta);
[T_mat2, center1_2, center2_2, center3_2, center4_2, center5_2, center6_2] = FK(theta1, theta2_, theta3_, theta4_, beta);
[T_mat3, center1_3, center2_3, center3_3, center4_3, center5_3, center6_3] = FK(theta1_, theta2, theta3, theta4, beta);
[T_mat4, center1_4, center2_4, center3_4, center4_4, center5_4, center6_4] = FK(theta1_, theta2_, theta3_, theta4_, beta);
T_end_pos1 = T_mat1(1:3, 4);
T_end_pos2 = T_mat2(1:3, 4);
T_end_pos3 = T_mat3(1:3, 4);
T_end_pos4 = T_mat4(1:3, 4);
diff1 = T_end_pos1 - target_end_pos;
diff2 = T_end_pos2 - target_end_pos;
diff3 = T_end_pos3 - target_end_pos;
diff4 = T_end_pos4 - target_end_pos;
res = res2;
centers_2 = [center1_2, center2_2, center3_2, center4_2, center5_2, center6_2];
centers = centers_2;
end

function [T_final, center1, center2, center3, center4, center5, center6] = FK(theta1, theta2, theta3, theta4, beta)
T0 = eye(4);
T1 = T0 * DH(0, 0, 7.7, theta1);
center1 = T1(:, 4);
T2 = T1 * DH(-90, 0, 0, -90);
center2 = T2(:, 4);
T3 = T2 * DH(0, 0, 0, -beta-theta2);
T3 = T3 * DH(0, 13, 0, 0);
center3 = T3(:, 4);
T4 = T3 * DH(0, 0, 0, beta-90);
center4 = T4(:, 4);
T5 = T4 * DH(0, 0, 0, theta3);
T5 = T5 * DH(0, 12.4, 0, 0);
center5 = T5(:, 4);
T6 = T5 * DH(0, 0, 0, theta4);
T6 = T6 * DH(0, 12.6, 0, 0);
center6 = T6(:, 4);
T_final = T6;
end

function final_mat = DH(alpha, a, d, theta)
final_mat = [cosd(theta), -sind(theta), 0, a;
sind(theta)*cosd(alpha), cosd(theta)*cosd(alpha), -sind(alpha), -sind(alpha)*d;
sind(theta)*sind(alpha), cosd(theta)*sind(alpha), cosd(alpha), cosd(alpha)*d;
0, 0, 0, 1];
end

function traj = trajectory(theta, t, tf)
a0 = theta(1);
a1 = 0;
a2 = (3/tf^2) * (theta(2) - theta(1));
a3 = (-2/tf^3) * (theta(2) - theta(1));
traj = a0 + a1 .* t + a2 .* t.^2 + a3 .* t.^3;
end