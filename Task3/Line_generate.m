clear all;
centre = [-15.1663,-3.2769,8];
phi = 0;
tf = 6;
t = 0:1:tf;
full_traj = [];

y = 10;
y_step = 0.5;
start_pos = [centre(1), centre(2), centre(3)];
for i = 1:y_step:y
desired_pos = [centre(1)+0.5, centre(2)+0.5, centre(3)];
start_ang = IK(start_pos, 90);
desired_ang = IK(desired_pos, 90);
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

x = 10;
x_step = 1;
for i = 1:x_step:x
desired_pos = [centre(1)-10+i, centre(2)-10, centre(3)];
start_ang = IK(start_pos, 0);
desired_ang = IK(desired_pos, 0);
theta1 = [start_ang(1), desired_ang(1)];
theta2 = [start_ang(2), desired_ang(2)];
theta3= [start_ang(3), desired_ang(3)];
theta4 = [start_ang(4), desired_ang(4)];
theta1_traj = trajectory(theta1, t, tf);
theta2_traj = trajectory(theta2, t, tf);
theta3_traj = trajectory(theta3, t, tf);
theta4_traj = trajectory(theta4, t, tf);
full_traj = [full_traj, [theta1_traj; theta2_traj; theta3_traj; theta4_traj]];
start_pos = desired_pos;
end

y1 = 10;
y_step = 1;
for i = 1:y_step:y1
desired_pos = [centre(1), centre(2)-10+i, centre(3)];
start_ang = IK(start_pos, 0);
desired_ang = IK(desired_pos, 0);
theta1 = [start_ang(1), desired_ang(1)];
theta2 = [start_ang(2), desired_ang(2)];
theta3= [start_ang(3), desired_ang(3)];
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

function res = IK(target_end_pos, phi)
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
res = res2;
;

end

function traj = trajectory(theta, t, tf)
a0 = theta(1);
a1 = 0;
a2 = (3/tf^2) * (theta(2) - theta(1));
a3 = (-2/tf^3) * (theta(2) - theta(1));
traj = a0 + a1 .* t + a2 .* t.^2 + a3 .* t.^3;
end