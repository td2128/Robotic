clc
clear all
close all

% x-axis: GREEN
% y-axis: BLUE
% z-axis: RED

%% FK
%Const
beta = atand(0.024/0.128);
T0 = eye(4);
min_lim = -50;
max_lim = 50;

initial_pos = [0, 0, 0, 0];
target_pos = [50,50,50,50];

theta1 = [initial_pos(1), target_pos(1)];
theta2 = [initial_pos(2), target_pos(2)];
theta3 = [initial_pos(3), target_pos(3)];
theta4 = [initial_pos(4), target_pos(4)];

% theta(t) = a0 + a1*t + a2 * t^2 + a3 * t^3
tf = 30;
t = 1:1:tf;
% theta1 
a0_1 = theta1(1);
a1 = 0;
a2_1 = (3/tf^2) * (theta1(2) - theta1(1));
a3_1 = (-2/tf^3) * (theta1(2) - theta1(1));
theta1_traj = a0_1 + a1 .* t + a2_1 .* t.^2 + a3_1 .* t.^3;
theta1_v = a1 + 2*a2_1 .* t + 3*a3_1 .* t.^2;

% theta2
a0_2 = theta2(1);
a1 = 0;
a2_2 = (3/tf^2) * (theta2(2) - theta2(1));
a3_2 = (-2/tf^3) * (theta2(2) - theta2(1));
theta2_traj = a0_2 + a1 .* t + a2_2 .* t.^2 + a3_2 .* t.^3;
theta2_v = a1 + 2*a2_2 .* t + 3*a3_2 .* t.^2;

% theta3
a0_3 = theta3(1);
a1 = 0;
a2_3 = (3/tf^2) * (theta3(2) - theta3(1));
a3_3 = (-2/tf^3) * (theta3(2) - theta3(1));
theta3_traj = a0_3 + a1 .* t + a2_3 .* t.^2 + a3_3 .* t.^3;
theta3_v = a1 + 2*a2_3 .* t + 3*a3_3 .* t.^2;

% theta4 
a0_4 = theta4(1);
a1 = 0;
a2_4 = (3/tf^2) * (theta4(2) - theta4(1));
a3_4 = (-2/tf^3) * (theta4(2) - theta4(1));
theta4_traj = a0_4 + a1 .* t + a2_4 .* t.^2 + a3_4 .* t.^3;
theta4_v = a1 + 2*a2_4 .* t + 3*a3_4 .* t.^2;
%Variable
theta1_temp = 50 ;
theta2_temp =  50 ;
theta3_temp = 50 ;
theta4_temp = 50 ;
theta1 = theta1_traj;
theta2 = theta2_traj;
theta3 = theta3_traj;
theta4 = theta4_traj;
for i = 1:length(theta1)
% first transformation (i=1)
T1 = T0 * threeDTransform(0, 0, 7.7, theta1(i)) ;
x_dir1 = T1(1:3, 1)*5;
y_dir1 = T1(1:3, 2)*5;
z_dir1 = T1(1:3, 3)*5;
center1 = T1(:, 4);

% frame change (i=2)
T2 = T1 * threeDTransform(-90, 0, 0, -90) ;
T2 = T2 * threeDTransform(0, 0, 0, -beta-0);
x_dir2 = T2(1:3, 1)*5;
y_dir2 = T2(1:3, 2)*5;
z_dir2 = T2(1:3, 3)*5;
center2 = T2(:, 4);

% second transformation (i=3)
T3 = T2 * threeDTransform(0, 13, 0, 0);
x_dir3 = T3(1:3, 1)*5;
y_dir3 = T3(1:3, 2)*5;
z_dir3 = T3(1:3, 3)*5;
center3 = T3(:, 4);

% frame change (i=4)
T4 = T3 * threeDTransform(0, 0, 0, beta-90) ;
x_dir4 = T4(1:3, 1)*5;
y_dir4 = T4(1:3, 2)*5;
z_dir4 = T4(1:3, 3)*5;
center4 = T4(:, 4);

% third transformation (i=5)
T5 = T4 * threeDTransform(0, 0, 0, 0) ;
T5 = T5 * threeDTransform(0, 12.4, 0, 0);
x_dir5 = T5(1:3, 1)*5;
y_dir5 = T5(1:3, 2)*5;
z_dir5 = T5(1:3, 3)*5;
center5 = T5(:, 4);

% fourth transformation (i=6)
T6 = T5 * threeDTransform(0, 0, 0, 0) ;
T6 = T6 * threeDTransform(0, 12.6, 0, 0) ;
x_dir6= T6(1:3, 1)*5;
y_dir6 = T6(1:3, 2)*5;
z_dir6 = T6(1:3, 3)*5;
center6 = T6(:, 4);

min_lim = -50;
max_lim = 50;

% base frame (i=0)
plot3([0 5], [0 0], [0 0], 'LineWidth', 2, 'Color', 'g');
hold on
plot3([0 0], [0 5], [0 0], 'LineWidth', 2, 'Color', 'b');
hold on
plot3([0 0], [0 0], [0 5], 'LineWidth', 2, 'Color', 'r');
hold on

line0 = plot3([0 center1(1)], [0, center1(2)], [0, center1(3)], 'LineWidth', 1.5, 'Color', 'black');

% change frame! (i=2)
frame0_1 = plot3([center2(1) center2(1)+ x_dir2(1)], [center2(2) center2(2)+x_dir2(2)], [center2(3) center2(3)+x_dir2(3)], 'LineWidth', 2, 'Color', 'g');
hold on
frame0_2 = plot3([center2(1) center2(1)+ y_dir2(1)], [center2(2) center2(2)+y_dir2(2)], [center2(3) center2(3)+y_dir2(3)], 'LineWidth', 2, 'Color', 'b');
hold on
frame0_3 = plot3([center2(1) center2(1)+ z_dir2(1)], [center2(2) center2(2)+z_dir2(2)], [center2(3) center2(3)+z_dir2(3)], 'LineWidth', 2, 'Color', 'r');
hold on

line1 = plot3([center2(1) center3(1)], [center2(2) center3(2)], [center2(3) center3(3)], 'LineWidth', 1.5, 'Color', 'black');


% change frame! (i=4)
frame1_1 = plot3([center4(1) center4(1)+x_dir5(1)], [center4(2) center4(2)+x_dir5(2)], [center4(3) center4(3)+x_dir5(3)], 'LineWidth', 2, 'Color', 'g');
hold on
frame1_2 = plot3([center4(1) center4(1)+ y_dir5(1)], [center4(2) center4(2)+y_dir5(2)], [center4(3) center4(3)+y_dir5(3)], 'LineWidth', 2, 'Color', 'b');
hold on
frame1_3 = plot3([center4(1) center4(1)+ z_dir5(1)], [center4(2) center4(2)+z_dir5(2)], [center4(3) center4(3)+z_dir5(3)], 'LineWidth', 2, 'Color', 'r');
hold on

line2 = plot3([center4(1) center5(1)], [center4(2) center5(2)], [center4(3) center5(3)], 'LineWidth', 1.5, 'Color', 'black');

% third transformation (i=5)
frame2_1 = plot3([center5(1) center5(1)+x_dir6(1)], [center5(2) center5(2)+x_dir6(2)], [center5(3) center5(3)+x_dir6(3)], 'LineWidth', 2.5, 'Color', 'g');
hold on
frame2_2 = plot3([center5(1) center5(1)+y_dir6(1)], [center5(2) center5(2)+y_dir6(2)], [center5(3) center5(3)+y_dir6(3)], 'LineWidth', 2, 'Color', 'b');
hold on
frame2_3 = plot3([center5(1) center5(1)+z_dir6(1)], [center5(2) center5(2)+z_dir6(2)], [center5(3) center5(3)+z_dir6(3)], 'LineWidth', 2, 'Color', 'r');
hold on

line3 = plot3([center5(1) center6(1)], [center5(2) center6(2)], [center5(3) center6(3)], 'LineWidth', 1.5, 'Color', 'black');

% fourth transformation (i=6)
hold on
line4 = scatter3(center6(1), center6(2), center6(3), 'o', 'LineWidth', 2.5,'MarkerEdgeColor','k');

xlim([min_lim max_lim]);
ylim([min_lim max_lim]);
zlim([min_lim max_lim]);
xlabel('x')
ylabel('y')
zlabel('z')
view(-175,16)
grid on
title('Forward Kinematics', 'FontSize', 15);
pause(0.05)
delete(line0)
delete(frame0_1)
delete(frame0_2)
delete(frame0_3)
delete(frame1_1)
delete(frame1_2)
delete(frame1_3)
delete(line1)
delete(frame2_1)
delete(frame2_2)
delete(frame2_3)
delete(line2)
delete(line3)
delete(line4)
end


for i = 1:length(theta2)
% first transformation (i=1)
T1 = T0 * threeDTransform(0, 0, 7.7, theta1_temp); 
x_dir1 = T1(1:3, 1)*5;
y_dir1 = T1(1:3, 2)*5;
z_dir1 = T1(1:3, 3)*5;
center1 = T1(:, 4);

% frame change (i=2)
T2 = T1 * threeDTransform(-90, 0, 0, -90) ;
T2 = T2 * threeDTransform(0, 0, 0, -beta-theta2(i));
x_dir2 = T2(1:3, 1)*5;
y_dir2 = T2(1:3, 2)*5;
z_dir2 = T2(1:3, 3)*5;
center2 = T2(:, 4);

% second transformation (i=3)
T3 = T2 * threeDTransform(0, 13, 0, 0);
x_dir3 = T3(1:3, 1)*5;
y_dir3 = T3(1:3, 2)*5;
z_dir3 = T3(1:3, 3)*5;
center3 = T3(:, 4);

% frame change (i=4)
T4 = T3 * threeDTransform(0, 0, 0, beta-90); 
x_dir4 = T4(1:3, 1)*5;
y_dir4 = T4(1:3, 2)*5;
z_dir4 = T4(1:3, 3)*5;
center4 = T4(:, 4);

% third transformation (i=5)
T5 = T4 * threeDTransform(0, 0, 0, 0) ;
T5 = T5 * threeDTransform(0, 12.4, 0, 0);
x_dir5 = T5(1:3, 1)*5;
y_dir5 = T5(1:3, 2)*5;
z_dir5 = T5(1:3, 3)*5;
center5 = T5(:, 4);

% fourth transformation (i=6)
T6 = T5 * threeDTransform(0, 0, 0, 0) ;
T6 = T6 * threeDTransform(0, 12.6, 0, 0) ;
x_dir6= T6(1:3, 1)*5;
y_dir6 = T6(1:3, 2)*5;
z_dir6 = T6(1:3, 3)*5;
center6 = T6(:, 4);

min_lim = -50;
max_lim = 50;

% base frame (i=0)
plot3([0 5], [0 0], [0 0], 'LineWidth', 2, 'Color', 'g');
hold on
plot3([0 0], [0 5], [0 0], 'LineWidth', 2, 'Color', 'b');
hold on
plot3([0 0], [0 0], [0 5], 'LineWidth', 2, 'Color', 'r');
hold on

line0 = plot3([0 center1(1)], [0, center1(2)], [0, center1(3)], 'LineWidth', 1.5, 'Color', 'black');

% change frame! (i=2)
frame0_1 = plot3([center2(1) center2(1)+x_dir2(1)], [center2(2) center2(2)+x_dir2(2)], [center2(3) center2(3)+x_dir2(3)], 'LineWidth', 2, 'Color', 'g');
hold on
frame0_2 = plot3([center2(1) center2(1)+ y_dir2(1)], [center2(2) center2(2)+y_dir2(2)], [center2(3) center2(3)+y_dir2(3)], 'LineWidth', 2, 'Color', 'b');
hold on
frame0_3 = plot3([center2(1) center2(1)+ z_dir2(1)], [center2(2) center2(2)+z_dir2(2)], [center2(3) center2(3)+z_dir2(3)], 'LineWidth', 2, 'Color', 'r');
hold on

line1 = plot3([center2(1) center3(1)], [center2(2) center3(2)], [center2(3) center3(3)], 'LineWidth', 1.5, 'Color', 'black');


% change frame! (i=4)
frame1_1 = plot3([center4(1) center4(1)+x_dir5(1)], [center4(2) center4(2)+x_dir5(2)], [center4(3) center4(3)+x_dir5(3)], 'LineWidth', 2, 'Color', 'g');
hold on
frame1_2 = plot3([center4(1) center4(1)+ y_dir5(1)], [center4(2) center4(2)+y_dir5(2)], [center4(3) center4(3)+y_dir5(3)], 'LineWidth', 2, 'Color', 'b');
hold on
frame1_3 = plot3([center4(1) center4(1)+ z_dir5(1)], [center4(2) center4(2)+z_dir5(2)], [center4(3) center4(3)+z_dir5(3)], 'LineWidth', 2, 'Color', 'r');
hold on

line2 = plot3([center4(1) center5(1)], [center4(2) center5(2)], [center4(3) center5(3)], 'LineWidth', 1.5, 'Color', 'black');

% third transformation (i=5)
frame2_1 = plot3([center5(1) center5(1)+x_dir6(1)], [center5(2) center5(2)+x_dir6(2)], [center5(3) center5(3)+x_dir6(3)], 'LineWidth', 2.5, 'Color', 'g');
hold on
frame2_2 = plot3([center5(1) center5(1)+y_dir6(1)], [center5(2) center5(2)+y_dir6(2)], [center5(3) center5(3)+y_dir6(3)], 'LineWidth', 2, 'Color', 'b');
hold on
frame2_3 = plot3([center5(1) center5(1)+z_dir6(1)], [center5(2) center5(2)+z_dir6(2)], [center5(3) center5(3)+z_dir6(3)], 'LineWidth', 2, 'Color', 'r');
hold on

line3 = plot3([center5(1) center6(1)], [center5(2) center6(2)], [center5(3) center6(3)], 'LineWidth', 1.5, 'Color', 'black');

% fourth transformation (i=6)
hold on
line4 = scatter3(center6(1), center6(2), center6(3), 'o', 'LineWidth', 2.5,'MarkerEdgeColor','k');

xlim([min_lim max_lim])
ylim([min_lim max_lim])
zlim([min_lim max_lim])
xlabel('x')
ylabel('y')
zlabel('z')
view(-175,16)
grid on
title('Forward Kinematics', 'FontSize', 15)
pause(0.05)

delete(line0)
delete(frame0_1)
delete(frame0_2)
delete(frame0_3)
delete(frame1_1)
delete(frame1_2)
delete(frame1_3)
delete(line1)
delete(frame2_1)
delete(frame2_2)
delete(frame2_3)
delete(line2)
delete(line3)
delete(line4)
end

for i = 1:length(theta3)
% first transformation (i=1)
T1 = T0 * threeDTransform(0, 0, 7.7, theta1_temp) ;
x_dir1 = T1(1:3, 1)*5;
y_dir1 = T1(1:3, 2)*5;
z_dir1 = T1(1:3, 3)*5;
center1 = T1(:, 4);

% frame change (i=2)
T2 = T1 * threeDTransform(-90, 0, 0, -90) ;
T2 = T2 * threeDTransform(0, 0, 0, -beta-theta2_temp);
x_dir2 = T2(1:3, 1)*5;
y_dir2 = T2(1:3, 2)*5;
z_dir2 = T2(1:3, 3)*5;
center2 = T2(:, 4);

% second transformation (i=3)
T3 = T2 * threeDTransform(0, 13, 0, 0);
x_dir3 = T3(1:3, 1)*5;
y_dir3 = T3(1:3, 2)*5;
z_dir3 = T3(1:3, 3)*5;
center3 = T3(:, 4);

% frame change (i=4)
T4 = T3 * threeDTransform(0, 0, 0, beta-90) ;
x_dir4 = T4(1:3, 1)*5;
y_dir4 = T4(1:3, 2)*5;
z_dir4 = T4(1:3, 3)*5;
center4 = T4(:, 4);

% third transformation (i=5)
T5 = T4 * threeDTransform(0, 0, 0, theta3(i)) ;
T5 = T5 * threeDTransform(0, 12.4, 0, 0);
x_dir5 = T5(1:3, 1)*5;
y_dir5 = T5(1:3, 2)*5;
z_dir5 = T5(1:3, 3)*5;
center5 = T5(:, 4);

% fourth transformation (i=6)
T6 = T5 * threeDTransform(0, 0, 0, 0) ;
T6 = T6 * threeDTransform(0, 12.6, 0, 0) ;
x_dir6= T6(1:3, 1)*5;
y_dir6 = T6(1:3, 2)*5;
z_dir6 = T6(1:3, 3)*5;
center6 = T6(:, 4);

min_lim = -50;
max_lim = 50;

% base frame (i=0)
plot3([0 5], [0 0], [0 0], 'LineWidth', 2, 'Color', 'g');
hold on
plot3([0 0], [0 5], [0 0], 'LineWidth', 2, 'Color', 'b');
hold on
plot3([0 0], [0 0], [0 5], 'LineWidth', 2, 'Color', 'r');
hold on

line0 = plot3([0 center1(1)], [0, center1(2)], [0, center1(3)], 'LineWidth', 1.5, 'Color', 'black');

% change frame! (i=2)
frame0_1 = plot3([center2(1) center2(1)+x_dir2(1)], [center2(2) center2(2)+x_dir2(2)], [center2(3) center2(3)+x_dir2(3)], 'LineWidth', 2, 'Color', 'g');
hold on
frame0_2 = plot3([center2(1) center2(1)+ y_dir2(1)], [center2(2) center2(2)+y_dir2(2)], [center2(3) center2(3)+y_dir2(3)], 'LineWidth', 2, 'Color', 'b');
hold on
frame0_3 = plot3([center2(1) center2(1)+ z_dir2(1)], [center2(2) center2(2)+z_dir2(2)], [center2(3) center2(3)+z_dir2(3)], 'LineWidth', 2, 'Color', 'r');
hold on

line1 = plot3([center2(1) center3(1)], [center2(2) center3(2)], [center2(3) center3(3)], 'LineWidth', 1.5, 'Color', 'black');


% change frame! (i=4)
frame1_1 = plot3([center4(1) center4(1)+x_dir5(1)], [center4(2) center4(2)+x_dir5(2)], [center4(3) center4(3)+x_dir5(3)], 'LineWidth', 2, 'Color', 'g');
hold on
frame1_2 = plot3([center4(1) center4(1)+ y_dir5(1)], [center4(2) center4(2)+y_dir5(2)], [center4(3) center4(3)+y_dir5(3)], 'LineWidth', 2, 'Color', 'b');
hold on
frame1_3 = plot3([center4(1) center4(1)+ z_dir5(1)], [center4(2) center4(2)+z_dir5(2)], [center4(3) center4(3)+z_dir5(3)], 'LineWidth', 2, 'Color', 'r');
hold on

line2 = plot3([center4(1) center5(1)], [center4(2) center5(2)], [center4(3) center5(3)], 'LineWidth', 1.5, 'Color', 'black');

% third transformation (i=5)
frame2_1 = plot3([center5(1) center5(1)+x_dir6(1)], [center5(2) center5(2)+x_dir6(2)], [center5(3) center5(3)+x_dir6(3)], 'LineWidth', 2.5, 'Color', 'g');
hold on
frame2_2 = plot3([center5(1) center5(1)+y_dir6(1)], [center5(2) center5(2)+y_dir6(2)], [center5(3) center5(3)+y_dir6(3)], 'LineWidth', 2, 'Color', 'b');
hold on
frame2_3 = plot3([center5(1) center5(1)+z_dir6(1)], [center5(2) center5(2)+z_dir6(2)], [center5(3) center5(3)+z_dir6(3)], 'LineWidth', 2, 'Color', 'r');
hold on

line3 = plot3([center5(1) center6(1)], [center5(2) center6(2)], [center5(3) center6(3)], 'LineWidth', 1.5, 'Color', 'black');

% fourth transformation (i=6)
hold on
line4 = scatter3(center6(1), center6(2), center6(3), 'o', 'LineWidth', 2.5,'MarkerEdgeColor','k');

xlim([min_lim max_lim]);
ylim([min_lim max_lim]);
zlim([min_lim max_lim]);
xlabel('x');
ylabel('y');
zlabel('z');
view(-175,16)
grid on
title('Forward Kinematics', 'FontSize', 15);
pause(0.05)
delete(line0)
delete(frame0_1)
delete(frame0_2)
delete(frame0_3)
delete(frame1_1)
delete(frame1_2)
delete(frame1_3)
delete(line1)
delete(frame2_1)
delete(frame2_2)
delete(frame2_3)
delete(line2)
delete(line3)
delete(line4)
end

for i = 1:length(theta4)
% first transformation (i=1)
T1 = T0 * threeDTransform(0, 0, 7.7, theta1_temp) ;
x_dir1 = T1(1:3, 1)*5;
y_dir1 = T1(1:3, 2)*5;
z_dir1 = T1(1:3, 3)*5;
center1 = T1(:, 4);

% frame change (i=2)
T2 = T1 * threeDTransform(-90, 0, 0, -90) ;
T2 = T2 * threeDTransform(0, 0, 0, -beta-theta2_temp);
x_dir2 = T2(1:3, 1)*5;
y_dir2 = T2(1:3, 2)*5;
z_dir2 = T2(1:3, 3)*5;
center2 = T2(:, 4);

% second transformation (i=3)
T3 = T2 * threeDTransform(0, 13, 0, 0);
x_dir3 = T3(1:3, 1)*5;
y_dir3 = T3(1:3, 2)*5;
z_dir3 = T3(1:3, 3)*5;
center3 = T3(:, 4);

% frame change (i=4)
T4 = T3 * threeDTransform(0, 0, 0, beta-90) ;
x_dir4 = T4(1:3, 1)*5;
y_dir4 = T4(1:3, 2)*5;
z_dir4 = T4(1:3, 3)*5;
center4 = T4(:, 4);

% third transformation (i=5)
T5 = T4 * threeDTransform(0, 0, 0, theta3_temp) ;
T5 = T5 * threeDTransform(0, 12.4, 0, 0);
x_dir5 = T5(1:3, 1)*5;
y_dir5 = T5(1:3, 2)*5;
z_dir5 = T5(1:3, 3)*5;
center5 = T5(:, 4);

% fourth transformation (i=6)
T6 = T5 * threeDTransform(0, 0, 0, theta4(i)) ;
T6 = T6 * threeDTransform(0, 12.6, 0, 0) ;
x_dir6= T6(1:3, 1)*5;
y_dir6 = T6(1:3, 2)*5;
z_dir6 = T6(1:3, 3)*5;
center6 = T6(:, 4);

min_lim = -50;
max_lim = 50;

% base frame (i=0)
plot3([0 5], [0 0], [0 0], 'LineWidth', 2, 'Color', 'g');
hold on
plot3([0 0], [0 5], [0 0], 'LineWidth', 2, 'Color', 'b');
hold on
plot3([0 0], [0 0], [0 5], 'LineWidth', 2, 'Color', 'r');
hold on

line0 = plot3([0 center1(1)], [0, center1(2)], [0, center1(3)], 'LineWidth', 1.5, 'Color', 'black');

% change frame! (i=2)
frame0_1 = plot3([center2(1) center2(1)+x_dir2(1)], [center2(2) center2(2)+x_dir2(2)], [center2(3) center2(3)+x_dir2(3)], 'LineWidth', 2, 'Color', 'g');
hold on
frame0_2 = plot3([center2(1) center2(1)+ y_dir2(1)], [center2(2) center2(2)+y_dir2(2)], [center2(3) center2(3)+y_dir2(3)], 'LineWidth', 2, 'Color', 'b');
hold on
frame0_3 = plot3([center2(1) center2(1)+ z_dir2(1)], [center2(2) center2(2)+z_dir2(2)], [center2(3) center2(3)+z_dir2(3)], 'LineWidth', 2, 'Color', 'r');
hold on

line1 = plot3([center2(1) center3(1)], [center2(2) center3(2)], [center2(3) center3(3)], 'LineWidth', 1.5, 'Color', 'black');


% change frame! (i=4)
frame1_1 = plot3([center4(1) center4(1)+x_dir5(1)], [center4(2) center4(2)+x_dir5(2)], [center4(3) center4(3)+x_dir5(3)], 'LineWidth', 2, 'Color', 'g');
hold on
frame1_2 = plot3([center4(1) center4(1)+ y_dir5(1)], [center4(2) center4(2)+y_dir5(2)], [center4(3) center4(3)+y_dir5(3)], 'LineWidth', 2, 'Color', 'b');
hold on
frame1_3 = plot3([center4(1) center4(1)+ z_dir5(1)], [center4(2) center4(2)+z_dir5(2)], [center4(3) center4(3)+z_dir5(3)], 'LineWidth', 2, 'Color', 'r');
hold on

line2 = plot3([center4(1) center5(1)], [center4(2) center5(2)], [center4(3) center5(3)], 'LineWidth', 1.5, 'Color', 'black');

% third transformation (i=5)
frame2_1 = plot3([center5(1) center5(1)+x_dir6(1)], [center5(2) center5(2)+x_dir6(2)], [center5(3) center5(3)+x_dir6(3)], 'LineWidth', 2.5, 'Color', 'g');
hold on
frame2_2 = plot3([center5(1) center5(1)+y_dir6(1)], [center5(2) center5(2)+y_dir6(2)], [center5(3) center5(3)+y_dir6(3)], 'LineWidth', 2, 'Color', 'b');
hold on
frame2_3 = plot3([center5(1) center5(1)+z_dir6(1)], [center5(2) center5(2)+z_dir6(2)], [center5(3) center5(3)+z_dir6(3)], 'LineWidth', 2, 'Color', 'r');
hold on

line3 = plot3([center5(1) center6(1)], [center5(2) center6(2)], [center5(3) center6(3)], 'LineWidth', 1.5, 'Color', 'black');

% fourth transformation (i=6)
hold on
line4 = scatter3(center6(1), center6(2), center6(3), 'o', 'LineWidth', 2.5,'MarkerEdgeColor','k');

xlim([min_lim max_lim]);
ylim([min_lim max_lim]);
zlim([min_lim max_lim]);
xlabel('x');
ylabel('y');
zlabel('z');
view(-175,16)
grid on
title('Forward Kinematics', 'FontSize', 15);
pause(0.05)
delete(line0)
delete(frame0_1)
delete(frame0_2)
delete(frame0_3)
delete(frame1_1)
delete(frame1_2)
delete(frame1_3)
delete(line1)
delete(frame2_1)
delete(frame2_2)
delete(frame2_3)
delete(line2)
delete(line3)
delete(line4)
end

% first transformation (i=1)
T1 = T0 * threeDTransform(0, 0, 7.7, theta1_temp) ;
x_dir1 = T1(1:3, 1)*5;
y_dir1 = T1(1:3, 2)*5;
z_dir1 = T1(1:3, 3)*5;
center1 = T1(:, 4);

% frame change (i=2)
T2 = T1 * threeDTransform(-90, 0, 0, -90) ;
T2 = T2 * threeDTransform(0, 0, 0, -beta-theta2_temp);
x_dir2 = T2(1:3, 1)*5;
y_dir2 = T2(1:3, 2)*5;
z_dir2 = T2(1:3, 3)*5;
center2 = T2(:, 4);

% second transformation (i=3)
T3 = T2 * threeDTransform(0, 13, 0, 0);
x_dir3 = T3(1:3, 1)*5;
y_dir3 = T3(1:3, 2)*5;
z_dir3 = T3(1:3, 3)*5;
center3 = T3(:, 4);

% frame change (i=4)
T4 = T3 * threeDTransform(0, 0, 0, beta-90) ;
x_dir4 = T4(1:3, 1)*5;
y_dir4 = T4(1:3, 2)*5;
z_dir4 = T4(1:3, 3)*5;
center4 = T4(:, 4);

% third transformation (i=5)
T5 = T4 * threeDTransform(0, 0, 0, theta3_temp) ;
T5 = T5 * threeDTransform(0, 12.4, 0, 0);
x_dir5 = T5(1:3, 1)*5;
y_dir5 = T5(1:3, 2)*5;
z_dir5 = T5(1:3, 3)*5;
center5 = T5(:, 4);

% fourth transformation (i=6)
T6 = T5 * threeDTransform(0, 0, 0, theta4_temp) ;
T6 = T6 * threeDTransform(0, 12.6, 0, 0) ;
x_dir6= T6(1:3, 1)*5;
y_dir6 = T6(1:3, 2)*5;
z_dir6 = T6(1:3, 3)*5;
center6 = T6(:, 4);

min_lim = -50;
max_lim = 50;

% base frame (i=0)
plot3([0 5], [0 0], [0 0], 'LineWidth', 2, 'Color', 'g');
hold on
plot3([0 0], [0 5], [0 0], 'LineWidth', 2, 'Color', 'b');
hold on
plot3([0 0], [0 0], [0 5], 'LineWidth', 2, 'Color', 'r');
hold on

line0 = plot3([0 center1(1)], [0, center1(2)], [0, center1(3)], 'LineWidth', 1.5, 'Color', 'black');

% change frame! (i=2)
frame0_1 = plot3([center2(1) center2(1)+x_dir2(1)], [center2(2) center2(2)+x_dir2(2)], [center2(3) center2(3)+x_dir2(3)], 'LineWidth', 2, 'Color', 'g');
hold on
frame0_2 = plot3([center2(1) center2(1)+ y_dir2(1)], [center2(2) center2(2)+y_dir2(2)], [center2(3) center2(3)+y_dir2(3)], 'LineWidth', 2, 'Color', 'b');
hold on
frame0_3 = plot3([center2(1) center2(1)+ z_dir2(1)], [center2(2) center2(2)+z_dir2(2)], [center2(3) center2(3)+z_dir2(3)], 'LineWidth', 2, 'Color', 'r');
hold on

line1 = plot3([center2(1) center3(1)], [center2(2) center3(2)], [center2(3) center3(3)], 'LineWidth', 1.5, 'Color', 'black');


% change frame! (i=4)
frame1_1 = plot3([center4(1) center4(1)+x_dir5(1)], [center4(2) center4(2)+x_dir5(2)], [center4(3) center4(3)+x_dir5(3)], 'LineWidth', 2, 'Color', 'g');
hold on
frame1_2 = plot3([center4(1) center4(1)+ y_dir5(1)], [center4(2) center4(2)+y_dir5(2)], [center4(3) center4(3)+y_dir5(3)], 'LineWidth', 2, 'Color', 'b');
hold on
frame1_3 = plot3([center4(1) center4(1)+ z_dir5(1)], [center4(2) center4(2)+z_dir5(2)], [center4(3) center4(3)+z_dir5(3)], 'LineWidth', 2, 'Color', 'r');
hold on

line2 = plot3([center4(1) center5(1)], [center4(2) center5(2)], [center4(3) center5(3)], 'LineWidth', 1.5, 'Color', 'black');

% third transformation (i=5)
frame2_1 = plot3([center5(1) center5(1)+x_dir6(1)], [center5(2) center5(2)+x_dir6(2)], [center5(3) center5(3)+x_dir6(3)], 'LineWidth', 2.5, 'Color', 'g');
hold on
frame2_2 = plot3([center5(1) center5(1)+y_dir6(1)], [center5(2) center5(2)+y_dir6(2)], [center5(3) center5(3)+y_dir6(3)], 'LineWidth', 2, 'Color', 'b');
hold on
frame2_3 = plot3([center5(1) center5(1)+z_dir6(1)], [center5(2) center5(2)+z_dir6(2)], [center5(3) center5(3)+z_dir6(3)], 'LineWidth', 2, 'Color', 'r');
hold on

line3 = plot3([center5(1) center6(1)], [center5(2) center6(2)], [center5(3) center6(3)], 'LineWidth', 1.5, 'Color', 'black');

% fourth transformation (i=6)
hold on
line4 = scatter3(center6(1), center6(2), center6(3), 'o', 'LineWidth', 2.5,'MarkerEdgeColor','k');

xlim([min_lim max_lim]);
ylim([min_lim max_lim]);
zlim([min_lim max_lim]);
xlabel('x');
ylabel('y');
zlabel('z');
view(-175,16)
grid on
title('Forward Kinematics - Trajectory Planning', 'FontSize', 15);
pause(0.05)


%% 

function final_mat = threeDTransform(alpha, a, d, theta)
    final_mat = [cosd(theta),             -sind(theta),            0,            a;
                 sind(theta)*cosd(alpha), cosd(theta)*cosd(alpha), -sind(alpha), -sind(alpha)*d;
                 sind(theta)*sind(alpha), cosd(theta)*sind(alpha), cosd(alpha),  cosd(alpha)*d;
                 0,                       0,                       0,            1];
end
