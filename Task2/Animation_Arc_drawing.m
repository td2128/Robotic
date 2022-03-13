clc
clear all
close all

% x-axis: GREEN
% y-axis: BLUE
% z-axis: 

%% FK
%Const
load full_traj_morepoints.mat
beta = atand(0.024/0.128);
T0 = eye(4);
min_lim = -20;
max_lim = 20;
center6_comp = [];

theta1 = full_traj(1, :);
theta2 = full_traj(2, :);
theta3 = full_traj(3, :);
theta4 = full_traj(4, :);
for i = 1:length(theta1)
% first transformation (i=1)
T1 = T0 * threeDTransform(0, 0, 7.7, theta1(i)) ;
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
T6 = T5 * threeDTransform(0, 0, 0, theta4(i)) ;
T6 = T6 * threeDTransform(0, 12.6, 0, 0) ;
x_dir6= T6(1:3, 1)*5;
y_dir6 = T6(1:3, 2)*5;
z_dir6 = T6(1:3, 3)*5;
center6 = T6(:, 4);
center6_comp = [center6_comp,center6];
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
line4 = scatter3(center6(1), center6(2), center6(3), 'o', 'LineWidth', 0.5,'MarkerEdgeColor','k');

column_curr = center6_comp(:,i);
z = i-1;
if i == 1 
    plot3(column_curr(1),column_curr(2),column_curr(3),'LineWidth', 1.5, 'Color','black')
else
    column_previous = center6_comp(:,z);
    plot3([column_previous(1) column_curr(1)], [column_previous(2) column_curr(2)], [column_previous(3) column_curr(3)], 'LineWidth', 1.5, 'Color','black');
end
view(-176,77);
pause(1/100000);
xlim([-35 10]);
ylim([-30 20]);
zlim([-10 35]);
xlabel('x')
ylabel('y')
zlabel('z')
grid on
title('Square Plotting', 'FontSize', 15);
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
scatter3(center6(1), center6(2), center6(3), 'o', 'LineWidth', 0.5,'MarkerEdgeColor','k');
plot3([column_previous(1) column_curr(1)], [column_previous(2) column_curr(2)], [column_previous(3) column_curr(3)], 'LineWidth', 1.5, 'Color','black');
view(-176,77);
xlim([-35 10]);
ylim([-30 20]);
zlim([-10 35]);
xlabel('x')
ylabel('y')
zlabel('z')
grid on
title('Square Plotting', 'FontSize', 15);

%% 

function final_mat = threeDTransform(alpha, a, d, theta)
    final_mat = [cosd(theta),             -sind(theta),            0,            a;
                 sind(theta)*cosd(alpha), cosd(theta)*cosd(alpha), -sind(alpha), -sind(alpha)*d;
                 sind(theta)*sind(alpha), cosd(theta)*sind(alpha), cosd(alpha),  cosd(alpha)*d;
                 0,                       0,                       0,            1];
end