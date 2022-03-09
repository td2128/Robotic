clc
clear all
close all

% x-axis: GREEN
% y-axis: BLUE
% z-axis: 

%% FK
%Const
load square_drawing_side1.mat
xz1 = angles_all_side1;
load square_drawing_side2.mat
xz2 = angles_all_side2;
load square_drawing_side3.mat
xz3 = angles_all_side3;
load square_drawing_side4.mat
xz4 = angles_all_side4;

load square_yz_side1.mat
yz1 = angles_all_side1;
load square_yz_side2.mat
yz2 = angles_all_side2;
load square_yz_side3.mat
yz3 = angles_all_side3;
load square_yz_side4.mat
yz4 = angles_all_side4;

load square_xy_side1.mat
xy1 = angles_all_side1;
load square_xy_side2.mat
xy2 = angles_all_side2;
load square_xy_side3.mat
xy3 = angles_all_side3;
load square_xy_side4.mat
xy4 = angles_all_side4;

beta = atand(0.024/0.128);
T0 = eye(4);
min_lim = -20;
max_lim = 20;
center6_comp = [];

theta1_all_side1 = [xz1(1, :) xz2(1,:) xz3(1,:) xz4(1,:) yz1(1, :) yz2(1,:) yz3(1,:) yz4(1,:) xy1(1, :) xy2(1,:) xy3(1,:) xy4(1,:)];
theta2_all_side1 = [xz1(2, :) xz2(2,:) xz3(2,:) xz4(2,:) yz1(2, :) yz2(2,:) yz3(2,:) yz4(2,:) xy1(2, :) xy2(2,:) xy3(2,:) xy4(2,:)];
theta3_all_side1 = [xz1(3, :) xz2(3,:) xz3(3,:) xz4(3,:) yz1(3, :) yz2(3,:) yz3(3,:) yz4(3,:) xy1(3, :) xy2(3,:) xy3(3,:) xy4(3,:)];
theta4_all_side1 = [xz1(4, :) xz2(4,:) xz3(4,:) xz4(4,:) yz1(4, :) yz2(4,:) yz3(4,:) yz4(4,:) xy1(4, :) xy2(4,:) xy3(4,:) xy4(4,:)];
for i = 1:length(theta1_all_side1)
% first transformation (i=1)
T1 = T0 * threeDTransform(0, 0, 7.7, theta1_all_side1(i)) ;
x_dir1 = T1(1:3, 1)*5;
y_dir1 = T1(1:3, 2)*5;
z_dir1 = T1(1:3, 3)*5;
center1 = T1(:, 4);

% frame change (i=2)
T2 = T1 * threeDTransform(-90, 0, 0, -90) ;
T2 = T2 * threeDTransform(0, 0, 0, -beta-theta2_all_side1(i));
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
T5 = T4 * threeDTransform(0, 0, 0, theta3_all_side1(i)) ;
T5 = T5 * threeDTransform(0, 12.4, 0, 0);
x_dir5 = T5(1:3, 1)*5;
y_dir5 = T5(1:3, 2)*5;
z_dir5 = T5(1:3, 3)*5;
center5 = T5(:, 4);

% fourth transformation (i=6)
T6 = T5 * threeDTransform(0, 0, 0, theta4_all_side1(i)) ;
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
if i <= 11
    column1 = center6_comp(:,1)
elseif i<= 22
    column1 = center6_comp(:,12)
elseif i<= 33
    column1 = center6_comp(:,23)
elseif i <= 44  
    column1 = center6_comp(:,34)
elseif i <= 55  
    column1 = center6_comp(:,45)
elseif i <= 66  
    column1 = center6_comp(:,56)
elseif i <= 77  
    column1 = center6_comp(:,67)
elseif i <= 88 
    column1 = center6_comp(:,78)
elseif i <= 99 
    column1 = center6_comp(:,89)
elseif i <= 110 
    column1 = center6_comp(:,100)
elseif i <= 121 
    column1 = center6_comp(:,111)
else
    column1 = center6_comp(:,122)
end

column_curr = center6_comp(:,i)
plot3([column1(1) column_curr(1)], [column1(2) column_curr(2)], [column1(3) column_curr(3)], 'LineWidth', 1.5, 'Color','black');

xlim([-35 10]);
ylim([-10 20]);
zlim([-10 35]);

xlabel('x')
ylabel('y')
zlabel('z')
view(-22,14)
grid on
title('Square Plotting', 'FontSize', 15);
pause(0.1)
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
line_plot = plot3([column1(1) column_curr(1)], [column1(2) column_curr(2)], [column1(3) column_curr(3)], 'LineWidth', 1.5, 'Color','black');

xlim([-35 10]);
ylim([-10 20]);
zlim([-10 35]);
view(-22,14)
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