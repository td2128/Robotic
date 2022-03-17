%%
clear all
clc

%% 
tf = 6;
t = 0:1:tf;
full_traj = [];
%%
height = 8.8407;
pos1_initial = [60,200,height];
pos2_initial = [140,200,height];
pos3_initial= [140,125,height];
phi = 0;
r_initial = 40;
centre_initial = [100,200,8.8407];
%% Translate it to our model
pos1 = [-pos1_initial(2)/10,-pos1_initial(1)/10, height];%[-20,-6,8.8407]
pos2 = [-pos2_initial(2)/10,-pos2_initial(1)/10, height];%[-20,-14,8.8407]
pos3 = [-pos3_initial(2)/10,-pos3_initial(1)/10, height];%[-12.5,-14,8.8407]
% centre = [-centre_initial(2)/10,-centre_initial(1)/10, height];%[-20,-8,8.8407];
% r = r_initial/10;
%% 
%full_traj = robot_line(pos1, pos2, 1,t,tf, 2);
%full_traj = robot_line(pos2, pos3, 1,t,tf, 1);
% full_traj = robot_line(pos3, pos1, 1,t,tf, 3);
angle = 150;
step = 10;
centre = [15,16,6];
r = 15;
%full_traj = robot_arc(centre, angle, step, r, t, tf);
full_traj = robot_sweep(centre,r,angle,t,tf,1);
%% 
function arc_traj = robot_sweep(centre,r,angle,t,tf,type)
            if type == 1
                start_pos = [centre(1), centre(2)+r, centre(3)]';
                arc_traj = [];
                for i = 0:10:angle
                desired_pos = [centre(1)+r*sind(i), centre(2)+r*cosd(i), centre(3)]';
                start_ang = IK(start_pos(1),start_pos(2),start_pos(3), 0);
                desired_ang = IK(desired_pos(1),desired_pos(2),desired_pos(3), 0);
                theta1 = [start_ang(1), desired_ang(1)];
                theta2 = [start_ang(2), desired_ang(2)];
                theta3 = [start_ang(3), desired_ang(3)];
                theta4 = [start_ang(4), desired_ang(4)];
                theta1_traj = trajectory(theta1, t, tf);
                theta2_traj = trajectory(theta2, t, tf);
                theta3_traj = trajectory(theta3, t, tf);
                theta4_traj = trajectory(theta4, t, tf);
                arc_traj = [arc_traj, [theta1_traj; theta2_traj; theta3_traj; theta4_traj]];
                start_pos = desired_pos;
                end   
            end 
        end

%% 
function traj = robot_line(start_pos, end_pos, number_iteration,step,t,tf, status)
            [start, start_mid] = robot_pick_angle(start_pos,0,3);
            [final, final_mid] = robot_pick_angle(end_pos,0,3);
            %status1 = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, start_mid, 1);
            %status1 = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, start, 1);
            if status == 1 
                x_diff = end_pos(status) - start_pos(status);
                initial_pos = start_pos;
                start_pos1 = start_pos;
                if x_diff < 0
                    traj = trajectory_degree(-step, x_diff, 1, 0, initial_pos, start_pos1, t, tf);
                else
                    traj = trajectory_degree(step, x_diff, 1, 0, initial_pos, start_pos1, t, tf);
                end
            elseif status == 2
                y_diff = end_pos(status) - start_pos(status);
                initial_pos = start_pos;
                start_pos1 = start_pos;
                if y_diff < 0
                    traj = trajectory_degree(-step, y_diff, 0, 1, initial_pos, start_pos1, t, tf);
                else
                    traj = trajectory_degree(step, y_diff, 0, 1, initial_pos, start_pos1, t, tf);
                end
            elseif status == 3
                x_diff = end_pos(1) - start_pos(1);
                y_diff = end_pos(2) - start_pos(2);
                initial_pos = start_pos;
                start_pos1 = start_pos;
                if x_diff < 0 && y_diff > 0
                    y_step = y_diff / number_iteration;
                    x_step = x_diff / number_iteration;
                    x_change = x_step/y_step;
                    traj = trajectory_degree(y_step, y_diff, x_change, 1, initial_pos, start_pos1, t, tf);
                elseif x_diff < 0 && y_diff < 0
                    y_step = y_diff / number_iteration;
                    x_step = x_diff / number_iteration;
                    x_change = x_step/y_step;
                    traj = trajectory_degree(y_step, y_diff, x_change, 1, initial_pos, start_pos1, t, tf);
                elseif x_diff > 0 && y_diff < 0
                    y_step = y_diff / number_iteration;
                    x_step = x_diff / number_iteration;
                    y_change = y_step/x_step;
                    traj = trajectory_degree(x_step, x_diff, 1, y_change, initial_pos, start_pos1, t, tf);
                elseif x_diff > 0 && y_diff > 0
                    y_step = y_diff / number_iteration;
                    x_step = x_diff / number_iteration;
                    y_change = y_step/x_step;
                    traj = trajectory_degree(x_step, x_diff, 1, y_change, initial_pos, start_pos1, t, tf);
                end
            end
        end
%% 
function arc_traj = robot_arc(centre, angle, step, r, t, tf)
            start_pos = [centre(1)+r*sind(0), centre(2)+r*cosd(0), centre(3)]';
            arc_traj = [];
            arc_traj1 = [];
            [start, start_mid] = robot_pick_angle(centre,0,3);
%             status1 = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, start_mid, 1);
%             status1 = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, start, 1);
            arc_traj = robot_line(centre, start_pos, 10,1,t,tf, 2);
            for i = 0:step:angle
            desired_pos = [centre(1)+r*sind(i), centre(2)+r*cosd(i), centre(3)]';
            start_ang = IK(start_pos(1),start_pos(2),start_pos(3), 0);
            desired_ang = IK(desired_pos(1),desired_pos(2),desired_pos(3), 0);
            theta1 = [start_ang(1), desired_ang(1)];
            theta2 = [start_ang(2), desired_ang(2)];
            theta3 = [start_ang(3), desired_ang(3)];
            theta4 = [start_ang(4), desired_ang(4)];
            theta1_traj = trajectory(theta1, t, tf);
            theta2_traj = trajectory(theta2, t, tf);
            theta3_traj = trajectory(theta3, t, tf);
            theta4_traj = trajectory(theta4, t, tf);
            arc_traj = [arc_traj, [theta1_traj; theta2_traj; theta3_traj; theta4_traj]];
            start_pos = desired_pos;
            end
            if angle == 180
                arc_traj1 = robot_line(desired_pos, centre, 10,1,t,tf, 2);
            else
                arc_traj1 = robot_line(desired_pos, centre, 10,1,t,tf, 3);
            end
            arc_traj = [arc_traj,arc_traj1];
        end
%% 
function IK_deg = robot_angle(pos_coordinate,phi)
            pos_angle = IK(pos_coordinate(1),pos_coordinate(2),pos_coordinate(3),phi);
            IK_deg1 = (pos_angle(1) + 180) / 0.088;
            IK_deg2 = (pos_angle(2) + 180) / 0.088;
            IK_deg3 = (-pos_angle(3) + 180) / 0.088;
            IK_deg4 = (-pos_angle(4) + 180) / 0.088;
            IK_deg = [IK_deg1,IK_deg2,IK_deg3,IK_deg4];
        end
%% 

function result = IK(pos_x,pos_y,pos_z,phi)
            target_end_pos = [pos_x,pos_y,pos_z];
            a2 = 13;
            a3 = 12.4;
            a4 = 12.6;
            beta = atand(0.024/0.128);
            r3 = sqrt(pos_x^2 + pos_y^2);
            z3 = pos_z - 7.7;
            % randomly chose a sum for all angles: 49.4 - 34.38 - 60 (temp val)
            r2 = r3 - a4*cosd(phi);
            z2 = z3 - a4*sind(phi); 
            cos_theta3 = (r2^2+z2^2-a2^2-a3^2) / (2*a2*a3);
            % no solution case
            if cos_theta3 < -1 || cos_theta3 > 1
                msgbox('No solution exists')
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
            theta1 = atand(pos_y/pos_x);
            theta1_ = -atand(pos_y/pos_x);
            if theta1 == 0
                theta1 = 0;
                theta1_ = 180;
            end
            res1 = [theta1, theta2, theta3, theta4];
            res2 = [theta1, theta2_, theta3_, theta4_];
            res3 = [theta1_, theta2, theta3, theta4];
            res4 = [theta1_, theta2_, theta3_, theta4_];
            result = res2;
end


function traj = trajectory(theta, t, tf)
a0 = theta(1);
a1 = 0;
a2 = (3/tf^2) * (theta(2) - theta(1));
a3 = (-2/tf^3) * (theta(2) - theta(1));
traj = a0 + a1 .* t + a2 .* t.^2 + a3 .* t.^3;
end

function [IK_deg, IK_mid_deg] = robot_pick_angle(pos_coordinate,phi,height)
            pos_angle = IK(pos_coordinate(1),pos_coordinate(2),pos_coordinate(3),phi);
            pos_mid_angle = IK(pos_coordinate(1),pos_coordinate(2),pos_coordinate(3)+height,phi);
            IK_deg1 = (pos_angle(1) + 180) / 0.088;
            IK_deg2 = (pos_angle(2) + 180) / 0.088;
            IK_deg3 = (-pos_angle(3) + 180) / 0.088;
            IK_deg4 = (-pos_angle(4) + 180) / 0.088;
            IK_mid_deg1 = (pos_mid_angle(1) + 180) / 0.088;
            IK_mid_deg2 = (pos_mid_angle(2) + 180) / 0.088;
            IK_mid_deg3 = (-pos_mid_angle(3) + 180) / 0.088;
            IK_mid_deg4 = (-pos_mid_angle(4) + 180) / 0.088;
            IK_deg = [IK_deg1,IK_deg2,IK_deg3,IK_deg4];
            IK_mid_deg = [IK_mid_deg1,IK_mid_deg2,IK_mid_deg3,IK_mid_deg4];
end


function traj = trajectory_degree(step, diff, x, y, initial_pos, start_pos, t, tf)
    traj = [];
    for i = 0:step:diff
    desired_pos = [initial_pos(1)+i*x, initial_pos(2)+i*y, initial_pos(3)];
    start_ang = IK(start_pos(1), start_pos(2), start_pos(3), 0);
    desired_ang = IK(desired_pos(1), desired_pos(2), desired_pos(3), 0);
    theta1 = [start_ang(1), desired_ang(1)];
    theta2 = [start_ang(2), desired_ang(2)];
    theta3 = [start_ang(3), desired_ang(3)];
    theta4 = [start_ang(4), desired_ang(4)];
    theta1_traj = trajectory(theta1, t, tf);
    theta2_traj = trajectory(theta2, t, tf);
    theta3_traj = trajectory(theta3, t, tf);
    theta4_traj = trajectory(theta4, t, tf);
    traj = [traj, [theta1_traj; theta2_traj; theta3_traj; theta4_traj]];
    start_pos = desired_pos;
    end
end
