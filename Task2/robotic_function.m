classdef robotic_function
    methods(Static)
        % robot_pick_angle provide angles for robot to pick and lift up by
        % height in z domain
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

        function [IK_deg, IK_mid_deg] = robot_pick_angle_y(pos_coordinate,phi,height)
            pos_angle = IK2(pos_coordinate(1),pos_coordinate(2),pos_coordinate(3),phi);
            pos_mid_angle = IK2(pos_coordinate(1),pos_coordinate(2),pos_coordinate(3)+height,phi);
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

        function IK_deg = robot_angle(pos_coordinate,phi)
            pos_angle = IK(pos_coordinate(1),pos_coordinate(2),pos_coordinate(3),phi);
            IK_deg1 = (pos_angle(1) + 180) / 0.088;
            IK_deg2 = (pos_angle(2) + 180) / 0.088;
            IK_deg3 = (-pos_angle(3) + 180) / 0.088;
            IK_deg4 = (-pos_angle(4) + 180) / 0.088;
            IK_deg = [IK_deg1,IK_deg2,IK_deg3,IK_deg4];
        end

        function status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_deg, status)
            %id15 = 1 close; id15 = 0 open
            if status == 1
                id15 = 230/0.088;
            elseif status == 2
                %id15 = 233/0.088;
                id15 = 185/0.088;
            elseif status == 3
                id15 = 148/0.088;
            elseif status == 4% book
                id15 = 220/0.088;
            elseif status == 0
                id15 = 134/0.088;
            end
            write4ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_PRO_GOAL_POSITION, IK_deg(1));
            write4ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_PRO_GOAL_POSITION, IK_deg(2));
            write4ByteTxRx(port_num, PROTOCOL_VERSION, 13, ADDR_PRO_GOAL_POSITION, IK_deg(3));
            pause(2)
            write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, IK_deg(4));
            pause(3)
            write4ByteTxRx(port_num, PROTOCOL_VERSION, 15, ADDR_PRO_GOAL_POSITION, id15);
            pause(2.5)
        end

        function status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_deg, status)
            %id15 = 1 close; id15 = 0 open
            if status == 1%% box
                id15 = 230/0.088;
            elseif status == 5
                id15 = 220/0.088;
            elseif status == 6 % trash
                id15 = 194/0.088;
            elseif status == 3%water bottle
                id15 = 150/0.088;
            elseif status == 4
                id15 = 160/0.088;% water bottle angry
            elseif status == 0
                id15 = 130/0.088;
            end
            write4ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_PRO_GOAL_POSITION, IK_deg(1));
            write4ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_PRO_GOAL_POSITION, IK_deg(2));
            write4ByteTxRx(port_num, PROTOCOL_VERSION, 13, ADDR_PRO_GOAL_POSITION, IK_deg(3));
            write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, IK_deg(4));
            pause(2)
            write4ByteTxRx(port_num, PROTOCOL_VERSION, 15, ADDR_PRO_GOAL_POSITION, id15);
            pause(2)
        end

        % status = 1, front; status = 2, down; status = 3, toward;
        % the pick/drop phi: either -80,0 or 0,-80
        function status = robot_rotate(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, pos,height,x,y,z,pick_phi,drop_phi,status)
            [IK_deg, IK_mid] = robot_pick_angle(pos,pick_phi,height);
            drop_pos = [pos(1)+x, pos(2)+y,pos(3)+z];
            [rotate_deg, rotate_mid] = robot_pick_angle(drop_pos,drop_phi,height);

            if status == 1
                status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_deg, 1);
                status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_mid, 1);
                write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, 95/0.088);
                pause(1)
                status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, rotate_deg, 0);
                status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, rotate_mid, 0);
            elseif status ==2
                status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_deg, 1);
                status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_mid, 1);
                write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, 95/0.088);
                pause(1)
                status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, rotate_deg, 0);
                status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_mid, 0);
                status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_deg, 1);
                status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_mid, 1);
                write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, 95/0.088);
                pause(1)
                status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, rotate_deg, 0);
                status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_mid, 0);
            elseif status == 3
                write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, 95/0.088);
                pause(1)
                status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_mid, 0);
                status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_deg, 1);
                status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_mid, 1);
                status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, rotate_mid, 1);
                status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, rotate_deg, 0);
                status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, rotate_mid, 0);
            end
        end
        
        % pos2 is the position where cube is placed
        % rotate means whether it needs to be rotated or not
        % 1 is yes, 0 is no
        %number indicates which number of cube we are stacking
        %1 indicates second cube, 2 means third cube
        function number = robot_stack(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, pos1, pos2, number, rotate)
            if rotate == 0
                pick_phi = -85;
                drop_phi = -85;
            elseif rotate == 1
                pick_phi = -85;
                drop_phi = 0;
            end
            if number == 1
                cube = 3;
            elseif number == 2
                cube = 5;
            end
            pos_end = [pos2(1), pos2(2),pos2(3) + cube];
            [start, start_mid] = robot_pick_angle(pos1,pick_phi,1);
            [final, final_mid] = robot_pick_angle(pos_end,drop_phi,1);
            status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, start_mid, 0);
            status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, start, 1);
            status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, start_mid, 1);
            status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, final_mid, 1);
            status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, final, 0);
            status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, final_mid, 0);
               if rotate == 0
                write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, 180/0.088);
                pause(1)
            elseif rotate == 1
                write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, 90/0.088);
                pause(1)
            end
        end

        % status = 1, x dim; status = 2, y dim; status = 3, diagonal
        % step has to be positive
        function status = robot_line(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, start_pos, end_pos, number_iteration,t,tf, status)
            [start, start_mid] = robot_pick_angle(start_pos,0,3);
           % [final, final_mid] = robot_pick_angle(end_pos,0,3);
            %status1 = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, start_mid, 1);
            status1 = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, start, 1);
            if status == 1 
                x_diff = end_pos(status) - start_pos(status);
                step = x_diff / number_iteration;
                initial_pos = start_pos;
                start_pos1 = start_pos;
                traj = trajectory_degree(step, x_diff, 1, 0, initial_pos, start_pos1, t, tf);
                
            elseif status == 2
                y_diff = end_pos(status) - start_pos(status);
                step = y_diff / number_iteration;
                initial_pos = start_pos;
                start_pos1 = start_pos;
                traj = trajectory_degree(step, y_diff, 0, 1, initial_pos, start_pos1, t, tf);
               
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
            theta1_traj = (traj(1, :) + 180)./0.088;
            theta2_traj = (traj(2, :) + 180)./0.088;
            theta3_traj = (-traj(3, :) + 180)./0.088;
            theta4_traj = (-traj(4, :) + 180)./0.088;
            a = trajectory_movement(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, theta1_traj,theta2_traj,theta3_traj,theta4_traj);
            pause(2)
            %status1 = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, final_mid, 1);
        end
        %If with other drawing, only arc = 0; otherwise only_arc = 1
        function angle = robot_arc(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION,start_pos,centre, angle, step, r, t, tf)
            start_pos = [centre(1)+r*cosd(0), centre(2)+r*sind(0), centre(3)]';
            arc_traj = [];
            [centre_deg, centre_mid] = robot_pick_angle(centre,0,3);
            [start, start_mid] = robot_pick_angle(start_pos,0,3);
            
            status1 = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, centre_mid, 1);
            status1 = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, centre_deg, 1);
            status = robot_line(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, centre, start_pos, 5,t,tf, 1);
            for i = 0:step:angle
            desired_pos = [centre(1)+r*cosd(i), centre(2)+r*sind(i), centre(3)]';
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
            [deg, mid] = robot_pick_angle(desired_pos,0,3);
            theta1_arc = (arc_traj(1, :) + 180)./0.088;
            theta2_arc = (arc_traj(2, :) + 180)./0.088;
            theta3_arc = (-arc_traj(3, :) + 180)./0.088;
            theta4_arc = (-arc_traj(4, :) + 180)./0.088;
            theta1_arc = trajectory_movement(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, theta1_arc,theta2_arc,theta3_arc,theta4_arc);
            status1 = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, mid, 1);          

        end

        % type = 1, arc with forward and backward motion; 
        % type = 2, horizontal sweep
        function angle = robot_sweep(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, centre,r,angle,t,tf)
                start_pos = [centre(1), centre(2)+r, centre(3)]';
                arc_traj = [];
                centre_deg = robot_angle(centre,0);
                status1 = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, centre_deg, 1);
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
                theta1_arc = (arc_traj(1, :) + 180)./0.088;
                theta2_arc = (arc_traj(2, :) + 180)./0.088;
                theta3_arc = (-arc_traj(3, :) + 180)./0.088;
                theta4_arc = (-arc_traj(4, :) + 180)./0.088;
                for i = 1:1:length(theta1_traj)
                    write4ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_PRO_GOAL_POSITION, theta1_traj(i));
                    write4ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_PRO_GOAL_POSITION, theta2_traj(i));
                    write4ByteTxRx(port_num, PROTOCOL_VERSION, 13, ADDR_PRO_GOAL_POSITION, theta3_traj(i));
                    write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, 195/0.088);
                    pause(0.2)
                    write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, 165/0.088);
                    pause(0.2)
                end
        end
        function ENABLE = torque(port_num, PROTOCOL_VERSION, ADDR_PRO_TORQUE_ENABLE, ENABLE)
            write1ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_PRO_TORQUE_ENABLE, ENABLE);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_PRO_TORQUE_ENABLE, ENABLE);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, 13, ADDR_PRO_TORQUE_ENABLE, ENABLE);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_TORQUE_ENABLE, ENABLE);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, 15, ADDR_PRO_TORQUE_ENABLE, ENABLE);
        end

        function MAX_POS_id0 = max_pos_limit(port_num,PROTOCOL_VERSION,ADDR_MAX_POS,MAX_POS_id0)
            write4ByteTxRx(port_num,PROTOCOL_VERSION,11,ADDR_MAX_POS,MAX_POS_id0);
            write4ByteTxRx(port_num,PROTOCOL_VERSION,12,ADDR_MAX_POS,MAX_POS_id0);
            write4ByteTxRx(port_num,PROTOCOL_VERSION,13,ADDR_MAX_POS,MAX_POS_id0);
            write4ByteTxRx(port_num,PROTOCOL_VERSION,14,ADDR_MAX_POS,MAX_POS_id0);
        end

        function MIN_POS_id0 = min_pos_limit(port_num,PROTOCOL_VERSION,ADDR_MIN_POS,MIN_POS_id0)
            write4ByteTxRx(port_num,PROTOCOL_VERSION,11,ADDR_MIN_POS,MIN_POS_id0);
            write4ByteTxRx(port_num,PROTOCOL_VERSION,12,ADDR_MIN_POS,MIN_POS_id0);
            write4ByteTxRx(port_num,PROTOCOL_VERSION,13,ADDR_MIN_POS,MIN_POS_id0);
            write4ByteTxRx(port_num,PROTOCOL_VERSION,14,ADDR_MIN_POS,MIN_POS_id0);
        end

        function mode = operating_mode(port_num, PROTOCOL_VERSION, ADDR_PRO_OPERATING_MODE, mode)
            write1ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_PRO_OPERATING_MODE, mode);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_PRO_OPERATING_MODE, mode);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, 13, ADDR_PRO_OPERATING_MODE, mode);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_OPERATING_MODE, mode);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, 15, ADDR_PRO_OPERATING_MODE, mode);
        end

        function mode = drive_mode(port_num, PROTOCOL_VERSION, ADDR_PRO_DRIVE_MODE, mode)
            write1ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_PRO_DRIVE_MODE, mode);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_PRO_DRIVE_MODE, mode);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, 13, ADDR_PRO_DRIVE_MODE, mode);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_DRIVE_MODE, mode);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, 15, ADDR_PRO_DRIVE_MODE, mode);
        end

        function speed = profile_velocity(port_num,PROTOCOL_VERSION,ADDR_PRO_PROFILE_VELOCITY,speed,speed_grab)
            write4ByteTxRx(port_num,PROTOCOL_VERSION,11,ADDR_PRO_PROFILE_VELOCITY,speed);
            write4ByteTxRx(port_num,PROTOCOL_VERSION,12,ADDR_PRO_PROFILE_VELOCITY,speed);
            write4ByteTxRx(port_num,PROTOCOL_VERSION,13,ADDR_PRO_PROFILE_VELOCITY,speed);
            write4ByteTxRx(port_num,PROTOCOL_VERSION,14,ADDR_PRO_PROFILE_VELOCITY,speed);
            write4ByteTxRx(port_num,PROTOCOL_VERSION,15,ADDR_PRO_PROFILE_VELOCITY,speed_grab);
        end

    end
end

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
            % verfity end_pos
            T_mat1 = DHTransform(theta1, theta2, theta3, theta4, beta);
            T_mat2 = DHTransform(theta1, theta2_, theta3_, theta4_, beta);
            T_mat3 = DHTransform(theta1_, theta2, theta3, theta4, beta);
            T_mat4 = DHTransform(theta1_, theta2_, theta3_, theta4_, beta);
            T_end_pos1 = T_mat1(1:3, 4);
            T_end_pos2 = T_mat2(1:3, 4);
            T_end_pos3 = T_mat3(1:3, 4);
            T_end_pos4 = T_mat4(1:3, 4);
            diff1 = T_end_pos1 - target_end_pos;
            diff2 = T_end_pos2 - target_end_pos;
            diff3 = T_end_pos3 - target_end_pos;
            diff4 = T_end_pos4 - target_end_pos;
            result = res2;
end

function result = IK2(pos_x,pos_y,pos_z,phi)
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
            % verfity end_pos
            T_mat1 = DHTransform(theta1, theta2, theta3, theta4, beta);
            T_mat2 = DHTransform(theta1, theta2_, theta3_, theta4_, beta);
            T_mat3 = DHTransform(theta1_, theta2, theta3, theta4, beta);
            T_mat4 = DHTransform(theta1_, theta2_, theta3_, theta4_, beta);
            T_end_pos1 = T_mat1(1:3, 4);
            T_end_pos2 = T_mat2(1:3, 4);
            T_end_pos3 = T_mat3(1:3, 4);
            T_end_pos4 = T_mat4(1:3, 4);
            diff1 = T_end_pos1 - target_end_pos;
            diff2 = T_end_pos2 - target_end_pos;
            diff3 = T_end_pos3 - target_end_pos;
            diff4 = T_end_pos4 - target_end_pos;
            result = res4;
end

function final_mat = threeDTransform(alpha, a, d, theta)
final_mat = [cosd(theta),             -sind(theta),            0,            a;
             sind(theta)*cosd(alpha), cosd(theta)*cosd(alpha), -sind(alpha), -sind(alpha)*d;
             sind(theta)*sind(alpha), cosd(theta)*sind(alpha), cosd(alpha),  cosd(alpha)*d;
             0,                       0,                       0,            1];
end

function T_final = DHTransform(theta1, theta2, theta3, theta4, beta)
T0 = eye(4);
T1 = T0 * threeDTransform(0, 0, 7.7, theta1);
T2 = T1 * threeDTransform(-90, 0, 0, -90);
T3 = T2 * threeDTransform(0, 0, 0, -beta-theta2);
T3 = T3 * threeDTransform(0, 13, 0, 0);
T4 = T3 * threeDTransform(0, 0, 0, beta-90);
T5 = T4 * threeDTransform(0, 0, 0, theta3); 
T5 = T5 * threeDTransform(0, 12.4, 0, 0);
T6 = T5 * threeDTransform(0, 0, 0, theta4);
T6 = T6 * threeDTransform(0, 12.6, 0, 0);
T_final = T6;
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

function status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_deg, status)
            %id15 = 1 close; id15 = 0 open
            if status == 1
                id15 = 223/0.088;
            elseif status == 2
                id15 = 233/0.088;
            elseif status == 0
                id15 = 137/0.088;
            end
            write4ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_PRO_GOAL_POSITION, IK_deg(1));
            write4ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_PRO_GOAL_POSITION, IK_deg(2));
            write4ByteTxRx(port_num, PROTOCOL_VERSION, 13, ADDR_PRO_GOAL_POSITION, IK_deg(3));
            pause(2)
            write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, IK_deg(4));
            pause(3)
            write4ByteTxRx(port_num, PROTOCOL_VERSION, 15, ADDR_PRO_GOAL_POSITION, id15);
            pause(2.5)
end

function traj = trajectory(theta, t,tf)
            a0 = theta(1);
            a1 = 0;
            a2 = (3/tf^2) * (theta(2) - theta(1));
            a3 = (-2/tf^3) * (theta(2) - theta(1));
            traj = a0 + a1 .* t + a2 .* t.^2 + a3 .* t.^3;
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

function theta1_traj = trajectory_movement(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, theta1_traj,theta2_traj,theta3_traj,theta4_traj)
            for i = 1:1:length(theta1_traj)
                write4ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_PRO_GOAL_POSITION, theta1_traj(i));
                write4ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_PRO_GOAL_POSITION, theta2_traj(i));
                write4ByteTxRx(port_num, PROTOCOL_VERSION, 13, ADDR_PRO_GOAL_POSITION, theta3_traj(i));
                write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, theta4_traj(i));
                pause(0.2)
            end
end

function status = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_deg, status)
            %id15 = 1 close; id15 = 0 open
            if status == 1
                %id15 = 233/0.088;
                id15 = 233/0.088;% pen
            elseif status == 0
                id15 = 137/0.088;
            end
            write4ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_PRO_GOAL_POSITION, IK_deg(1));
            write4ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_PRO_GOAL_POSITION, IK_deg(2));
            write4ByteTxRx(port_num, PROTOCOL_VERSION, 13, ADDR_PRO_GOAL_POSITION, IK_deg(3));
            write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, IK_deg(4));
            pause(3)
            write4ByteTxRx(port_num, PROTOCOL_VERSION, 15, ADDR_PRO_GOAL_POSITION, id15);
            pause(2.5)
end

function status = robot_line(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, start_pos, end_pos, number_iteration,t,tf, status)
            [start, start_mid] = robot_pick_angle(start_pos,0,3);
           % [final, final_mid] = robot_pick_angle(end_pos,0,3);
            %status1 = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, start_mid, 1);
            status1 = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, start, 1);
            if status == 1 
                x_diff = end_pos(status) - start_pos(status);
                step = x_diff / number_iteration;
                initial_pos = start_pos;
                start_pos1 = start_pos;
                traj = trajectory_degree(step, x_diff, 1, 0, initial_pos, start_pos1, t, tf);
                
            elseif status == 2
                y_diff = end_pos(status) - start_pos(status);
                step = y_diff / number_iteration;
                initial_pos = start_pos;
                start_pos1 = start_pos;
                traj = trajectory_degree(step, y_diff, 0, 1, initial_pos, start_pos1, t, tf);
               
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
            theta1_traj = (traj(1, :) + 180)./0.088;
            theta2_traj = (traj(2, :) + 180)./0.088;
            theta3_traj = (-traj(3, :) + 180)./0.088;
            theta4_traj = (-traj(4, :) + 180)./0.088;
            a = trajectory_movement(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, theta1_traj,theta2_traj,theta3_traj,theta4_traj);
            pause(2)
            %status1 = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, final_mid, 1);
end

function angle = robot_arc(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION,start_pos,centre, angle, step, r, t, tf, only_arc)
            start_pos = [centre(1)+r*sind(0), centre(2)+r*cosd(0), centre(3)]';
            arc_traj = [];
            [centre_deg, centre_mid] = robot_pick_angle(centre,0,3);
            [start, start_mid] = robot_pick_angle(start_pos,0,3);
            if only_arc == 0
            status1 = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, start_mid, 1);
            end
            status1 = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, centre_mid, 1);
            status1 = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, centre_deg, 1);
            status = robot_line(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, centre, start_pos, 1,t,tf, 2);
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
            theta1_arc = (arc_traj(1, :) + 180)./0.088;
            theta2_arc = (arc_traj(2, :) + 180)./0.088;
            theta3_arc = (-arc_traj(3, :) + 180)./0.088;
            theta4_arc = (-arc_traj(4, :) + 180)./0.088;
            theta1_arc = trajectory_movement(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, theta1_arc,theta2_arc,theta3_arc,theta4_arc);
            if angle == 180
                status = robot_line(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, desired_pos, centre, 1,t,tf, 2);
            else
                status = robot_line(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, desired_pos, centre, 1,t,tf, 3);
            end
            status1 = robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, centre_mid, 1);

end

function IK_deg = robot_angle(pos_coordinate,phi)
            pos_angle = IK(pos_coordinate(1),pos_coordinate(2),pos_coordinate(3),phi);
            IK_deg1 = (pos_angle(1) + 180) / 0.088;
            IK_deg2 = (pos_angle(2) + 180) / 0.088;
            IK_deg3 = (-pos_angle(3) + 180) / 0.088;
            IK_deg4 = (-pos_angle(4) + 180) / 0.088;
            IK_deg = [IK_deg1,IK_deg2,IK_deg3,IK_deg4];
        end