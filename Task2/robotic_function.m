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
                %id15 = 223/0.088;
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

        % status = 1, front; status = 2, down; status = 3, toward;
        % the pick/drop phi: either -80,0 or 0,-80
        function status = robot_rotate(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, pos,height,x,y,z,pick_phi,drop_phi,status)
            [IK_deg, IK_mid] = robot_pick_angle(pos,pick_phi,height);
            drop_pos = [pos(1)+x, pos(2)+y,pos(3)+z];
            [rotate_deg, rotate_mid] = robot_pick_angle(drop_pos,drop_phi,height);

            if status == 1
                status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_deg, 1);
                status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_mid, 1);
                write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, 95/0.088);
                pause(2)
                status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, rotate_deg, 0);
                status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, rotate_mid, 0);
            elseif status ==2
                status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_deg, 1);
                status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_mid, 1);
                write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, 95/0.088);
                pause(2)
                status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, rotate_deg, 0);
                status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_mid, 0);
                status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_deg, 1);
                status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_mid, 1);
                write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, 95/0.088);
                pause(2)
                status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, rotate_deg, 0);
                status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_mid, 0);
            elseif status == 3
                write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, 95/0.088);
                pause(2)
                %status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_mid, 0);
                status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_deg, 1);
                status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_mid, 1);
                status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, rotate_deg, 0);
                status = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, rotate_mid, 0);
            end
        end
        
        % pos2 is the position where cube is placed
        % rotate means whether it needs to be rotated or not
        % 1 is yes, 0 is no
        %number indicates which number of cube we are stacking
        %1 indicates second cube, 2 means third cube
        function number = robot_stack(pos1, pos2, transit_z, port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, pos_start,pos_end, number, rotate)
            if number == 1 
                cube_height = 2.5;
            elseif number == 2
                cube_height = 5;
            end

            if rotate == 0
                phi1 = -85;
                phi2 = -85;
            elseif rotate == 1
                phi1 = -85;
                phi2 = 0;
            end

            start_pos = IK(pos1(1), pos1(2),pos1(3),phi1);
            mid_pos1   = IK(pos1(1), pos1(2),pos1(3)+3,phi1);
            transit_angle = IK(pos2(1), pos2(2), pos2(3) + cube_height + transit_z, phi2);
            end_pos = IK(pos2(1), pos2(2), pos2(3)+cube_height, phi2);
            mid_pos2 = IK(pos2(1), pos2(2), pos2(3)+cube_height+3, phi2);





           
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

        function traj = trajectory(theta, tf)
            t = 1:tf:5;
            a0 = theta(1);
            a1 = 0;
            a2 = (3/tf^2) * (theta(2) - theta(1));
            a3 = (-2/tf^3) * (theta(2) - theta(1));
            traj = a0 + a1 .* t + a2 .* t.^2 + a3 .* t.^3;
        end

        function traj = robot_traj(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, theta1_traj,theta2_traj,theta3_traj,theta4_traj)
            for i = 1:1:length(theta1_traj)
                write4ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_PRO_GOAL_POSITION, theta1_traj(i));
                pause(2)
                write4ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_PRO_GOAL_POSITION, theta2_traj(i));
                pause(2)
                write4ByteTxRx(port_num, PROTOCOL_VERSION, 13, ADDR_PRO_GOAL_POSITION, theta3_traj(i));
                pause(2)
                write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, theta4_traj(i));
                pause(3)
                traj = theta1_traj;
            end
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
            % if all(abs(diff1) < 0.1)
            %     result = res1;
            % end
            % if all(abs(diff2)< 0.1) 
            %     result = res2;
            % end
            % if all(abs(diff3)< 0.1) 
            %     result = res3;
            % end
            % if all(abs(diff4)< 0.1) 
            %     result = res4;
            % end
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
                %id15 = 223/0.088;
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

