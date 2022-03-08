classdef robotic_function
    methods
        function IK_deg = robot_angle(pos_coordinate,phi)
            pos_angle = IK(pos_coordinate(1),pos_coordinate(2),pos_coordinate(3),phi);
            IK_deg1 = (pos_angle(1) + 180) / 0.088;
            IK_deg2 = (pos_angle(2) + 180) / 0.088;
            IK_deg3 = (-pos_angle(3) + 180) / 0.088;
            IK_deg4 = (-pos_angle(4) + 180) / 0.088;
            IK_deg = [IK_deg1,IK_deg2,IK_deg3,IK_deg4];
        end

        function IK_deg = robot_mid_angle(pos_coordinate,phi,height)
            pos_angle = IK(pos_coordinate(1),pos_coordinate(2),pos_coordinate(3)+height,phi);
            IK_deg1 = (pos_angle(1) + 180) / 0.088;
            IK_deg2 = (pos_angle(2) + 180) / 0.088;
            IK_deg3 = (-pos_angle(3) + 180) / 0.088;
            IK_deg4 = (-pos_angle(4) + 180) / 0.088;
            IK_deg = [IK_deg1,IK_deg2,IK_deg3,IK_deg4];
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

        function cube = robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK_deg, status)
            %id15 = 1 close; id15 = 0 open
            if status == 1
                id15 = 213/0.088;
            elseif status == 0
                id15 = 137/0.088;
            end
            write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_GOAL_POSITION, IK_deg(1));
            pause(2)
            write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID12, ADDR_PRO_GOAL_POSITION, IK_deg(2));
            pause(2)
            write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID13, ADDR_PRO_GOAL_POSITION, IK_deg(3));
            pause(2)
            write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID14, ADDR_PRO_GOAL_POSITION, IK_deg(4));
            pause(5)
            write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID15, ADDR_PRO_GOAL_POSITION, id15);
            pause(2)
            cube = 0;
        end

        function ENABLE = torque(port_num, PROTOCOL_VERSION, ADDR_PRO_TORQUE_ENABLE, ENABLE)
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_TORQUE_ENABLE, ENABLE);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID12, ADDR_PRO_TORQUE_ENABLE, ENABLE);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID13, ADDR_PRO_TORQUE_ENABLE, ENABLE);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID14, ADDR_PRO_TORQUE_ENABLE, ENABLE);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID15, ADDR_PRO_TORQUE_ENABLE, ENABLE);
        end

        function MAX_POS_id0 = max_pos_limit(port_num,PROTOCOL_VERSION,ADDR_MAX_POS,MAX_POS_id0)
            write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID11,ADDR_MAX_POS,MAX_POS_id0);
            write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID12,ADDR_MAX_POS,MAX_POS_id0);
            write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID13,ADDR_MAX_POS,MAX_POS_id0);
            write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID14,ADDR_MAX_POS,MAX_POS_id0);
        end

        function MIN_POS_id0 = min_pos_limit(port_num,PROTOCOL_VERSION,ADDR_MIN_POS,MIN_POS_id0)
            write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID11,ADDR_MIN_POS,MIN_POS_id0);
            write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID12,ADDR_MIN_POS,MIN_POS_id0);
            write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID13,ADDR_MIN_POS,MIN_POS_id0);
            write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID14,ADDR_MIN_POS,MIN_POS_id0);
        end

        function mode = operating_mode(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_OPERATING_MODE, mode)
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_OPERATING_MODE, mode);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID12, ADDR_PRO_OPERATING_MODE, mode);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID13, ADDR_PRO_OPERATING_MODE, mode);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID14, ADDR_PRO_OPERATING_MODE, mode);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID15, ADDR_PRO_OPERATING_MODE, mode);
        end

        function mode = drive_mode(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_DRIVE_MODE, mode)
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_DRIVE_MODE, mode);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID12, ADDR_PRO_DRIVE_MODE, mode);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID13, ADDR_PRO_DRIVE_MODE, mode);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID14, ADDR_PRO_DRIVE_MODE, mode);
            write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID15, ADDR_PRO_DRIVE_MODE, mode);
        end

        function speed = profile_velocity(port_num,PROTOCOL_VERSION,DXL_ID11,ADDR_PRO_PROFILE_VELOCITY,speed,speed_grab)
            write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID11,ADDR_PRO_PROFILE_VELOCITY,speed);
            write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID12,ADDR_PRO_PROFILE_VELOCITY,speed);
            write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID13,ADDR_PRO_PROFILE_VELOCITY,speed);
            write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID14,ADDR_PRO_PROFILE_VELOCITY,speed);
            write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID15,ADDR_PRO_PROFILE_VELOCITY,speed_grab);
        end

        function traj = trajectory(theta, tf)
            t = 1:1:tf;
            a0 = theta(1);
            a1 = 0;
            a2 = (3/tf^2) * (theta(2) - theta(1));
            a3 = (-2/tf^3) * (theta(2) - theta(1));
            traj = a0 + a1 .* t + a2 .* t.^2 + a3 .* t.^3;
        end

        function traj = robot_traj(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_TORQUE_ENABLE, theta1_traj,theta2_traj,theta3_traj,theta4_traj)
            for i = 1:1:length(theta1_traj)
                write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_TORQUE_ENABLE, 1);
                write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_GOAL_POSITION, theta1_traj(i));
                pause(2)
                write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID12, ADDR_PRO_TORQUE_ENABLE, 1);
                write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID12, ADDR_PRO_GOAL_POSITION, theta2_traj(i));
                pause(2)
                write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID13, ADDR_PRO_TORQUE_ENABLE, 1);
                write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID13, ADDR_PRO_GOAL_POSITION, theta3_traj(i));
                pause(2)
                write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID14, ADDR_PRO_TORQUE_ENABLE, 1);
                write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID14, ADDR_PRO_GOAL_POSITION, theta4_traj(i));
                pause(3)
                traj = 0;
            end
        end



    end
end