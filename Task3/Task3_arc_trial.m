% Read the position of the dynamixel horn with the torque off
% The code executes for a given amount of time then terminates


clc;
clear all;

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

%% ---- Control Table Addresses ---- %%

ADDR_PRO_TORQUE_ENABLE       = 64;           % Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 116; 
ADDR_PRO_PROFILE_VELOCITY    = 112;
ADDR_PRO_PRESENT_POSITION    = 132; 
ADDR_PRO_OPERATING_MODE      = 11;
ADDR_PRO_DRIVE_MODE          = 10;

%% ---- Other Settings ---- %%

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL_ID11                     = 11;            % Dynamixel ID: 1
DXL_ID12                     = 12;
DXL_ID13                     = 13;
DXL_ID14                     = 14;
DXL_ID15                     = 15;
BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM10';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
DEFAULT_POS = [2048,2048,2048,2048];                                            
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE2  = -150000;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE2  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MINIMUM_POSITION_VALUE1  = 600;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE1  = 3400; 
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

%% ------------------ %%

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE1 DXL_MAXIMUM_POSITION_VALUE1];         % Goal position

dxl_error = 0;                              % Dynamixel error
dxl_present_position0 = 0;                   % Present position
dxl_present_position1 = 0;

% Open port
if (openPort(port_num))
    fprintf('Port Open\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port\n');
    input('Press any key to terminate...\n');
    return;
end


% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% Set motion limits
ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;

MAX_POS_id0 = 3070; % 270
MIN_POS_id0 = 1068;  % 90
dxl_present_position11 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_PRESENT_POSITION);
dxl_present_position12 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID12, ADDR_PRO_PRESENT_POSITION);
dxl_present_position13 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID13, ADDR_PRO_PRESENT_POSITION);
dxl_present_position14 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID14, ADDR_PRO_PRESENT_POSITION);

% Set max pos limit
max = robotic_function.max_pos_limit(port_num,PROTOCOL_VERSION,ADDR_MAX_POS,MAX_POS_id0);
% Set min pos limit
MIN = robotic_function.min_pos_limit(port_num,PROTOCOL_VERSION,ADDR_MIN_POS,MIN_POS_id0);
% Put actuator into Position Control Mode
mode = robotic_function.operating_mode(port_num, PROTOCOL_VERSION, ADDR_PRO_OPERATING_MODE, 3);
%Enable Torque
%torque_disbale = robotic_function.torque(port_num, PROTOCOL_VERSION, ADDR_PRO_TORQUE_ENABLE,0);
% Drive mode
mode = robotic_function.drive_mode(port_num, PROTOCOL_VERSION, ADDR_PRO_DRIVE_MODE,4);
pause(0.5)
% set Profile Velocity
speed = robotic_function.profile_velocity(port_num,PROTOCOL_VERSION,ADDR_PRO_PROFILE_VELOCITY,1600,1200);
%Enable Torque
torque_enable = robotic_function.torque(port_num, PROTOCOL_VERSION, ADDR_PRO_TORQUE_ENABLE,1);

%Default position for all servos
% default1 = robotic_function.trajectory([dxl_present_position11,180/0.088],5);
% default2 = robotic_function.trajectory([dxl_present_position12,180/0.088],5);
% default3 = robotic_function.trajectory([dxl_present_position13,180/0.088],5);
% default4 = robotic_function.trajectory([dxl_present_position14,180/0.088],5);
% traj = robotic_function.robot_traj(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, default1,default2,default3,default4);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, 15, ADDR_PRO_GOAL_POSITION, 137/0.088);

cube = robotic_function.robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, DEFAULT_POS, 1);

dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);

if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end

% T_0 Matrix base frame
%pos_1 = input('Please input desired [] position /n');
%pos_2 = input('Please input final [] position');
%first cube pos
%% Need to measure location again, it is off
centre = [-27.3958,0,20.4773];
phi = 0;
%% Pick up all cubes and place them
%First cube
%IK_start = robotic_function.robot_angle(centre,phi);
%% Clculate all IK 
tf = 6;
t = 0:1:tf;
r = 10;
full_traj = [];
angle_step = 10;
angle = 30;
start_pos = [centre(1)+r*cosd(0), centre(2)+r*sind(0), centre(3)]';
for i = 1:angle_step:angle
desired_pos = [centre(1)+r*cosd(i), centre(2)+r*sind(i), centre(3)]';
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

theta1_draw = (full_traj(1, :) + 180)./0.088;
theta2_draw = (full_traj(2, :) + 180)./0.088;
theta3_draw = (-full_traj(3, :) + 180)./0.088;
theta4_draw = (-full_traj(4, :) + 180)./0.088;

for i = 1:length(theta1_draw)
    write4ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_PRO_GOAL_POSITION, theta1_draw(i));
    write4ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_PRO_GOAL_POSITION, theta2_draw(i));
    write4ByteTxRx(port_num, PROTOCOL_VERSION, 13, ADDR_PRO_GOAL_POSITION, theta3_draw(i));
    write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, theta4_draw(i));
end
%% 
    j = 0;
    while (j<3000)
        j = j+1;

        % Read present position
        dxl_present_position0 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_PRESENT_POSITION);
        dxl_present_position1 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID12, ADDR_PRO_PRESENT_POSITION);

        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        
        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
        
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end

        if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position0), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
            break;
        end

       
    end

% Disable Dynamixel Torque
%cube1 = robotic_function.torque(port_num, PROTOCOL_VERSION, ADDR_PRO_TORQUE_ENABLE,0);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Close port
closePort(port_num);
fprintf('Job Finished! \n');


% Unload Library
unloadlibrary(lib_name);

close all;
%clear all;
function  res = IK(target_end_pos, phi)
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