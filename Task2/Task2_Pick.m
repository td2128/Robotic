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
BAUDRATE                    = 115200;
DEVICENAME                  = 'COM4';       % Check which port is being used on your controller
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

% Set max pos limit
max = robotic_function.max_pos_limit(port_num,PROTOCOL_VERSION,ADDR_MAX_POS,MAX_POS_id0);
% Set min pos limit
MIN = robotic_function.min_pos_limit(port_num,PROTOCOL_VERSION,ADDR_MIN_POS,MIN_POS_id0);
% Put actuator into Position Control Mode
mode = robotic_function.operating_mode(port_num, PROTOCOL_VERSION, ADDR_PRO_OPERATING_MODE, 3);
%Enable Torque
torque_disbale = robotic_function.torque(port_num, PROTOCOL_VERSION, ADDR_PRO_TORQUE_ENABLE,0);
% Drive mode
mode = robotic_function.drive_mode(port_num, PROTOCOL_VERSION, ADDR_PRO_DRIVE_MODE,4);
pause(0.5)
% set Profile Velocity
speed = robotic_function.profile_velocity(port_num,PROTOCOL_VERSION,ADDR_PRO_PROFILE_VELOCITY,4000,3000);
%Enable Torque
torque_enable = robotic_function.torque(port_num, PROTOCOL_VERSION, ADDR_PRO_TORQUE_ENABLE,1);

% Default position for all servos
default = robotic_function.robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, DEFAULT_POS,0);

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
pos1 = [-7.6,20.2,5];
pos2 = [-12.9,11.9,6];
%second cube pos
pos3 = [-21.7,-0.5,5.5];
pos4 = [-10,-0.3,5.5];
%third cube pos
pos5 = [-15,-15,5];
%pos6 = [-4,-5,6];%mid point so won't touch ground
pos7 = [-0.2,-10.2,5];
phi = -50;
%% Pick up all cubes and place them
%First cube
[IK1_start_deg, IK1_mid_deg] = robotic_function.robot_pick_angle(pos1,phi,3);
[IK1_end_deg, IK1_transit_deg] = robotic_function.robot_pick_angle(pos2,phi,3);
%Second cube
[IK2_start_deg, IK2_mid_deg] = robotic_function.robot_pick_angle(pos3,phi,3);
[IK2_end_deg, IK2_transit_deg] = robotic_function.robot_pick_angle(pos4,phi,3);
%Third cube
[IK3_start_deg, IK3_mid_deg] = robotic_function.robot_pick_angle(pos5,phi,3);
[IK3_end_deg, IK3_transit_deg] = robotic_function.robot_pick_angle(pos7,phi,3);
%% Normal Operation with linear velocity
% cube1 = robotic_function.robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK1_start_deg,1);
% cube1 = robotic_function.robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK1_mid_deg,1);
% cube1 = robotic_function.robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK1_end_deg,0);
% cube1 = robotic_function.robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK1_transit_deg,0);
% 
% cube2 = robotic_function.robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK2_start_deg,1);
% cube2 = robotic_function.robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK2_mid_deg,1);
% cube2 = robotic_function.robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK2_end_deg,0);
% cube2 = robotic_function.robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK2_transit_deg,0);
% 
% cube3 = robotic_function.robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK3_start_deg,1);
% cube3 = robotic_function.robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK3_mid_deg,1);
% cube3 = robotic_function.robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK3_end_deg,0);
% cube3 = robotic_function.robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, IK3_transit_deg,0);
% 
% default = robotic_function.robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, DEFAULT_POS,0);
%% 
% % trajectory trial
% theta1_traj = robotic_function.trajectory([IK1_start_deg(1),IK1_end_deg(1)],20);
% theta2_traj = robotic_function.trajectory([IK1_start_deg(2),IK1_end_deg(2)],20);
% theta3_traj = robotic_function.trajectory([IK1_start_deg(3),IK1_end_deg(3)],20);
% theta4_traj = robotic_function.trajectory([IK1_start_deg(4),IK1_end_deg(4)],20);
% traj = robotic_function.robot_traj(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, theta1_traj,theta2_traj,theta3_traj,theta4_traj);
%% Rotation
status = robotic_function.robot_rotate(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, pos2, 1);

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




