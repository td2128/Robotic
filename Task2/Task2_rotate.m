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

%cube = robotic_function.robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, DEFAULT_POS, 0);

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
pos1 = [-17.3,17.7,6.1];
pos2 = [-17.5,-5,5.4];
%pos1_2 = [-22.2,0,8];
%second cube pos
pos3 = [0,17.5,6.3];
pos4 = [0,-15,5.5];
%third cube pos
pos5 = [-10, 0,5.6];
pos6 = [-22.5,0,4.9];
phi = -80;
%% Pick up all cubes and place them
%First
%% Normal Operation with linear velocity
%status = robotic_function.robot_rotate(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, pos1,1,0,0,0,0,-80,3);
status = robotic_function.robot_rotate(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, pos2,5,0.4,0.7,1.5,-80,0,2);
status = robotic_function.robot_rotate(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, pos3,3,0,-0.5,0.7,-80,0,1);
% default = robotic_function.robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, DEFAULT_POS,0);
%% 
% % trajectory trial
%From default to first cube
% tf = 5;
% tf_small = 1;
% theta1_1 = robotic_function.trajectory([180/0.088,IK1_start_deg(1)],tf);
% theta2_1 = robotic_function.trajectory([180/0.088,IK1_start_deg(2)],tf);
% theta3_1 = robotic_function.trajectory([180/0.088,IK1_start_deg(3)],tf);
% theta4_1 = robotic_function.trajectory([180/0.088,IK1_start_deg(4)],tf);
% traj1 = robotic_function.robot_traj(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, theta1_1,theta2_1,theta3_1,theta4_1);
% pause(3)
% write4ByteTxRx(port_num, PROTOCOL_VERSION, 15, ADDR_PRO_GOAL_POSITION, 215/0.088);
% pause(2)
% theta1_11 = robotic_function.trajectory([IK1_start_deg(1),IK1_mid_deg(1)],tf_small);
% theta2_11 = robotic_function.trajectory([IK1_start_deg(2),IK1_mid_deg(2)],tf_small);
% theta3_11 = robotic_function.trajectory([IK1_start_deg(3),IK1_mid_deg(3)],tf_small);
% theta4_11 = robotic_function.trajectory([IK1_start_deg(4),IK1_mid_deg(4)],tf_small);
% traj2 = robotic_function.robot_traj(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, theta1_11,theta2_11,theta3_11,theta4_11);
% pause(3)
% theta1_111 = robotic_function.trajectory([IK1_mid_deg(1),IK1_end_deg(1)],tf_small);
% theta2_111 = robotic_function.trajectory([IK1_mid_deg(2),IK1_end_deg(2)],tf_small);
% theta3_111 = robotic_function.trajectory([IK1_mid_deg(3),IK1_end_deg(3)],tf_small);
% theta4_111 = robotic_function.trajectory([IK1_mid_deg(4),IK1_end_deg(4)],tf_small);
% traj3 = robotic_function.robot_traj(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, theta1_111,theta2_111,theta3_111,theta4_111);
% pause(3)
% write4ByteTxRx(port_num, PROTOCOL_VERSION, 15, ADDR_PRO_GOAL_POSITION, 137/0.088);
% pause(2)
%% Rotation
%%status = robotic_function.robot_rotate(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, pos2, 1);

%% 
   

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




