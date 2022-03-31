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
MIN_POS_id0 = 950;  % 90
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
speed = robotic_function.profile_velocity(port_num,PROTOCOL_VERSION,ADDR_PRO_PROFILE_VELOCITY,700,400);
%Enable Torque
torque_enable = robotic_function.torque(port_num, PROTOCOL_VERSION, ADDR_PRO_TORQUE_ENABLE,1);

%Default position for all servos
% default1 = robotic_function.trajectory([dxl_present_position11,180/0.088],5);
% default2 = robotic_function.trajectory([dxl_present_position12,180/0.088],5);
% default3 = robotic_function.trajectory([dxl_present_position13,180/0.088],5);
% default4 = robotic_function.trajectory([dxl_present_position14,180/0.088],5);
% traj = robotic_function.robot_traj(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, default1,default2,default3,default4);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, 15, ADDR_PRO_GOAL_POSITION, 137/0.088);

% cube = robotic_function.robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, DEFAULT_POS, 0);

dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);

if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end
%% 
tf = 6;
t = 0:1:tf;
%% Input Coordinate
height = 5.8;
A_initial = [175,100,height];
B_initial = [175,200,height];
C_initial= [125,150,height];
D_initial= [175,150,height];
phi = 0;
r_initial = 25;
centre_initial = [175,175,height];
%% Translate it to our model
A = [-A_initial(2)/10,-A_initial(1)/10, height];%[-20,-6,8.8407]
B = [-B_initial(2)/10,-B_initial(1)/10, height];%[-20,-14,8.8407]
C = [-C_initial(2)/10,-C_initial(1)/10, height];%[-12.5,-14,8.8407]
D = [-D_initial(2)/10,-D_initial(1)/10, height];
pos_pick = [-22,2.2,6.5];
iteration = 5;
centre = [-centre_initial(2)/10,-centre_initial(1)/10, height];%[-20,-8,8.8407];
r = r_initial/10;
%% Get all degrees for servos to draw the triangle
%First line change in x in Ad perspective
 [start, start_mid] = robotic_function.robot_pick_angle(A,0,3);
 [pick_deg,pick_mid] = robotic_function.robot_pick_angle(pos_pick,0,10);
 status1 = robotic_function.robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, pick_mid, 0);
 status1 = robotic_function.robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, pick_deg, 1);
 status1 = robotic_function.robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, pick_mid, 1);
% 
status1 = robotic_function.robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, start_mid, 1);
status = robotic_function.robot_line(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, A, B, iteration,t,tf, 1);

%Second line change in y
status = robotic_function.robot_line(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, B, C, iteration,t,tf, 3);
% Final Diagonal line
status = robotic_function.robot_line(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, C, D, iteration,t,tf, 2);
%% Arc Genrate
%  centre = [-15,-16,height];
angle_step = 10;
angle = 270;
%pause(0.5)
speed = robotic_function.profile_velocity(port_num,PROTOCOL_VERSION,ADDR_PRO_PROFILE_VELOCITY,300,400);
angle = robotic_function.robot_arc(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION,D,centre, angle, angle_step, r, t, tf);
%% Default
pos_drop = [-21.5,2.8,6.1];
[drop_deg,drop_mid] = robotic_function.robot_pick_angle(pos_drop,0,10);
status1 = robotic_function.robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, drop_mid, 1);
status1 = robotic_function.robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, drop_deg, 0);
status1 = robotic_function.robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, drop_mid, 0);
% cube = robotic_function.robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, DEFAULT_POS, 0);


%% Disable Dynamixel Torque
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