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

MAX_POS_id0 = 3300; % 270
MIN_POS_id0 = 600;  % 90
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
% Drive mode
mode = robotic_function.drive_mode(port_num, PROTOCOL_VERSION, ADDR_PRO_DRIVE_MODE,4);
pause(0.5)
% set Profile Velocity
speed = robotic_function.profile_velocity(port_num,PROTOCOL_VERSION,ADDR_PRO_PROFILE_VELOCITY,1200,1000);
%Enable Torque
torque_enable = robotic_function.torque(port_num, PROTOCOL_VERSION, ADDR_PRO_TORQUE_ENABLE,1);

cube = robotic_function.robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, DEFAULT_POS, 0);

dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);

if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end
%% Angry
%angry book
pos1 = [-12.3,-12.5,3.8];
drop11 = [-15.2,-15.2,3.1];
drop14 = [-16.9,-16.9,2.6];
drop12 = [-16.2,-14.4,0.8];
drop13 = [-17.8,-16.8,0.8];

[pick1_deg, pick1_mid] = robotic_function.robot_pick_angle(pos1,-90,3);
[drop11_deg,drop11_mid] = robotic_function.robot_pick_angle(drop11,-10,10);
[drop12_deg,drop12_mid] = robotic_function.robot_pick_angle(drop12,-20,5);
[drop13_deg,drop13_mid] = robotic_function.robot_pick_angle(drop13,-20,5);
drop14_deg = robotic_function.robot_angle(drop14,-10);
speed = robotic_function.profile_velocity(port_num,PROTOCOL_VERSION,ADDR_PRO_PROFILE_VELOCITY,600,300);
status1 = robotic_function.robot_pick(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, pick1_mid, 0);
status1 = robotic_function.robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, pick1_deg, 5);
status1 = robotic_function.robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, pick1_mid, 5);

write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, 135/0.088);
pause(0.08)
write4ByteTxRx(port_num, PROTOCOL_VERSION, 15, ADDR_PRO_GOAL_POSITION, 135/0.088);
pause(1)
% Angry water mode
speed = robotic_function.profile_velocity(port_num,PROTOCOL_VERSION,ADDR_PRO_PROFILE_VELOCITY,1000,900);
pos1 = [0,-19.5,7];
[water_deg, water_mid] = robotic_function.robot_pick_angle(pos1,0,9);
pos_smash = [-18,0,13];
smash_deg = robotic_function.robot_angle(pos_smash,0);
pos2 = [-9.2,20.2,5.9];
[plant_deg1,plant_mid1] = robotic_function.robot_pick_angle(pos2,-80,2);
pos3 = [-7.7,13,15];
plant_deg2 = robotic_function.robot_angle(pos3,0);
pour_deg1 = robotic_function.robot_angle(pos2,-30);
pour_deg2 = robotic_function.robot_angle(pos2,-50);
status = robotic_function.robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, plant_mid1, 0);
status = robotic_function.robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, plant_deg1, 4);
status = robotic_function.robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, plant_deg2, 4);
status = robotic_function.robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, water_mid, 0);

%Angry Trash
speed = robotic_function.profile_velocity(port_num,PROTOCOL_VERSION,ADDR_PRO_PROFILE_VELOCITY,600,300);
pos_pick = [-13.1,7,1.85];
[pick_deg, pick_mid] = robotic_function.robot_pick_angle(pos_pick,-85,8);
%high up, crawl together
pos_ready = [-2,12.6,30];
% another 2 angles or ratating and pretend to aim
pos_aim1 = [3,13,30];
pos_aim2 = [-6,7.3,32.3];
%extend with a phi of 30
pos_throw = [-21,-7.9,37.3];
ready_deg = robotic_function.robot_angle(pos_ready,90);
aim_deg1 = robotic_function.robot_angle(pos_aim1,90);
aim_deg2 = robotic_function.robot_angle(pos_aim2,90);
throw_deg = robotic_function.robot_angle(pos_throw,50);

status = robotic_function.robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, pick_mid, 0);
status = robotic_function.robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, pick_deg, 6);
status = robotic_function.robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, pick_mid, 6);

%pick
write4ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_PRO_GOAL_POSITION, ready_deg(1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_PRO_GOAL_POSITION, ready_deg(2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, 13, ADDR_PRO_GOAL_POSITION, ready_deg(3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, ready_deg(4));
write4ByteTxRx(port_num, PROTOCOL_VERSION, 15, ADDR_PRO_GOAL_POSITION, 194/0.088);
pause(3)
%"aim"
write4ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_PRO_GOAL_POSITION, aim_deg1(1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_PRO_GOAL_POSITION, aim_deg1(2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, 13, ADDR_PRO_GOAL_POSITION, aim_deg1(3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, aim_deg1(4));
pause(2)
write4ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_PRO_GOAL_POSITION, aim_deg2(1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_PRO_GOAL_POSITION, aim_deg2(2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, 13, ADDR_PRO_GOAL_POSITION, aim_deg2(3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, aim_deg2(4));
pause(3)
speed = robotic_function.profile_velocity(port_num,PROTOCOL_VERSION,ADDR_PRO_PROFILE_VELOCITY,300,100);
%throw
write4ByteTxRx(port_num, PROTOCOL_VERSION, 11, ADDR_PRO_GOAL_POSITION, throw_deg(1));
write4ByteTxRx(port_num, PROTOCOL_VERSION, 12, ADDR_PRO_GOAL_POSITION, throw_deg(2));
write4ByteTxRx(port_num, PROTOCOL_VERSION, 13, ADDR_PRO_GOAL_POSITION, throw_deg(3));
write4ByteTxRx(port_num, PROTOCOL_VERSION, 14, ADDR_PRO_GOAL_POSITION, throw_deg(4));
pause(0.03)
write4ByteTxRx(port_num, PROTOCOL_VERSION, 15, ADDR_PRO_GOAL_POSITION, 135/0.088);
%% Default
cube = robotic_function.robot_draw(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, DEFAULT_POS, 0);


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


