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
dxl_present_position11 = 0;                   % Present position
dxl_present_position12 = 0;
dxl_present_position13 = 0;                   % Present position
dxl_present_position14 = 0;                   % Present position
dxl_present_position15 = 0;

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

MAX_POS_id0 = 3400; % 270
MIN_POS_id0 = 600;  % 90

% Set max pos limit
write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID11,ADDR_MAX_POS,MAX_POS_id0);
write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID12,ADDR_MAX_POS,MAX_POS_id0);
write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID13,ADDR_MAX_POS,MAX_POS_id0);
write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID14,ADDR_MAX_POS,MAX_POS_id0);
%write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID15,ADDR_MAX_POS,MAX_POS_id0);

% Set min pos limit
write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID11,ADDR_MIN_POS,MIN_POS_id0);
write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID12,ADDR_MIN_POS,MIN_POS_id0);
write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID13,ADDR_MIN_POS,MIN_POS_id0);
write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID14,ADDR_MIN_POS,MIN_POS_id0);
%write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID15,ADDR_MIN_POS,MIN_POS_id0);




% Put actuator into Position Control Mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID12, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID13, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID14, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID15, ADDR_PRO_OPERATING_MODE, 3);

write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_TORQUE_ENABLE, 0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID12, ADDR_PRO_TORQUE_ENABLE, 0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID13, ADDR_PRO_TORQUE_ENABLE, 0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID14, ADDR_PRO_TORQUE_ENABLE, 0);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID15, ADDR_PRO_TORQUE_ENABLE, 0);

% Drive mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_DRIVE_MODE, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID12, ADDR_PRO_DRIVE_MODE, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID13, ADDR_PRO_DRIVE_MODE, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID14, ADDR_PRO_DRIVE_MODE, 4);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID15, ADDR_PRO_DRIVE_MODE, 4);
pause(0.5)
% set Profile Velocity
write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID11,ADDR_PRO_PROFILE_VELOCITY,5000);
write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID12,ADDR_PRO_PROFILE_VELOCITY,5000);
write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID13,ADDR_PRO_PROFILE_VELOCITY,5000);
write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID14,ADDR_PRO_PROFILE_VELOCITY,5000);
write4ByteTxRx(port_num,PROTOCOL_VERSION,DXL_ID15,ADDR_PRO_PROFILE_VELOCITY,5000);

write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID12, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID13, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID14, ADDR_PRO_TORQUE_ENABLE, 1);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID15, ADDR_PRO_TORQUE_ENABLE, 1);

% Default position for all servos
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_GOAL_POSITION, 2048);
pause(2)
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID12, ADDR_PRO_GOAL_POSITION, 2048);
pause(2)
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID13, ADDR_PRO_GOAL_POSITION, 2048);
pause(2)
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID14, ADDR_PRO_GOAL_POSITION, 2048);
pause(5)
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID15, ADDR_PRO_GOAL_POSITION, 137/0.088);

% % Disable Dynamixel Torque
% %write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_TORQUE_ENABLE, 0);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID12, ADDR_PRO_TORQUE_ENABLE, 0);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID13, ADDR_PRO_TORQUE_ENABLE, 0);
% write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID14, ADDR_PRO_TORQUE_ENABLE, 0);
% %write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID15, ADDR_PRO_TORQUE_ENABLE, 0);

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
pos_1 = [-5,20,6];
pos_2 = [-10,10.5,6];
initial_pos = IK(pos_1(1), pos_1(2),pos_1(3),-50);
mid_pos = IK(pos_1(1), pos_1(2),pos_1(3),-90);
target_pos = IK(pos_2(1), pos_2(2),pos_2(3),-90);

IK_deg1 = (initial_pos(1) + 180) / 0.088;
IK_deg2 = (initial_pos(2) + 180) / 0.088;
IK_deg3 = (-initial_pos(3) + 180) / 0.088;
IK_deg4 = (-initial_pos(4) + 180) / 0.088;

IK_mid1 = (mid_pos(1) + 180) / 0.088;
IK_mid2 = (mid_pos(2) + 180) / 0.088;
IK_mid3 = (-mid_pos(3) + 180) / 0.088;
IK_mid4 = (-mid_pos(4) + 180) / 0.088;

IK2_deg1 = (target_pos(1) + 180) / 0.088;
IK2_deg2 = (target_pos(2) + 180) / 0.088;
IK2_deg3 = (-target_pos(3) + 180) / 0.088;
IK2_deg4 = (-target_pos(4) + 180) / 0.088;

pause(4)
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_TORQUE_ENABLE, 1);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_GOAL_POSITION, IK_deg1);
pause(2)
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID12, ADDR_PRO_TORQUE_ENABLE, 1);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID12, ADDR_PRO_GOAL_POSITION, IK_deg2);
pause(2)
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID13, ADDR_PRO_TORQUE_ENABLE, 1);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID13, ADDR_PRO_GOAL_POSITION, IK_deg3);
pause(2)
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID14, ADDR_PRO_TORQUE_ENABLE, 1);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID14, ADDR_PRO_GOAL_POSITION, IK_deg4);
pause(7)
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID15, ADDR_PRO_GOAL_POSITION, 213/0.088);
pause(5)
   t5 = [];
    j = 0;
    while (j<5000)
        j = j+1;
        
        % Read present position
        dxl_present_position11 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_PRESENT_POSITION);
        dxl_present_position12 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID12, ADDR_PRO_PRESENT_POSITION);
        dxl_present_position13 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID13, ADDR_PRO_PRESENT_POSITION);
        dxl_present_position14 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID14, ADDR_PRO_PRESENT_POSITION);
        dxl_present_position15 = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID15, ADDR_PRO_PRESENT_POSITION);
        

        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        
        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
        
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end

        dxl11_angle = dxl_present_position11 * 0.088;
        dxl12_angle = dxl_present_position12 * 0.088;
        dxl13_angle = dxl_present_position13 * 0.088;
        dxl14_angle = dxl_present_position14 * 0.088;
        dxl15_angle = dxl_present_position15 * 0.088;

       %fprintf('ID 11 Position in angles:')
        IK_deg1 = dxl11_angle + 180;
        %fprintf('ID 12 Position in angles:')
        IK_deg2 = dxl12_angle +180;
        %fprintf('ID 13 Position in angles:')
        IK_deg3 = -dxl13_angle + 180;
        %fprintf('ID 14 Position in angles:')
        IK_deg4 = -dxl14_angle + 180;

       

        t5 = DHTransform(IK_deg1,IK_deg2,IK_deg3,IK_deg4,10.6197);
        disp(t5)

%         fprintf(['ID: 11 :', num2str(IK_deg1)])
%         fprintf(['ID: 12 :', num2str(IK_deg2)])
%         fprintf(['ID: 13 :', num2str(IK_deg3)])
%         fprintf(['ID: 14 :', num2str(IK_deg4)])

      

%           fprintf('[ID:%03d] ID 11 Position in angles: %03d\n', DXL_ID11, typecast(uint32(IK_deg1), 'int32'));
%           fprintf('[ID:%03d] ID 12 Position in angles: %03d\n', DXL_ID12, typecast(uint32(IK_deg2), 'int32'));
%           fprintf('[ID:%03d] ID 13 Position in angles: %03d\n', DXL_ID13, typecast(uint32(IK_deg3), 'int32'));
%           fprintf('[ID:%03d] ID 14 Position in angles: %03d\n', DXL_ID14, typecast(uint32(IK_deg4), 'int32'));
          fprintf('[ID:%03d] ID 15 Position in angles: %03d\n', DXL_ID15, typecast(uint32(dxl15_angle), 'int32'));


        if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position11), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
            break;
        end

       
    end

% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID11, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID12, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID13, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID14, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID15, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);

dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Close port
closePort(port_num);
fprintf('Port Closed \n');
fprintf('time':time)
fprintf('position':current_pos)

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;
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


res1 = [theta1, theta2, theta3, theta4]
res2 = [theta1, theta2_, theta3_, theta4_]
res3 = [theta1_, theta2, theta3, theta4]
res4 = [theta1_, theta2_, theta3_, theta4_]

% verfity end_pos
T_mat1 = DHTransform(theta1, theta2, theta3, theta4, beta);
T_mat2 = DHTransform(theta1, theta2_, theta3_, theta4_, beta);
T_mat3 = DHTransform(theta1_, theta2, theta3, theta4, beta);
T_mat4 = DHTransform(theta1_, theta2_, theta3_, theta4_, beta);

T_end_pos1 = T_mat1(1:3, 4)
T_end_pos2 = T_mat2(1:3, 4)
T_end_pos3 = T_mat3(1:3, 4)
T_end_pos4 = T_mat4(1:3, 4)

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

%% 
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
%% 

function final_mat = threeDTransform(alpha, a, d, theta)
    final_mat = [cosd(theta),             -sind(theta),            0,            a;
                 sind(theta)*cosd(alpha), cosd(theta)*cosd(alpha), -sind(alpha), -sind(alpha)*d;
                 sind(theta)*sind(alpha), cosd(theta)*sind(alpha), cosd(alpha),  cosd(alpha)*d;
                 0,                       0,                       0,            1];
end


