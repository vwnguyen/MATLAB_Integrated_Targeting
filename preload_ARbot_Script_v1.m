
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INITIALIZE DIRECTORY AND FILE PATHS
% systemDirectory = genpath(pwd);
% addpath(systemDirectory);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INITIALIZE ROS CONNECTIONS
% rosshutdown;
% ipaddress = '169.254.215.90'
% username = 'arbot';
% password = 'arbot';
% rosinit(ipaddress);
% hwobj = jetson(ipaddress, username, password);
% % CREATE ROS SUBSCRIBES AND PUBLISHERS
% % listen to the talker on VM, in topic called chatter
% %msgSub = rossubscriber('/chatter','std_msgs/String');
% %receive(msgSub,10); % Wait to receive first message
% myPub = rospublisher('/my_target_topic','node_example/TargetTemplate');
% myMsg = rosmessage('node_example/TargetTemplate');
% Specify Link Lengths
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TRAJECTORY VARIABLES 
L1=2;L2=5;L3=4;L4=3;
% adjust time resolution accodingly
tf = 1; % 1 second(s), duration time of trajectory
tv = tf/2; % time to complete the trajectory, generically set to tf/2
if(tf<=1)
    dt=.05;
else
    dt=0.1;
end
t=0:tv/12:tf;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SIMULATION VARIABLES
% Constructing Links
% L(1) = Link('revolute', 'd', L1, 'a', 0, 'alpha', pi/2, 'standard');
% L(2) = Link('revolute', 'd', 0, 'a', L2, 'alpha', 0, 'standard');
% L(3) = Link('revolute', 'd', 0, 'a', L3, 'alpha', 0, 'standard');
% L(4) = Link('revolute', 'd', 0, 'a', L4, 'alpha', 0, 'standard');
% robot=SerialLink(L, 'name', '4DOF');    
q0 = [ 0, pi/4, -pi/4, pi/4 ]; % generic starting position 
qf = [-pi/2 pi/4 -pi/4 -pi/2]; % some generic bin location    
% webcam and detection variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% webcam and detection variables
cam = webcam;
cam.Resolution = '320x240';
inputSize = [244 244 3];
YOLO_Object_Classifier = load('resnet101_28_preprocessed.mat');
% rotation angles from Robot to Camera
theta = 0;
phi = 0;
psi = 0;
% vector from robot to camera origin
P_A_BORG = [ 24; 0; 0 ];
% how many targets we can handle in the camera... subject to change
total_Targets_Possible = 3;
% Catching Parameters (robotic limitation)
belt_rate = 12; % measures in inches per second for now
X_Catch_Line_Position = 5;

delta_T_Threshold = 0;  % time needed at minimum for the robot to catch
                        % the object 
max_Catching_Time = 0; % time required by the robot to make the a 
                       % successful catch

endEffectorOrientation = deg2rad(-90);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% misc variables 
bboxAreaThreshold = 1000; % bbox areas in pixels to remove
analytics = false; % controls whether the simulation plays
display_Video = true;
display_Simulation = false;
overlapThreshold = 0.2; % selectStrongestBox, set smaller for more 
                        % filtering, but at the cost of potentially     
                        % missing objects close together
