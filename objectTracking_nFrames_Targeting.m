close all
clear all 
clc

% %% SETUP
if ~exist('cam','var')
    run('preload_ARbot_Script_v1.m');
end
%% MAIN WHILE LOOP
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
% WEBCAM AND DETECTOR VARIABLES
%cam = webcam;
%cam.Resolution = '320x240';
inputSize = [244 244 3];
% YOLO_Object_Classifier = load('resnet101_28_preprocessed.mat');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% POSITIONING VARIABLES
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

delta_T_Threshold = 0.5;  % time needed at minimum for the robot to catch
                        % the object 
max_Catching_Time = delta_T_Threshold; % time required by the robot to make the a 
                       % successful catch

endEffectorOrientation = deg2rad(-90);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Object Tracking Variables 

n_frame = 8;
label_cnt = 1;
min_distance = 30;

% pre allocate memory
ref_n_frame_axies = 0;
% coder.varsize('ref_n_frame_axies', [ 1 n_frame ] );
ref_n_frame_labels = 0;
% coder.varsize('ref_n_frame_labels', [ 1 n_frame ] );
ref_n_frame_axies_flatten = [];
coder.varsize('ref_n_frame_axies_flatten', [ 50 2 ] );
ref_n_frame_labels_flatten = [];
coder.varsize('ref_n_frame_labels_flatten', [ 1 50 ] );
deletionLengths = [];
coder.varsize('deletionLengths', [ 1 50 ] );

n_frame = 8;
label_cnt = 1;
min_distance = 30;

% pre allocate memory
ref_n_frame_axies = 0;
% coder.varsize('ref_n_frame_axies', [ 1 n_frame ] );
ref_n_frame_labels = 0;
% coder.varsize('ref_n_frame_labels', [ 1 n_frame ] );
ref_n_frame_axies_flatten = [];
coder.varsize('ref_n_frame_axies_flatten', [ 50 2 ] );
ref_n_frame_labels_flatten = [];
coder.varsize('ref_n_frame_labels_flatten', [ 1 50 ] );
deletionLengths = [];
coder.varsize('deletionLengths', [ 1 50 ] );

filename = 'JointAngles.txt';
fileID = fopen(filename,'w');

curr_frame_axies = [];
coder.varsize('curr_frame_axies', [ 50 2 ] );
curr_frame_labels = [];
coder.varsize('curr_frame_labels', [ 1 50 ] );
unique_traj_targets = [];
coder.varsize('unique_traj_targets', [ 1 50 ] );
filtered_bboxes = [];
coder.varsize('filtered_bboxes', [ 50 2 ] );

curr_label_max = -1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% misc variables 
bboxAreaThreshold = 1000; % bbox areas in pixels to remove
analytics = false; % controls whether the simulation plays
display_Video = true;
display_Simulation = true;
overlapThreshold = 0.2; % selectStrongestBox, set smaller for more 
                        % filtering, but at the cost of potentially     
                        % missing objects close together

% t = timer; 
for z = 1:1e5
    
    %% clear the curr frames
    curr_frame_axies = [];
    curr_frame_labels = [];
    filtered_bboxes = [];
    unique_traj_targets = [];
    %% Detect And Extract Centers
    img = snapshot(cam);
    img = imresize(img,[224 224]);
    [bboxes,scores] = detect(YOLO_Object_Classifier.detector,img,'Threshold',0.6);
    [bboxes,scores] = selectStrongestBbox(bboxes,scores,'OverlapThreshold',0.2);
    %[selectedBbox,selectedScore] = selectStrongestBbox(bbox,score);
    xCenter = bboxes(:,1)   + (bboxes(:,3)/2) ; % note bboxes(:,1) = LHS X
    yCenter = bboxes(:,2)  + (bboxes(:,4)/2); % note bboxes(:,2) = Y from bottom
    %% Extract Centers
    A = [xCenter yCenter];
    %% Update Current Frames By Evaluating Current bboxes detected from image
    [ label_cnt,curr_frame_labels,curr_frame_axies,A,ref_n_frame_labels_flatten,ref_n_frame_axies_flatten ] = ...
     trackCurrentObjects(min_distance,label_cnt,curr_frame_labels,curr_frame_axies,...
     A,ref_n_frame_labels_flatten,ref_n_frame_axies_flatten);
%     %% Delete Oldest Reference Frame Value
    [ ref_n_frame_axies ref_n_frame_labels ref_n_frame_labels_flatten ref_n_frame_axies_flatten deletionLengths ] = ...
    deleteOldReferenceFrames(deletionLengths,curr_frame_labels, ...
    n_frame,ref_n_frame_axies,ref_n_frame_labels,ref_n_frame_labels_flatten,ref_n_frame_axies_flatten);
    %% Append To Reference Frame Value
    [ref_n_frame_labels  ref_n_frame_axies ref_n_frame_axies_flatten ref_n_frame_labels_flatten] = ...
    appendReferenceFrames(ref_n_frame_labels,ref_n_frame_axies,curr_frame_labels,curr_frame_axies,...
    ref_n_frame_axies_flatten,ref_n_frame_labels_flatten);

    %% Targeting Logic
    % of the current labels, check if there are any new ones. store that 
    % label value into an array, set the new max accordingly
    for i=1:length(curr_frame_labels)
        if (  curr_frame_labels(i) > curr_label_max )
            % We have to generate a trajectory for this target
            unique_traj_targets(i) = curr_frame_labels(i);
            curr_label_max = curr_frame_labels(i);
        end
    end
    % need to remove the redundant targets
    for i=1:length(unique_traj_targets) 
        newIdx = find( curr_frame_labels == unique_traj_targets(i))
        if ( ~isempty(newIdx) ) 
            filtered_bboxes(i,:) = A(newIdx(1),:);
        end
    end
    % if there are unique targets, generate coordinates to get them
    if ~isempty(filtered_bboxes)
        %% Phase 1: Extract Coordinates, Map To Catch Line
        coordinates = sense(img,filtered_bboxes,theta, phi, psi,P_A_BORG,total_Targets_Possible);
        [catching_coordinates,valid_Targets] = mapToCatchLine(coordinates,X_Catch_Line_Position,0,belt_rate,delta_T_Threshold,max_Catching_Time);
         if valid_Targets > 0 
            %% Phase 2: Generate Final Target Points
            ikTargetAngles = inverseKineTargets(catching_coordinates,endEffectorOrientation,L1,L2,L3,L4,valid_Targets);
            
            %% Phase 3: Generate a trajectories for each set of valid targets
            % Then print and set the initial position to where we were
            % last. 
            trajectories = trajGenTargets(ikTargetAngles,valid_Targets,q0,qf,tf,tv,analytics,catching_coordinates);
            printJointAngles(fileID,trajectories);
            q0 = qf;
         end
    end 
    
    % Annotate detections in the image.
    if ~isempty(bboxes)
        outImg = insertObjectAnnotation(img,'Rectangle',bboxes,curr_frame_labels);
    else
        outImg = img;
    end
    
%    ref_n_frame_labels_flatten
%    % ref_n_frame_axies_flatten
%    
%    fprintf("ref_axies_flatten:\n");
%    sizeRefAx = size(ref_n_frame_axies_flatten);
%    for i=1:(sizeRefAx(1))
%       for j=1:sizeRefAx(2)
%          fprintf("%f,%f \r\n", ref_n_frame_axies_flatten(1), ref_n_frame_axies_flatten(2));
%       end
%    end
   
    imshow(outImg);
%     title("Frame: " + z );

%% End file 
end
fclose(fileID);