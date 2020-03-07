close all
clear all 
clc

% %% SETUP
if ~exist('cam','var')
    run('preload_ARbot_Script_Tracking.m');
end

% curr_label_max = -1;
%% MAIN WHILE LOOP
for  i = 1:10000
    % tic
    curr_frame_axies = [];
    curr_frame_labels = [];
    
    unique_traj_targets = [];
    filtered_bboxes = [];
    %% SENSE
    img = snapshot(cam);
    img = imresize(img,inputSize(1:2));
    % [bboxes scores] = detect(img,inputSize,overlapThreshold,YOLO_Object_Classifier);
    [bboxes,scores] = detect(YOLO_Object_Classifier.detector,img);
    % filter by selecting the strongest bounding box in the case of the
    % overlapping ones
    [bboxes,scores] = selectStrongestBbox(bboxes,scores,'OverlapThreshold',overlapThreshold);
    % coordinates = sense(img,bboxes,scores,theta, phi, psi,P_A_BORG,total_Targets_Possible,bboxAreaThreshold);
    %% PROCESS   
    xCenter = bboxes(:,1)   + (bboxes(:,3)/2) ; % note bboxes(:,1) = LHS X
    yCenter = bboxes(:,2)  + (bboxes(:,4)/2); % note bboxes(:,2) = Y from bottom
    
    A = [xCenter yCenter];
    
    %% Update Current Frames By Evaluating Current bboxes detected from image
    [ label_cnt,curr_frame_labels,curr_frame_axies,A,ref_n_frame_labels_flatten,ref_n_frame_axies_flatten ] = ...
     trackCurrentObjects(min_distance,label_cnt,curr_frame_labels,curr_frame_axies,...
     A,ref_n_frame_labels_flatten,ref_n_frame_axies_flatten);
    %% Delete Oldest Reference Frame Value
    [ ref_n_frame_axies ref_n_frame_labels ref_n_frame_labels_flatten ref_n_frame_axies_flatten deletionLengths ] = ...
    deleteOldReferenceFrames(deletionLengths,curr_frame_labels, ...
    n_frame,ref_n_frame_axies,ref_n_frame_labels,ref_n_frame_labels_flatten,ref_n_frame_axies_flatten);
     %% Append To Reference Frame Value
    [ref_n_frame_labels  ref_n_frame_axies ref_n_frame_axies_flatten ref_n_frame_labels_flatten] = ...
    appendReferenceFrames(ref_n_frame_labels,ref_n_frame_axies,curr_frame_labels,curr_frame_axies,...
    ref_n_frame_axies_flatten,ref_n_frame_labels_flatten);

    for i=1:length(curr_frame_labels)
        if (  curr_frame_labels(i) > curr_label_max )
            % We have to generate a trajectory for this target
            unique_traj_targets(i) = curr_frame_labels(i);
            curr_label_max = curr_frame_labels(i);
        end
    end
   
    % need to remove the redundant targets
    for i=1:length(unique_traj_targets) 
        filtered_bboxes(i,:) = bboxes(i,:);
    end
    
    if ~isempty(filtered_bboxes)
        P_A = sense(img,filtered_bboxes,theta, phi, psi,P_A_BORG,total_Targets_Possible,bboxAreaThreshold);
    end
    % only process the unique trajectory targets
    
    if ~isempty(bboxes)
    %% Display the Image
        img = insertObjectAnnotation(img,'rectangle',bboxes,curr_frame_labels);
    end
    imshow(img);

end



