close all
clear all 
clc

% %% SETUP
if ~exist('cam','var')
    run('preload_ARbot_Script_v1.m');
end
%% MAIN WHILE LOOP

n_frame = 8;
label_cnt = 1;
min_distance = 30;

% pre allocate memory
ref_n_frame_axies = {};
ref_n_frame_labels = {};
ref_n_frame_axies_flatten = [];
ref_n_frame_labels_flatten = [];
deletionLengths = [];

for  i = 1:10000
    % tic
    curr_frame_axies = [];
    curr_frame_labels = [];
    %% SENSE
    img = snapshot(cam);
    img = imresize(img,inputSize(1:2));
    % [bboxes scores] = detect(img,inputSize,overlapThreshold,YOLO_Object_Classifier);
    [bboxes,scores] = detect(YOLO_Object_Classifier.detector,img);
    % filter by selecting the strongest bounding box in the case of the
    % overlapping ones
    [bboxes,scores] = selectStrongestBbox(bboxes,scores,'OverlapThreshold',overlapThreshold);
    xCenter = bboxes(:,1)   + (bboxes(:,3)/2) ; % note bboxes(:,1) = LHS X
    yCenter = bboxes(:,2)  + (bboxes(:,4)/2); % note bboxes(:,2) = Y from bottom
    
    A = [xCenter yCenter];
    % A_Size = size(A);
    
    %% track objects
    
     [ label_cnt,curr_frame_labels,curr_frame_axies,A,ref_n_frame_labels_flatten,ref_n_frame_axies_flatten ] = ...
     trackCurrentObjects(min_distance,label_cnt,curr_frame_labels,curr_frame_axies,...
     A,ref_n_frame_labels_flatten,ref_n_frame_axies_flatten);
    
    
%     for i=1:A_Size(1)
%         lbl = 'NaN';
%         ref_n_frame_labels_flatten_Size = size(ref_n_frame_labels_flatten);
%         if ( ref_n_frame_labels_flatten_Size(1) > 0 ) 
%             distArray = ( ref_n_frame_axies_flatten - A(i,:) );
%             dist_Size = size(distArray);
%             for j=1:dist_Size(1)
%                 normArray(j) = norm(distArray(j));
%             end
%             min_value = min(normArray);
%             if (min_value < min_distance)
%                 idx = find( normArray == min_value );
%                 % if there happen to be multiple matches, just choose the
%                 % first one
%                 if (length(idx) > 1)
%                    idx = idx(1); 
%                 end
%                 lbl = ref_n_frame_labels_flatten(idx);
%             end
%         end
%         if (lbl == 'NaN')
%            lbl = label_cnt;
%            label_cnt = label_cnt + 1;
%         end                                                                    
%         %append(curr_frame_labels,lbl);
%         curr_frame_labels = [ curr_frame_labels lbl ];
%         %append(curr_frame_axies,A(i,:));
%         curr_frame_axies = [ curr_frame_axies ; A(i,:) ] ;
% 
%     end
    
     %% DELETE %% pop off old reference labels 
    [ ref_n_frame_axies ref_n_frame_labels ref_n_frame_labels_flatten ref_n_frame_axies_flatten deletionLengths ] = ...
    deleteOldReferenceFrames(deletionLengths,curr_frame_labels, ...
    n_frame,ref_n_frame_axies,ref_n_frame_labels,ref_n_frame_labels_flatten,ref_n_frame_axies_flatten);

    %% APPEND %% append to the reference labels 
    % if deletion was needed we now have room to append other wise we were
    % going to append to the reference labels and axies until we reach
    % n_frame
    [ref_n_frame_labels  ref_n_frame_axies ref_n_frame_axies_flatten ref_n_frame_labels_flatten] = ...
    appendReferenceFrames(ref_n_frame_labels,ref_n_frame_axies,curr_frame_labels,curr_frame_axies,...
    ref_n_frame_axies_flatten,ref_n_frame_labels_flatten);
    
    if ~isempty(bboxes)
        img = insertObjectAnnotation(img,'rectangle',bboxes,curr_frame_labels);
    end
    imshow(img);
    normArray = [];
    % toc
end



