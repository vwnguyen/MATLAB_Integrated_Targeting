close all
clear all 
clc

% %% SETUP
if ~exist('cam','var')
    run('preload_ARbot_Script_v1.m');
end
%% MAIN WHILE LOOP
ref_frame_axies = [];
ref_frame_labels = [];
label_cnt = 1;
min_distance = 30;

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
    A_Size = size(A);
    for i=1:A_Size(1)
        lbl = 'NaN';
        if ( length(ref_frame_labels) > 0 ) 
            % distance = norm( ref_frame_axies - A , 1 ); % take axis = 1 max norm
            
            % compute norms for THE REFERENCE TO CURRENT
            %for i=1:length(A)
                % distArray = ( ref_frame_axies(i,:) - A );
                distArray = ( ref_frame_axies - A(i,:) );
            %end
            
%             if ( length(A) > length(ref_frame_axies) ) 
%                 distArray = ( ref_frame_axies - A(i,:) );
%             else
%                 distArray = ( ref_frame_axies(i,:) - A );
%             end
            
            dist_Size = size(distArray);
            for j=1:dist_Size(1)
                normArray(j) = norm(distArray(j));
            end
            
            min_value = min(normArray);
                    
            if (min_value < min_distance)
                idx = find( normArray == min_value );
                lbl = ref_frame_labels(idx);
            end
            
        end
        if (lbl == 'NaN')
           lbl = label_cnt;
           label_cnt = label_cnt + 1;
        end

        %append(curr_frame_labels,lbl);
        curr_frame_labels = [ curr_frame_labels lbl ];
        %append(curr_frame_axies,A(i,:));
        curr_frame_axies = [ curr_frame_axies ; A(i,:) ] ;

    end
    
    if ~isempty(bboxes)
        img = insertObjectAnnotation(img,'rectangle',bboxes,curr_frame_labels);
    end
    imshow(img);
    
    ref_frame_labels = curr_frame_labels;
    ref_frame_axies = curr_frame_axies;
    normArray = [];
    
    % toc
end



