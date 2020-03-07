function [ label_cnt,curr_frame_labels,curr_frame_axies,bboxes,ref_n_frame_labels_flatten,ref_n_frame_axies_flatten ] = trackCurrentObjects(min_distance,label_cnt,curr_frame_labels,curr_frame_axies,bboxes,ref_n_frame_labels_flatten,ref_n_frame_axies_flatten)
    % #codegen
    bbox_Size = size(bboxes);
    % pre allocate memory for speed
    iouArray = zeros(50,1);
    overlapThreshold = 0.6;
%     distArray = [];
%     coder.varsize('distArray', [ 50 2 ] );
    for i=1:bbox_Size(1)
        lbl = -1;
        ref_n_frame_labels_flatten_Size = size(ref_n_frame_labels_flatten);
        if ( ref_n_frame_labels_flatten_Size(2) > 0 ) 
            % distArray = ( ref_n_frame_axies_flatten - bboxes(i,:) );
            % need to compare the IoU of all current bboxes and those in 
            % the reference_n_frames
            
            % set max to the most recent to 0, then look through all
            % previous frames for an IoU match. set the greatest one 
            % accordingly
            
            overlapMax = 0;
            overlapIndex = 0;
            for z = 1:ref_n_frame_labels_flatten_Size(2)
                currOverlap = bboxOverlapRatio(bboxes(i,:),ref_n_frame_axies_flatten(z,:),'Min');
                if currOverlap > overlapMax
                   overlapMax = currOverlap;
                   overlapIndex = z;
                end
            end
            if (overlapMax > overlapThreshold)
               fprintf("Overlap Ratio Met With: %f\r\n",overlapMax);
               idx = overlapIndex; 
               lbl = ref_n_frame_labels_flatten(idx);
            else
                 fprintf("Overlap Ratio NOT Met With Unknown Target: %f\r\n",overlapMax);
            end
        end
        if (lbl == -1)
          
           lbl = label_cnt;
           label_cnt = label_cnt + 1;
        end                                                                    
        %append(curr_frame_labels,lbl);
        curr_frame_labels = [ curr_frame_labels lbl ];
        %append(curr_frame_axies,A(i,:));
        curr_frame_axies = [ curr_frame_axies ; bboxes(i,:) ] ;

    end
    
end