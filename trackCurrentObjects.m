function [ label_cnt,curr_frame_labels,curr_frame_axies,A,ref_n_frame_labels_flatten,ref_n_frame_axies_flatten ] = trackCurrentObjects(min_distance,label_cnt,curr_frame_labels,curr_frame_axies,A,ref_n_frame_labels_flatten,ref_n_frame_axies_flatten)
    % #codegen
    A_Size = size(A);
    % pre allocate memory for speed
    normArray = Inf(50,1);
%     distArray = [];
%     coder.varsize('distArray', [ 50 2 ] );
    for i=1:A_Size(1)
        lbl = -1;
        ref_n_frame_labels_flatten_Size = size(ref_n_frame_labels_flatten);
        if ( ref_n_frame_labels_flatten_Size(2) > 0 ) 
            distArray = ( ref_n_frame_axies_flatten - A(i,:) );
            dist_Size = size(distArray);
            for j=1:dist_Size(1)
                normArray(j) = norm(distArray(j));
            end
            min_value = min(normArray);
            if (min_value < min_distance)
                idx = find( normArray == min_value );
                % if there happen to be multiple matches, just choose the
                % first one
                %if (length(idx) > 1)
                   idx = idx(1); 
                %end
                lbl = ref_n_frame_labels_flatten(idx);
            end
        end
        if (lbl == -1)
           lbl = label_cnt;
           label_cnt = label_cnt + 1;
        end                                                                    
        %append(curr_frame_labels,lbl);
        curr_frame_labels = [ curr_frame_labels lbl ];
        %append(curr_frame_axies,A(i,:));
        curr_frame_axies = [ curr_frame_axies ; A(i,:) ] ;

    end



end