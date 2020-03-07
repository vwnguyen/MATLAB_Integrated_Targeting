function [ ref_n_frame_axies ref_n_frame_labels ref_n_frame_labels_flatten ref_n_frame_axies_flatten deletionLengths ] = deleteOldReferenceFrames(deletionLengths,curr_frame_labels,n_frame,ref_n_frame_axies,ref_n_frame_labels,ref_n_frame_labels_flatten,ref_n_frame_axies_flatten)
    itemsToDelete = length(curr_frame_labels);
    deletionLengths = [deletionLengths itemsToDelete];
%     if we reached the length n, we need to make space for new incoming
%     entries. First element leaves the FIFO queue
    if ( ref_n_frame_axies == n_frame ) 
        ref_n_frame_axies = ref_n_frame_axies - 1;
        ref_n_frame_labels = ref_n_frame_labels - 1;
        for x = 1:deletionLengths(1)
            if (x <= length(ref_n_frame_labels_flatten))
                ref_n_frame_labels_flatten(x) = [];
                ref_n_frame_axies_flatten(x,:) = [];
            end
        end
        % remove length from the deletion queue
        deletionLengths(1) = [];
    end
     
end