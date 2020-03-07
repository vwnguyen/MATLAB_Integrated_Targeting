function [ ref_n_frame_labels  ref_n_frame_axies ref_n_frame_axies_flatten ref_n_frame_labels_flatten] = appendReferenceFrames(ref_n_frame_labels,ref_n_frame_axies,curr_frame_labels,curr_frame_axies,ref_n_frame_axies_flatten,ref_n_frame_labels_flatten)
    ref_n_frame_labels =  ref_n_frame_labels + 1;
    ref_n_frame_axies = ref_n_frame_axies + 1;
    ref_n_frame_axies_flatten = [ ref_n_frame_axies_flatten ; curr_frame_axies ];
    ref_n_frame_labels_flatten = [ ref_n_frame_labels_flatten curr_frame_labels ]; 
end