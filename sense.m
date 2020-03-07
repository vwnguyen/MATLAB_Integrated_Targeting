%% sense() takes in the arguments as shown below and outputs a cell array 
% of n x total_Targets_Possible array that contains the X, Y, Z, and time 
% spotted 

function P_A = sense(img,bboxes,theta, phi, psi,P_A_BORG,total_Targets_Possible)

    P_A = zeros(total_Targets_Possible,4);
    bboxSize = size(bboxes);
    for j = 1:bboxSize(1);
        P_A(j,:) = mapToRobotBase(theta,phi,psi,bboxes(j,1),bboxes(j,1),P_A_BORG);

    end
end

    