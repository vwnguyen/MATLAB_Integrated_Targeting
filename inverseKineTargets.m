%% inverseKineTargets(catching_coordinates)
% Desc: performs iKine on all coordinates in an array

% Params: 
% catching_coordinates: the array of coordinates after ETA estimation
% phi: the orientation of the end effector
% Outputs: 
% ikAngles: array of angles per each joint to accomplish the target
% 
function ikAngles = inverseKineTargets(catching_coordinates,phi,L1,L2,L3,L4,valid_Targets)
    i = 1;   
    % four is a constant to ensure we have x y z and time
    ikAngles = zeros(valid_Targets,4);
    for i=1:valid_Targets
            px = catching_coordinates(i,1);
            py = catching_coordinates(i,2);
            pz = catching_coordinates(i,3);
            ikAngles(i,:) = inverseKineRBT(px,py,pz,phi,L1,L2,L3,L4);
    end
end