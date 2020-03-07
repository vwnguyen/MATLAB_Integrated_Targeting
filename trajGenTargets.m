%% trajGenTargets(catching_coordinates)
% Desc: performs iKine on all coordinates in an array
% if there is more than one target the trajectory will pick up after the
% last one to pick up the target and return to target bin

% Params: 
% ikAngles: an array storing joint angles J1, J2, J3, J4, to the target
%           at time of treaching the catching line. 
% valid_Targets: the number of total valid targets determined earlier
% Outputs:
%   trajectories: an array of [ th th_dot th_ddot ], th is an array of angles
%   th_dot is the velocity and th_ddot is the acceleration.
%   q0: joint angles at starting position
%   qv: joint angles at the via point
%   qf: joint angles at final position
%   tf: duration of the trajectory (final time)
%   tv: time at which via point occurs
%   analytics: boolean true or false, if true, calculate the velocity,
%   acceleration and jerk for the given ik angle.


function trajectories = trajGenTargets(ikTargetAngles,valid_Targets,q0,qf,tf,tv,analytics,catching_coordinates)
    
    joints = 4;
    timeColumn = 4; 
    % run the traj once to get lengths and initialize properly
    % grab the first trajectory, the initial one should take tf to 1
    % second, next calls in the for loop should chain these to obtain all
    % targets
    

    
    
    [theta theta_dot theta_ddot] = traj6_v2(q0,ikTargetAngles(1,:),qf,tf,tv,analytics);
    theta_length = length(theta);
    
    % create array with valid memory for all the spots
    trajectories = zeros(theta_length*valid_Targets,joints);
    trajIndex = 1;
    for i=1:valid_Targets
        [theta theta_dot theta_ddot] = traj6_v2(q0,ikTargetAngles(1,:),qf,tf,tv,analytics);
        for j=1:length(theta)
            trajectories(trajIndex,1:4) = theta(j,:);
            % fill the last column with the time of arrival of that target
            trajectories(trajIndex,5) = catching_coordinates(i,4);
            trajIndex = trajIndex + 1;
        end
    end
end