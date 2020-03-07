close all
clear all 
clc

% %% SETUP
if ~exist('cam','var')
    run('preload_ARbot_Script_v1.m');
end
%% MAIN WHILE LOOP
for  i = 1:10000
    % tic
    %% SENSE
    img = snapshot(cam);
    img = imresize(img,inputSize(1:2));
    % [bboxes scores] = detect(img,inputSize,overlapThreshold,YOLO_Object_Classifier);
    [bboxes,scores] = detect(YOLO_Object_Classifier.detector,img);
    % filter by selecting the strongest bounding box in the case of the
    % overlapping ones
    [bboxes,scores] = selectStrongestBbox(bboxes,scores,'OverlapThreshold',overlapThreshold);
    coordinates = sense(img,bboxes,scores,theta, phi, psi,P_A_BORG,total_Targets_Possible,bboxAreaThreshold);
    %% PROCESS   
    
    if length(bboxes) > 0 
        img = insertObjectAnnotation(img,'rectangle',bboxes,scores);
        % Phase 0: Extract vector from robot base to the target(s)
        
       
        % Phase 1: Map the coordinates to their final coordinates and times 
        % along the cathcing line.
        % disp("final coordinates and catching time\n")
        [catching_coordinates,valid_Targets] = mapToCatchLine(coordinates,X_Catch_Line_Position,0,belt_rate,delta_T_Threshold,max_Catching_Time);
        % catching_coordinates
        
        
        
        
        % Phase 2: extract the joint angles needed to get to the target at the
        % estimated time.
        % after mapping it into the final catching coordinates, we can now feed
        % this into the IK to display the joint angles needed to get there.
        % uses closed loop algebraic solution.
        if valid_Targets > 0 
            ikTargetAngles = inverseKineTargets(catching_coordinates,endEffectorOrientation,L1,L2,L3,L4,valid_Targets);
        
        % Phase 3: Generate a trajectories for each set of valid targets
        % [theta theta_dot theta_ddot] = traj6_v2(q0,qv,qf,tf,tv,analytics)
            trajectories = trajGenTargets(ikTargetAngles,valid_Targets,q0,qf,tf,tv,analytics);
            
            %% CONTROL
            for i=1:length(trajectories)
%                 myMsg.Joint1Angle = rad2deg(trajectories(i,1));
%                 myMsg.Joint2Angle = rad2deg(trajectories(i,2));
%                 myMsg.Joint3Angle = rad2deg(trajectories(i,3));
%                 myMsg.Joint4Angle = rad2deg(trajectories(i,4));
%                 % Control the O-Drive by Publishing
%                 send(myPub,myMsg);
            end
            
%             if (valid_Targets >= 2) && (display_Simulation == true)
%                 visualize(robot,trajectories,dt);
%             end
            % update current position, save the last valid target
            
            
            q0 = qf;
        end
    
    end 
    imshow(img);
    % toc
end



