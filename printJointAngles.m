function printJointAngles(fileID,trajectories)
        
    trajSize = size(trajectories);
    for i=1:trajSize(1)
        Joint1Angle = trajectories(i,1);
        Joint2Angle = trajectories(i,2);
        Joint3Angle = trajectories(i,3);
        Joint4Angle = trajectories(i,4);
        toA = trajectories(i,5);
        fprintf(fileID,"%f,%f,%f,%f,%f\r\n",Joint1Angle,Joint2Angle,Joint3Angle,Joint4Angle,toA);
    end
    fprintf(fileID,"STOP\r\n",Joint1Angle,Joint2Angle,Joint3Angle,Joint4Angle);
end