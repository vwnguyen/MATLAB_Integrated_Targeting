%% TARGET FINALIZATION
% Assumtption: Catching line is placed parallel to Y Frame, adjust code if
% needed...

% Params:
% P_A = an array of targets in the frame of the robot, formatted as
% [x y z time_spotted]
% Camera_X_Catch_Line = x coordinate at which the catching line is placed
% Camera_Y_Catch_Line = y coordinate at which the catching line is placed
% Would only use the camera X or camera Y. In this instance the catch
% line is supposed to be parallel to robot's Y reference frame. 

% Outputs: 
% catching coordinates = an array with time of arrival

function [ catching_coordinates, valid_Targets ] = mapToCatchLine(P_A,Camera_X_Catch_Line,Camera_Y_Catch_Line,belt_rate,delta_T_Threshold,max_Catching_Time)
    i = 1;
    valid_Targets = 0;
    coordSize = size(P_A);
    coordLength = length(P_A);
    catching_coordinates = zeros(coordSize);
    distanceToCatchLine = 0;
    timeToCatchLine = 0;
    
    for i=1:coordSize(1)
        distanceToCatchLine = P_A(i,1) - Camera_X_Catch_Line;
        timeToCatchLine = distanceToCatchLine / belt_rate;
        if timeToCatchLine >= delta_T_Threshold
            catching_coordinates(i,1) = Camera_X_Catch_Line;
            catching_coordinates(i,2) = P_A(i,2);
            catching_coordinates(i,3) = P_A(i,3);
            catching_coordinates(i,4) = timeToCatchLine;
          
            
        else % overwrite the row
            catching_coordinates(i,:) = zeros(1,coordLength);
            % catching_coordinates(i,:) = [];
        end
    end
    
     % eliminate rows with zeros
     catching_coordinates = catching_coordinates(any(catching_coordinates,2),:);
     % sort rows based on time
     catching_coordinates = sortrows(catching_coordinates,4);
     catchMatrixSize = size(catching_coordinates);
     
     currIndex = 2;
     tempCoordinates = zeros(catchMatrixSize);
     i = 1;
     
     % filter objects that are too close to each other. 
     if catchMatrixSize > 0
         tempCoordinates(1,:) = catching_coordinates(1,:);
         while currIndex <= catchMatrixSize(1)
             timeBetweenObjects =  abs(catching_coordinates(currIndex,4) - tempCoordinates(1,i)); 
             if timeBetweenObjects >= max_Catching_Time
                i = i + 1;
                tempCoordinates(i,:) = catching_coordinates(currIndex,:);
               
             end
             % prevIndex = prevIndex + 1;
             currIndex = currIndex + 1;
         end
     end
     
     catching_coordinates = tempCoordinates;
     catching_coordinates = catching_coordinates(any(catching_coordinates,2),:);
     catchMatrixSize = size(catching_coordinates);
     valid_Targets = (catchMatrixSize(1));
     % toc