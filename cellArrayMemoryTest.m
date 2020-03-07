m = 1;
n = 1;
myCell = preAllocateVariabeCellArrayZeros(m,n)

% change a cell to have an entry
myCell{1} = [ 1 2 3 4; 5 6 7 8];

% append to that cell
matToAppend = [ 9 10 11 12]

tempVar = [myCell{1,1} ; matToAppend] 

% size(myCell)
% myCell
% 
% myCell2 = preAllocateVariabeCellArrayZeros(1,10)
% myCell2{end+1}= 2