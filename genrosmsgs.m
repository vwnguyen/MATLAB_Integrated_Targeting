% examplePackages = 'C:\Users\Visitor\Desktop\dummy'

% examplePackages = fullfile(fileparts(which('rosgenmsg')), 'examples', 'packages')

% examplePackages = fullfile('C:\Users\Visitor\Downloads\joint_angles_message_pkg');


userFolder = 'C:\ros_msgs\custom_msgs\'

folderpath = userFolder;
rosgenmsg(folderpath)