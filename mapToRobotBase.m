  function P_A = mapToRobotBase(theta,phi,psi,xCenter,yCenter,P_A_BORG)
  % initialize variables for better C code generation
  T_A_B = [ 0 0 0 0;
      0 0 0 0;
      0 0 0 0;
      0 0 0 1;
      ];
  xPixelToInches = 0.1919; % constant subject to change, dependant on
  yPixelToInches = 0.1094; % resolution, distance of camera, etc.
  % you place the camera and its measurements
  xInInches = xCenter * xPixelToInches;
  yInInches = yCenter * yPixelToInches;
  P_B = [ xInInches; yInInches; 0; 1]; % vector from camera to target
  Rx = [ 1 0 0;
      0 cosd(theta) -sind(theta) ;
      0 sind(theta) cosd(theta) ];
  Ry = [ cosd(phi) 0 sind(phi);
      0 1 0;
      -sind(phi) 0 cosd(phi)];
  Rz = [ cosd(psi) -sind(psi) 0 ;
      sind(psi) cosd(psi) 0;
      0 0 1];
  % rotation from camera to robot
  R_A_B =  Rz * Ry * Rx;
  T_A_B(1:3,1:3) = R_A_B;
  T_A_B(1:3,4) = P_A_BORG;
  P_A = T_A_B * P_B;
  end