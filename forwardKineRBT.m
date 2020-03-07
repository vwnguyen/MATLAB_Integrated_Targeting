function [pos] = forwardKineRBT(th1, th2, th3, th4)
%forwardKine returns the EE position after computing the forward kinematics
%with the joint angles provided th1 th2 th3 th4
%   Detailed explanation goes here
% Link Lengths
L1=.2;L2=.5;L3=.4;L4=.3;
%L1=2;L2=5;L3=4;L4=3;
% forward kinematic tranformation matrix from WristFrame{4} to BaseFrame{0}

T04=[ cos(th2 + th3 + th4)*cos(th1), -sin(th2 + th3 + th4)*cos(th1),  sin(th1), cos(th1)*(L3*cos(th2 + th3) + L2*cos(th2) + L4*cos(th2 + th3 + th4));
      cos(th2 + th3 + th4)*sin(th1), -sin(th2 + th3 + th4)*sin(th1), -cos(th1), sin(th1)*(L3*cos(th2 + th3) + L2*cos(th2) + L4*cos(th2 + th3 + th4));
          sin(th2 + th3 + th4),           cos(th2 + th3 + th4),         0,       L1 + L3*sin(th2 + th3) + L2*sin(th2) + L4*sin(th2 + th3 + th4);
                             0,                              0,         0,                                                                    1];  
% pos=[px py pz]
pos=T04(1:3,4);%+[0;0;.3];
%{
T=@(th,a,d,alph)[ cos(th) -sin(th)*cos(alph)  sin(th)*sin(alph) a*cos(th);
    sin(th)  cos(th)*cos(alph) -cos(th)*sin(alph) a*sin(th);
      0         sin(alph)           cos(alph)         d;
      0            0                   0              1];
T05=T04*T(0,0,L4,0);
pos=T05(1:3,4);
%}
%pos(3)=pos(3)+.1;
%{
T04=[ cos(th2 + th3 + th4)*cos(th1), -sin(th2 + th3 + th4)*cos(th1),  sin(th1), cos(th1)*(L2 + L4*cos(th2 + th3) + L3*cos(th2));
 cos(th2 + th3 + th4)*sin(th1), -sin(th2 + th3 + th4)*sin(th1), -cos(th1), sin(th1)*(L2 + L4*cos(th2 + th3) + L3*cos(th2));
          sin(th2 + th3 + th4),           cos(th2 + th3 + th4),         0,            L1 + L4*sin(th2 + th3) + L3*sin(th2);
                             0,                              0,         0,                                               1];
pos=T04(1:3,4);
%}
end

