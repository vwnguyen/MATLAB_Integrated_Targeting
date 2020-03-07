function [th] = inverseKineRBT(px,py,pz,phi,L1,L2,L3,L4)
% Solves IK problem and returns joint angle vector
% th=[th1 th2 th3 th4]
% Link Lengths
%L1=2;L2=5;L3=4;L4=3;
%L1=.2;L2=.5;L3=.4;L4=.3;
%pz=-pz+L3;

th1=atan2(py,px);
A=px-L4*cos(th1)*cos(phi);
B=py-L4*sin(th1)*cos(phi);
C=pz-L1-L4*sin(phi);
th3=real(acos( complex((A^2 + B^2 + C^2 - L2^2 - L3^2)/(2*L2*L3))) );

a=L3*sin(th3);
b=L2+L3*cos(th3);
c=pz-L1-L4*sin(phi);
r=sqrt(a^2 + b^2);
th2=atan2(c,real(sqrt(r^2-c^2)))-atan2(a,b);

th4=phi-th2-th3;
th=[th1 th2 th3 th4];
%{
gamma=atan2(pz,px);
xq=px-L4*cos(phi);
yq=py;
zq=pz--L4*sin(phi)
%}
end

