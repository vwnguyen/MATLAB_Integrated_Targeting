function [theta theta_dot theta_ddot] = traj6_v2(q0,qv,qf,tf,tv,analytics)
%traj6 generates a trajectory with a via point. The trajectory will
%complete in the time specified. The arm will pass through the via point at
%the specified time. If tv is unspecified it will default to tf/2. The
%trajectory uses a six degree polynomial to calculate the path. The
%trajectory gaurantees continuous jerk thus reducing mechanical wear.
%   q0: joint angles at starting position
%   qv: joint angles at the via point
%   qf: joint angles at final position
%   tf: duration of the trajectory (final time)
%   tv: time at which via point occurs
%   analytics: boolean true or false, if true, calculate the velocity,
%   acceleration and jerk for the given ik angle.

%% Adjust time step resolution for short time intervals
if(tf<=1)
    dt=.05;
else
    dt=0.1;
end
t=0:tv/12:tf;

%% Path for joint 1
a10=q0(1);
a13 = ( 1/(tf-tv)^3 ) * ( ( ((qv(1) - q0(1))*tf^3) / tv^3 ) - ( tv*( (qf(1)-q0(1)) * (15*tf^2 - 24*tf*tv + 10*tv^2) ) / tf^3 ) );
a14 = ( 3/(tf-tv)^3 ) * ( -1*( ((qv(1) - q0(1))*tf^2) / tv^3 ) + ( ( (qf(1)-q0(1)) * (5*tf^3 - 9*tf*tv^2 + 5*tv^3) ) / tf^4 ) );
a15 = ( 3/(tf-tv)^3 ) * ( ( ((qv(1) - q0(1))*tf) / tv^3 ) - ( ( (qf(1)-q0(1)) * (8*tf^3 - 9*tf^2*tv + 2*tv^3) ) / tf^5 ) );
a16 = -( 1/(tf-tv)^3 ) * ( ( (qv(1) - q0(1)) / tv^3 ) - ( ( (qf(1)-q0(1)) * (10*tf^2 - 15*tf*tv + 6*tv^2) ) / tf^5 ) );
th1=@(t) a16*t^6 + a15*t^5 + a14*t^4 + a13*t^3 + a10;        % position
th1_d =@(t) 6*a16*t^5 + 5*a15*t^4 + 4*a14*t^3 + 3*a13*t^2;   % velocity 
th1_dd =@(t) 30*a16*t^4 + 20*a15*t^3 + 12*a14*t^2 + 6*a13*t; % acceleration
th1_ddd =@(t) 120*a16*t^3 + 60*a15*t^2 + 24*a14*t;           % jerk 

%% Path for joint 2
a20=q0(2);
a23 = ( 1/(tf-tv)^3 ) * ( ( ((qv(2) - q0(2))*tf^3) / tv^3 ) - ( tv*( (qf(2)-q0(2)) * (15*tf^2 - 24*tf*tv + 10*tv^2) ) / tf^3 ) );
a24 = ( 3/(tf-tv)^3 ) * ( -1*( ((qv(2) - q0(2))*tf^2) / tv^3 ) + ( ( (qf(2)-q0(2)) * (5*tf^3 - 9*tf*tv^2 + 5*tv^3) ) / tf^4 ) );
a25 = ( 3/(tf-tv)^3 ) * ( ( ((qv(2) - q0(2))*tf) / tv^3 ) - ( ( (qf(2)-q0(2)) * (8*tf^3 - 9*tf^2*tv + 2*tv^3) ) / tf^5 ) );
a26 = -( 1/(tf-tv)^3 ) * ( ( (qv(2) - q0(2)) / tv^3 ) - ( ( (qf(2)-q0(2)) * (10*tf^2 - 15*tf*tv + 6*tv^2) ) / tf^5 ) );
th2=@(t) a26*t^6 + a25*t^5 + a24*t^4 + a23*t^3 + a20;
th2_d =@(t) 6*a26*t^5 + 5*a25*t^4 + 4*a24*t^3 + 3*a23*t^2;   % velocity 
th2_dd =@(t) 30*a26*t^4 + 20*a25*t^3 + 12*a24*t^2 + 6*a23*t; % acceleration
th2_ddd =@(t) 120*a26*t^3 + 60*a25*t^2 + 24*a24*t;           % jerk 

%% Path for joint 3
a30=q0(3);
a33 = ( 1/(tf-tv)^3 ) * ( ( ((qv(3) - q0(3))*tf^3) / tv^3 ) - ( tv*( (qf(3)-q0(3)) * (15*tf^2 - 24*tf*tv + 10*tv^2) ) / tf^3 ) );
a34 = ( 3/(tf-tv)^3 ) * ( -1*( ((qv(3) - q0(3))*tf^2) / tv^3 ) + ( ( (qf(3)-q0(3)) * (5*tf^3 - 9*tf*tv^2 + 5*tv^3) ) / tf^4 ) );
a35 = ( 3/(tf-tv)^3 ) * ( ( ((qv(3) - q0(3))*tf) / tv^3 ) - ( ( (qf(3)-q0(3)) * (8*tf^3 - 9*tf^2*tv + 2*tv^3) ) / tf^5 ) );
a36 = -( 1/(tf-tv)^3 ) * ( ( (qv(3) - q0(3)) / tv^3 ) - ( ( (qf(3)-q0(3)) * (10*tf^2 - 15*tf*tv + 6*tv^2) ) / tf^5 ) );
th3=@(t) a36*t^6 + a35*t^5 + a34*t^4 + a33*t^3 + a30;
th3_d =@(t) 6*a36*t^5 + 5*a35*t^4 + 4*a34*t^3 + 3*a33*t^2;   % velocity 
th3_dd =@(t) 30*a36*t^4 + 20*a35*t^3 + 12*a34*t^2 + 6*a33*t; % acceleration
th3_ddd =@(t) 120*a36*t^3 + 60*a35*t^2 + 24*a34*t;           % jerk

%% Path for joint 4
a40=q0(4);
a43 = ( 1/(tf-tv)^3 ) * ( ( ((qv(4) - q0(4))*tf^3) / tv^3 ) - ( tv*( (qf(4)-q0(4)) * (15*tf^2 - 24*tf*tv + 10*tv^2) ) / tf^3 ) );
a44 = ( 3/(tf-tv)^3 ) * ( -1*( ((qv(4) - q0(4))*tf^2) / tv^3 ) + ( ( (qf(4)-q0(4)) * (5*tf^3 - 9*tf*tv^2 + 5*tv^3) ) / tf^4 ) );
a45 = ( 3/(tf-tv)^3 ) * ( ( ((qv(4) - q0(4))*tf) / tv^3 ) - ( ( (qf(4)-q0(4)) * (8*tf^3 - 9*tf^2*tv + 2*tv^3) ) / tf^5 ) );
a46 = -( 1/(tf-tv)^3 ) * ( ( (qv(4) - q0(4)) / tv^3 ) - ( ( (qf(4)-q0(4)) * (10*tf^2 - 15*tf*tv + 6*tv^2) ) / tf^5 ) );
th4=@(t) a46*t^6 + a45*t^5 + a44*t^4 + a43*t^3 + a40;
th4_d =@(t) 6*a46*t^5 + 5*a45*t^4 + 4*a44*t^3 + 3*a43*t^2;   % velocity 
th4_dd =@(t) 30*a46*t^4 + 20*a45*t^3 + 12*a44*t^2 + 6*a43*t; % acceleration
th4_ddd =@(t) 120*a46*t^3 + 60*a45*t^2 + 24*a44*t;           % jerk

%% Compute the joint path polynomials
n=length(t);
th=zeros(n,4);%pre allocate
th_d=zeros(n,4);%pre allocate
th_dd=zeros(n,4);%pre allocate
if analytics == true
    for i=1:n
        %th is the trajectory
        th(i,:)=[th1(t(i)) th2(t(i)) th3(t(i)) th4(t(i))];
%         th_d(i,:)   = [th1_d(t(i)) th2_d(t(i)) th3_d(t(i)) th4_d(t(i))];
%         th_dd(i,:)  = [th1_dd(t(i)) th2_dd(t(i)) th3_dd(t(i)) th4_dd(t(i))];
%         th_ddd(i,:) = [th1_ddd(t(i)) th2_ddd(t(i)) th3_ddd(t(i)) th4_ddd(t(i))];
    end
else
    for i=1:n
        %th is the trajectory
        th(i,:)=[th1(t(i)) th2(t(i)) th3(t(i)) th4(t(i))];
        %th_d(i,:)   = [th1_d(t(i)) th2_d(t(i)) th3_d(t(i)) th4_d(t(i))];
        %th_dd(i,:)  = [th1_dd(t(i)) th2_dd(t(i)) th3_dd(t(i)) th4_dd(t(i))];
        %th_ddd(i,:) = [th1_ddd(t(i)) th2_ddd(t(i)) th3_ddd(t(i)) th4_ddd(t(i))];
    end
end
theta=th;
theta_dot=th_d;
theta_ddot=th_dd;
end














