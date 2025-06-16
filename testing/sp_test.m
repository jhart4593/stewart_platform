% stewart platform code
clear all;
clc;


% constants based on my build
s = 10.824; %length of operating leg (in)
a = 2; %length of servo operating arm (in)

% coordinates of servo motor rotation point in base frame
b1 = [-2.43 -4.84 0]';
b2 = [2.43 -4.84 0]';
b3 = [5.03 -.33 0]';
b4 = [2.6 3.876 0]';
b5 = [-2.6 3.876 0]';
b6 = [-5.03 -.33 0]';
b = [b1 b2 b3 b4 b5 b6];

% coordinates of the home position of platform joints, platform frame
p01 = [-2.1875 -2.49 0]';
p02 = [2.1875 -2.49 0]';
p03 = [3.0625 -.974 0]';
p04 = [.875 2.815 0]';
p05 = [-.875 2.815 0]';
p06 = [-3.0625 -.974 0]';
p0 = [p01 p02 p03 p04 p05 p06];

% angle of servo arm plane relative to x-axis
beta1 = deg2rad(0);
beta2 = deg2rad(180);
beta3 = deg2rad(120);
beta4 = deg2rad(300);
beta5 = deg2rad(240);
beta6 = deg2rad(60);
beta = [beta1 beta2 beta3 beta4 beta5 beta6];

% plot(b(2,:),-b(1,:))
% hold on
% plot(p(2,:),-p(1,:))
% hold off

% calculate h_0 and alpha_0 - home locations of platform height and servo
% arm angle
h_01 = sqrt(s^2+a^2-(p01(1)-b1(1))^2-(p01(2)-b1(2))^2)-p01(3);
h_03 = sqrt(s^2+a^2-(p03(1)-b3(1))^2-(p03(2)-b3(2))^2)-p03(3);
h_05 = sqrt(s^2+a^2-(p05(1)-b5(1))^2-(p05(2)-b5(2))^2)-p05(3);
h_0 = (h_01+h_03+h_05)/3;

l_01 = sqrt((p01(1)-b1(1))^2+(p01(2)-b1(2))^2+(h_0+p01(3))^2);
L01 = 2*a^2;
M01 = 2*a*(cos(beta1)*(p01(1)-b1(1))+sin(beta1)*(p01(2)-b1(2)));
N01 = 2*a*(h_0+p01(3));
alpha_01 = asin(L01/sqrt(M01^2+N01^2)) - atan2(M01,N01);

l_03 = sqrt((p03(1)-b3(1))^2+(p03(2)-b3(2))^2+(h_0+p03(3))^2);
L03 = 2*a^2;
M03 = 2*a*(cos(beta3)*(p03(1)-b3(1))+sin(beta3)*(p03(2)-b3(2)));
N03 = 2*a*(h_0+p03(3));
alpha_03 = asin(L03/sqrt(M03^2+N03^2)) - atan2(M03,N03);

l_05 = sqrt((p05(1)-b5(1))^2+(p05(2)-b5(2))^2+(h_0+p05(3))^2);
L05 = 2*a^2;
M05 = 2*a*(cos(beta5)*(p05(1)-b5(1))+sin(beta5)*(p05(2)-b5(2)));
N05 = 2*a*(h_0+p05(3));
alpha_05 = asin(L05/sqrt(M05^2+N05^2)) - atan2(M05,N05);

l_02 = sqrt((p02(1)-b2(1))^2+(p02(2)-b2(2))^2+(h_0+p02(3))^2);
L02 = 2*a^2;
M02 = 2*a*(p02(1)-b2(1));
N02 = 2*a*(h_0+p02(3));
alpha_02 = asin(L02/sqrt(M02^2+N02^2)) - atan2(M02,N02);

alpha_0 = (alpha_01 + alpha_03 + alpha_05)/3;
alpha_0d = rad2deg(alpha_0);

% calculate home position of platform joints in base frame
q1 = [0 0 h_0]' + p01;
q2 = [0 0 h_0]' + p02;
q3 = [0 0 h_0]' + p03;
q4 = [0 0 h_0]' + p04;
q5 = [0 0 h_0]' + p05;
q6 = [0 0 h_0]' + p06;

% position and orientation of platform origin (in platform coords)
%desired
pos_d = [0.5 0.5 0]';
or_d = [.5 0 0]';

%current 
pos = [0 0 0]';
or = [0 0 0]';

%delta
pos_del = pos_d - pos;
or_del = or_d - or;

% calculate rotational matrix for platform relative to the base
phi = or_d(1);
theta = or_d(2);
psi = or_d(3);

Rz = [cos(psi) -sin(psi) 0;sin(psi) cos(psi) 0;0 0 1];
Ry = [cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
Rx = [1 0 0;0 cos(phi) -sin(phi);0 sin(phi) cos(phi)];
R_PB = Rz*Ry*Rx;

% calculate effective leg lengths 
T = [pos_d(1) pos_d(2) pos_d(3)+h_0]'; %translation vector of platform frame wrt base frame

l1 = T+R_PB*p01-b1;
l2 = T+R_PB*p02-b2;
l3 = T+R_PB*p03-b3;
l4 = T+R_PB*p04-b4;
l5 = T+R_PB*p05-b5;
l6 = T+R_PB*p06-b6;
l = [l1 l2 l3 l4 l5 l6];

% calculate angles required for each servo
alpha = zeros(6,1);

% uses p coordinates as in equation 9 of 'maths' paper
% for idx = 1:6
%     ll = sqrt(l(1,idx)^2+l(2,idx)^2+l(3,idx)^2);
%     LL = ll^2 - (s^2 - a^2);
%     MM = 2*a*(p0(3,idx)-b(3,idx));
%     NN = 2*a*(cos(beta(idx))*(p0(1,idx)-b(1,idx))+sin(beta(idx))*(p0(2,idx)-b(2,idx)));
%     al = asin(LL/sqrt(MM^2+NN^2)) - atan2(NN,MM);
%     alpha(idx) = al;
% end

% uses q coordinates as mentioned by instructables post by thiagohersan
plat_jts = zeros(3,6);
for idx = 1:6
    q = T + R_PB * p0(:,idx);
    plat_jts(:,idx) = q;
    ll = sqrt(l(1,idx)^2+l(2,idx)^2+l(3,idx)^2);
    LL = ll^2 - (s^2 - a^2);
    MM = 2*a*(q(3)-b(3,idx));
    NN = 2*a*(cos(beta(idx))*(q(1)-b(1,idx))+sin(beta(idx))*(q(2)-b(2,idx)));
    al = asin(LL/sqrt(MM^2+NN^2)) - atan2(NN,MM);
    alpha(idx) = al;
end

%% Sim Test
x_a = a*cos(alpha).*cos(beta') + b(1,:)';
y_a = a*cos(alpha).*sin(beta') + b(2,:)';
z_a = a*sin(alpha);

base_pts = [b,b(:,1)];
plat = [plat_jts,plat_jts(:,1)];

plot3(base_pts(1,:),base_pts(2,:),zeros(7,1))
hold on
plot3(plat(1,:),plat(2,:),plat(3,:))
hold on

for ii = 1:6
    plot3([b(1,ii) x_a(ii)],[b(2,ii) y_a(ii)],[0 z_a(ii)],'g')
    hold on
    plot3([x_a(ii) plat(1,ii)],[y_a(ii) plat(2,ii)],[z_a(ii) plat(3,ii)],'m')
    hold on
end

grid on
xlabel("X")
ylabel("Y")
zlabel("Z")



%% Test simulation
% given desired pose of platform over time, output animation of its
% movement
clear all;
clc;

t = 100;
ball = [0 0];

[s,a,b,p0,beta,h_0,alpha_0] = initialize();

h = figure(1);
h.Visible = 'off';
for mm = 1:t
    idx = mm-1;
    ang = 2*pi/t * idx;
    rot = -pi/6 + (2*pi/6 -2*pi/6*(abs(t/2-idx)/(t/2)));
    z_move = (1.5-(1.5*abs(t/2-idx)/(t/2)));
    pose_des = [6*cos(ang),6*sin(ang),0,0,0,0]; %only translation
    % pose_des = [0,0,0,rot,0]; %only rotation
    % pose_des = [cos(ang),sin(ang),0,0,rot,0]; %combined
    % pose_des = [0,0,z_move,0,0,0]; %only vertical
    [alpha_cmd,plat_jts,ball_loc] = inv_kin(pose_des,h_0,p0,b,s,a,beta,ball,alpha_0);
    plot_sp(alpha_cmd,beta,b,plat_jts,a,ball_loc);
    drawnow
    F(mm) = getframe;
end

h.Visible = 'on';
movie(F,2,30)

%%
% Test Rotation for T
clear;
clc;

T_0 = [0 0 10]';
phi = pi()/6;
theta = pi()/6;
psi = 0;

Rz = [cos(psi) -sin(psi) 0;sin(psi) cos(psi) 0;0 0 1];
Ry = [cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
Rx = [1 0 0;0 cos(phi) -sin(phi);0 sin(phi) cos(phi)];
R_PB = Rz*Ry*Rx;

T_new = R_PB*T_0

norm(T_new)

%% Controller simulation
% NEED TO ADD SAFETIES IF BALL IS OFF THE PLATFORM OR IF BALL IS NOT
% DETECTED ON THE PLATFORM FOR PLATFORM TO MOVE TO HOME POS

% issue with ball plotting is that location of ball at previous time is
% being calculated at next time step - so not accurate

clear all;
clc;

origin = [144 108];
ball_pos_prev = [300 60];
ball_pos = [280 50];
err_prev = [0 0];
err_prev(1) = origin(1) - ball_pos_prev(1);
err_prev(2) = ball_pos_prev(2) - origin(2);
dt = 100;



[s,a,b,p0,beta,h_0,alpha_0] = initialize();
[cmd,err_prev] = PD(origin,ball_pos,err_prev,dt);
pose_des = [cmd(1),cmd(2),0,cmd(3),cmd(4),0]; 
[alpha_cmd,plat_jts,ball_loc] = inv_kin(pose_des,h_0,p0,b,s,a,beta,ball_pos,alpha_0);
plot_sp(alpha_cmd,beta,b,plat_jts,a,ball_loc);

function [cmd, err_prev] = PD(origin,ball_pos,err_prev,dt)
    pos_max = 0.5;
    or_max = pi/4;

    kp_p = 0.0006; kd_p = 0.56;
    kp_o = .0013; kd_o = 1.1;
    err = [0 0];
    err(1) = origin(1) - ball_pos(1);
    err(2) = ball_pos(2) - origin(2);
    d_err = (err-err_prev)/dt;

    cmd_p = kp_p*err + kd_p*d_err;
    cmd_o = kp_o*err + kd_o*d_err;

    %cmd = [cmd_p -cmd_o(1) cmd_o(2)];
    cmd = [cmd_p 0 0];

    if abs(cmd(1)) > pos_max
        cmd(1) = pos_max*sign(cmd(1));
    end

    if abs(cmd(2)) > pos_max
        cmd(2) = pos_max*sign(cmd(2));
    end

    if abs(cmd(3)) > or_max
        cmd(3) = or_max*sign(cmd(3));
    end

    if abs(cmd(4)) > 2
        cmd(4) = or_max*sign(cmd(4));
    end

    err_prev = err;
end

%% Code Consolidation into Functions
% function to calculate all base values
function [s,a,b,p0,beta,h_0,alpha_0] = initialize()
    % constants based on my build
    s = 10.824; %length of operating leg (in)
    a = 2; %length of servo operating arm (in)
    
    % coordinates of servo motor rotation point in base frame
    b1 = [-2.43 -4.84 0]';
    b2 = [2.43 -4.84 0]';
    b3 = [5.03 -.33 0]';
    b4 = [2.6 3.876 0]';
    b5 = [-2.6 3.876 0]';
    b6 = [-5.03 -.33 0]';
    b = [b1 b2 b3 b4 b5 b6];
    
    % coordinates of the home position of platform joints, platform frame
    p01 = [-2.1875 -2.49 0]';
    p02 = [2.1875 -2.49 0]';
    p03 = [3.0625 -.974 0]';
    p04 = [.875 2.815 0]';
    p05 = [-.875 2.815 0]';
    p06 = [-3.0625 -.974 0]';
    p0 = [p01 p02 p03 p04 p05 p06];
    
    % angle of servo arm plane relative to x-axis
    beta1 = deg2rad(0);
    beta2 = deg2rad(180);
    beta3 = deg2rad(120);
    beta4 = deg2rad(300);
    beta5 = deg2rad(240);
    beta6 = deg2rad(60);
    beta = [beta1 beta2 beta3 beta4 beta5 beta6];

    % calculate h_0 and alpha_0 - home locations of platform height and servo
    % arm angle
    h_01 = sqrt(s^2+a^2-(p01(1)-b1(1))^2-(p01(2)-b1(2))^2)-p01(3);
    h_03 = sqrt(s^2+a^2-(p03(1)-b3(1))^2-(p03(2)-b3(2))^2)-p03(3);
    h_05 = sqrt(s^2+a^2-(p05(1)-b5(1))^2-(p05(2)-b5(2))^2)-p05(3);
    h_0 = (h_01+h_03+h_05)/3;
    
    L01 = 2*a^2;
    M01 = 2*a*(cos(beta1)*(p01(1)-b1(1))+sin(beta1)*(p01(2)-b1(2)));
    N01 = 2*a*(h_0+p01(3));
    alpha_01 = asin(L01/sqrt(M01^2+N01^2)) - atan2(M01,N01);
    
    L03 = 2*a^2;
    M03 = 2*a*(cos(beta3)*(p03(1)-b3(1))+sin(beta3)*(p03(2)-b3(2)));
    N03 = 2*a*(h_0+p03(3));
    alpha_03 = asin(L03/sqrt(M03^2+N03^2)) - atan2(M03,N03);
    
    L05 = 2*a^2;
    M05 = 2*a*(cos(beta5)*(p05(1)-b5(1))+sin(beta5)*(p05(2)-b5(2)));
    N05 = 2*a*(h_0+p05(3));
    alpha_05 = asin(L05/sqrt(M05^2+N05^2)) - atan2(M05,N05);
    
    alpha_0 = (alpha_01 + alpha_03 + alpha_05)/3;

end

% function to calculate alpha angles of arms based on desired pose as well
% as platform joints when arms are at these angles
function [alpha_cmd,plat_jts,ball_loc] = inv_kin(pose_des,h_0,p0,b,s,a,beta,ball,alpha_0)
    % max and min alpha values
    al_max = alpha_0 + pi/4;
    al_min = alpha_0 - pi/4;
    % calculate rotational matrix for platform relative to the base
    phi = pose_des(4);
    theta = pose_des(5);
    psi = pose_des(6);
    
    Rz = [cos(psi) -sin(psi) 0;sin(psi) cos(psi) 0;0 0 1];
    Ry = [cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
    Rx = [1 0 0;0 cos(phi) -sin(phi);0 sin(phi) cos(phi)];
    R_PB = Rz*Ry*Rx;
    
    % calculate effective leg lengths 
    T = [pose_des(1) pose_des(2) pose_des(3)+h_0]'; %translation vector of platform frame wrt base frame
    
    l1 = T+R_PB*p0(:,1)-b(:,1);
    l2 = T+R_PB*p0(:,2)-b(:,2);
    l3 = T+R_PB*p0(:,3)-b(:,3);
    l4 = T+R_PB*p0(:,4)-b(:,4);
    l5 = T+R_PB*p0(:,5)-b(:,5);
    l6 = T+R_PB*p0(:,6)-b(:,6);
    l = [l1 l2 l3 l4 l5 l6];

    ball_loc = T+R_PB*([ball 0]');
    
    % calculate angles required for each servo
    alpha = zeros(6,1);
    plat_jts = zeros(3,6);

    for idx = 1:6
        q = T + R_PB * p0(:,idx);
        plat_jts(:,idx) = q;
        ll = sqrt(l(1,idx)^2+l(2,idx)^2+l(3,idx)^2);
        LL = ll^2 - (s^2 - a^2);
        MM = 2*a*(q(3)-b(3,idx));
        NN = 2*a*(cos(beta(idx))*(q(1)-b(1,idx))+sin(beta(idx))*(q(2)-b(2,idx)));
        al = asin(LL/sqrt(MM^2+NN^2)) - atan2(NN,MM);
        if al > al_max
            al = al_max;
        end
        if al < al_min
            al = al_min;
        end
        alpha(idx) = al;
    end

    alpha_cmd = alpha;
end

% function to plot stewart platform
function [] = plot_sp(alpha,beta,b,plat_jts,a,ball)
    x_a = a*cos(alpha).*cos(beta') + b(1,:)';
    y_a = a*cos(alpha).*sin(beta') + b(2,:)';
    z_a = a*sin(alpha);
    
    base_pts = [b,b(:,1)];
    plat = [plat_jts,plat_jts(:,1)];
    
    plot3(base_pts(1,:),base_pts(2,:),zeros(7,1))
    hold on
    plot3(plat(1,:),plat(2,:),plat(3,:))
    hold on
    plot3(ball(1),ball(2),ball(3),'ro')
    hold on
    
    for ii = 1:6
        plot3([b(1,ii) x_a(ii)],[b(2,ii) y_a(ii)],[0 z_a(ii)],'g')
        hold on
        plot3([x_a(ii) plat(1,ii)],[y_a(ii) plat(2,ii)],[z_a(ii) plat(3,ii)],'m')
        hold on
    end
    
    grid on
    xlabel("X")
    ylabel("Y")
    zlabel("Z")
    xlim([-5 5]);
    ylim([-5 5]);
    zlim([0 15]);
    hold off

end


