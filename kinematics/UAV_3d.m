clear all; clc; close all;

epsilon = 0.5;
zeta = 0.05;

delta = 3;
eta = 0.5;
end_condition = [0.1 0.1 0 0 0 3];

obstacles = [6, 3, 0;
            6, 3.2, 0;
            6, 3.5, 0;
            6, 3.7, 0;
            6, 3.9, 0;
            6, 4.2, 0
            ];

%% Main 
l1 = 3;
l2 = 0.5;

global UAV;
UAV = [0, 0, 0; %무게중심
        l1, 0, 0; %앞쪽 정점 
        0, l2, 0; %왼쪽 정점 
        0, -l2, 0; %오른쪽 정점
        ];
%initial point
P = [0, 0, 0;
    0, 0, 0;
    0, 0, 0;
    ];

% q = [x, y, z, yaw, pitch, roll];
init_q = [2, 2, 0, 0, 0, 0];
goal_q = [8, 8, 0, 0.5, 0.3, 0.5];
q = init_q;

% p(q)
p_init_q = FromQ_ToP(init_q);

%pause()

p_goal_q = FromQ_ToP(goal_q);

while true
    p_q = FromQ_ToP(q); %호모지니어스를 이용한 이동 

    PlotGraph(p_init_q, p_goal_q, p_q, obstacles);
    
    % F_att(q), F_rep(p(q)) 각점에 대해서 attribute랑 repulsive를 해준다. 
    f_att = [F_att(p_goal_q(1, :), p_q(1, :), epsilon, zeta)'];
    f_att = [f_att; F_att(p_goal_q(2, :), p_q(2, :), epsilon, zeta)'];
    f_att = [f_att; F_att(p_goal_q(3, :), p_q(3, :), epsilon, zeta)'];
    f_att = [f_att; F_att(p_goal_q(4, :), p_q(4, :), epsilon, zeta)'];

    f_rep = F_rep(obstacles, p_q(1, :), delta, eta);
    f_rep = [f_rep;F_rep(obstacles, p_q(2, :), delta, eta)];
    f_rep = [f_rep;F_rep(obstacles, p_q(3, :), delta, eta)];  
    f_rep = [f_rep;F_rep(obstacles, p_q(4, :), delta, eta)];  

    % F_Q
    F_Q = [0, 0, 0, 0, 0, 0]';
    F_Q = F_Q + J1(q)' * f_att(1,:)'; %translate 
    F_Q = F_Q + J2(q)' * f_att(2,:)'; %rotate 
    F_Q = F_Q + J2(q)' * f_att(3,:)'; %rotate 
    F_Q = F_Q + J2(q)' * f_att(4,:)'; %rotate 

    for jdx = 0:length(obstacles)-1
        F_Q = F_Q + J1(q)' * f_rep(jdx * 4 + 1, :)'; %jacobian에 현재 position을 넣네 -> 즉, workspace를 미분한거에 현재지점을 넣게 되면 속도가 나오게 된다. ->jacobian은 workspace인 거리를 미분하여 속도로 나타냄 -> 근데 회전 할때 속도가 다르기 때문에 필요한거네
        F_Q = F_Q + J2(q)' * f_rep(jdx * 4 + 2, :)';
        F_Q = F_Q + J2(q)' * f_rep(jdx * 4 + 3, :)';
        F_Q = F_Q + J2(q)' * f_rep(jdx * 4 + 4, :)';
    end

    % update q 원래 q에대가 attractive function' * jacobian' + jacobian' * repulsive function' 를 update만큼 곱해줘서 더해준다.  
    q = q + 1.2 * F_Q'

    % end condition
    if abs(goal_q - q) < end_condition
        break
    end
   
   %pause(0.01)
   %break
end


%% Functions
function f_rep = F_rep(obstacles, p_q, delta, eta)
    f_rep = zeros(size(obstacles));

    for o_idx = 1:length(obstacles)
        o_q = p_q(1:3) - obstacles(o_idx, :);
        dist_o_q = sqrt(o_q(1)^2 + o_q(2)^2 + o_q(3)^2) + 0.0000001;

        if dist_o_q < delta
            f_rep(o_idx, :) = eta * (1/dist_o_q - 1/delta) * (1 /(dist_o_q^2)) * (o_q / dist_o_q);
        else
            f_rep(o_idx, :) = 0;
        end
    end
end

function Ryaw = YAW(yaw)
    Ryaw = [
    1, 0, 0, 0;
    0, cos(yaw), -sin(yaw), 0;
    0, sin(yaw), cos(yaw), 0;
    0, 0, 0, 1];
end

function Rpitch = PITCH(pitch)
    Rpitch = [
    cos(pitch), 0, sin(pitch), 0;
    0, 1, 0, 0;
    -sin(pitch), 0, cos(pitch), 0;
    0, 0, 0, 1];
end

function Rroll = ROLL(roll)
    Rroll = [
    cos(roll), -sin(roll), 0, 0;
    sin(roll), cos(roll), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];
end

function Txyz = TXYZ(txyz)
    tx = txyz(1);
    ty = txyz(2);
    tz = txyz(3);

    Txyz = [
    1, 0, 0, tx;
    0, 1, 0, ty;
    0, 0, 1, tz;
    0, 0, 0, 1];
end

%상대좌표계
function p = FromQ_ToP(q)
    global UAV
    H01 = TXYZ([q(1), q(2), q(3)]);  %무게중심은 앞으로만 나아가고 
    H12 = ROLL(q(6)) * PITCH(q(5)) * YAW(q(4)); %다른 부분들은 돌기만 한다. 

    p = zeros(length(UAV),4);
    p1 = (H01 * [UAV(1, :), 1].')';
    p2 = (H12 * [UAV(2, :), 1].')';
    p3 = (H12 * [UAV(3, :), 1].')';
    p4 = (H12 * [UAV(4, :), 1].')';

    p(1, :) = p1;
    p(2, :) = p(1, :) + p2;
    p(3, :) = p(1, :) + p3;
    p(4, :) = p(1, :) + p4;

    p(:, 4) = ones([length(UAV), 1]);    
end

function f_att = F_att(p_goal_q, p_q, epsilon, zeta)
    diff_pq = (p_q(1:3)' - p_goal_q(1:3)');
    dist_pq = sqrt(diff_pq(1)^2 + diff_pq(2)^2 + diff_pq(3)^2) + 0.0000001;

    if dist_pq > epsilon
        f_att = -(epsilon*zeta*diff_pq)/dist_pq;
    else
        f_att = -zeta * diff_pq;
    end
end

function J_q = J1(q)
    J_q=[1 0 0 0 0 0;
        0 1 0 0 0 0;
        0 0 1 0 0 0];
end

%기준 좌표계 pre-mulitply
function J_q = J2(q)
    J_q=[cos(q(6))*cos(q(5)), cos(q(6))*sin(q(5))*sin(q(4))-sin(q(6))*cos(q(4)), cos(q(6))*sin(q(5))*cos(q(4))+sin(q(6))*sin(q(4)), q(2)*cos(q(6))*sin(q(5))*cos(q(4))+q(2)*sin(q(6))*sin(q(4))-q(3)*cos(q(6))*sin(q(5))*sin(q(4))+q(3)*sin(q(6))*cos(q(4)), -q(1)*cos(q(6))*sin(q(5))+q(2)*cos(q(6))*cos(q(5))*sin(q(4))+q(3)*cos(q(6))*cos(q(5))*cos(q(4)), -q(1)*sin(q(6))*cos(q(5))-q(2)*sin(q(6))*sin(q(5))*sin(q(4))-q(2)*cos(q(6))*cos(q(4))-q(3)*sin(q(6))*sin(q(5))*cos(q(4))+q(3)*cos(q(6))*sin(q(4));
    sin(q(6))*cos(q(5)), sin(q(6))*sin(q(5))*sin(q(4))+cos(q(6))*cos(q(4)), sin(q(6))*sin(q(5))*cos(q(4))-cos(q(6))*sin(q(4)), q(2)*sin(q(6))*sin(q(5))*cos(q(4))-q(2)*cos(q(6))*sin(q(4))-q(3)*sin(q(6))*sin(q(5))*sin(q(4))-q(3)*cos(q(6))*cos(q(4)), -q(1)*sin(q(6))*sin(q(5))+q(2)*sin(q(6))*cos(q(5))*sin(q(4))+q(3)*sin(q(6))*cos(q(5))*cos(q(4)), q(1)*cos(q(6))*cos(q(5))+q(2)*cos(q(6))*sin(q(5))*sin(q(4))-q(2)*sin(q(6))*cos(q(4))+q(3)*cos(q(6))*sin(q(5))*cos(q(4))+q(3)*sin(q(6))*sin(q(4));
    -sin(q(5)), cos(q(5))*sin(q(4)), cos(q(5))*cos(q(4)), q(2)*cos(q(5))*cos(q(4))-q(3)*cos(q(5))*sin(q(4)), -q(1)*cos(q(5))-q(2)*sin(q(5))*sin(q(4))+q(3)*sin(q(5))*cos(q(4)), 0];
end

function PlotGraph(p_init_q, p_goal_q, p_q, obstacles)
    % UAV
    Px = p_q(:, 1);
    Py = p_q(:, 2);
    Pz = p_q(:, 3);
    Pc = p_q(:, 4);

    Px=Px(2:4);
    Py=Py(2:4);
    Pz=Pz(2:4);
    Pc=Pc(2:4);

    % Init Points
    P_in_x = p_init_q(:, 1);
    P_in_y = p_init_q(:, 2);
    P_in_z = p_init_q(:, 3);

    P_in_x = P_in_x(2:4);
    P_in_y = P_in_y(2:4);
    P_in_z = P_in_z(2:4);

    % Goal Points
    P_go_x = p_goal_q(:, 1);
    P_go_y = p_goal_q(:, 2);
    P_go_z = p_goal_q(:, 3);

    P_go_x = P_go_x(2:4);
    P_go_y = P_go_y(2:4);
    P_go_z = P_go_z(2:4);

    subplot(2, 2, [1])
    hold off
    grid on;

    fill3(Px, Py, Pz, Pc)
    hold on
    fill3(P_in_x, P_in_y, P_in_z, 'blue')
    fill3(P_go_x, P_go_y, P_go_z, 'red')

    plot3(obstacles(:, 1), obstacles(:, 2), obstacles(:, 3), 'go', MarkerSize=10)

    fig=gcf;
    fig.Position(1:4) = [100, 400, 600, 600];
    axis([0, 12, 0 ,12, -6, 6])

    xlabel('X')
    ylabel('Y')
    zlabel('Z')

    % x z
    subplot(2, 2, [2])
    hold off
    grid on;

    fill(Px, Pz, Pc)
    hold on
    fill(P_in_x, P_in_z, 'blue')
    fill(P_go_x, P_go_z, 'red')
    plot(obstacles(:, 1), obstacles(:, 3), 'go', MarkerSize=10)
    axis([0, 12, -6, 6])

    xlabel('X')
    ylabel('Z')

    % y z
    subplot(2, 2, [3])
    hold off
    grid on;

    fill(Py, Pz, Pc)
    hold on
    fill(P_in_y, P_in_z, 'blue')
    fill(P_go_y, P_go_z, 'red')
    plot(obstacles(:, 2), obstacles(:, 3), 'go', MarkerSize=10)
    axis([0 ,12, -6, 6])
    xlabel('Y')
    ylabel('Z')

    % x y
    subplot(2, 2, [4])
    hold off
    grid on;

    fill(Px, Py, Pc)
    hold on
    fill(P_in_x, P_in_y, 'blue')
    fill(P_go_x, P_go_y, 'red')
    plot(obstacles(:, 1), obstacles(:, 2), 'go', MarkerSize=10)
    xlabel('X')
    ylabel('Y')
    axis([0, 12, 0 ,12])

    pause(0.01)
end


