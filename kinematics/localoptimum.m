clear all; clc; close all;

%% MAIN

% Hyper Parameters
epsilon = 20;
zeta = 0.01;

delta = 15;
eta = 3;
end_condition = [0.1, 0.1];

% q
goal_q = [95, 50];

obstacles = [40, 60;
            50, 50;
            60, 40
            ];

% Kinematics
H = ROLL(0);

%% U_att, U_rep MeshGrid


xlabel('X')
ylabel('Y')
%zlabel('z')

xy_lb = 0;
xy_ub = 100;

[X, Y] = meshgrid(linspace(xy_lb, xy_ub, 101), linspace(xy_lb, xy_ub, 101));
U_att_mesh = zeros(size(X));
U_rep_mesh = zeros(size(X));


p_goal_q = FromQ_ToP(H, goal_q);

for i=1:(length(X))
    for j=1:(length(X))
        q = [X(i, j), Y(i, j)];

        p_q = FromQ_ToP(H, q);

        u_att = U_att(p_goal_q, p_q, epsilon, zeta)';
        u_rep = U_rep(obstacles, p_q, delta, eta)';

        U_Q = u_att;
        for jdx = 1:length(u_rep)
            U_Q = U_Q + u_rep(jdx);
        end

        if U_Q > 20
            U_Q = 20;
        end
    
        U_att_mesh(i, j) = u_att;
        U_rep_mesh(i, j) = U_Q;
        
    end
end

% fig 3
figure(3)
fig=gcf;

fig.Position(1:4) = [400, 000, 400, 400];

axis([0 100 0 100 0 20])
surf(X, Y, U_att_mesh);

% fig 1
figure(1)
fig=gcf;

fig.Position(1:4) = [800, 000, 400, 400];

axis([0 100 0 100 0 20])
surf(X, Y, U_rep_mesh);

%% Update 

init_q = [7, 50];
p_init_q = FromQ_ToP(H, init_q);

q = init_q;

while true
    p_q = FromQ_ToP(H, q);
    p_init_q = FromQ_ToP(H, init_q);
    p_goal_q = FromQ_ToP(H, goal_q);

    PlotGraph(p_init_q, p_goal_q, p_q, obstacles);
    
    % F_att(q), F_rep(p(q))
    f_att = F_att(p_goal_q, p_q, epsilon, zeta)';
    f_rep = F_rep(obstacles, p_q, delta, eta)';

    % F_Q
    F_Q = J(q)' * f_att;
    for jdx = 1:length(obstacles)
         F_Q = F_Q + J(q)' * f_rep(:, jdx);
    end

    % update q
    q = q + F_Q';

    % end condition
    if abs(goal_q - q) < end_condition
        break
    end

    pause(0.01)
end

%% Functions
function Rroll = ROLL(roll)
    Rroll = [
    cos(roll), -sin(roll), 0;
    sin(roll), cos(roll), 0;
    0, 0, 1;];
end

function p = FromQ_ToP(H, q)
    q = [q, 1];
    p = H * q';
end

function u_att = U_att(p_goal_q, p_q, epsilon, zeta)
    u_att = [0, 0]';
    diff_pq = p_goal_q(1:2)' - p_q(1:2)';
    dist_pq = sqrt(diff_pq(1)^2 + diff_pq(2)^2) + 0.0000001;

    if dist_pq > epsilon
        u_att = (epsilon*zeta*dist_pq - 0.5*zeta*epsilon^2);
    else
        u_att = 0.5 * zeta * dist_pq ^ 2;
    end

end

function u_rep = U_rep(obstacles, p_q, delta, eta)
    u_rep = [];

    for o_idx = 1:length(obstacles)
        o_q = p_q(1:2)' - obstacles(o_idx, :);
        dist_o_q = sqrt(o_q(1)^2 + o_q(2)^2) + 0.0000001;

        if dist_o_q < delta
            u_rep(o_idx) = eta * 0.5 * (1/dist_o_q - 1/delta)^2;
        else
            u_rep(o_idx) = 0;
        end
    end

end

function f_att = F_att(p_goal_q, p_q, epsilon, zeta)
    diff_pq = (p_q(1:2)' - p_goal_q(1:2)');
    dist_pq = sqrt(diff_pq(1)^2 + diff_pq(2)^2) + 0.0000001;

    if dist_pq > epsilon
        f_att = - (epsilon*zeta*diff_pq)/dist_pq;
    else
        f_att = -zeta * diff_pq;
    end

end

function f_rep = F_rep(obstacles, p_q, delta, eta)
    f_rep = zeros(size(obstacles));

    for o_idx = 1:length(obstacles)
        o_q = p_q(1:2)' - obstacles(o_idx, :);
        dist_o_q = sqrt(o_q(1)^2 + o_q(2)^2) + 0.0000001;

        if dist_o_q < delta
            f_rep(o_idx, :) = eta * (1/dist_o_q - 1/delta) * (1 /(pow2(dist_o_q))) * (o_q / dist_o_q);
        else
            f_rep(o_idx, :) = 0;
        end
    end
end

function J_q = J(q)
    J_q=[1, 0;
        0, 1];
end

function PlotGraph(p_init_q, p_goal_q, p_q, obstacles)
    figure(2)    

    fig=gcf;

    fig.Position(1:4) = [800, 600, 400, 400];

    xlabel('X')
    ylabel('Y')
    grid on;
    
    axis([0, 100, 0, 100])

    hold off

    plot(p_q(1), p_q(2), 'ko', MarkerSize=5)
    hold on
    plot(p_init_q(1), p_init_q(2), 'ro', MarkerSize=5)
    plot(p_goal_q(1), p_goal_q(2), 'bo', MarkerSize=5)

    plot(obstacles(:, 1), obstacles(:, 2), 'go', MarkerSize=10)

    pause(0.01)
    
end
