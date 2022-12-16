clear all; clc; close all;

%% MAIN
l1 = 2;
l2 = 1;
l3 = 1;
init_P = [0, 0, 0, 1;
        l1, 0, 0, 1;
        l2, 0, 0, 1;
        l3, 0, 0, 1;
        ];

P = [0, 0, 0, 1;
    0, 0, 0, 1;
    0, 0, 0, 1;
    0, 0, 0, 1;
    ];


% Roll(z - based), Pitch(y - based), Yaw(x - based)
p0_txyz=[0, 0, 0];

p1_rpy=[0, 0, 0];
p1_txyz=[0, 0, 0];

p2_rpy=[0, 0, 0];
p2_txyz=[0, 0, 0];

p3_rpy=[0, 0, 0];
p3_txyz=[0, 0, 0];

% history
hist_t = [];
hist_p1_rpy = [];
hist_p2_rpy = [];
hist_p3_rpy = [];

% main
for t = 0:500
    % update input rpy, txyz
    p0_txyz(1) = p0_txyz(1) + 0.01;

    p1_rpy(1) = mod(p1_rpy(1) + deg2rad(1), deg2rad(360))
    p2_rpy(1) = mod(p2_rpy(1) + deg2rad(3), deg2rad(360));
    p3_rpy(1) = mod(p3_rpy(1) + deg2rad(5), deg2rad(360));

    % update kinematics
    P(1, :) = (TXYZ(p0_txyz) * init_P(1, :)')';
    P(2, :) = P(1, :) + (ROLL(p1_rpy(1)) * init_P(2, :)')';
    P(3, :) = P(2, :) + (ROLL(p1_rpy(1)+p2_rpy(1)) * init_P(3, :)')';
    P(4, :) = P(3, :) + (ROLL(p1_rpy(1)+p2_rpy(1)+p3_rpy(1)) * init_P(4, :)')';

    % update history
    hist_t = [hist_t; t];
    hist_p1_rpy = [hist_p1_rpy; p1_rpy];
    hist_p2_rpy = [hist_p2_rpy; p2_rpy];
    hist_p3_rpy = [hist_p3_rpy; p3_rpy];
    
    % Plot point
    PlotGraph(hist_t, P, hist_p1_rpy, hist_p2_rpy,hist_p3_rpy)
    pause(0)

end

%% Functions

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

function PlotGraph(t, P, p1_rpy, p2_rpy,p3_rpy)
    fig=gcf;

    % x, y, width, height
    fig.Position(1:4) = [400, 000, 400, 800];


    % 1번 그림
    subplot(4, 2, [1,2,3,4])

    plot3(P(:, 1), P(:, 2), P(:, 3), LineWidth=3)
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    
    grid on;
    view([0 0 90])
    axis([-5, 5, -5, 5, -5, 5])
    
    
    % 2번 그림
    subplot(4, 2, [5, 6])

    plot(t, p1_rpy(:, 1), 'ro')
    axis([0, 500, -10, 10])
    grid on;

    % 3번 그림
    subplot(4, 2, [7, 8])

    plot(t, p2_rpy(:, 1), 'ko')
    axis([0, 500, -10, 10])
    grid on;

    plot(t, p3_rpy(:, 1), 'g')
    axis([0, 500, -10, 10])
    grid on;

    
end