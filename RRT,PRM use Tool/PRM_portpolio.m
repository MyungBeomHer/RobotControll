clf;
clear all;
close all;

global Segments;
Segments = 50;
%q_init = [-2, -10, 0, deg2rad(0), deg2rad(0), deg2rad(0)];
%q_goal = [2, 10, 0, deg2rad(0), deg2rad(0), deg2rad(0)];
q_init = [-2, 0, 0, deg2rad(0), deg2rad(0), deg2rad(0)];
q_goal = [3, 12, 10, deg2rad(0), deg2rad(0), deg2rad(0)];

robot = Robot1();
init_robot = Robot1();
global obss;
obss=Map3();

Kinematics(init_robot, robot, q_init);
Simulate(robot, obss, false)
Draw_Init_Goal(init_robot, robot, q_init, q_goal)

pause()

%%sampling
%1은 장애물 0은 not exist, 2는 exist 
nodes = 0; %지금까지 sampling 된 갯수
Node = [];
%connect_node = [];
range = 1;

nodes = nodes + 1;
Node = [Node;q_init]; %Node에다가 집어넣음->configuration space의 노드
%connect_node = [connect_node,inf];

%flag = 0; %goal_not_found == 0, goal_found = 1
%%cSpace -> not_exist == 0, obstacle == 1,exist == 2 

%%sampling
global X
X = 9;
global Y
Y = 20;
global Z
Z = 9;
x_inter = 5;
y_inter = 5;
z_inter = 5;

subplot(1,4,4);
hold on
%xlim([-4.5 4.5]);
%ylim([-10 10]);
xlim([-5 15]);
ylim([-4 15]);
title("6-dof robot phase space")    
xlabel("x");
ylabel("y");

%X_ran = 6;
%Y_ran = 20;
%Z_ran = 6;

X_ran = 20;
Y_ran = 19;
Z_ran = 15;

sample = 100
for i = 1:x_inter
    for j = 1:y_inter
        for k = 1:z_inter
            n = 0;
            while n <= sample/(x_inter * y_inter * z_inter) 
                %x = X/x_inter * (i-1) + (-X/x_inter * (i-1) + X/x_inter * i) * rand() - 4.5;
                %y = Y/y_inter * (j-1) + (-Y/y_inter * (j-1) + Y/y_inter * j) * rand() - 10;
                %z = Z/z_inter * (k-1) + (-Z/z_inter * (k-1) + Z/z_inter * k) * rand() - 4.5;
                
                %q_sample(1) = -5 + (10-(-5)) * rand();
                %q_sample(2) = -4 + (15-(-4)) * rand();
                %q_sample(3) = -3 + (12-(-3)) * rand();
                x = X_ran/x_inter * (i-1) + ((X_ran/x_inter * i) - X_ran/x_inter * (i-1)) * rand() -5;
                y = Y_ran/y_inter * (j-1) + ((Y_ran/y_inter * j) - Y_ran/y_inter * (j-1)) * rand() -4;
                z = Z_ran/z_inter *(k -1) + ((Z_ran/z_inter * k) - Z_ran/z_inter * (k -1)) * rand() -3;
                q1 = -90 + (90 -(-90)) * rand();
                q2 = -90 + (90 -(-90)) * rand();
                q3 = -90 + (90 -(-90)) * rand();
                q1 = deg2rad(q1);
                q2 = deg2rad(q2);
                q3 = deg2rad(q3);
                Q = [x,y,z,q1,q2,q3];
                Kinematics(init_robot, robot, Q);
                if Collision_check_robot(robot) ~= 0
                    continue
                end
                Node = [Node;x,y,z,q1,q2,q3];
                nodes = nodes + 1;
                n = n + 1;
                subplot(1,4,4)
                hold on 
                plot(x,y,'r*');
                pause(0.0001)
            end
        end
    end
end
%%Expansion
Node = [Node;q_goal];
nodes = nodes + 1;

s = [];
t = [];
weights = [];

G = zeros(nodes,nodes); %connect node 
for i = 1:nodes 
    for j = i + 1:nodes
        if i == j 
            continue
        end
        dis = vecnorm(Node(i,:) - Node(j,:));
        if dis > range
            continue
        end

        if  Collision_check(robot,init_robot,Node(i,:),Node(j,:)) == 1
              continue
        end
        G(i,j) = dis;
        G(j,i) = dis;
        
        %{
        s = [s,i];
        t = [t,j];
        s = [s,j];
        t = [t,i];
        weights = [weights,dis];
        weights = [weights,dis];
        %}
        Cspace_plot(Node(i,1),Node(i,2),Node(j,1),Node(j,2));
    end
end

[cost,Roadmap] = dijkstra(G,1,nodes);
%Graph = graph(s,t,weights);
%[P,d] = shortestpath(Graph,1,nodes);
%Roadmap = P;
Roadmap = fliplr(Roadmap) %Roadmap은 Node의 번호임

for i = 1:numel(Roadmap) - 1
    parent = Roadmap(i)
    child = Roadmap(i + 1)
    subplot(1,4,4)
    hold on
    line([Node(parent,1),Node(child,1)],[Node(parent,2),Node(child,2)],'Color','green','Linewidth',2)
    pause(0.00000001)
end
pause()
segments = 50;
%% roadmap을 따라서 이동 
for i = 1:numel(Roadmap) - 1
    child_index = Roadmap(i);
    child_q = Node(child_index,:);

    j = Roadmap(i + 1);
    parent_q = Node(j,:);
    vector = parent_q - child_q;
    for k = 0:segments
        q = child_q + k * vector/segments;
        Kinematics(init_robot, robot, q)
        
        %만약을 대비해서 
        if Collision_check_robot(robot) ~= 0
                    break
        end

        Simulate(robot, obss, false)
        pause(0.000000001)
    end
end


%%Function
%%collision여부 두 점 사이에 collision 여부 
function check = Collision_check(robot,init_robot,Q1,Q2)
    check = 0;
    global Segments
    V = (Q2 - Q1)/Segments; %vector 
    for i = 0:Segments 
        update_q = Q1 + V * i;
        Kinematics(init_robot,robot,update_q);
        if Collision_check_robot(robot) ~=  0
            check = 1;
            break
        end
    end
end
%현재 configuration 상에 collision 여부
function check = Collision_check_robot(robot)
    check = 0;
    global obss
    for i = 1:length(obss)
        isCol = checkCollision(robot,obss(i)); 
        if  isCol == true
            check = 1;
            break
        end
    end
end

%%homogeneous transform
function Kinematics(init_robot, robot, q)
    H01 = TXYZ([q(1), q(2), q(3)]) * ROLL(q(4)) * PITCH(q(5)) * YAW(q(6));
    robot.Pose = H01 * init_robot.Pose;
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

%%draw
function Draw_Init_Goal(init_robot, robot, q_init, q_goal)
    Kinematics(init_robot, robot, q_init)
    [~, q_init_robot] = show(robot);
    q_init_robot.FaceColor = [0 1 0];
    q_init_robot.FaceAlpha = 1;
    
    hold on;

    Kinematics(init_robot, robot, q_goal)
    [~, q_goal_robot] = show(robot);
    q_goal_robot.FaceColor = [1 0 0];
    q_goal_robot.FaceAlpha = 1;
end

function bool = Cspace_plot(xHat,yHat,x,y)
    subplot(1,4,4) %122
    hold on 
    title("6-dof robot phase space")    
    xlabel("x");
    ylabel("y");
    plot(x,y,'r*')
    line([xHat,x],[yHat,y])
    pause(0.00000000001)
end
%%
function Simulate(robot, obss, collView)
    fig=gcf;
    fig.Position(1:4) = [100, 100, 1800, 600];

    if collView == true
        SimulRobot(robot);
        hold on
%         view(-78, 8)
        view(2)
        
        xlim([-20 20])
        ylim([-30 30])
        zlim([-20 20])
        SimulObss(obss);

    else
        subplot(1,4,1)
        SimulRobot(robot);
        hold on
        view(-90, 0)
        
        xlim([-20 20])
        ylim([-30 30])
        zlim([-20 20])
        SimulObss(obss);
    
        subplot(1,4,2)
        SimulRobot(robot);
        hold on
        view(-90, 90)
        
        xlim([-20 20])
        ylim([-30 30])
        zlim([-20 20])
        SimulObss(obss);
    
        subplot(1,4,3)
        SimulRobot(robot);
        hold on
        view(3)
    
        xlim([-20 20])
        ylim([-30 30])
        zlim([-20 20])
        SimulObss(obss);
    end

end
%{
function Simulate(robot, obss, collView)
    fig=gcf;
    fig.Position(1:4) = [200, 200, 1800, 600];

    if collView == true
        SimulRobot(robot);
        hold on
%         view(-78, 8)
        view(2)
        
        xlim([-4.5 4.5])
        ylim([-10 10])
        zlim([-4.5 4.5])
        SimulObss(obss);
    else
         subplot(1,3,1)
        SimulRobot(robot);
        hold on
        view(-90, 0)
        
        xlim([-20 20])
        ylim([-30 30])
        zlim([-20 20])
        SimulObss(obss);
    
        subplot(1,3,2)
        SimulRobot(robot);
        hold on
        view(-90, 90)
        
        xlim([-20 20])
        ylim([-30 30])
        zlim([-20 20])
        SimulObss(obss);
    
        subplot(1,3,3)
        SimulRobot(robot);
        hold on
        view(3)
    
        xlim([-20 20])
        ylim([-30 30])
        zlim([-20 20])
        SimulObss(obss);
    end
end
%}
function SimulRobot(robot)
    [~, q_robot] = show(robot);
    q_robot.FaceColor = [255/255, 192/255, 203/255];
    q_robot.FaceAlpha = 0.8;
    q_robot.EdgeColor = 'none';
end

function SimulObss(obss)
    for i=1:length(obss)
        [~, obs_paint] = show(obss(i));
        obs_paint.FaceAlpha = 0.1;
        obs_paint.FaceColor = [255/255, 255/255, 0/255];
        obs_paint.EdgeColor = 'none';
    end
end

%%dijkstra
function [e L] = dijkstra(A,s,d)
if s==d
    e=0;
    L=[s];
else
A = setupgraph(A,inf,1);
if d==1
    d=s;
end
A=exchangenode(A,1,s);
lengthA=size(A,1);
W=zeros(lengthA);
for i=2 : lengthA
    W(1,i)=i;
    W(2,i)=A(1,i);
end
for i=1 : lengthA
    D(i,1)=A(1,i);
    D(i,2)=i;
end
D2=D(2:length(D),:);
L=2;
while L<=(size(W,1)-1)
    L=L+1;
    D2=sortrows(D2,1);
    k=D2(1,2);
    W(L,1)=k;
    D2(1,:)=[];
    for i=1 : size(D2,1)
        if D(D2(i,2),1)>(D(k,1)+A(k,D2(i,2)))
            D(D2(i,2),1) = D(k,1)+A(k,D2(i,2));
            D2(i,1) = D(D2(i,2),1);
        end
    end
    
    for i=2 : length(A)
        W(L,i)=D(i,1);
    end
end
if d==s
    L=[1];
else
    L=[d];
end
e=W(size(W,1),d);
L = listdijkstra(L,W,s,d);
end
end

function G = exchangenode(G,a,b)
%Exchange element at column a with element at column b;
buffer=G(:,a);
G(:,a)=G(:,b);
G(:,b)=buffer;
%Exchange element at row a with element at row b;
buffer=G(a,:);
G(a,:)=G(b,:);
G(b,:)=buffer;
end

function L = listdijkstra(L,W,s,d)
index=size(W,1);
while index>0
    if W(2,d)==W(size(W,1),d)
        L=[L s];
        index=0;
    else
        index2=size(W,1);
        while index2>0
            if W(index2,d)<W(index2-1,d)
                L=[L W(index2,1)];
                L=listdijkstra(L,W,s,W(index2,1));
                index2=0;
            else
                index2=index2-1;
            end
            index=0;
        end
    end
end
end

function G = setupgraph(G,b,s)
if s==1
    for i=1 : size(G,1)
        for j=1 :size(G,1)
            if G(i,j)==0
                G(i,j)=b;
            end
        end
    end
end
if s==2
    for i=1 : size(G,1)
        for j=1 : size(G,1)
            if G(i,j)==b
                G(i,j)=0;
            end
        end
    end
end
end
