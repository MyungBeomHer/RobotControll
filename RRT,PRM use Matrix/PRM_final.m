clf;
clear all;
close all;


%%workspace 
global l1;
global l2;
global l3;
global l4;
global X;
global Y;
X = 100;
Y = 100;
l1 = 4;
l2 = 4;
l3 = 4;
l4 = 4;
%각 점들의 초기위치 
init_wSpace = [10,10;
    l1,0;
    l2,0;
    l3,0;
    l4,0;
    ];

epsilon = 0.25;
zeta = 0.05;

init_q = [10,10,0.3,0.3,0.3,0.3];
goal_q = [65,80,0.2,0.2,0.2,0.2]; %theta 같은 경우 1당 1/18 * 2 *pi 이라고 생각 


q = init_q;
p_init_q = FromQ_ToP(init_q);
p_goal_q = FromQ_ToP(goal_q);
%%
obstacle = false(X,Y);
[x,y] = meshgrid(1:X,1:Y);

obstacle(y<=1 | y>=100 | x<=1 | x>=100) = true;

%obstacle(x > 30 & x<60 & y> 30 & y <60) = true;
obstacle(x > 35 & x<40 & y > 0 & y <30) = true;
obstacle(x > 35 & x<40 & y > 60 & y <100) = true;
obstacle(x > 45 & x<50 & y > 0 & y <40) = true;
obstacle(x > 45 & x<50 & y > 70 & y <100) = true;
obstacle(x > 55 & x<60 & y > 0 & y <40) = true;
obstacle(x > 55 & x<60 & y > 80 & y <100) = true;

global wSpace;
wSpace = zeros(size(obstacle));
wSpace(obstacle) = 1;


%%
hf = subplot(1,2,1);
imshow(~obstacle);
%axis xy; %행 열을 x축 y축 처럼 변경 
%axis on;
xlabel('y');
ylabel('x');
title('workspace');
hold on 
plot(p_init_q(:, 1), p_init_q(:, 2), LineWidth=3)
plot(p_goal_q(:, 1), p_goal_q(:, 2), LineWidth=3)
grid on
%%

%%configuration space
Theta1 = 18;
Theta2 = 18; %2pi를 18개로 나눴기 때문에 (인덱스 * 18 * 2 *pi) 해줘야된다.
Theta3 = 18;
Theta4 = 18;

C = false(X,Y,Theta1,Theta2,Theta3,Theta4);

global cSpace; 
cSpace = zeros(size(C));

%%init_q, goal_q 
hf2 = subplot(1,2,2);
plot3(hf2,init_q(1),init_q(2),init_q(3),'mv',goal_q(1),goal_q(2),goal_q(3),'mv');
title("6-dof robot phase space")
xlabel('x');
ylabel('y');
zlabel('theta1');
hold on
pause(2)


%%sampling
%1은 장애물 0은 not exist, 2는 exist 
nodes = 0; %지금까지 sampling 된 갯수
Node = [];
sampling = 3000; %총 sampling할 갯수

nodes = nodes + 1;
Node = [Node;init_q]; %Node에다가 집어넣음->configuration space의 노드

%while (nodes <= sampling + 1)

n_x = 10; %x 구간을 나눌 갯수
n_y = 10; %y 구간을 나눌 갯수 
n = sampling /(n_x * n_y); %구간 한개에 들어가야되는 샘플의 갯수
for i = 0:n_x -1
    for j = 0:n_y -1
        for add = 1:n
            x = randi([10 * i + 1, 10 * (i + 1)]);
            y = randi([10 * j + 1, 10 * (j + 1)]);
            theta1 = randi(Theta1);
            theta2 = randi(Theta2);
            theta3 = randi(Theta3);
            theta4 = randi(Theta4);

    if(wSpace(x,y) == 1)
        cSpace(x,y,theta1,theta2,theta3,theta4) = 1;
        continue
    end

    if([x,y,theta1] == [2,2,0])
        continue
    end
    
    if([x,y,theta1] == [80,90,7])
        continue
    end
    %not observation 
    if (cSpace(x,y,theta1,theta2,theta3,theta4) == 0)
        Q = [x,y,theta1,theta2,theta3,theta4];
        %workspace로 넘어감 
        if(wSpaceCollision(Q))
            cSpace(x,y,theta1,theta2,theta3,theta4) = 1; % obstacle 
            plot3(hf2,x,y,theta1,'o')
            %zlabel('theta1');
            continue
        else
            cSpace(x,y,theta1,theta2,theta3,theta4) = 2; % not obstacle -> exist
            nodes = nodes + 1;
            Node = [Node;Q]; %Node에다가 집어넣음 
            
            title("6-dof robot phase space")
            plot3(hf2,x,y,theta1,'r*')
            pause(0.0000001)
        end
        %점을 찍어줌

    %already observation
    else
        continue
    end
        end
    end
end

nodes = nodes + 1;
Node = [Node;goal_q]; %Node에다가 집어넣음 첫번째 노드에는 init_q가 마지막 노드에는 goal_q가 들어있음 

%%Expansion 
connect_node = inf(nodes,nodes); %connect_Graph 원래 Inf

range = 10;

for i = 1:nodes
    for j = i + 1:nodes
        %range를 벗어남 Node는 x,y,theta1,...,theta6을 열 값으로 가지고 있는 행렬임
        
        dis = norm(Node(i,1:2) - Node(j,1:2)) + norm(Node(i,3:6) - Node(j,3:6)) * 1/20;
        if(dis > range)
            continue
        %obstacle exist
        elseif(cObstacle(Node(i,:),Node(j,:)))
            continue
        elseif(wSpaceCollision(Node(i,:)))
            continue
        elseif(wSpaceCollision(Node(j,:)))
            continue
        %그래프인 connect_node 에 집어넣어줌 선 연결 connect_node는 이 Node들의 연결 됬는지 안됬는지만
        %판단해주는 행렬 
        else
            %i번째 노드와 j번째 노드를 연결 !!! 헷갈리지 말자 
            connect_node(i,j) = dis; %연결 됨 
            connect_node(j,i) = dis; %%%%
            %연결 선 그리기     
            %hold on 
            %line(hf2,[Node(i,1),Node(j,1)],[Node(i,2),Node(j,2)],[Node(i,3),Node(j,3)])
            %pause(0.0000000000001)
        end
    end
end
%plot3(hf2,init_q(1),init_q(2),init_q(3),'mv',goal_q(1),goal_q(2),goal_q(3),'mv');
hold off

%%Connect PRM Graph,Q_init, Q_init 
min_init = 100;
min_goal = 100;
init_node_index = 0;
goal_node_index = 0;
for i = 1:nodes

    dis_init = norm(Node(i,1:2)-init_q(1:2)) + norm(Node(i,3:6)-init_q(3:6)) * 1/20; %init과 점과의 거리
    dis_goal = norm(Node(i,1:2)-goal_q(1:2)) + norm(Node(i,3:6)-goal_q(3:6)) * 1/20; %goal과 점과의 거리
    if(dis_init < min_init && i~= 1)
        min_init = dis_init;
        init_node_index = i;

    end
    if(dis_goal < min_goal && i~= nodes)
        min_goal = dis_goal;
        goal_node_index = i;
    end
end
connect_node(1,init_node_index) = min_init;
connect_node(init_node_index,1) = min_init;
connect_node(i,goal_node_index) = min_goal;
connect_node(goal_node_index,i) = min_goal;

hold on 
line([init_q(1),Node(init_node_index,1)],[init_q(2),Node(init_node_index,2)],[init_q(3),Node(init_node_index,3)],'Color','green','Linewidth',2);
line([goal_q(1),Node(goal_node_index,1)],[goal_q(2),Node(goal_node_index,2)],[goal_q(3),Node(goal_node_index,3)],'Color','green','Linewidth',2);
hold off

%%Roadmap 
%set up Graph
for i = 1:nodes
    for j = 1:nodes 
        if (connect_node(i,j) == Inf)
        connect_node(i,j) = 0;
        end
    end
end

goal_node = nodes;
[cost,Roadmap] = dijkstra(connect_node,1,goal_node);
%[cost,Roadmap] = dijkstra(connect_node,init_node_index,goal_node_index);
Roadmap = fliplr(Roadmap) %Roadmap은 Node의 번호임
for i = 1:numel(Roadmap) - 1
    parent = Roadmap(i)
    child = Roadmap(i + 1)
    hold on
    line(hf2,[Node(parent,1),Node(child,1)],[Node(parent,2),Node(child,2)],[Node(parent,3),Node(child,3)],'Color','green','Linewidth',2)
    pause(0.0001)
end
hold off 

segments = 50;
%% roadmap을 따라서 이동 
for i = 1:numel(Roadmap) - 1
    child_index = Roadmap(i);
    child_q = Node(child_index,:);

    j = Roadmap(i + 1);
    parent_q = Node(j,:);
    vector = parent_q - child_q;
    for k = 1:segments
        q = child_q + k * vector/segments;
        %p_goal_q = FromQ_ToP(goal_q); 
        p_q = FromQ_ToP(q); 
        PlotGraph(p_init_q, p_goal_q, p_q,hf);
    end
end

function K = PlotGraph(p_init_q, p_goal_q, p_q,hf)

    hold on 
    plot(hf,p_init_q(:, 1), p_init_q(:, 2), LineWidth=3)
    plot(hf,p_goal_q(:, 1), p_goal_q(:, 2), LineWidth=3)
    plot(hf,p_q(:, 1), p_q(:, 2), LineWidth=3)
    pause(0.0000000000000000000000001)
end

%%workspace function
function p = FromQ_ToP(Q)
    global l1
    global l2
    global l3
    global l4
    global init_wSpace
    Q(3) = Q(3)/18 * 2 *pi;
    Q(4) = Q(4)/18 * 2 *pi;
    Q(5) = Q(5)/18 * 2 *pi;
    Q(6) = Q(6)/18 * 2 *pi;

    p = zeros(length(init_wSpace),2)
    p1 = [Q(1), Q(2)];
    p2 = p1 + [l1 *cos(Q(3)),l1 * sin(Q(3))];
    p3 = p2 + [l2 *cos(Q(3) + Q(4)),l2 * sin(Q(3) + Q(4))];
    p4 = p3 + [l3 *cos(Q(3) + Q(4) + Q(5)),l3 * sin(Q(3) + Q(4) + Q(5))];
    p5 = p4 + [l4 *cos(Q(3) + Q(4) + Q(5) + Q(6)),l4 * sin(Q(3) + Q(4) + Q(5) + Q(6))];

    p(1,:) = p1;
    p(2,:) = p2;
    p(3,:) = p3;
    p(4,:) = p4;
    p(5,:) = p5;
end

%%configuration space function
function check = wSpaceCollision(Q)
    check = 0;
    global l1
    global l2
    global l3
    global l4
    
    Q(3) = Q(3)/18 * 2 *pi;
    Q(4) = Q(4)/18 * 2 *pi;
    Q(5) = Q(5)/18 * 2 *pi;
    Q(6) = Q(6)/18 * 2 *pi;


    p1 = [Q(1), Q(2)];
    p2 = p1 + [l1 *cos(Q(3)),l1 * sin(Q(3))];
    p3 = p2 + [l2 *cos(Q(3) + Q(4)),l2 * sin(Q(3) + Q(4))];
    p4 = p3 + [l3 *cos(Q(3) + Q(4) + Q(5)),l3 * sin(Q(3) + Q(4) + Q(5))];
    p5 = p4 + [l4 *cos(Q(3) + Q(4) + Q(5) + Q(6)),l4 * sin(Q(3) + Q(4) + Q(5) + Q(6))];


    if(wObstacle(p1,p2) == 1) 
        check = 1;
    elseif(wObstacle(p2,p3) == 1) 
            check = 1;
    elseif(wObstacle(p3,p4) == 1) 
            check = 1;
    elseif(wObstacle(p4,p5) == 1) 
            check = 1;
    end
end


function bool = wObstacle(q1,q2)
    bool = 0;
    global wSpace
    Segments = 10;
    %방향벡터

    V_x = q2(1) - q1(1); %1은 행 -> y축을 의미
    V_y = q2(2) - q1(2); %2는 열 -> x축을 의미
    V_x_Step = V_x/Segments;
    V_y_Step = V_y/Segments;
    for i = 1:Segments
        q1_x = round(q1(1) + V_x_Step * i);
        q1_y = round(q1(2) + V_y_Step * i);

        if(q1_x <= 0)
            q1_x = 1;
        end
        
        if(q1_y <= 0)
            q1_y = 1;
        end        
        if(wSpace(q1_x, q1_y) == 1)
            bool = 1;
            break
        end
      
    end
    
end

function bool = cObstacle(q1,q2)
    bool = 0;
    global cSpace
    Segments = 10;
    %방향벡터
    V_x = q2(1) - q1(1);
    V_y = q2(2) - q1(2);
    V_theta1 = q2(3) - q1(3);
    V_theta2 = q2(4) - q1(4);
    V_theta3 = q2(5) - q1(5);
    V_theta4 = q2(6) - q1(6);

    V_x_Step = V_x/Segments;
    V_y_Step = V_y/Segments;
    V_theta1_Step = V_theta1/Segments;
    V_theta2_Step = V_theta2/Segments;
    V_theta3_Step = V_theta3/Segments;
    V_theta4_Step = V_theta4/Segments;
    
    for i = 1:Segments - 1
        q1_x = round(q1(1) + V_y_Step * i);
        q1_y = round(q1(2) + V_x_Step * i);
        q1_theta1 = round(q1(3) + V_theta1_Step * i);
        q1_theta2 = round(q1(4) + V_theta2_Step * i);
        q1_theta3 = round(q1(5) + V_theta3_Step * i);
        q1_theta4 = round(q1(6) + V_theta4_Step * i);

        if q1_x <=0
            q1_x = 1;
        end
        if q1_x >100
            q1_x = 100;
        end

        if q1_y <=0
            q1_y = 1;
        end
        if q1_y >100
            q1_y = 100;
        end

        if q1_theta1 <=0
            q1_theta1 = 1;
        end
        if q1_theta1 >18
            q1_theta1 = 18;
        end

        if q1_theta2 <=0
            q1_theta2 = 1;
        end
        if q1_theta2 >18
            q1_theta2 = 18;
        end

        if q1_theta3 <=0
            q1_theta3 = 1;
        end
        if q1_theta3 >18
            q1_theta3 = 18;
        end
        if q1_theta4 <=0
            q1_theta4 = 1;
        end
        if q1_theta4 >18
            q1_theta4 = 18;
        end
        
        if(cSpace(q1_x, q1_y,q1_theta1,q1_theta2,q1_theta3,q1_theta4) == 1)
            bool = 1;
            break
        end
      
    end
end

%%dijkstra functions
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







