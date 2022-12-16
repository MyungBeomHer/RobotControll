clf;
clear all;
close all;

global Segments;
Segments = 10;
q_init = [-2, -10, 0, deg2rad(0), deg2rad(0), deg2rad(0)];
q_goal = [2, 5, 0, deg2rad(0), deg2rad(0), deg2rad(0)];

robot = Robot1();
init_robot = Robot1();
global obss;
obss=Map5();

Kinematics(init_robot, robot, q_init);
Simulate(robot, obss, false)
Draw_Init_Goal(init_robot, robot, q_init, q_goal)

pause()

%%sampling
%1은 장애물 0은 not exist, 2는 exist 
nodes = 0; %지금까지 sampling 된 갯수
Node = [];
connect_node = [];
range = 3.5;

nodes = nodes + 1;
Node = [Node;q_init]; %Node에다가 집어넣음->configuration space의 노드
connect_node = [connect_node,inf];

%flag = 0; %goal_not_found == 0, goal_found = 1
%%cSpace -> not_exist == 0, obstacle == 1,exist == 2 
while true
    x = -4.4 + (4.4 + 4.4) * rand();
    y = -10 + (10 + 10) * rand();
    z = -4.4 + (4.4 + 4.4) * rand();
    q1 = -90 + (90 -(-90)) * rand();
    q2 = -90 + (90 -(-90)) * rand();
    q3 = -90 + (90 -(-90)) * rand();
    q1 = deg2rad(q1);
    q2 = deg2rad(q2);
    q3 = deg2rad(q3);
    
    Q = [x,y,z,q1,q2,q3];
    Kinematics(init_robot, robot, Q);
    if 1 == Collision_check_robot(robot)
        continue
    end
    min_dis = inf;
    %연결할 노드 찾음   
    for i = 1:nodes
        dis = vecnorm(Node(i) - Q);
        %연결 못함 
       if range < dis
          continue
       end
       if(dis < min_dis)
         %두 configuration space사이를 segement로 나눠 점을 만든 다음 그 점 안의 workspace 상에서 collision 발생하는지 확인해야된다.
            if  Collision_check(robot,init_robot,Node(i,:),Q) == true
              continue
            end
            min_dis = dis;
            closest_index = i;
       end
    end
            %연결 할 수 있는 부모가 없다.-> drop
      if min_dis == inf 
         continue
      else
         nodes = nodes + 1;
         Node = [Node;Q]; %Node에다가 집어넣음 -> Configuration space를 넣는다. 
         connect_node = [connect_node;closest_index]; %connect Node와 Node의 순서는 Node번호로 똑같다. 단 부모를 넣는다.
      end   
         Cspace_plot(Node(closest_index,1),Node(closest_index,2),x,y);
         
      %부모에 노드를 연결했다. -> 이 노드는 살아남았다. -> 이 노드가 goal_q에 닿으면 rrt algorithm 종료
      goal_dis = vecnorm(q_goal - Q);
      if(goal_dis < range)
          disp("second")
            if Collision_check(robot,init_robot,Node(closest_index,:),q_goal) == 1
                disp("first")
               continue
            end
      nodes = nodes + 1;
      Node = [Node;q_goal]; %마지막 노드에 Configuration space를 넣는다. 
      connect_node = [connect_node;nodes-1];
      break
      %flag = 1; %for 문 나가는 flag -> 제일 마지막 노드와 connect_node는 goal_node다.    
      end
end

%%Roadmap 
route = [];
route = construct_route(nodes,Node,connect_node); %여기서 Nodes는 Goal 지점을 의미한다.

Roadmap = fliplr(route);
for i = 1:numel(Roadmap) - 1
    parent = Roadmap(i)
    child = Roadmap(i + 1)
    subplot(1,3,3)
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
    for k = 1:segments
        q = child_q + k * vector/segments;
        Kinematics(init_robot, robot, q)
        if Collision_check_robot(robot) == true
                    break
        end
        Simulate(robot, obss, false)
        pause(0.1)
    end
end

%%Make Roadmap 
function route = construct_route(nodes,Node,connect_node)
route = [nodes, connect_node(nodes,1)];
cur = connect_node(nodes,1);
    while connect_node(cur,1)~= Inf 
        cur = connect_node(cur,1);
        route = [route,cur];
    end
end

%%Function
%%collision여부 두 점 사이에 collision 여부 
function check = Collision_check(robot,init_robot,q1,q2)
    check = 0;
    global Segments
    V = (q2 - q1)/Segments; %vector 
    for i = 0:Segments 
        update_q = q1 + V * i;
        Kinematics(init_robot,robot,update_q);
        if Collision_check_robot(robot) ==  1
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
    subplot(1,3,3)
    hold on 
    title("6-dof robot phase space")    
    plot(x,y,'r*')
    line([xHat,x],[yHat,y])
    pause(0.0000001)
end

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
        
        xlim([-4.5 4.5])
        ylim([-30 30])
        zlim([-4.5 4.5])
        SimulObss(obss);

        subplot(1,3,2) %122
        SimulRobot(robot);
        hold on
        view(3)
    
        xlim([-4.5 4.5])
        ylim([-30 30])
        zlim([-4.5 4.5])
        SimulObss(obss);
    end

end

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
