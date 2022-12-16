function obss=Map3()
    obs1 = collisionBox(10, 4, 0.1);
    obs2 = collisionBox(10, 0.1, 4);
    obs3 = collisionBox(10, 4, 0.1);
    obs4 = collisionBox(6, 0.1, 4);
    obs5 = collisionBox(0.1, 10, 4);
    obs6 = collisionBox(4, 6, 0.1);
    obs7 = collisionBox(4, 6, 0.1);
    obs8 = collisionBox(0.1, 6, 4);

    obs9 = collisionBox(4, 6, 0.1);
    obs10 = collisionBox(4, 10, 0.1);
    obs11 = collisionBox(0.1, 6, 4);
    obs12 = collisionBox(0.1, 6, 4);
    obs13= collisionBox(0.1, 4, 10);
    obs14 = collisionBox(0.1, 4, 10);
    obs15 = collisionBox(4, 0.1, 10);
    obs16 = collisionBox(4, 0.1, 6);
    
   
    % 중심점 이동
    obs1.Pose = trvec2tform([0 0 2]);
    obs2.Pose = trvec2tform([0 -2 0]);
    obs3.Pose = trvec2tform([0 0 -2]);
    obs4.Pose = trvec2tform([-2 2 0]);
    obs5.Pose = trvec2tform([5 3 0]);
    obs6.Pose = trvec2tform([3 5 2]);
    obs7.Pose = trvec2tform([3 5 -2]);
    obs8.Pose = trvec2tform([1 5 0]);

    obs9.Pose = trvec2tform([3 10 2]);
    obs10.Pose = trvec2tform([3 11 -2]);
    obs11.Pose = trvec2tform([1 10 0]);
    obs12.Pose = trvec2tform([5 10 0]);
    obs13.Pose = trvec2tform([1 14 3]);
    obs14.Pose = trvec2tform([5 14 3]);
    obs15.Pose = trvec2tform([3 16 3]);
    obs16.Pose = trvec2tform([3 12 5]);
    

    obss = [obs1, obs2, obs3, obs4, obs5, obs6, obs7, obs8, obs9, obs10, obs11, obs12, obs13, obs14, obs15, obs16];
end
