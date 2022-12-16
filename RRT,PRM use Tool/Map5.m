function obss = Map5()
    obs1 = collisionBox(3,1,3);
    obs2 = collisionBox(3,1,3);
    obs3 = collisionBox(3,1,3);
    obs4 = collisionBox(3,1,3);
    obs6 = collisionBox(3,1,3);
    obs7 = collisionBox(3,1,3);
    obs8 = collisionBox(3,1,3);
    obs9 = collisionBox(3,1,3);
    obs10 = collisionBox(3,1,3);
    obs11 = collisionBox(3,1,3);
    obs12 = collisionBox(3,1,3);
    obs13 = collisionBox(3,1,3);
    obs14 = collisionBox(3,1,3);
    obs15 = collisionBox(3,1,3);
    obs16 = collisionBox(3,1,3);
    
    obs1.Pose = trvec2tform([-3 0 -3]);
    obs2.Pose = trvec2tform([0 0 -3]);
    obs3.Pose = trvec2tform([3 0 -3]);
    obs4.Pose = trvec2tform([-3 0 0]);
    obs5.Pose = trvec2tform([3 0 -0]);
    obs6.Pose = trvec2tform([-3 0 3]);
    obs7.Pose = trvec2tform([0 0 3]);
    obs8.Pose = trvec2tform([3 0 3]);

    obs9.Pose = trvec2tform([-3 3 -3]);
    obs10.Pose = trvec2tform([0 3 -3]);
    obs11.Pose = trvec2tform([3 3 -3]);
    obs12.Pose = trvec2tform([-3 3 0]);
    obs13.Pose = trvec2tform([3 3 -0]);
    obs14.Pose = trvec2tform([-3 3 3]);
    obs15.Pose = trvec2tform([0 3 3]);
    obs16.Pose = trvec2tform([3 3 3]);

   obss = [obs1,obs2,obs3,obs4,obs6,obs7,obs8,obs9,obs10,obs11,obs12,obs13,obs14,obs15,obs16]
end


