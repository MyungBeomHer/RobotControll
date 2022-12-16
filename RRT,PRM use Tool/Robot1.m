function robot = Robot1()
    link = [3 1 0.1];
    robot = collisionBox(link(1), link(2), link(3));       % x, y, z
end