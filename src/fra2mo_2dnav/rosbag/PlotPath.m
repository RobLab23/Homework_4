bag = rosbag('/home/lucabor/Scrivania/RoboticsLab/src/fra2mo_2dnav/rosbag/Path.bag');


    pose = bag.select("Topic", "/fra2mo/pose");
    poseTs = timeseries(pose);
    t = poseTs.Time - poseTs.Time(1);
    x = poseTs.Data(:, 4);
    y = poseTs.Data(:, 5);
    % z = poseTs.Data(:, 6);
    yaw = quat2eul([poseTs.Data(:, 10), poseTs.Data(:, 7:9)]);
    yaw = yaw(:, 1);


    % trajectory
    idxs = 1:round(length(x)/50):length(x);
    ql = 0.5;
    qx = x(idxs);
    qy = y(idxs);
    qu = cos(yaw(idxs));
    qv = sin(yaw(idxs));
    figure(Name="Trajectory")
    plot(x, y)
    hold on
    quiver(qx, qy, qu, qv, ql)
    scatter(-3, 5, 50, 'filled', 'MarkerFaceColor', 'r'); 
    scatter(-6, 8, 50, 'filled', 'MarkerFaceColor', 'g'); 
    scatter(-17.5, 3, 50, 'filled', 'MarkerFaceColor', 'b'); 
    scatter(-15, 7, 50, 'filled', 'MarkerFaceColor', 'c'); 
    scatter(-10, 3, 50, 'filled', 'MarkerFaceColor', 'k');

    grid on
    title("Trajectory")
    axis equal
    xlabel("x [m]")
    ylabel("y [m]")


