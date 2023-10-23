%% Function to drive robot forwards
function DriveRobot(dx)
   % Shut down any existing ROS nodes
    rosshutdown;
    
    % Initialize ROS node in MATLAB
    rosinit;
    
    % Create a publisher for sending velocity commands
    cmd_vel_pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
    
    % Create a subscriber for the robot's current pose (for orientation feedback)
    pose_sub = rossubscriber('/odom', 'nav_msgs/Odometry');
    
    % Create a Twist message for the desired velocity command
    cmd_msg = rosmessage(cmd_vel_pub)
    
    % Set the forward / backward velocity to make the TurtleBot move
    % forward
    cmd_msg.Linear.X = 0.2; % Adjust the value as needed
    if dx < 0
        % If rotation is negative, swap direction
        cmd_msg.Linear.X = -cmd_msg.Linear.X;
    end

    if dx == 0
        cmd_msg.Linear.X = 0;
    end

    send(cmd_vel_pub, cmd_msg);

    %Get the current pose of the robot
    pose = receive(pose_sub);

    currentPositionX = pose.Pose.Pose.Position.X;
    currentPositionY = pose.Pose.Pose.Position.Y;

    % Find the local displacement based on pose
    CurrentRobotPoseZ = pose.Pose.Pose.Orientation.Z;

    delta_x_global = dx * cos(CurrentRobotPoseZ);
    delta_y_global = dx * sin(CurrentRobotPoseZ);

    %Calculate the target position
    targetPositionX = currentPositionX + delta_x_global; 
    targetPositionY = currentPositionY + delta_y_global; 

    % Define angle tolerance for stopping the rotation
    Tolerance = 0.005; % Adjust the tolerance as needed

    diffX = abs(targetPositionX - currentPositionX);
    diffY = abs(targetPositionY - currentPositionY);

    disp("displacement request"); disp(dx);
    disp("Current X Y position");    disp(currentPositionX);    disp(currentPositionY);
    disp("Goal X Y");    disp(targetPositionX);        disp(targetPositionY);
    disp("Delta X Y");    disp(delta_x_global); disp(delta_y_global); 
    disp("Diff X Y");    disp(diffX); disp(diffY); 
    
    %Continue moving forwards until displacement is met
    while diffX > Tolerance % & diffY > Tolerance
        %get the current pose from the pose subscriber
        pose = receive(pose_sub);

        diffX = abs(targetPositionX - pose.Pose.Pose.Position.X);
        diffY = abs(targetPositionY - pose.Pose.Pose.Position.Y);

        disp("Difference X and Y");        disp(diffX);        disp(diffY);

        pause(0.001);
    end

    %stop the robot by sending linear velocity command
    cmd_msg.Linear.X = 0;
    send(cmd_vel_pub, cmd_msg);

    % Shutdown the ROS node when done
    rosshutdown;
end
