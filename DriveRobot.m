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

    if dx = 0
        cmd_msg.Linear.X = 0;
    end

    send(cmd_vel_pub, cmd_msg);

    %Get the current pose of the robot
    pose = receive(pose_sub);
    
    currentOrientation = quat2eul([pose.Pose.Pose.Orientation.W, ...
        pose.Pose.Pose.Orientation.X, pose.Pose.Pose.Orientation.Y, ...
        pose.Pose.Pose.Orientation.Z]);
    currentPositionX = pose.Pose.Pose.Position.X;

    % Find the local displacement based on pose
    CurrentRobotPoseZ = pose.Pose.Pose.

    delta_x_global = dx * cos(initial_global_pose.Orientation.Z) - dy * sin(initial_global_pose.Orientation.Z);
    delta_y_global = dx * sin(initial_global_pose.Orientation.Z) + dy * cos(initial_global_pose.Orientation.Z);


    %Calculate the target position
    targetPositionX = currentPositionX + displacement; 

    % Define angle tolerance for stopping the rotation
    Tolerance = 0.05; % Adjust the tolerance as needed
    
    %Continue moving forwards until displacement is met
    while abs(targetPositionX - currentPositionX) > Tolerance
        %get the current pose from the pose subscriber
        pose = receive(pose_sub);
        currentPositionX = pose.Pose.Pose.Position.X;

        pause(0.05);
    end

    %stop the robot by sending linear velocity command
    cmd_msg.Linear.X = 0;
    send(cmd_vel_pub, cmd_msg);

    % Shutdown the ROS node when done
    rosshutdown;
end
