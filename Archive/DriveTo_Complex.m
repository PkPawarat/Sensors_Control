% New function in testing to control the robot all the way to the goal.
% First movement will be to rotate towards the goal and then drive towards
% it. While driving, minor slippage may occur and therefore must be
% accounted for
function DriveTo_Complex(target_x , target_y)
   % Shut down any existing ROS nodes
    rosshutdown;

    turningSpeed = 0.4;
    drivingSpeed = 0.1;
    angleTolerance = 0.005;
    MaxAngle = 0.5;
    
    % Initialize ROS node in MATLAB
    rosinit;
    
    % Create a publisher for sending velocity commands
    cmd_vel_pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
    
    % Create a subscriber for the robot's current pose (for orientation feedback)
    pose_sub = rossubscriber('/odom', 'nav_msgs/Odometry');
    
    % Create a Twist message for the desired velocity command
    cmd_msg = rosmessage(cmd_vel_pub);

    send(cmd_vel_pub, cmd_msg);

    %Get the current pose of the robot
    pose = receive(pose_sub);

    current_x = pose.Pose.Pose.Position.X
    current_y = pose.Pose.Pose.Position.Y

    delta_x = target_x - current_x
    delta_y = target_y - current_y

    targetYaw = atan2(delta_y, delta_x)

    currentOrientation = quat2eul([pose.Pose.Pose.Orientation.W, ...
        pose.Pose.Pose.Orientation.X, pose.Pose.Pose.Orientation.Y, ...
        pose.Pose.Pose.Orientation.Z]);

    % Extract the current yaw angle
    CurrentRotation = currentOrientation(1)

    delta_yaw = targetYaw - CurrentRotation
    delta_yaw_deg = rad2deg(delta_yaw)

    % Set the angular velocity to make the TurtleBot rotate
    cmd_msg.Angular.Z = turningSpeed; % Adjust the value as needed
    if delta_yaw_deg < 0
        % If rotation is negative, swap direction
        cmd_msg.Angular.Z = -cmd_msg.Angular.Z;
    end


    %Now that we are facing the correct direction, start the drive
    %but consider slippage in the rotation

    distance = sqrt(delta_x * delta_x + delta_y * delta_y)

    distanceTolerance = 0.05;
    while distance > distanceTolerance
        cmd_msg.Angular.Z = 0; %reset angular to start
        cmd_msg.Linear.X = drivingSpeed;

        % Get the current orientation from the pose subscriber
        pose = receive(pose_sub);
        currentOrientation = quat2eul([pose.Pose.Pose.Orientation.W, ...
            pose.Pose.Pose.Orientation.X, pose.Pose.Pose.Orientation.Y, ...
            pose.Pose.Pose.Orientation.Z]);
        CurrentRotation = currentOrientation(1); % Extract the yaw angle


        current_x = pose.Pose.Pose.Position.X;
        current_y = pose.Pose.Pose.Position.Y;
    
        delta_x = target_x - current_x;
        delta_y = target_y - current_y;

        distance = sqrt(delta_x * delta_x + delta_y * delta_y)

        % Calculate the updated angle difference
        delta_yaw = targetYaw - CurrentRotation;
        if delta_yaw > pi * 2
            delta_yaw = delta_yaw - pi * 2;
        end

        delta_yaw_deg = rad2deg(delta_yaw)
        % Set the angular velocity to make the TurtleBot rotate

        if abs(delta_yaw_deg) > angleTolerance
            % If rotation is negative, swap direction
            cmd_msg.Angular.Z = turningSpeed;
            if delta_yaw_deg < 0
                cmd_msg.Angular.Z = -cmd_msg.Angular.Z;
            end
        end

        send(cmd_vel_pub, cmd_msg);
        % Pause briefly to control the loop rate
        pause(0.02);
    end

    %stop the robot by sending linear velocity command
    cmd_msg.Linear.X = 0;
    cmd_msg.Angular.Z = 0;
    send(cmd_vel_pub, cmd_msg);
    
    % Shutdown the ROS node when done
    rosshutdown;

end