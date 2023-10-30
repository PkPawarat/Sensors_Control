%% New script
function RotateRobot(rotateAngle)
   % Shut down any existing ROS nodes
    rosshutdown;
    
    % Initialize ROS node in MATLAB
    rosinit;
    
    % Create a publisher for sending velocity commands
    cmd_vel_pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
    
    % Create a subscriber for the robot's current pose (for orientation feedback)
    pose_sub = rossubscriber('/odom', 'nav_msgs/Odometry');
    
    % Create a Twist message for the desired velocity command
    cmd_msg = rosmessage(cmd_vel_pub);
    
    % Set the angular velocity to make the TurtleBot rotate
    cmd_msg.Angular.Z = 0.2; % Adjust the value as needed
    if rotateAngle < 0
        % If rotation is negative, swap direction
        cmd_msg.Angular.Z = -cmd_msg.Angular.Z;
    end

    % Initialize the initial orientation
    initialOrientation = 0;
    % Publish the command to start the rotation
    send(cmd_vel_pub, cmd_msg);
    
    % Define angle tolerance for stopping the rotation
    angleTolerance = 0.05; % Adjust the tolerance as needed
    
    % Get the current pose of the TurtleBot
    pose = receive(pose_sub);
    currentOrientation = quat2eul([pose.Pose.Pose.Orientation.W, ...
        pose.Pose.Pose.Orientation.X, pose.Pose.Pose.Orientation.Y, ...
        pose.Pose.Pose.Orientation.Z]);
    
    % Extract the current yaw angle
    CurrentRotation = currentOrientation(1);
    
    % Calculate the target angle for rotation
    targetAngle = CurrentRotation + deg2rad(rotateAngle); % Convert degrees to radians
    
    % Calculate the initial angle difference
    diff = abs(targetAngle - CurrentRotation);
    
    % Keep rotating until the angle difference is within tolerance
    while diff > angleTolerance
        % Get the current orientation from the pose subscriber
        pose = receive(pose_sub);
        currentOrientation = quat2eul([pose.Pose.Pose.Orientation.W, ...
            pose.Pose.Pose.Orientation.X, pose.Pose.Pose.Orientation.Y, ...
            pose.Pose.Pose.Orientation.Z]);
        CurrentRotation = currentOrientation(1); % Extract the yaw angle
        
        % Calculate the updated angle difference
        diff = abs(targetAngle - CurrentRotation);
        if diff > pi * 2
            diff = diff - pi * 2
        end
        disp(diff);
        
        % Pause briefly to control the loop rate
        pause(0.02);
    end
    
    % Stop the TurtleBot by sending a zero angular velocity command
    cmd_msg.Angular.Z = 0;
    send(cmd_vel_pub, cmd_msg);
    
    % Shutdown the ROS node when done
    rosshutdown;
end