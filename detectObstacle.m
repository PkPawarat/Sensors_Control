% function [isObstacle, distance] = detectFrontObstacle()
%     % Shut down any existing ROS nodes
%     rosshutdown;
    
%     % Initialize ROS node in MATLAB
%     rosinit;
    
%     % Create a subscriber for the LIDAR scan data
%     scan_sub = rossubscriber('/scan', 'sensor_msgs/LaserScan');
    
%     % Retrieve the latest scan data
%     scan_msg = receive(scan_sub, 10); % wait up to 10 seconds for data
    
%     % Get the range and angle information
%     ranges = scan_msg.Ranges;
%     angle_increment = scan_msg.AngleIncrement;
%     angle_min = scan_msg.AngleMin;
%     angle_max = scan_msg.AngleMax;

%     % Identify indices corresponding to front of robot (-20° to 20° around the front)
%     front_indicesLeft = find((angle_min:angle_increment:angle_max) >= 340*pi/180 & (angle_min:angle_increment:angle_max) <= 360*pi/180);
%     front_indicesRight = find((angle_min:angle_increment:angle_max) >= 0*pi/180 & (angle_min:angle_increment:angle_max) <= 20*pi/180);

%     % Extract the front scan readings from both sides
%     front_rangesLeft = ranges(front_indicesLeft);
%     front_rangesRight = ranges(front_indicesRight);

%     % Combine ranges from both sides
%     front_ranges = [front_rangesLeft; front_rangesRight];
    
%     % Check if there's an obstacle close by
%     threshold_distance = 2; % Set your desired threshold
%     min_front_distance = min(front_ranges);

%     if min_front_distance < threshold_distance
%         isObstacle = true;
%         distance = min_front_distance
%     else
%         isObstacle = false;
%         distance = min_front_distance
%     end

%     % Shut down ROS node
%     rosshutdown;
% end


function [isObstacle, distance, obstacle_location] = detectFrontObstacle()

    % Shut down any existing ROS nodes
    rosshutdown;

    % Initialize ROS node in MATLAB
    rosinit;

    % Create a subscriber for the LIDAR scan data
    scan_sub = rossubscriber('/scan', 'sensor_msgs/LaserScan');

    % Create a subscriber for the robot's pose
    pose_sub = rossubscriber('/odom', 'nav_msgs/Odometry');

    % Retrieve the latest scan data
    scan_msg = receive(scan_sub, 10); % wait up to 10 seconds for data

    % Retrieve the robot's current pose
    pose_msg = receive(pose_sub, 10);
    
    % Get the range and angle information
    ranges = scan_msg.Ranges;
    angle_increment = scan_msg.AngleIncrement;
    angle_min = scan_msg.AngleMin;
    angle_max = scan_msg.AngleMax;

    % Identify indices corresponding to front of robot (-20° to 20° around the front)
    front_indicesLeft = find((angle_min:angle_increment:angle_max) >= 340*pi/180 & (angle_min:angle_increment:angle_max) <= 360*pi/180);
    front_indicesRight = find((angle_min:angle_increment:angle_max) >= 0*pi/180 & (angle_min:angle_increment:angle_max) <= 20*pi/180);

    % Extract the front scan readings from both sides
    front_rangesLeft = ranges(front_indicesLeft);
    front_rangesRight = ranges(front_indicesRight);

    % Combine ranges from both sides
    front_ranges = [front_rangesLeft; front_rangesRight];
    
    % Check if there's an obstacle close by
    threshold_distance = 2; % Set your desired threshold
    [min_front_distance, index] = min(front_ranges);

    if min_front_distance < threshold_distance
        isObstacle = true;
        distance = min_front_distance;
        
        % Calculate the angle to the closest obstacle
        if index <= length(front_rangesLeft)
            obstacle_angle = angle_min + (front_indicesLeft(index) - 1) * angle_increment;
        else
            obstacle_angle = angle_min + (front_indicesRight(index - length(front_rangesLeft)) - 1) * angle_increment;
        end
        
        % Calculate obstacle's position in the robot's frame
        x_local = distance * cos(obstacle_angle);
        y_local = distance * sin(obstacle_angle);
        
        % Convert the obstacle's location to the global frame
        robot_x = pose_msg.Pose.Pose.Position.X;
        robot_y = pose_msg.Pose.Pose.Position.Y;
        robot_orientation = quat2eul([pose_msg.Pose.Pose.Orientation.W, pose_msg.Pose.Pose.Orientation.X, pose_msg.Pose.Pose.Orientation.Y, pose_msg.Pose.Pose.Orientation.Z]);
        yaw = robot_orientation(1);

        x_global = robot_x + x_local*cos(yaw) - y_local*sin(yaw);
        y_global = robot_y + x_local*sin(yaw) + y_local*cos(yaw);

        obstacle_location = [x_global, y_global];
        DriveTo(x_global-0.3, y_global)
    else
        isObstacle = false;
        distance = min_front_distance;
        obstacle_location = [NaN, NaN]; % No obstacle detected, so return NaN for location
    end

    % Shut down ROS node
    rosshutdown;
end
