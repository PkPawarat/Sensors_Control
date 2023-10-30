%Connor Keogh, Pawarat Phatthanaphusakun & Lauren Seeto
% Sensors and Control
% Group Project

%Pseudo Code
% 1. Scan environment for QR Code
% 2. If QR code = NA, turn 45 degrees counter clockwise and go back to (1)
% 3. Calculate the normal of the plane of detected QR code and robots
% required turn and movement to get to line
% 4. Rotate robot towards line normal and drive to it
% 5. Rotate robot to QR code again
% 6. Recalculate (3) and check error. If within +- X, continue
% 7. Drive towards QR code until within X distance

%For complex trajectory, recommend arc steering
%4. 


classdef TurtlebotFollowStraightLine < handle

    properties
    QRCodeFound = false; % On startup, this will be false. 
    pub_vel;% = rospublisher('/cmd_vel','DataFormat','geometry_msgs/Twist');
    odom_;% = rossubscriber('/odom','DataFormat','struct');
    lidar_;% = rossubscriber('/scan','DataFormat','struct');
    camera_rgb_;% = rossubscriber('/camera/rgb/image_raw');

    camera_image;

    turn_duration = 5; % Turning duration in seconds

    ImageSub;
    VideoPlay;
    velPub;
    velMsg;

    QRCodeOrder = {'1. Follow me', '2. Follow me', '3. Follow me'};

    GetOrder = 1;

    ipAddress = '127.0.0.1'%'192.168.0.101'

    %% Set advance to true for complex trajectory 
    Advance = false;

    end

    methods
        function self = TurtlebotFollowStraightLine()
            
            %Clear all, close all etc
            close all;
            clc;


            %% SETUP
            % Connect to ROS master
            rosshutdown;

            rosinit(self.ipAddress);
            % rosinit;

            self.pub_vel = rospublisher('/cmd_vel','geometry_msgs/Twist');
            self.odom_ = rossubscriber('/odom','DataFormat','struct');
            self.lidar_ = rossubscriber('/scan','DataFormat','struct');
            self.camera_rgb_ = rossubscriber('/camera/rgb/image_raw', 'sensor_msgs/Image');
            
            pose = receive(self.odom_);
            
            [isObstacle, distances, obstacle_locations] = self.detectAllObstacles(self);

            if ~self.Advance
                for i = 1:size(self.QRCodeOrder, 2)
                    disp("Start find QR code.");
                    self.ScanForQR(self);
                    disp("QR Code found.");
                    disp(self.QRCodeOrder{i});
                    [isObstacle, distance, obstacle_location] = self.detectFrontObstacle(self);
                    disp("Rotating robot towards QR");
                    disp(obstacle_location(1))
                    disp(obstacle_location(2))
                    self.DriveTo(self, obstacle_location(1),obstacle_location(2), 0.4, false)
                    disp("Rotate robot back to home position");
                    self.RotateRobot(self, 170);
                    disp("Drive robot back to home position");
                    self.DriveTo(self, pose.Pose.Pose.Position.X, pose.Pose.Pose.Position.Y, 0.1, true);
                    disp("Rotate robot back to start position");
                    self.RotateRobot(self, 170);
                end
            else
                for i = 1:size(self.QRCodeOrder, 2)-1
                    disp("Drive robot to QR code location with advance mode");
                    disp('//')
                    disp(obstacle_locations(i,1))
                    disp(obstacle_locations(i,2))
                    disp('//')
                    self.DriveTo_Complex(self, obstacle_locations(i,1),obstacle_locations(i,2), 0.4)
                    disp("Rotate robot back");
                    self.RotateRobot(self, 170);
                end
                self.DriveTo_Complex(self, pose.Pose.Pose.Position.X, pose.Pose.Pose.Position.Y, 0.2);
            end
            disp("ALL DONE Thanks for watching");
        end
    end

    methods(Static)

         % If the QR code can be found, then change 'QRCodeFound' to true
        function ScanForQR(self)
            detectQRCode = true;
            while detectQRCode
                msg = receive(self.camera_rgb_,10);
                self.ImageSub = readImage(msg);
                [zoomedROI, detect] = self.detectSquare(self);
                if detect
                    [msg, detectedFormat, loc] = readBarcode(zoomedROI);
                    % Logic for identifying QR code
                    currentQRCodeTarget = self.QRCodeOrder(self.GetOrder);
                    if currentQRCodeTarget == msg
                        self.GetOrder = self.GetOrder+1;
                        self.StopRobot(self);
                        self.QRCodeFound = true;
                        detectQRCode = false;

                        return;
                    end
                end
                %If QR not found
                self.RotateRobot(self, -5);
                % pause(0.1);
            end
        end
        
        %Rotate the robot 45 degrees counter-clockwise. 
        function RotateRobot(self, rotateAngle)
            % Create a Twist message for the desired velocity command
            cmd_msg = rosmessage(self.pub_vel);
            
            % Set the angular velocity to make the TurtleBot rotate
            cmd_msg.Angular.Z = 0.4; % Adjust the value as needed
            if rotateAngle < 0
                % If rotation is negative, swap direction
                cmd_msg.Angular.Z = -cmd_msg.Angular.Z;
            end
            
            % Initialize the initial orientation
            initialOrientation = 0;
            % Publish the command to start the rotation
            send(self.pub_vel, cmd_msg);
            
            % Define angle tolerance for stopping the rotation
            angleTolerance = 0.05; % Adjust the tolerance as needed
            
            % Get the current pose of the TurtleBot
            pose = receive(self.odom_);
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
                pose = receive(self.odom_);
                currentOrientation = quat2eul([pose.Pose.Pose.Orientation.W, ...
                    pose.Pose.Pose.Orientation.X, pose.Pose.Pose.Orientation.Y, ...
                    pose.Pose.Pose.Orientation.Z]);
                CurrentRotation = currentOrientation(1); % Extract the yaw angle
                
                % Calculate the updated angle difference
                diff = abs(targetAngle - CurrentRotation);
                if diff > pi * 2
                    diff = diff - pi * 2;
                end
                % disp(diff);
                
                % Pause briefly to control the loop rate
                send(self.pub_vel, cmd_msg);
                pause(0.02);
            end
            
            % Stop the TurtleBot by sending a zero angular velocity command
            cmd_msg.Angular.Z = 0;
            send(self.pub_vel, cmd_msg);
        end
        
        % Drive the robot to target x and y location base on /odom with
        % distance tolerance
        function DriveTo(self, target_x , target_y, distanceTolerance, back)
            turningSpeed = 0.1;
            drivingSpeed = 0.1;
            
            % Create a Twist message for the desired velocity command
            cmd_msg = rosmessage(self.pub_vel);
            send(self.pub_vel, cmd_msg);
            
            %Get the current pose of the robot
            pose = receive(self.odom_);
            
            current_x = pose.Pose.Pose.Position.X;
            current_y = pose.Pose.Pose.Position.Y;
            
            delta_x = target_x - current_x;
            delta_y = target_y - current_y;
            
            targetYaw = atan2(delta_y, delta_x);
            
            currentOrientation = quat2eul([pose.Pose.Pose.Orientation.W, ...
                pose.Pose.Pose.Orientation.X, pose.Pose.Pose.Orientation.Y, ...
                pose.Pose.Pose.Orientation.Z]);
            
            % Extract the current yaw angle
            CurrentRotation = currentOrientation(1);
            
            delta_yaw = targetYaw - CurrentRotation;
            delta_yaw_deg = rad2deg(delta_yaw);
            
            % Set the angular velocity to make the TurtleBot rotate
            cmd_msg.Angular.Z = turningSpeed; % Adjust the value as needed
            if delta_yaw_deg < 0
                % If rotation is negative, swap direction
                cmd_msg.Angular.Z = -cmd_msg.Angular.Z;
            end
            
            % Publish the command to start the rotation
            send(self.pub_vel, cmd_msg);
            
            angleTolerance = 0.05;
            % Keep rotating until the angle difference is within tolerance
            while abs(delta_yaw_deg) > angleTolerance
                % Get the current orientation from the pose subscriber
                pose = receive(self.odom_);
                currentOrientation = quat2eul([pose.Pose.Pose.Orientation.W, ...
                    pose.Pose.Pose.Orientation.X, pose.Pose.Pose.Orientation.Y, ...
                    pose.Pose.Pose.Orientation.Z]);
                CurrentRotation = currentOrientation(1); % Extract the yaw angle
                    
                % Calculate the updated angle difference
                delta_yaw_deg = targetYaw - CurrentRotation;
                if delta_yaw_deg > pi * 2
                    delta_yaw_deg = delta_yaw_deg - pi * 2;
                end
                % disp(delta_yaw_deg);
                    
                % Pause briefly to control the loop rate
                send(self.pub_vel, cmd_msg);
                % pause(0.02);
            end
            
            cmd_msg.Angular.Z = 0;
            send(self.pub_vel, cmd_msg);
            
            %Now that we are facing the correct direction, start the drive
            %but consider slippage in the rotation
            
            distance = sqrt(delta_x * delta_x + delta_y * delta_y);
            countcheckLine = 0;
            % distanceTolerance = 0.3;
            cmd_msg.Angular.Z = 0; %reset angular to start
            while distance > distanceTolerance
                cmd_msg.Linear.X = drivingSpeed;
            
                % Get the current orientation from the pose subscriber
                pose = receive(self.odom_);
                currentOrientation = quat2eul([pose.Pose.Pose.Orientation.W, ...
                    pose.Pose.Pose.Orientation.X, pose.Pose.Pose.Orientation.Y, ...
                    pose.Pose.Pose.Orientation.Z]);
                CurrentRotation = currentOrientation(1); % Extract the yaw angle

                current_x = pose.Pose.Pose.Position.X;
                current_y = pose.Pose.Pose.Position.Y;
                
                delta_x = target_x - current_x;
                delta_y = target_y - current_y;
            
                distance = sqrt(delta_x * delta_x + delta_y * delta_y);
            
                % Calculate the updated angle difference
                delta_yaw = targetYaw - CurrentRotation;
                if delta_yaw > pi * 2
                    delta_yaw = delta_yaw - pi * 2;
                end
            
                delta_yaw_deg = rad2deg(delta_yaw);
                % Set the angular velocity to make the TurtleBot rotate
                if back
                    if abs(delta_yaw_deg) > angleTolerance
                        % If rotation is negative, swap direction
                        cmd_msg.Angular.Z = turningSpeed;
                        if delta_yaw_deg < 0
                            cmd_msg.Angular.Z = -cmd_msg.Angular.Z;
                        end
                    end
                % Pause briefly to control the loop rate
                % pause(0.02);
                else
                    if countcheckLine == 5
                        turn = self.CalculateNormal(self);
                        if turn == 1
                            cmd_msg.Angular.Z = -turningSpeed;
                        elseif turn == 2
                            cmd_msg.Angular.Z = turningSpeed;
                        elseif turn == 0
                            cmd_msg.Angular.Z = 0;
                        end
                        countcheckLine = 0;
                    end
                end
                
                countcheckLine = countcheckLine+1;
                
                send(self.pub_vel, cmd_msg);
            end
            
            %stop the robot by sending linear velocity command
            cmd_msg.Linear.X = 0;
            cmd_msg.Angular.Z = 0;
            send(self.pub_vel, cmd_msg);
        end
    
        function DriveTo_Complex(self, target_x , target_y, distanceTolerance)
            turningSpeed = 0.4;
            drivingSpeed = 0.1;
            angleTolerance = 0.05;
            MaxAngle = 0.5;
            distanceTolerance = distanceTolerance;
            
            % Create a publisher for sending velocity commands
            cmd_vel_pub = self.pub_vel;
            
            % Create a subscriber for the robot's current pose (for orientation feedback)
            pose_sub = self.odom_;
            
            % Create a Twist message for the desired velocity command
            cmd_msg = rosmessage(cmd_vel_pub);
        
            send(cmd_vel_pub, cmd_msg);
        
            %Get the current pose of the robot
            pose = receive(pose_sub);
        
            current_x = pose.Pose.Pose.Position.X;
            current_y = pose.Pose.Pose.Position.Y;
        
            delta_x = target_x - current_x;
            delta_y = target_y - current_y;
        
            targetYaw = atan2(delta_y, delta_x);
        
            currentOrientation = quat2eul([pose.Pose.Pose.Orientation.W, ...
                pose.Pose.Pose.Orientation.X, pose.Pose.Pose.Orientation.Y, ...
                pose.Pose.Pose.Orientation.Z]);
        
            % Extract the current yaw angle
            CurrentRotation = currentOrientation(1);
        
            delta_yaw = targetYaw - CurrentRotation;
            delta_yaw_deg = rad2deg(delta_yaw);
        
            % Set the angular velocity to make the TurtleBot rotate
            cmd_msg.Angular.Z = turningSpeed; % Adjust the value as needed
            if delta_yaw_deg < 0
                % If rotation is negative, swap direction
                cmd_msg.Angular.Z = -cmd_msg.Angular.Z;
            end
        
        
            %Now that we are facing the correct direction, start the drive
            %but consider slippage in the rotation
        
            distance = sqrt(delta_x * delta_x + delta_y * delta_y);
        
            
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
        
                distance = sqrt(delta_x * delta_x + delta_y * delta_y);
        
                % Calculate the updated angle difference
                delta_yaw = targetYaw - CurrentRotation;
                if delta_yaw > pi * 2
                    delta_yaw = delta_yaw - pi * 2;
                end
        
                delta_yaw_deg = rad2deg(delta_yaw);
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

                % self.CalculateNormal(self);

                pause(0.02);
            end
        
            %stop the robot by sending linear velocity command
            cmd_msg.Linear.X = 0;
            cmd_msg.Angular.Z = 0;
            send(cmd_vel_pub, cmd_msg);
            
        end

        %Store all necessary values as properties within the class rather than
        %trying to pass them in and out of functions
        function move = CalculateNormal(self)
            % disp("QR Code not found, turning X amount and trying again");
            [msg2,status,statustext] = receive(self.camera_rgb_,10);
            msg2 = readImage(msg2);
            grey_image = rgb2gray(msg2);
            image_size = size(grey_image);
            cropped_image = imcrop(grey_image, [0 image_size(1,1)*3/4  image_size(1,2) image_size(1,1)]);
            edges = edge(cropped_image);
            [y_points, x_points] = find(edges);

            largest_x_point = 0;
            largest_y_point = 0;
            
            for i = 1:size(x_points)
                if x_points(i) > largest_x_point
                    largest_x_point = x_points(i);
                end
            end
            
            for i = 1:size(y_points)
                if y_points(i) > largest_y_point
                    largest_y_point = y_points(i);
                end
            end

            lowest_x_point = largest_x_point;
            lowest_y_point = largest_y_point;

            for i = 1:size(x_points)
                if x_points(i) < lowest_x_point
                    lowest_x_point = x_points(i);
                end
            end
            for i = 1:size(y_points)
                if y_points(i) < lowest_y_point
                    lowest_y_point = y_points(i);
                end
            end

            line_centre_x = (lowest_x_point+largest_x_point)/2;
            line_centre_y = (lowest_y_point+largest_y_point)/2;
            
            cropped_image_size = size(cropped_image);

            robot_centre_x = cropped_image_size(1,2)/2;
            robot_centre_y = cropped_image_size(1,1)/2;
            
            if (robot_centre_x - line_centre_x) < -100
                move = 1;%'turn right';
            elseif (robot_centre_x - line_centre_x) > 100
                move = 2;%'turn left';
            else
                move = 0;%'straight';
            end 
            % disp(robot_centre_x - line_centre_x)
            disp(move)
              
        end

        % Send a command to increase or decrease velocity and angular
        function sendBotVel(linear_x, linear_y, angular_x, angular_y)
            bot_vel = rosmessage('geometry_msgs/Twist');
            bot_vel.twist.twist.linear.x = linear_x;
            bot_vel.twist.twist.linear.y = linear_y;
            bot_vel.twist.twist.angular.x = angular_x;
            bot_vel.twist.twist.angular.y = angular_y;
            send(pub_vel,bot_vel);
        end
        
        % Move the robot back a little bit
        function MoveBack(self, linear_x)
            bot_vel = rosmessage('geometry_msgs/Twist');
            bot_vel.Linear.X = linear_x;
            for i = 1:10
                send(self.pub_vel, bot_vel);
                %pause(2);
            end
        end
        
        % Stop the robot movement 
        function StopRobot(self) %automatically merge. Don’t worry, 
            % Create a Twist message for the desired velocity command
            cmd_msg = rosmessage(self.pub_vel);
            % Stop the TurtleBot by sending a zero angular velocity command
            cmd_msg.Angular.Z = 0;
            send(self.pub_vel, cmd_msg);
        end

        % detect obstacle in front of the robot both vertical and
        % horizontal
        function [isObstacle, distance, obstacle_location] = detectFrontObstacle(self)
            scan_msg = receive(self.lidar_, 10); % wait up to 10 seconds for data
            % Retrieve the robot's current pose
            pose_msg = receive(self.odom_, 10);
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
                % DriveTo(x_global-0.3, y_global)
            else
                isObstacle = false;
                distance = min_front_distance;
                obstacle_location = [NaN, NaN]; % No obstacle detected, so return NaN for location
            end
        end

        % Detect Square object in front of robot
        function [zoomedROI,  detect] = detectSquare(self)
            % Detect Harris corner points
            I = rgb2gray(self.ImageSub);
            cornerPoints1 = detectHarrisFeatures(I);
            locations = cornerPoints1.Location;
        
            % Sort by y-value
            [~, sortedIndices] = sort(locations(:, 2));
        
            % Take top and bottom subsets
            numPoints = size(locations, 1);
            topSubset = sortedIndices(1:round(0.05 * numPoints)); 
            bottomSubset = sortedIndices(end-round(0.05 * numPoints)+1:end);
        
            % Determine top-left, top-right, bottom-left, bottom-right
            [~, idxTopLeft] = min(locations(topSubset, 1));
            [~, idxTopRight] = max(locations(topSubset, 1));
            [~, idxBottomLeft] = min(locations(bottomSubset, 1));
            [~, idxBottomRight] = max(locations(bottomSubset, 1));
        
            topLeft = locations(topSubset(idxTopLeft), :);
            topRight = locations(topSubset(idxTopRight), :);
            bottomLeft = locations(bottomSubset(idxBottomLeft), :);
            bottomRight = locations(bottomSubset(idxBottomRight), :);
        
            % Initialize zoomedROI as an empty array
            zoomedROI = [];
            detect = false;
            % Check if top corner points (topLeft and topRight) are detected
            if ~isempty(topLeft) && ~isempty(topRight)
                % Calculate orientation based on the top line
                dy = topRight(2) - topLeft(2);
                dx = topRight(1) - topLeft(1);
                angleRadians = atan2(dy, dx);
                angleDegrees = rad2deg(angleRadians);
                rotationAngle = angleDegrees;
                correctedImage = imrotate(I, rotationAngle, 'bicubic', 'crop');
        
                % Calculate the bounding box of the detected corners
                minX = min([topLeft(1), topRight(1), bottomLeft(1), bottomRight(1)]);
                maxX = max([topLeft(1), topRight(1), bottomLeft(1), bottomRight(1)]);
                minY = min([topLeft(2), topRight(2), bottomLeft(2), bottomRight(2)]);
                maxY = max([topLeft(2), topRight(2), bottomLeft(2), bottomRight(2)]);
        
                % Define a region of interest (ROI) based on the bounding box
                roi = [minX, minY, maxX - minX, maxY - minY];
        
                % Extract the ROI from the corrected image
                zoomedROI = imcrop(correctedImage, roi);
                detect = true;
            end
        end

        function [isObstacle, distances, grouped_obstacle_locations] = detectAllObstacles(self)
            scan_msg = receive(self.lidar_, 10); % wait up to 10 seconds for data
            % Retrieve the robot's current pose
            pose_msg = receive(self.odom_, 10);
            % Get the range and angle information
            ranges = scan_msg.Ranges;
            angle_increment = scan_msg.AngleIncrement;
            angle_min = scan_msg.AngleMin;
            
            % Threshold distance to consider an obstacle
            threshold_distance = 2;
            
            obstacle_indices = find(ranges < threshold_distance);
            isObstacle = ~isempty(obstacle_indices); % Return true if any obstacle detected, otherwise false
        
            distances = ranges(obstacle_indices);
            obstacle_locations = [];
            
            robot_x = pose_msg.Pose.Pose.Position.X;
            robot_y = pose_msg.Pose.Pose.Position.Y;
            robot_orientation = quat2eul([pose_msg.Pose.Pose.Orientation.W, pose_msg.Pose.Pose.Orientation.X, pose_msg.Pose.Pose.Orientation.Y, pose_msg.Pose.Pose.Orientation.Z]);
            yaw = robot_orientation(1);
        
            for i = 1:length(obstacle_indices)
                idx = obstacle_indices(i);
                distance = ranges(idx);
                obstacle_angle = angle_min + (idx-1) * angle_increment;
                
                % Calculate obstacle's position in the robot's frame
                x_local = distance * cos(obstacle_angle);
                y_local = distance * sin(obstacle_angle);
                % Convert the obstacle's location to the global frame
                x_global = robot_x + x_local*cos(yaw) - y_local*sin(yaw);
                y_global = robot_y + x_local*sin(yaw) + y_local*cos(yaw);
                obstacle_locations = [obstacle_locations; x_global, y_global];
            end
        
            % Grouping close locations
            proximity_threshold = 0.2;
            is_visited = zeros(size(obstacle_locations, 1), 1);
            grouped_obstacle_locations = [];
        
            for i = 1:size(obstacle_locations, 1)
                if ~is_visited(i)
                    is_visited(i) = 1;
                    current_point = obstacle_locations(i, :);
                    nearby_points = current_point; % Start with the current point
                    
                    % Check for points close to the current one
                    for j = 1:size(obstacle_locations, 1)
                        if ~is_visited(j)
                            distance = norm(obstacle_locations(j, :) - current_point);
                            if distance < proximity_threshold
                                is_visited(j) = 1;
                                nearby_points = [nearby_points; obstacle_locations(j, :)];
                            end
                        end
                    end
                    
                    % Average the nearby points to get a representative point
                    avg_point = mean(nearby_points, 1);
                    grouped_obstacle_locations = [grouped_obstacle_locations; avg_point];
                end
            end
        end


    end

end