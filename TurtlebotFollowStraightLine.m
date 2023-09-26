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



classdef TurtlebotFollowStraightLine < handle

    properties
    QRCodeFound = false; % On startup, this will be false. 
    pub_vel;% = rospublisher('/cmd_vel','DataFormat','geometry_msgs/Twist');
    odom_;% = rossubscriber('/odom','DataFormat','struct');
    lidar_;% = rossubscriber('/scan','DataFormat','struct');
    camera_rgb_;% = rossubscriber('/camera/rgb/image_raw');
    turn_duration = 5; % Turning duration in seconds

    ImageSub;
    VideoPlay;
    velPub;
    velMsg;

    QRCodeOrder = ['1. Follow me' '2. Follow me' '3. Follow me']
    GetOrder = 1;

    end

    methods
        function self = TurtlebotFollowStraightLine()
            
            %Clear all, close all etc
            close all;
            clc;


            %% SETUP
            % Connect to ROS master
            rosshutdown;

            % rosinit(turtlebotIp);
            rosinit;

            self.pub_vel = rospublisher('/cmd_vel','geometry_msgs/Twist');
            self.odom_ = rossubscriber('/odom','DataFormat','struct');
            self.lidar_ = rossubscriber('/scan','DataFormat','struct');
            self.camera_rgb_ = rossubscriber('/camera/rgb/image_raw');
    

            self.ImageSub = rossubscriber('/camera/rgb/image_raw');
            disp('Subscribed to /camera/rgb/image_raw.');
            
            try
                receive(self.ImageSub,10); % Wait to receive first message
            catch
                disp('Fail to read ImageSub.');
            end
            [self.velPub,self.velMsg] = rospublisher('/cmd_vel');
            disp('Initialized ROS publishers and subscribers.');


        
            %Initialise ROS
            figure
            try
                lidar_scan_data = receive(self.lidar_, 100);
            catch
                disp('Fail to read lidar_.');
            end

            rosPlot(lidar_scan_data,"MaximumRange",10);
        
            disp("1. Searching for QR Code");
            %While QR not found, 
            while (~(self.QRCodeFound))
                self.ScanForQR(self);
                pause(5);
            end
            disp("1. QR Code found.");
        
    
    
            disp("3. Calculating normal line from QR code");
            CalculateNormal(self);
    
    
            disp("4. Turning robot towards normal line");
    
    
    
            disp("4. Driving robot towards normal line");
    
    
            disp("5. Rotating robot towards QR");
    
            %Do a cross check here to make sure we are in line
            disp("6. Cross check position");
    
        end
    end

    methods(Static)

         % If the QR code can be found, then change 'QRCodeFound' to true
        function ScanForQR(self)
            % TODO: Store QR location
            % ImageSub = imread("Test3.png");
            % [msg, detectedFormat, loc] = readBarcode(img);
            % % disp(msg);
            % % disp(detectedFormat);
            % % disp(loc);
            % 
            % currentQRCodeTarget = self.QRCodeOrder(self.GetOrder);
            % if currentQRCodeTarget ~= msg
            %     self.Rotate45CC(self)
            %     return
            % 
            % else
            %     %move to target by checking straighLine
            % end

            % for i = 0:45:360
                rgb_image = receive(self.camera_rgb_,10);
                [msg, detectedFormat, loc] = readBarcode(rgb_image);
                % grey_image = rgb2gray(rgb_image);
                % Logic for identifying QR code
                currentQRCodeTarget = self.QRCodeOrder(self.GetOrder);
                if currentQRCodeTarget == msg
                    self.QRCodeFound = true;
                    return;
                end
                %If QR not found
                self.Rotate45CC(self);
                pause(5);
            % end     
            
    
            % If QR not found, you can add your handling logic here
            % For example, call the Rotate45CC function
            % Rotate45CC(self);
        end
        
        
        
        %Rotate the robot 45 degrees counter-clockwise. 
        function Rotate45CC(self)
            disp("2. QR Code not found, turning X amount and trying again");
               
            %CK NOTES
                % May need to take readings of the odom to determine the
                % current pose of the turtlebot and do a while loop to check
                % that we are stopping the rotation at the correct time... 

            % Initialize ROS node in MATLAB
            rosinit;

            % Create a publisher for sending velocity commands
            cmd_vel_pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');

            % Create a Twist message for the desired velocity command
            cmd_msg = rosmessage(cmd_vel_pub);

            % Set the angular velocity to make the TurtleBot rotate
            cmd_msg.Angular.Z = 0.5; % Adjust the value as needed for 45-degree rotation

            % Publish the command
            send(cmd_vel_pub, cmd_msg);

            % Sleep for a duration to rotate the TurtleBot
            pause(10); % Adjust the duration as needed for 45-degree rotation

            % Stop the TurtleBot by sending a zero velocity command
            cmd_msg.Angular.Z = 0;
            send(cmd_vel_pub, cmd_msg);

            % Shutdown the ROS node when done
            rosshutdown;


            %% Lauren part added
            for i = 1:turning_duration-1
                self.sendBotVel(0, 0, 0.157, 0);
            end    
            self.sendBotVel(0, 0, 0, 0);
        end
    
    
        %Store all necessary values as properties within the class rather than
        %trying to pass them in and out of functions
        function CalculateNormal(self)
            disp("QR Code not found, turning X amount and trying again");
            rgb_image = receive(camera_rgb_,10);
            grey_image = rgb2gray(rgb_image);
            % find the normal logic   
        end

        function sendBotVel(linear_x, linear_y, angular_x, angular_y)
            bot_vel = rosmessage('geometry_msgs/Twist');
            bot_vel.twist.twist.linear.x = linear_x;
            bot_vel.twist.twist.linear.y = linear_y;
            bot_vel.twist.twist.angular.x = angular_x;
            bot_vel.twist.twist.angular.y = angular_y;
            send(pub_vel,bot_vel);
        end

    end

end