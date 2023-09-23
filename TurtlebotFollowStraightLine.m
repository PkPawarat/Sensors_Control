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
    pub_vel = rospublisher('/cmd_vel','DataFormat','geometry_msgs/Twist');
    odom_ = rossubscriber('/odom','DataFormat','struct');
    lidar_ = rossubscriber('/scan','DataFormat','struct');
    camera_rgb_ = rossubscriber('/camera/rgb/image_raw');

    

    turn_duration = 5; % Turning duration in seconds
    end


    function main(self)
        
        %Clear all, close all etc
    
    
        %Initialise ROS
        figure
        lidar_scan_data = receive(lidar_, 1);
        rosPlot(lidar_scan_data,"MaximumRange",10);


        display("1. Searching for QR Code");
        %While QR not found, 
        % while (not(QRCodeFound))
        %     ScanForQR(self);
        %     pause(5);
        % end
        ScanForQR(self); % We can just use this to do a 360 scan
        disp("1. QR Code found.");

        disp("3. Calculating normal line from QR code");
        CalculateNormal(self);

        disp("4. Turning robot towards normal line");



        disp("4. Driving robot towards normal line");


        disp("5. Rotating robot towards QR");

        %Do a cross check here to make sure we are in line
        disp("6. Cross check position");



    end




    %If the QR code can be found, the change this 'QRCodeFound' to true
    function ScanForQR(self)
        QRCode_found = false;
        %TODO Store QR location
        for i = 0:45:360
            rgb_image = receive(camera_rgb_,10);
            grey_image = rgb2gray(rgb_image);
            % Logic for identifying QR code
            if %QR found
                break;
            end
            %If QR not found
            Rotate45CC(self);
            pause(5);
        end                
    end
    
    
    
    %Rotate the robot 45 degrees counter-clockwise. 
    function Rotate45CC(self)
        disp("2. QR Code not found, turning X amount and trying again");
        for i = 1:turning_duration-1
            sendBotVel(0, 0, 0.157, 0);
        end    
        sendBotVel(0, 0, 0, 0);
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