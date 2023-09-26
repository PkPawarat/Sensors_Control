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
    
    ImageSub;
    VideoPlay;
    velPub;
    velMsg;

    QRCodeOrder = ['1. Follow me' '2. Follow me' '3. Follow me']
    GetOrder = 1;

    end


    function main(self)
        
        %Clear all, close all etc
    
    
        %Initialise ROS
    
        display("1. Searching for QR Code");
        %While QR not found, 
        while (not(QRCodeFound))
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

    
    function SetupRos(self)
        %% SETUP
        % Connect to ROS master
        rosshutdown;
        % turtlebotIp = '172.19.115.28';
        % rosinit(turtlebotIp);

        % Create ROS subscribers and publishers
        self.ImageSub = rossubscriber('/camera/rgb/image_raw');
        receive(self.ImageSub,10); % Wait to receive first message
        [self.velPub,self.velMsg] = rospublisher('/cmd_vel');

        % Create video player for visualization
        self.VideoPlay = vision.DeployableVideoPlayer;

        % Load control parameters
        % params = turtlebotcontrol;
    end




     % If the QR code can be found, then change 'QRCodeFound' to true
    function ScanForQR(self)
        % TODO: Store QR location
        clc;
        ImageSub = imread("Test3.png");
        [msg, detectedFormat, loc] = readBarcode(img);
        disp(msg);
        disp(detectedFormat);
        disp(loc);

        currentQRCodeTarget = self.QRCodeOrder(self.GetOrder);
        if currentQRCodeTarget ~= msg
            self.Rotate45CC(self)
            return

        else
            %move to target by checking straighLine
        end
        

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
    end


    %Store all necessary values as properties within the class rather than
    %trying to pass them in and out of functions
    function CalculateNormal(self)
     disp("QR Code not found, turning X amount and trying again");
    
    
    
    end




end