%% 
disp("Test robot angle starting");
RotateRobot(45); 

disp("Test robot angle complete");
%pause(5); 
%disp("Test robot reverse angle starting");

%RotateRobot(-90);

%disp("Test robot reverse angle complete");

%% Drive robot
DriveRobot(0);
disp("Test robot drive starting");
DriveRobot(1); 

disp("Test robot angle complete");
pause(5); 
disp("Test robot reverse angle starting");

DriveRobot(-1);

disp("Test robot reverse angle complete");


%% Stop robot

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

    cmd_msg.Linear.X = 0;
    cmd_msg.Linear.Y = 0;
    cmd_msg.Linear.Z = 0;
    cmd_msg.Angular.X = 0;
    cmd_msg.Angular.Y = 0;
    cmd_msg.Angular.Z = 0;

    send(cmd_vel_pub, cmd_msg);

    % Shutdown the ROS node when done
    rosshutdown;
