% Initialize ROS
rosshutdown;
rosinit;

rostopic list
rostopic info /odom

% Create a ROS subscriber for the desired topic
laser = rossubscriber("/odom","DataFormat","struct");
pause(2)


% Specify the maximum time to wait for a message (in seconds)
timeout = 10;  % Adjust the timeout as needed

try
    % Wait for a message on the specified topic
    scandata = receive(laser,10);

    % Process the received message (e.g., access message fields)
    disp('Received a message:');
    disp(scandata);  % Display the received message

    % Access specific fields of the message (assuming it's of known type)
    % Example: Access the 'Data' field of a custom message
    % data = msg.Data;
    figure
    rosPlot(scandata,"MaximumRange",7)

catch
    disp('Failed to receive a message within the specified timeout.');
end

% Clean up
rosshutdown;
