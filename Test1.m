% % Initialize ROS
% rosshutdown;
% rosinit;
% 
% rostopic list
% rostopic info /odom
% 
% % Create a ROS subscriber for the desired topic
% sub = rossubscriber("/odom","DataFormat","struct");
% pause(2)
% 
% 
% % Specify the maximum time to wait for a message (in seconds)
% timeout = 10;  % Adjust the timeout as needed
% 
% try
%     % Wait for a message on the specified topic
%     [msg2,status,statustext] = receive(sub,10)
% 
%     % % Process the received message (e.g., access message fields)
%     % disp('Received a message:');
%     % disp(scandata);  % Display the received message
%     % 
%     % % Access specific fields of the message (assuming it's of known type)
%     % % Example: Access the 'Data' field of a custom message
%     % % data = msg.Data;
%     % figure
%     % rosPlot(scandata,"MaximumRange",7)
% 
% catch
%     disp('Failed to receive a message within the specified timeout.');
% end
% 
% % Clean up
% rosshutdown;
%%
ipAddress = '127.0.0.1';
try
    % Initialize ROS node
    rosshutdown;
    rosinit(ipAddress);

    % Create a subscriber
    self.camera_rgb_ = rossubscriber('/camera/rgb/image_raw', 'sensor_msgs/Image');
    
    figure(1);
    
    % while ishandle(1) % Keep running until the figure window is closed
        msg = receive(self.camera_rgb_, 10); % Receive image message
        i = msg.Data;
        % imwrite(readImage(msg), "testimage.png");
        figure(1);
        imshow(readImage(msg)); % Display the image
        disp('DONE');
        % Add a pause to control the refresh rate (e.g., pause(0.1) for 10 Hz)
        pause(10); 
    % end

catch exception
    disp(['Error: ' exception.message]);
end

% Shutdown ROS when finished
rosshutdown;


%%

clc;

cam = webcam(1);
preview(cam);
while(1)
    img = snapshot(cam);
    [msg,detectedFormat,loc] = readBarcode(img);
    disp(msg);
    % disp(loc);
end


%%

img = imread('QR Code/FollowMe1.jpg');

[msg,detectedFormat,loc] = readBarcode(img);
hold on;
imshow(img);
disp(msg);
