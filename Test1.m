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

%%
close all
clf
I = imread('test3.png');
I = rgb2gray(I);
% imshow(I)
a = size(I);
crop = imcrop(I, [0 a(1,1)*3/4  a(1,2) a(1,1)]);
imshow(crop)
cornerPoints = detectHarrisFeatures(crop, "MinQuality", 0.2);
BW = edge(crop);
[y, x] = find(BW);
highestx = 0;
highesty = 0;
for i = 1:size(x)
    if x(i) > highestx
        highestx = x(i);
    end
end

for i = 1:size(y)
    if y(i) > highesty
        highesty = y(i);
    end
end
lowestx = highestx;
lowesty = highesty;
for i = 1:size(x)
    if x(i) < lowestx
        lowestx = x(i);
    end
end
for i = 1:size(y)
    if y(i) < lowesty
        lowesty = y(i);
    end
end

centrex = (lowestx+highestx)/2;
centrey = (lowesty+highesty)/2;

imshow (crop)
hold on;
% plot (cornerPoints);
imshow(BW)
plot(centrex, centrey, 'd')

b = size(crop);

robotcentrex = b(1,2)/2;
robotcentrey = b(1,1)/2;

plot(robotcentrex, robotcentrey, 'd')

if (robotcentrex - centrex) < 0
    move = 'turn right'
elseif (robotcentrex - centrex) > 0
    move = 'turn left'
else
    move = 'straight'
end


% Q2
% I1gs = rgb2gray (imread('roofs1.jpg'));
% I2gs = rgb2gray (imread('roofs2.jpg'));
% 
% points1 = detectORBFeatures(I1gs);
% points2 = detectORBFeatures(I2gs);
% 
% [features1, validPoints1] = extractFeatures(I1gs,points1);
% [features2, validPoints2] = extractFeatures(I2gs,points2);
% 
% indexPairs = matchFeatures(features1,features2);
% 
% matchedPoints1 = validPoints1(indexPairs(:,1));
% matchedPoints2 = validPoints2(indexPairs(:,2));
% 
% showMatchedFeatures(I1gs,I2gs,matchedPoints1,matchedPoints2,'montage');

