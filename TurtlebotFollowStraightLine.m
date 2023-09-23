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


    end


    function main(self)
        
        %Clear all, close all etc
    
    
        %Initialise ROS
    
        display("1. Searching for QR Code");
        %While QR not found, 
        while (not(QRCodeFound))
            ScanForQR(self);
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




    %If the QR code can be found, the change this 'QRCodeFound' to true
    function ScanForQR(self)
        %TODO Store QR location
        
    
    
    
        %If QR not found
        %Rotate45CC(self)

        
                
    end
    
    
    
    %Rotate the robot 45 degrees counter-clockwise. 
    function Rotate45CC(self)
     disp("2. QR Code not found, turning X amount and trying again");
    
    
    
    end


    %Store all necessary values as properties within the class rather than
    %trying to pass them in and out of functions
    function CalculateNormal(self)
     disp("QR Code not found, turning X amount and trying again");
    
    
    
    end




end