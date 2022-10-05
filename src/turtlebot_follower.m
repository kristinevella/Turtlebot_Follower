rosinit()

%% Set velocity variable
velocity = 0.1;

%% Create publisher 
robotCmd = rospublisher("/robot1/cmd_vel","DataFormat","struct");
velMsg = rosmessage(robotCmd);

velMsg.Linear.X = velocity;
send(robotCmd,velMsg)
pause(4)
velMsg.Linear.X = 0;
send(robotCmd,velMsg)

%% Create subscribers
% Receive position and orientation
odomSub = rossubscriber("/robot1/odom","DataFormat","struct");

odomMsg = receive(odomSub,3);
pose = odomMsg.Pose.Pose;
x = pose.Position.X;
y = pose.Position.Y;
z = pose.Position.Z;

% display x, y, z values
[x y z] 

quat = pose.Orientation;
angles = quat2eul([quat.W quat.X quat.Y quat.Z]);

% display orientation
theta = rad2deg(angles(1))

% Recieve lidar data 
lidarSub = rossubscriber("/robot1/scan","DataFormat","struct");

scanMsg = receive(lidarSub);
figure
rosPlot(scanMsg)

velMsg.Angular.Z = velocity;
send(robotCmd,velMsg)
tic
while toc < 20
  scanMsg = receive(lidarSub);
  rosPlot(scanMsg)
end

velMsg.Angular.Z = 0;
send(robotCmd,velMsg)

%% Disconnect
clear 
rosshutdown