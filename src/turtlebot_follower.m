classdef turtlebot_follower
    properties
        RobotCmd;
        OdomSub;
        LidarSub;
        PoseSub;
    end
    methods
        function obj = turtlebot_follower()
            rosinit()

            obj.RobotCmd = rospublisher("/robot1/cmd_vel","DataFormat","struct");
            obj.OdomSub = rossubscriber("/robot1/odom","DataFormat","struct");
            obj.LidarSub = rossubscriber("/robot1/scan","DataFormat","struct");
            obj.PoseSub = rossubscriber("/robot2/odom","DataFormat","struct");
        end

        function ShutdownRos(obj)
            clear
            rosshutdown
        end

        function poseMsg = PoseCallback(obj) %standin for calling pose
            poseMsg = receive(obj.PoseSub,3);
            pose = poseMsg.Pose.Pose;
            x = pose.Position.X;
            y = pose.Position.Y;
            z = pose.Position.Z;
            
            % display x, y, z values
            [x y z] 
            
            quat = pose.Orientation;
            angles = quat2eul([quat.W quat.X quat.Y quat.Z]);

            % Rotation matrix
            R = [ 1-2*(quart.Y^2)-2*(quart.Z^2)        2*quart.X*quart.Y-2*quart.Z*quart.W  2*quart.X*quart.Z-2*quart.Y*quart.W; ...
                  2*quart.X*quart.Y-2*quart.Z*quart.W  1-2*(quart.X^2)-2*(quart.Z^2)        2*quart.Y*quart.Z-2*quart.X*quart.W; ...
                  2*quart.X*quart.Z-2*quart.Y*quart.W  2*quart.Y*quart.Z-2*quart.X*quart.W  1-2*(quart.X^2)-2*(quart.Y^2)        ]
            
            % display orientation
            theta = rad2deg(angles(1))
        end

        function goalPose = GoalPoseCallback(obj, poseMsg)
            pose = poseMsg.Pose.Pose;
            
        end

        function velMsg = GenerateVelocityMessage(obj, velocities)
            velMsg = rosmessage(obj.RobotCmd);
            velMsg.Linear.X = velocities(1,1);
            velMsg.Linear.Y = velocities(1,2);
            velMsg.Linear.Z = velocities(1,3);
            velMsg.Angular.X = velocities(1,4);
            velMsg.Angular.Y = velocities(1,5);
            velMsg.Angular.Z = velocities(1,6);
        end

        function PublishCmdVelocity(obj, velocities)
            velMsg = GenerateVelocityMessage(obj, velocities);
            send(obj.RobotCmd,velMsg)
        end

        function odomMsg = OdomCallback(obj)

            odomMsg = receive(obj.OdomSub,3);
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
        end

        function LidarCallback(obj)

            scanMsg = receive(obj.LidarSub);
            figure(1)
            rosPlot(scanMsg)
        end
    end
end

