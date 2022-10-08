classdef turtlebot_follower
    properties
        RobotCmd;
        OdomSub;
        LidarSub;
        CameraRgbSub;

        MarkerImg;
        Intrinsics;
        MarkerSize = 0.09;
    end
    methods
        function obj = turtlebot_follower()
            rosinit()
            
            focalLength    = [554 554]; 
            principalPoint = [320 240];
            imageSize      = [480 640];
            obj.Intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize); % would be better to get from camera_info topic

            obj.MarkerImg = rgb2gray (imread('../meshes/0.png'));

            obj.RobotCmd = rospublisher("/robot1/cmd_vel","DataFormat","struct");
            obj.OdomSub = rossubscriber("/robot1/odom","DataFormat","struct");
            obj.LidarSub = rossubscriber("/robot1/scan","DataFormat","struct");
            obj.CameraRgbSub = rossubscriber("/robot1/camera/rgb/image_raw","DataFormat","struct");
        end

        function FollowTheLeader(obj)
            tic;
            %r = rosrate(100);
            followLeader = true;
           
            %reset(r);
            while followLeader

                [markerPresent, pose] = AnalyseImage(obj);

                if markerPresent
                    % MoveTowardsMarker(obj, pose);
                else
                    % Do nothing
                end

                % stop after 5 minutes
                if toc > 5*60
                   followLeader = false;
                end
            end
        end

        function MoveTowardsMarker(obj, pose)
            goalPose = DetermineGoalPose(obj, pose);
            cmdVel = DetermineCmdVelocity(obj, goalPose);
            PublishCmdVelocity(obj, cmdVel);
        end

        function cmdVel = DetermineCmdVelocity(obj, goalPose)
            x_l = 0;
            y_l = 0;
            z_l = 0;
            x_a = 0;
            y_a = 0;
            z_a = 0; % TODO

            cmdVel = [x_l y_l z_l x_a y_a z_a];
        end

        function goalPose = DetermineGoalPose(obj, pose)
            % TODO
        end

        function [markerPresent,pose] = AnalyseImage(obj)
            rgbImgMsg = RobotCameraRgbCallback(obj);
            rgbImg = rosReadImage(rgbImgMsg);
            grayImage = rgb2gray(rgbImg);

            % adjust the image to allow for easier detection
            refinedImage = imadjust(grayImage);
            refinedImage = imlocalbrighten(refinedImage);
            refinedImage(refinedImage >= 10) = 255; % make grey pixels white to increase contrast
            %imshow(refinedImage);

            % april tag     
            I = undistortImage(rgbImg,obj.Intrinsics,OutputView="same");
            [id,loc,pose] = readAprilTag(refinedImage,"tag36h11", obj.Intrinsics,obj.MarkerSize)
            
            worldPoints = [0 0 0; obj.MarkerSize/2 0 0; 0 obj.MarkerSize/2 0; 0 0 obj.MarkerSize/2]
            for i = 1:length(pose)
                % Get image coordinates for axes.
                imagePoints = worldToImage(obj.Intrinsics,pose(i),worldPoints);
            
                % Draw colored axes.
                I = insertShape(I,Line=[imagePoints(1,:) imagePoints(2,:); ...
                    imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], ...
                    Color=["red","green","blue"],LineWidth=7);
            
                I = insertText(I,loc(1,:,i),id(i),BoxOpacity=1,FontSize=25);
            end

            figure;
            imshow(I);

            if isempty(id)
                markerPresent = false;
                disp("Marker was not detected");
            else 
                markerPresent = true;
                pose = pose(length(pose));
                disp("Marker detected at");
                disp(pose);
            end
        end

        function ShutdownRos(obj)
            clear
            rosshutdown
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

        function rbgImgMsg = RobotCameraRgbCallback(obj)
            rbgImgMsg = receive(obj.CameraRgbSub);
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

