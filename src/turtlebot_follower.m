classdef turtlebot_follower
    properties
        RobotCmd;
        OdomSub;
        LidarSub;
        CameraRgbSub;
        PoseSub;

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
            obj.PoseSub = rossubscriber("/robot2/odom","DataFormat","struct");
            obj.LidarSub = rossubscriber("/robot1/scan","DataFormat","struct");
            obj.CameraRgbSub = rossubscriber("/robot1/camera/rgb/image_raw","DataFormat","struct");
        end

        function FollowTheLeader(obj)
            tic;
            %r = rosrate(100);
            followLeader = true;
            timer = 5;
            readAprilTagTime = timer;
            markerPresent = 0;
            %reset(r);
            while followLeader
                % get values from robot
                % every ten seconds get new image
                if toc > readAprilTagTime
                rgbImgMsg = RobotCameraRgbCallback(obj);

                currentOdom = OdomCallback(obj);
                robotPose = currentOdom.Pose.Pose;

                [markerPresent, pose] = AnalyseImage(obj, rgbImgMsg, robotPose);

                readAprilTagTime = toc+timer;
                disp("Image Read")
                else
                    % do nothing
                end

                if markerPresent
                   MoveTowardsMarker(obj, pose, robotPose);
                else
                    velocities = [0,0,0,0,0,0];
                    PublishCmdVelocity(obj, velocities); % stand still if marker not present
                end

                % stop after 5 minutes
                if toc > 5*60
                   followLeader = false;
                end
            end
        end

        function MoveTowardsMarker(obj, pose, robotPose)
            
            goalPose = DetermineGoalPose(obj, pose);
            cmdVel = DetermineCmdVelocity(obj, goalPose, robotPose); 
            PublishCmdVelocity(obj, cmdVel);
        end

        function cmdVel = DetermineCmdVelocity(obj, goalPose, currentPose)
            cmdVel = [0 0 0 0 0 0];
            % proportional controller
            kp = 0.1;

            quat = currentPose.Orientation;
            angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
            theta = rad2deg(angles(1));

            xDiff = goalPose.Position.X - currentPose.Position.X;
            yDiff = goalPose.Position.Y - currentPose.Position.Y;
            angularError = atan2(yDiff,xDiff);

            linearError = sqrt(pow2(goalPose.Position.X - currentPose.Position.X) ...
                + pow2(goalPose.Position.Y - currentPose.Position.Y));

            pAng = kp * angularError;
            pLin = kp * linearError;

            if abs(angularError - theta) > 0.1
                cmdVel(1,6) = 0.3*pAng;
            elseif linearError > 0.5
                cmdVel(1,1) = 0.5*pLin;
            else
                cmdVel(1,6) = 0;
            end
        end

        function goalPose = DetermineGoalPose(obj, pose)
            % pose = pose from AR Tag
            % translate this pose to be about 1m away from leader turtlebot
            translate_x = 3;
            translate_y = 0;
            translate_z = 0;
            theta = 0;

            % convert from rigid3d to ros Pose
            goalPose = rosmessage("geometry_msgs/Pose","DataFormat","struct");
            goalPose.Position.X = pose(1,4)+translate_x;
            goalPose.Position.Y = pose(2,4)+translate_y;
            goalPose.Position.Z = pose(3,4)+translate_z;
            goalPose.Orientation = rotm2quat(pose(1:3,1:3));
        end

        function [markerPresent,worldPose] = AnalyseImage(obj, rgbImgMsg, robotPose)
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
            
            if isempty(id)
                markerPresent = false;
                worldPose = pose;
                disp("Marker was not detected");
            else 
                % rotate axis to coinside with robot frame
                rotation = eul2rotm([0 0 -pi/2]);
                tform = rigid3d(rotation,[0 0 0]);
                updatedR = pose.Rotation * tform.Rotation;
                pose = rigid3d(updatedR, pose.Translation);

                % display tag axis
                worldPoints = [0 0 0; obj.MarkerSize/2 0 0; 0 obj.MarkerSize/2 0; 0 0 obj.MarkerSize/2]
                %for i = 1:length(pose)
                i = 1
                    % Get image coordinates for axes.
                    imagePoints = worldToImage(obj.Intrinsics,pose(i),worldPoints);
                
                    % Draw colored axes.
                    I = insertShape(I,Line=[imagePoints(1,:) imagePoints(2,:); ...
                        imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], ...
                        Color=["red","green","blue"],LineWidth=7);
                
                    I = insertText(I,loc(1,:,i),id(i),BoxOpacity=1,FontSize=12);
                %end
    
                % find central tag image point
                uMin = min(loc(:,1));
                uMax = max(loc(:,1));
                vMin = min(loc(:,2));
                vMax = max(loc(:,2));
    
                centerPoint = [mean([uMax uMin]) mean([vMax vMin])];
                I = insertMarker(I,centerPoint,"circle","Size",10);
    
                figure(1);
                imshow(I);
    
                % convert image point to 3d points
                depth = 0; % get from sensor
                translation = [depth ...
                    depth * (centerPoint(1)-obj.Intrinsics.PrincipalPoint(1)/obj.Intrinsics.FocalLength(1)) ...
                    depth * (centerPoint(2)-obj.Intrinsics.PrincipalPoint(2)/obj.Intrinsics.FocalLength(2))];

                poseM = eul2rotm([0 0 0]);
                poseM(1:3,4) = translation';
                poseM(4,4) = 1;
    
                quat = robotPose.Orientation;
                worldPoseTr = quat2rotm([quat.W quat.X quat.Y quat.Z]);
                worldPoseTr(1:3,4) = [robotPose.Position.X;robotPose.Position.Y;robotPose.Position.Z];
                worldPoseTr(4,4) = 1;
    
                worldPose = inv(worldPoseTr) * poseM;

                markerPresent = true;
                disp("Marker detected at");
                disp(worldPose);
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

