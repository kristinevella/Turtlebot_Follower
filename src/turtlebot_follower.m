
classdef turtlebot_follower
    properties
        RobotCmd;
        OdomSub;
        LidarSub;
        CameraRgbSub;
        DepthSub;
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
            obj.DepthSub = rossubscriber("/robot1/camera/depth/image_raw","DataFormat","struct");
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
                rgbImgMsg = CameraRgbCallback(obj);
                depthMsg = CameraDepthCallback(obj);
                currentOdom = OdomCallback(obj);
                robotPose = currentOdom.Pose.Pose;

                [markerPresent, pose] = AnalyseImage(obj, rgbImgMsg, depthMsg, robotPose);

                readAprilTagTime = toc+timer;
                disp("Image Read")
                else
                    % do nothing
                end

                if markerPresent
                    currentOdom = OdomCallback(obj);
                    robotPose = currentOdom.Pose.Pose;
                    currentLeaderPose = PoseCallback(obj);
                    leaderPose = currentLeaderPose.Pose.Pose;
                    
                    MoveTowardsMarker(obj, leaderPose, robotPose); % leaderPose is temporary, change back to pose when done
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

            quatGoal = goalPose.Orientation;
            angles = quat2eul([quatGoal.W quatGoal.X quatGoal.Y quatGoal.Z]);
            thetaGoal = rad2deg(angles(1));

            quatCurrent = currentPose.Orientation;
            angles = quat2eul([quatCurrent.W quatCurrent.X quatCurrent.Y quatCurrent.Z]);
            thetaCurrent = rad2deg(angles(1));

            xDiff = goalPose.Position.X - currentPose.Position.X
            yDiff = goalPose.Position.Y - currentPose.Position.Y
            angularError = rad2deg(atan2(yDiff,xDiff));
            direction1 = (angularError-thetaCurrent)/(abs(angularError-thetaCurrent));
            direction2 = xDiff/abs(xDiff);
            direction3 = (thetaGoal-thetaCurrent)/(abs(thetaGoal-thetaCurrent));

            if abs(xDiff)<0.05 && abs(yDiff)<0.05
                if abs(thetaGoal-thetaCurrent)<1
                    % at goal and facing correct direction
                    % do nothing
                    cmdVel = [0 0 0 0 0 0];
                else
                    % at goal and not facing correct direction
                    % spin to correct direction
                    cmdVel = [0 0 0 0 0 direction3*0.3];
                end
            else
                if abs(angularError-thetaCurrent)<1
                    % facing direction of goal but not there yet
                    % drive towards goal
                    cmdVel = [direction2*0.5 0 0 0 0 0];
                else
                    % not facing direction of goal and not at goal
                    % turn to face goal
                    cmdVel = [0 0 0 0 0 direction1*0.3];
                end
            end


%             linearError = sqrt(pow2(goalPose.Position.X - currentPose.Position.X) ...
%                 + pow2(goalPose.Position.Y - currentPose.Position.Y));

%             pAng = kp * angularError;
%             pLin = kp * linearError;

%             if abs(angularError - theta) > 0.1
%                 cmdVel(1,6) = 0.3*pAng;
%             elseif linearError > 0.5
%                 cmdVel(1,1) = 0.5*pLin;
%             else
%                 cmdVel(1,6) = 0;
%             end
        end

        function goalPose = DetermineGoalPose(obj, pose)
            % pose = pose from AR Tag
            % translate this pose to be about 0.5m away from leader turtlebot
            distance = 0.5;
            % find angle of AR tag
            quat = pose.Orientation;
            angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
            theta = rad2deg(angles(1));
            % get x,y distance away based on angle
            translate_x = -distance*cos(angles(1));
            translate_y = -distance*sin(angles(1));

            % convert from rigid3d to ros Pose
            goalPose = rosmessage("geometry_msgs/Pose","DataFormat","struct");
            goalPose.Position.X = pose.Position.X+translate_x;
            goalPose.Position.Y = pose.Position.Y+translate_y;
            goalPose.Position.Z = pose.Position.Z;
%             goalPose.Orientation = rotm2quat(pose(1:3,1:3));  % Add back
%             in later when using AnalyseImage
            goalPose.Orientation = pose.Orientation;
        end

        function [markerPresent,worldPose] = AnalyseImage(obj, rgbImgMsg, depthMsg, robotPose)
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
                i = 1;
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
    
                centerPoint = [round(mean([uMax uMin])) round(mean([vMax vMin]))];
                I = insertMarker(I,centerPoint,"circle","Size",10,"Color","yellow");
    
                figure(1);
                imshow(I);
    
                % convert image point to 3d points
                depthImg = rosReadImage(depthMsg);
                depth = depthImg(centerPoint(1),centerPoint(2)); % get from sensor
                translation = [depth ...
                    depth * (centerPoint(1)-obj.Intrinsics.PrincipalPoint(1))/obj.Intrinsics.FocalLength(1) ...
                    depth * (centerPoint(2)-obj.Intrinsics.PrincipalPoint(2))/obj.Intrinsics.FocalLength(2)];

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

        function rbgImgMsg = CameraRgbCallback(obj)
            rbgImgMsg = receive(obj.CameraRgbSub);
        end

        function depthMsg = CameraDepthCallback(obj)
            depthMsg = receive(obj.DepthSub);
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
            theta = rad2deg(angles(1));
        end

        function poseMsg = PoseCallback(obj) %standin for calling pose
            poseMsg = receive(obj.PoseSub,3);
            pose = poseMsg.Pose.Pose;
            x = pose.Position.X;
            y = pose.Position.Y;
            z = pose.Position.Z;
            
            % display x, y, z values
            [x y z];
            
            quat = pose.Orientation;
            angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
            
            % display orientation
            theta = rad2deg(angles(1));
        end

        function scanMsg = LidarCallback(obj)
            scanMsg = receive(obj.LidarSub);
            figure(1)
            rosPlot(scanMsg)
        end
    end
end

