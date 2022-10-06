classdef turtlebot_follower
    properties
        RobotCmd;
        OdomSub;
        LidarSub;
        CameraRgbSub;
        DepthSub;

        MarkerImg;
    end
    methods
        function obj = turtlebot_follower()
            rosinit()

            obj.MarkerImg = rgb2gray (imread('../meshes/0.png'));

            obj.RobotCmd = rospublisher("/robot1/cmd_vel","DataFormat","struct");
            obj.OdomSub = rossubscriber("/robot1/odom","DataFormat","struct");
            obj.LidarSub = rossubscriber("/robot1/scan","DataFormat","struct");
            obj.CameraRgbSub = rossubscriber("/robot1/camera/rgb/image_raw","DataFormat","struct");
            obj.DepthSub = rossubscriber("/robot1/camera/rgb/image_raw/compressedDepth","DataFormat","struct");
        end

        function AnalyseImage(obj)
            rgbImgMsg = RobotCameraRgbCallback(obj);
            rgbImg = rosReadImage(rgbImgMsg);

            figure;
            imshow(rgbImg)

            original = obj.MarkerImg;
            robotCameraImage = rgb2gray (rgbImg);

            figure;
            imshowpair(original,robotCameraImage,'montage');
            
            ptsOriginal = detectSURFFeatures(original);
            ptsRobotCameraImage = detectSURFFeatures(robotCameraImage);
            
            [featuresOriginal, validPtsOriginal] = extractFeatures(original,ptsOriginal);
            [featuresRobotCameraImage, validPtsRobotCameraImage] = extractFeatures(robotCameraImage,ptsRobotCameraImage);
            
            indexPairs = matchFeatures(featuresOriginal,featuresRobotCameraImage);
            matchedOriginal = validPtsOriginal(indexPairs(:,1));
            matchedDistorted = validPtsRobotCameraImage(indexPairs(:,2));
            
            figure;
            showMatchedFeatures(original,robotCameraImage,matchedOriginal,matchedDistorted,'montage');
            
            [tform,inlierDistorted,inlierOriginal] = estimateGeometricTransform (matchedDistorted,matchedOriginal,'similarity');
            
            figure;
            showMatchedFeatures(original,robotCameraImage,inlierOriginal,inlierDistorted,'montage');
            title('Matching point (inliers only)');
            legend ('ptsOriginal','ptsDistorted');
            
            Tinv = tform.invert.T;
            ss = Tinv(2,1);
            sc = Tinv(1,1);
            scaleRecovered = sqrt(ss*ss+sc*sc);
            thetaRecovered = atan2(ss,sc)*180/pi;
            
            outputView = imref2d(size(original));
            recovered = imwarp(robotCameraImage,tform,'OutputView',outputView);
            figure;
            imshowpair(original,recovered,'montage');
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

