%% Import data
myfuns = Functions;
[sonarPoint,sonarRTheta,cameraPoint,dvl,imuEuler,imuQuart,depth,brightness] = ...
myfuns.pipeline_import('MC.txt');
Mid = 2.25;
load('SonarMapping.mat');

%% Iterate
start = 1;
test = [];
for i = start:(size(sonarRTheta,1)),
    
    % 3D Point
    SON = sonarPoint(i,:);
    CAM = cameraPoint(i,:);
    RVal = sonarRTheta(i,1);
    TVal = sonarRTheta(i,2);
    Roll = imuEuler(i,1)*-1;
    Pitch = imuEuler(i,2)*-1;
    Yaw = imuEuler(i,3);
    
    Pitch = imuEuler(i,2);
    if abs(Pitch) > 1.5,
        continue
    end
    test = [test [RVal(1),Pitch]];
    
end

size(test)
plot(test)