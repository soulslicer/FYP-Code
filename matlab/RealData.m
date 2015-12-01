%% Load Learning Model
eval('Model2');
functions = Functions;
l2norm = @(a,b)sqrt((a(1)-b(1)).^2 + (a(2)-b(2)).^2);
l2norm3 = @(a,b)sqrt((a(1)-b(1)).^2 + (a(2)-b(2)).^2 + (a(3)-b(3)).^2);
lineDistance = @(p1,p2,p3) abs( (p2(2)-p1(2))*p3(1) - (p2(1)-p1(1))*p3(2) + p2(1)*p1(2) - p2(2)*p1(1))/sqrt( (p2(2)-p1(2)).^2 + (p2(1)-p1(1)).^2 );
     
%% Extract data
data=functions.pipeline_import('BAG2',2.2180);
SON = data('sonarRTheta');
CAM = data('cameraPoint');
RPY = data('imuEuler');
DEPTH = data('correctedDepth');
REALDEPTH = data('depth');
DVL = data('dvl');

%% Process DVL data
% PROCESSEDDVL = zeros(size(DVL,1),3);
% for i = 2:(size(DVL,1)),
%    if ~(DVL(i-1,1)-DVL(i,1) == 0 && DVL(i-1,2)-DVL(i,2) == 0),
%        PROCESSEDDVL(i,:) = [DVL(i,1)-DVL(i-1,1) DVL(i,2)-DVL(i-1,2) DEPTH(i,1)-DEPTH(i-1,1)]; 
%    end
% end

%% Distortion correction
for i = 1:(size(CAM,1)),
   CAM(i,:) = functions.pointUndistort(CAM(i,:));
end

%% Parameters
% Initial Params (TX TY TZ TRoll TPitch TYaw LY LZ) Sonar WRT Camera
beta0 = [0.1 0.3 0.1 0 0 0 0.5 0.5];
% beta1 = [0.1399    0.2847    0.1484   -1.7687    2.0101    1.3405    0.1843    0.2292];
beta1 = [0.1394    0.2500    0.1413   -1.7862    2.0516    1.3287    0.1460    0.2514];
F = [735.4809         0  388.9476 0; ...
         0  733.6047  292.0895 0; ...
         0         0    1.0000 0];
% Initial thresholds
DistMax = 200;
PitchMax = 20;
% Cut Dataset
start=1;
skip=1;
% Optimization step
Opt = 1;
DrawPixelOptError = 1;
Draw3DOptError = 0;
DrawDistError = 1;

%% Data Preprocessing
% Removal of outliers
Data = [];
count = size(data('sonarRTheta'),1);
for i = 1:count,
    
    % Build Data
    RSonar = SON(i,1);
    ThetaSonar = SON(i,2);
    YVehicle = DEPTH(i,1);
    CamPoint = CAM(i,:);
    Roll = RPY(i,1)*-1;
    Pitch = RPY(i,2)*-1;
    Yaw = RPY(i,3);
    CurrData = [RSonar ThetaSonar YVehicle CamPoint(1) CamPoint(2) Roll Pitch Yaw];
    
    % Outlier removal
    TestPoint = [xModel(beta0, CurrData), yModel(beta0, CurrData)];
    if l2norm(CamPoint, TestPoint) > DistMax,
        continue
    end
    if abs(Pitch) > PitchMax,
        continue
    end
    
    % Assign Data
    Data = [Data; CurrData];
end

% Skip or view data
FinalData = [];
count = size(Data,1);
for i = start:skip:count,
    CurrData = Data(i,:);
    RSonar = CurrData(1); ThetaSonar = CurrData(2); YVehicle = CurrData(3);
    CamPoint = [CurrData(4) CurrData(5)];
    FinalData = [FinalData; CurrData];
end

%% Optimization Step
if Opt,
    y = zeros([size(FinalData,1), 1]);
    opts = statset('Display','iter','TolFun',1e-5);
    mdl = fitnlm(FinalData,y,fullModel,beta0,'Options',opts);
    beta1 = mdl.Coefficients.Estimate;
end
newErr = fullModel(beta1, FinalData);
oldErr = fullModel(beta0, FinalData);
if DrawPixelOptError,
    figure(2);
    title(sprintf('Pixel Distance Error (PDE) from Camera Ground Truth and Sonar \nBefore and After Optimization \nDATASET %d',start));
    xlabel('Data Points');
    ylabel('Pixel Distance Error');
    hold on;
    plot(oldErr,'Color','red');
    plot(newErr,'Color','blue');
    hold off;
    legend('Error Before Optimization','Error After Optimization');
end

%% Calculate 3D Error and Visualize
distErrArr = [];
yErrArr = [];
count = size(FinalData,1);
start=1;
aArr = [];
bArr = [];
if Draw3DOptError,
    figure(1);
    functions.Draw3D(0,0,0,'red');
    functions.Draw3D(0,0,0,'blue');
    title(sprintf('3D Error \nDATASET %d',start));
    legend('Ground Truth','Camera Sonar Estimated', 'Location','north');
    functions.Draw2D(290,390,'red');
    functions.Draw2D(290,390, 'blue');
    title(sprintf('2D Error \nDATASET %d',start));
    xlabel('U Camera');
    ylabel('V Camera');
    legend('Ground Truth','Sonar Estimated');
end
for i = start:count,
    
    % Extract Data
    CurrData = FinalData(i,:);
    RSonar = CurrData(1); ThetaSonar = CurrData(2); YVehicle = CurrData(3); Pitch = CurrData(7);
    CamPoint = [CurrData(4) CurrData(5)];
    
    % Stop for test
%     if uint16(CamPoint(1)) == 365,
%         disp(CamPoint)
%     end
%     if  uint16(CamPoint(2)) == 230 && uint16(CamPoint(1)) == 357,
%         CurrData
%         fag
%     end
    
    % Calculate Ground Truth Point
    YSonar = ySonarThroughYVehicle(RSonar*cosd(ThetaSonar), YVehicle, Pitch, beta1(7), beta1(8));
    SPACESONAR = [
        sqrt(RSonar.^2 - YSonar.^2)*sind(ThetaSonar);
        YSonar
        sqrt(RSonar.^2 - YSonar.^2)*cosd(ThetaSonar);
        1
    ];    
    SPACEVEHICLE = functions.GetRT(0,-Pitch,0,0,0,0)*functions.GetRT(0,0,0,0,beta1(7),beta1(8))*SPACESONAR;
    
    % Calculate Point from Camera
    YSonarCalculated = ySonarThroughV(RSonar, ThetaSonar, functions.GetRT(beta1(4),beta1(5),beta1(6),beta1(1),beta1(2),beta1(3)), CamPoint(2));
    YVehicleCalculated = yVehicleThroughYSonar(RSonar*cosd(ThetaSonar), YSonarCalculated, Pitch, beta1(7), beta1(8));
    PixelCalculated = [xModel(beta1, CurrData) yModel(beta1, CurrData)];
    SPACESONARCALCULATED = [
        sqrt(RSonar.^2 - YSonarCalculated.^2)*sind(ThetaSonar);
        YSonarCalculated
        sqrt(RSonar.^2 - YSonarCalculated.^2)*cosd(ThetaSonar);
        1
    ];    
    SPACEVEHICLECALCULATED = functions.GetRT(0,-Pitch,0,0,0,0)*functions.GetRT(0,0,0,0,beta1(7),beta1(8))*SPACESONARCALCULATED;
    
    % Draw 3D Error
    if Draw3DOptError,
        functions.Draw3D(SPACEVEHICLE(1),SPACEVEHICLE(2),SPACEVEHICLE(3),'red');
        functions.Draw3D(SPACEVEHICLECALCULATED(1),SPACEVEHICLECALCULATED(2),SPACEVEHICLECALCULATED(3),'blue');
        line([SPACEVEHICLE(1) SPACEVEHICLECALCULATED(1)], [SPACEVEHICLE(2) SPACEVEHICLECALCULATED(2)], [SPACEVEHICLE(3) SPACEVEHICLECALCULATED(3)], 'color', 'red');
        functions.Draw2D(CamPoint(1),CamPoint(2),'red');
        functions.Draw2D(PixelCalculated(1), PixelCalculated(2), 'blue');
        line([PixelCalculated(1) CamPoint(1)], [PixelCalculated(2) CamPoint(2)], 'color', 'red');
        pause(0.1);
    end

    % Computer 3D Distance error
    distErr = l2norm3(SPACEVEHICLE, SPACEVEHICLECALCULATED);
    distErrArr = [distErrArr; distErr];
    yErr = abs(YVehicle - YVehicleCalculated);
    yErrArr = [yErrArr; yErr];
    
end

% View 3D Opt Error
if DrawDistError,
    figure(3);
    title(sprintf('3D Distance Error \nDATASET %d',start));
    xlabel('Data Points');
    ylabel('3D Distance Error');
    hold on;
    plot(distErrArr);
    hold off;
end