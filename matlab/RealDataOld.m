eval('Model');
functions = Functions;
data=functions.pipeline_import('BAG2',2.2180);
beta0 = [0 0 0 0 0 0 0];
% betaA = [0.1818    0.2009    0.1682   -2.6178    3.2854    1.0292   -0.1589];
% beta0 = [0.1648    0.0188    0.1036   -2.1840    3.6191    1.1319]; % Trans then Rot
% beta1 = [0.0736    0.3127    0.2715   -2.4687    2.5581    1.8665   -0.2368];
% beta1 = [0.0742    0.3259    0.2718   -2.4682    2.5589    1.8655   -0.2180];
F = [735.4809         0  388.9476 0; ...
         0  733.6047  292.0895 0; ...
         0         0    1.0000 0];
l2norm = @(a,b)sqrt((a(1)-b(1)).^2 + (a(2)-b(2)).^2);
l2norm3 = @(a,b)sqrt((a(1)-b(1)).^2 + (a(2)-b(2)).^2 + (a(3)-b(3)).^2);
lineDistance = @(p1,p2,p3) abs( (p2(2)-p1(2))*p3(1) - (p2(1)-p1(1))*p3(2) + p2(1)*p1(2) - p2(2)*p1(1))/sqrt( (p2(2)-p1(2)).^2 + (p2(1)-p1(1)).^2 );
     
% Extract data
SON = data('sonarRTheta');
CAM = data('cameraPoint');
RPY = data('imuEuler');
DEPTH = data('correctedDepth');
REALDEPTH = data('depth');

% Distortion correction
smallestDist = 1000;
smallestIndex = 0; 
for i = 1:(size(CAM,1)),
   sD = l2norm(CAM(i,:),[389 292]);
   if sD < smallestDist,
       smallestDist = sD;
       smallestIndex = i;
   end
   CAM(i,:) = functions.pointUndistort(CAM(i,:));
end

% Removal of outliers
Data = [];
DistMax = 200;
PitchMax = 5;
Opt = 1;
count = size(data('sonarRTheta'),1);
for i = 1:count,
    
    % Build Data
    RSonar = SON(i,1);
    ThetaSonar = SON(i,2);
    YSonar = DEPTH(i,1);
    CamPoint = CAM(i,:);
    Roll = RPY(i,1)*-1;
    Pitch = RPY(i,2)*-1;
    Yaw = RPY(i,3);
    CurrData = [RSonar ThetaSonar YSonar CamPoint(1) CamPoint(2) Roll Pitch Yaw];
    
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
start=1;
skip=1;
for i = start:skip:count,
    
    CurrData = Data(i,:);
    RSonar = CurrData(1); ThetaSonar = CurrData(2); YSonar = CurrData(3);
    CamPoint = [CurrData(4) CurrData(5)];
    FinalData = [FinalData; CurrData];
    
end

% Optimization
if Opt,
    y = zeros([size(FinalData,1), 1]);
    opts = statset('Display','iter','TolFun',1e-5);
    mdl = fitnlm(FinalData,y,fullModel,beta0,'Options',opts);
    beta1 = mdl.Coefficients.Estimate;
    newErr = fullModel(beta1, FinalData);
    oldErr = fullModel(beta0, FinalData);
else,
    newErr = fullModel(beta1, FinalData);
    oldErr = fullModel(beta0, FinalData);
    hold on;
    plot(oldErr,'Color','red');
    plot(newErr,'Color','blue');
    hold off;
end

fag

% Skip or view data
distErrArr = [];
count = size(FinalData,1);
start=1;
figure(1);
for i = start:count,
    
    CurrData = FinalData(i,:);
    RSonar = CurrData(1); ThetaSonar = CurrData(2); YSonar = CurrData(3);
    CamPoint = [CurrData(4) CurrData(5)];
    
    YSonarCalculated = ySonarThroughV(RSonar, ThetaSonar, beta1(7), functions.GetRT(beta1(4),beta1(5),beta1(6),beta1(1),beta1(2),beta1(3)), CamPoint(2));
    PixelCalculated = [xModel(beta1, CurrData) yModel(beta1, CurrData)];
    
    SPACEREAL = [
        sqrt(RSonar.^2 - YSonar.^2)*sind(ThetaSonar);
        YSonar
        sqrt(RSonar.^2 - YSonar.^2)*cosd(ThetaSonar);
        1
    ];
    SPACECALCULATED = [
        sqrt(RSonar.^2 - YSonarCalculated.^2)*sind(ThetaSonar);
        YSonarCalculated
        sqrt(RSonar.^2 - YSonarCalculated.^2)*cosd(ThetaSonar);
        1
    ];
%     functions.Draw3D(SPACEREAL(1),SPACEREAL(2),SPACEREAL(3),'red');
%     functions.Draw3D(SPACECALCULATED(1),SPACECALCULATED(2),SPACECALCULATED(3),'blue');
%     line([SPACEREAL(1) SPACECALCULATED(1)], [SPACEREAL(2) SPACECALCULATED(2)], [SPACEREAL(3) SPACECALCULATED(3)], 'color', 'red');
%     functions.Draw2D(CamPoint(1),CamPoint(2),'red');
%     functions.Draw2D(PixelCalculated(1), PixelCalculated(2), 'blue');
%     line([PixelCalculated(1) CamPoint(1)], [PixelCalculated(2) CamPoint(2)], 'color', 'red');
%     pause(0.1);
    
    distErr = l2norm3(SPACEREAL, SPACECALCULATED);
    distErrArr = [distErrArr; distErr];

end