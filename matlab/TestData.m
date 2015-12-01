eval('Model2');
functions = Functions;
data=functions.generate_data(xModel, yModel, [0.1 0.1 0.2 0 0.2 0 0.5 0.5], 1, 0, 5);
beta0 = [0 0 0 0 0 0 0 0]; % Trans, Rot, YOffset
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

% Generate Data
Data = [];
count = size(data('sonarRTheta'),1);
start=1;
figure(1);
for i = start:count,
    RSonar = SON(i,1);
    ThetaSonar = SON(i,2);
    YSonar = DEPTH(i,1);
    CamPoint = CAM(i,:);
    RPYPoint = RPY(i,:);
    CurrData = [RSonar ThetaSonar YSonar CamPoint(1) CamPoint(2) RPYPoint(1) RPYPoint(2)];
    Data = [Data; CurrData];
end

% Optimization
y = zeros([count, 1]);
opts = statset('Display','iter','TolFun',1e-5);
mdl = fitnlm(Data,y,fullModel,beta0,'Options',opts);
beta1 = mdl.Coefficients.Estimate;
fullErr = fullModel(beta1, Data);

fag

% Solve back
for i = start:count,
    
    RSonar = SON(i,1);
    ThetaSonar = SON(i,2);
    YSonar = DEPTH(i,1);
    CamPoint = CAM(i,:);

    YSonarCalculated = ySonarThroughV(RSonar, ThetaSonar, beta1(7), functions.GetRT(beta1(4),beta1(5),beta1(6),beta1(1),beta1(2),beta1(3)), CamPoint(2));

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
    functions.Draw3D(SPACEREAL(1),SPACEREAL(2),SPACEREAL(3),'red');
    functions.Draw3D(SPACECALCULATED(1),SPACECALCULATED(2),SPACECALCULATED(3),'blue');
    line([SPACEREAL(1) SPACECALCULATED(1)], [SPACEREAL(2) SPACECALCULATED(2)], [SPACEREAL(3) SPACECALCULATED(3)], 'color', 'red');
    functions.Draw2D(CamPoint(1),CamPoint(2),'red');
    pause(0.1);
    
    l2norm3(SPACEREAL, SPACECALCULATED)
    
end
