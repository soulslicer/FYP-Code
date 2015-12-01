function funs = Functions
  funs.pipeline_import=@pipeline_import;
  funs.pf_import=@pf_import;
  funs.Draw3D=@Draw3D;
  funs.Draw2D=@Draw2D;
  funs.DrawRPY=@DrawRPY;
  funs.GetRT=@GetRT;
  funs.pointDistort=@pointDistort;
  funs.pointUndistort=@pointUndistort;
  funs.ModelConvertRT=@ModelConvertRT;
  funs.ModelConvertDegree=@ModelConvertDegree;
  funs.ModelConvertSymbol=@ModelConvertSymbol;
  funs.ModelConvertMatrixInput=@ModelConvertMatrixInput;
  funs.ModelWrapBrackets=@ModelWrapBrackets;
  funs.GetRTRad=@GetRTRad;
  funs.generate_data=@generate_data;
  funs.Draw3DSingle=@Draw3DSingle;
end

function Model = ModelWrapBrackets(Model),
    Model = strcat(Model,')');
    Model = strcat('(',Model);
end

function Model = ModelConvertMatrixInput(Model, string),
    Model = strrep(Model, strcat(string, ...
    '11'), strcat(string,'(1,1)'));
    Model = strrep(Model, strcat(string, ...
    '12'), strcat(string,'(1,2)'));
    Model = strrep(Model, strcat(string, ...
    '13'), strcat(string,'(1,3)'));
    Model = strrep(Model, strcat(string, ...
    '21'), strcat(string,'(2,1)'));
    Model = strrep(Model, strcat(string, ...
    '22'), strcat(string,'(2,2)'));
    Model = strrep(Model, strcat(string, ...
    '23'), strcat(string,'(2,3)'));
    Model = strrep(Model, strcat(string, ...
    '31'), strcat(string,'(3,1)'));
    Model = strrep(Model, strcat(string, ...
    '32'), strcat(string,'(3,2)'));
    Model = strrep(Model, strcat(string, ...
    '33'), strcat(string,'(3,3)'));
    Model = strrep(Model, strcat(string, ...
    '14'), strcat(string,'(1,4)'));
    Model = strrep(Model, strcat(string, ...
    '24'), strcat(string,'(2,4)'));
    Model = strrep(Model, strcat(string, ...
    '34'), strcat(string,'(3,4)'));
end

function Model = ModelConvertSymbol(Model, string, btype),
    Model = strrep(Model, string , btype);
end

function Model = ModelConvertDegree(Model),
    Model = strrep(Model, 'sin' ,'sind');
    Model = strrep(Model, 'cos' ,'cosd');
    Model = strrep(Model, 'acos' ,'acosd');
    Model = strrep(Model, 'asin' ,'asind');
    Model = strrep(Model, 'asindd' ,'asind');
    Model = strrep(Model, 'acosdd' ,'acosd');
end

function Model = ModelConvertRT(Model, string),
    Model = strrep(Model, strcat(string, ...
    '11'), '(cosd(b(4))*cosd(b(6)) + (-sind(b(5))*-sind(b(4))*-sind(b(6))))');
    Model = strrep(Model, strcat(string, ...
    '12'), '(cosd(b(5))*-sind(b(4)))');
    Model = strrep(Model, strcat(string, ...
    '13'), '(cosd(b(4))*sind(b(6)) + (-sind(b(5))*-sind(b(4))*cosd(b(6))))');
    Model = strrep(Model, strcat(string, ...
    '21'), '(sind(b(4))*cosd(b(6)) + (-sind(b(5))*cosd(b(4))*-sind(b(6))))');
    Model = strrep(Model, strcat(string, ...
    '22'), '(cosd(b(5))*cosd(b(4)))');
    Model = strrep(Model, strcat(string, ...
    '23'), '(sind(b(4))*sind(b(6)) + (-sind(b(5))*cosd(b(4))*cosd(b(6))))');
    Model = strrep(Model, strcat(string, ...
    '31'), '(cosd(b(5))*-sind(b(6)))');
    Model = strrep(Model, strcat(string, ...
    '32'), '(sind(b(5)))');
    Model = strrep(Model, strcat(string, ...
    '33'), '(cosd(b(5))*cosd(b(6)))');
    Model = strrep(Model, strcat(string, ...
    '14'), 'b(1)');
    Model = strrep(Model, strcat(string, ...
    '24'), 'b(2)');
    Model = strrep(Model, strcat(string, ...
    '34'), 'b(3)');
end

% Import data from CSV
function dict  = pipeline_import(filename, objectDepth)
    data = importdata(filename);    
    dict = containers.Map;
    dict('sonarPoint') = [data(:,1) data(:,2)];
    dict('sonarRTheta') = [data(:,3) data(:,4)];
    dict('cameraPoint') = [data(:,5) data(:,6)];
    dict('dvl') = [data(:,7) data(:,8) data(:,9)];
    dict('imuEuler') = [data(:,10) data(:,11) data(:,12)];
    dict('imuQuart') = [data(:,13) data(:,14) data(:,15) data(:,16)];
    dict('depth') = data(:,17);
    dict('correctedDepth') = objectDepth - data(:,17);
    dict('brightness') = data(:,18); 
end

% Import data from CSV
function dict  = pf_import(filename)
    data = importdata(filename);    
    dict = containers.Map;
    dict('mapPoint') = [data(:,1) data(:,2) data(:,3)];
    dict('pfPoint') = [data(:,4) data(:,5) data(:,6)];
end

% Generate Data
function dict = generate_data(xModel, yModel, beta, SonarNoise, RollEffect, PitchEffect)
    load('SonarMapping.mat');
    F = [735.4809         0  388.9476 0; ...
         0  733.6047  292.0895 0; ...
         0         0    1.0000 0];

    xLimit = 120;
    yLimit = 210;
    Data = zeros(40000,8);
    count = 0;
    for x = 142:20:142+xLimit

        for y = 218:20:218+yLimit
            index = x*480 + y + 1;
            RTheta = sonarpixel394480(index,:);
            RVal = RTheta(3);
            TVal = RTheta(4);

            % Generate Depth
            for YVal = -2:0.4:2,

                % Generate Noise on Sonar
                a = -SonarNoise; b = SonarNoise;
                xNoise = int32((b-a).*rand(1,1) + a);
                xN = x + xNoise;
                yNoise = int32((b-a).*rand(1,1) + a);
                yN = y + yNoise;
                index = xN*480 + yN + 1;
                RThetaNoised = sonarpixel394480(index,:);

                % RPY
                a = -RollEffect; b = RollEffect;
                Roll = (b-a).*rand(1,1) + a;
                a = -PitchEffect; b = PitchEffect;
                Pitch = (b-a).*rand(1,1) + a;
                Yaw = 0;
                
                % Generate Data
                CurrData = [
                    RVal TVal YVal ...
                    0 0 ...
                    Roll Pitch Yaw ...
                ];
                pointExpanded = [xModel(beta, CurrData), yModel(beta, CurrData)];
                CurrData(4) = pointExpanded(1);
                CurrData(5) = pointExpanded(2);
                CurrData(1) = RThetaNoised(3);
                CurrData(2) = RThetaNoised(4);
                count = count + 1;
                Data(count,:) = CurrData;                
                
            end
        end
    end
    
    FinalData = zeros(count,8);
    FinalData = Data(1:count,:);
    
    dict = containers.Map;
    dict('sonarRTheta') = [FinalData(:,1) FinalData(:,2)];
    dict('cameraPoint') = [FinalData(:,4) FinalData(:,5)];
    dict('imuEuler') = [FinalData(:,6) FinalData(:,7) FinalData(:,8)];
    dict('correctedDepth') = FinalData(:,3);
    
end

% Draw 3D Point
function x = Draw3DSingle(x,y,z,color,marker)
    hold on;
    set(gca,'proj','perspective'); grid on; 
    xlabel('x') % x-axis label
    ylabel('y') % y-axis label
    zlabel('z') % z-axis label
    axis([ -5 5 -5 5 0 8 ])
    plot3(x,y,z,'Marker',marker,'MarkerEdgeColor',color);
end

% Draw 3D Point
function x = Draw3D(x,y,z,color)
    hold on;
    subplot(1,2,2);
    set(gca,'proj','perspective'); grid on; 
    xlabel('x') % x-axis label
    ylabel('y') % y-axis label
    zlabel('z') % z-axis label
    axis([ -5 5 -5 5 0 8 ])
    plot3(x,y,z,'Marker','.','MarkerEdgeColor',color);
end

% Draw 2D Point
function x = Draw2D(x,y,color)
    hold on;
    subplot(1,2,1);
    axis([ 0 780 0 580])
    plot(x,y,'Marker','.','MarkerEdgeColor',color);
end

% Draw RPY
function x = DrawRPY(roll,pitch,yaw)
    hold on;
    subplot(1,3,3);
    set(gca,'proj','perspective'); grid on; 
    xlabel('roll') % x-axis label
    ylabel('pitch') % y-axis label
    zlabel('yaw') % z-axis label
    axis([ -1 1 -1 1 -1 1 ])
    cla;
    
    % your code
    n=6;
    load topo;
    [x,y,z]=sphere(n);
    y=y.*0.5;
    x=x.*0.5;
    sh=surface(x,y,z);
    rotate(sh,[1,0,0],roll*5);
    rotate(sh,[0,1,0],pitch*5);
    rotate(sh,[0,0,1],yaw*5);
end

% Generate RPY Matrix
function RotationMatrix = GetRT(Roll, Pitch, Yaw, X, Y, Z)
    Pitch = [	1       0           0       ; ...
                0       cosd(Pitch)     -sind(Pitch); ...
                0       sind(Pitch)     cosd(Pitch) ;	];
    Yaw = [		cosd(Yaw) 	0       sind(Yaw)	; ...
                0           1       0       ; ...
                -sind(Yaw)	0       cosd(Yaw)	;	];	
    Roll = [	cosd(Roll) 	-sind(Roll)    0	; ...
                sind(Roll)     cosd(Roll)     0 	; ...
                0           0           1	;	];
    RotationMatrix = Roll*Pitch*Yaw;
    RotationMatrix = [RotationMatrix [X Y Z]'];
    RotationMatrix = [RotationMatrix; [0 0 0 1]];
end

% Generate RPY Matrix Rad
function RotationMatrix = GetRTRad(Roll, Pitch, Yaw, X, Y, Z)
    Pitch = [	1       0           0       ; ...
                0       cos(Pitch)     -sin(Pitch); ...
                0       sin(Pitch)     cos(Pitch) ;	];
    Yaw = [		cos(Yaw) 	0       sin(Yaw)	; ...
                0           1       0       ; ...
                -sin(Yaw)	0       cos(Yaw)	;	];	
    Roll = [	cos(Roll) 	-sin(Roll)    0	; ...
                sin(Roll)     cos(Roll)     0 	; ...
                0           0           1	;	];
    RotationMatrix = Roll*Pitch*Yaw;
    RotationMatrix = [RotationMatrix [X Y Z]'];
    RotationMatrix = [RotationMatrix; [0 0 0 1]];
end

% Distort Point
function point = pointDistort(point)
    u = point(1);
    v = point(2);

    fx = 735.4809;
    fy = 733.6047;
    cx = 388.9476;
    cy = 292.0895;
    k1 = -0.0369; % BUT IT GAVE ME NEGATIVE INITIALLY
    k2 = 0.3870;
    k3 = 0;
    p1 = 0;
    p2 = 0;

    x = (u - cx) / fx;
    y = (v - cy) / fy; 

    r2 = x.^2 + y.^2;
    x = x.*(1+k1*r2 + k2*r2.^2) + 2*p1.*x.*y + p2*(r2 + 2*x.^2);
    y = y.*(1+k1*r2 + k2*r2.^2) + 2*p2.*x.*y + p1*(r2 + 2*y.^2);

    u = fx*x + cx;
    v = fy*y + cy;

    point = [u,v];
end

% Undistort Point
function point = pointUndistort(point)
    u = point(1);
    v = point(2);

    fx = 735.4809;
    fy = 733.6047;
    cx = 388.9476;
    cy = 292.0895;
    k1 = 0.0369; % BUT IT GAVE ME NEGATIVE INITIALLY
    k2 = -0.8870; % Has been change from 0.3870
    k3 = 0;
    p1 = 0;
    p2 = 0;

    x = (u - cx) / fx;
    y = (v - cy) / fy; 

    r2 = x.^2 + y.^2;
    x = x.*(1+k1*r2 + k2*r2.^2) + 2*p1.*x.*y + p2*(r2 + 2*x.^2);
    y = y.*(1+k1*r2 + k2*r2.^2) + 2*p2.*x.*y + p1*(r2 + 2*y.^2);

    u = fx*x + cx;
    v = fy*y + cy;

    point = [u,v];
end



