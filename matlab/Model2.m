%% Basic Model without Pitch
funs = Functions;

% Sonar Camera Properties
F = [sym('FX')         0  sym('UO') 0; ...
         0  sym('FY')  sym('VO') 0; ...
         0         0    1.0000 0];
SonarWRTCamera = sym('CTS%d%d', [4 4]);
SonarWRTCamera(4,1) = 0; SonarWRTCamera(4,2) = 0; SonarWRTCamera(4,3) = 0; SonarWRTCamera(4,4) = 1;

% Sonar to 3D
RSonar = sym('RSonar');
ThetaSonar = sym('ThetaSonar');
YSonar = sym('YSonar');
PointSonar = [
    sqrt(RSonar.^2 - YSonar.^2)*sin(ThetaSonar);
    YSonar
    sqrt(RSonar.^2 - YSonar.^2)*cos(ThetaSonar);
    1
];

% Map to Camera
UCamera = sym('UCamera');
VCamera = sym('VCamera');
pix = F*SonarWRTCamera*PointSonar;
UV = [pix(1)/pix(3) pix(2)/pix(3)];

% Minimization Model
XModel = UV(1);
YModel = UV(2);
FullModel = sqrt((XModel-UCamera).^2 + (YModel-VCamera).^2);

% Vehicle to Sonar Model
MValue = sym('MValue');
YVehicle = sym('YVehicle');
Pitch = sym('Pitch');
LZ = sym('LZ');
LY = sym('LY');
m = solve(RSonar*sin(Pitch+MValue) + LY*cos(Pitch) + LZ*sin(Pitch) - YVehicle, MValue);
x = RSonar*sin(m);
YSonarModel = x(2);
YVehicleModel = RSonar*sin(Pitch+asin(YSonar/RSonar)) + LY*cos(Pitch) + LZ*sin(Pitch);

% Convert to text
YSonarModelText = char(YSonarModel);
YVehicleModelText = char(YVehicleModel);
YSonarModelText = funs.ModelWrapBrackets(YSonarModelText);
YVehicleModelText = funs.ModelWrapBrackets(YVehicleModelText);
YVehicleModelOpt = strcat('yVehicleThroughYSonar = @(RSonar, YSonar, Pitch, LY, LZ)', funs.ModelConvertDegree(YVehicleModelText))
YSonarModelOpt = strcat('ySonarThroughYVehicle = @(RSonar, YVehicle, Pitch, LY, LZ)', funs.ModelConvertDegree(YSonarModelText))

FullModelText = char(vpa(FullModel,6));
FullModelText = funs.ModelConvertSymbol(FullModelText, 'YSonar', YSonarModelText);
FullModelText = funs.ModelConvertSymbol(FullModelText, 'LY', 'b(7)');
FullModelText = funs.ModelConvertSymbol(FullModelText, 'LZ', 'b(8)');
FullModelText = funs.ModelConvertDegree(FullModelText);
FullModelText = funs.ModelConvertRT(FullModelText, 'CTS'); % b1-6 used
FullModelText = funs.ModelConvertSymbol(FullModelText, '^', '.^');
FullModelText = funs.ModelConvertSymbol(FullModelText, '*', '.*');
FullModelText = funs.ModelConvertSymbol(FullModelText, '/', './');
FullModelText = funs.ModelConvertSymbol(FullModelText, 'FX', '735.4809');
FullModelText = funs.ModelConvertSymbol(FullModelText, 'FY', '733.6047');
FullModelText = funs.ModelConvertSymbol(FullModelText, 'UO', '388.9476');
FullModelText = funs.ModelConvertSymbol(FullModelText, 'VO', '292.0895');
FullModelDone = strcat('fullModel = @(b, RSonar, ThetaSonar, YVehicle, UCamera, VCamera, Pitch)', FullModelText);
FullModelText = funs.ModelConvertSymbol(FullModelText, 'RSonar', 'x(:,1)');
FullModelText = funs.ModelConvertSymbol(FullModelText, 'ThetaSonar', 'x(:,2)');
FullModelText = funs.ModelConvertSymbol(FullModelText, 'YVehicle', 'x(:,3)');
FullModelText = funs.ModelConvertSymbol(FullModelText, 'UCamera', 'x(:,4)');
FullModelText = funs.ModelConvertSymbol(FullModelText, 'VCamera', 'x(:,5)');
FullModelText = funs.ModelConvertSymbol(FullModelText, 'Pitch', 'x(:,7)');
FullModelOpt = strcat('fullModel = @(b, x)', FullModelText)

XModelText = char(vpa(XModel,6));
XModelText = funs.ModelConvertSymbol(XModelText, 'YSonar', YSonarModelText);
XModelText = funs.ModelConvertSymbol(XModelText, 'LY', 'b(7)');
XModelText = funs.ModelConvertSymbol(XModelText, 'LZ', 'b(8)');
XModelText = funs.ModelConvertDegree(XModelText);
XModelText = funs.ModelConvertRT(XModelText, 'CTS'); % b1-6 used
XModelText = funs.ModelConvertSymbol(XModelText, '^', '.^');
XModelText = funs.ModelConvertSymbol(XModelText, '*', '.*');
XModelText = funs.ModelConvertSymbol(XModelText, '/', './');
XModelText = funs.ModelConvertSymbol(XModelText, 'FX', '735.4809');
XModelText = funs.ModelConvertSymbol(XModelText, 'FY', '733.6047');
XModelText = funs.ModelConvertSymbol(XModelText, 'UO', '388.9476');
XModelText = funs.ModelConvertSymbol(XModelText, 'VO', '292.0895');
XModelDone = strcat('xModel = @(b, RSonar, ThetaSonar, YVehicle, UCamera, VCamera, Pitch)', XModelText);
XModelText = funs.ModelConvertSymbol(XModelText, 'RSonar', 'x(:,1)');
XModelText = funs.ModelConvertSymbol(XModelText, 'ThetaSonar', 'x(:,2)');
XModelText = funs.ModelConvertSymbol(XModelText, 'YVehicle', 'x(:,3)');
XModelText = funs.ModelConvertSymbol(XModelText, 'UCamera', 'x(:,4)');
XModelText = funs.ModelConvertSymbol(XModelText, 'VCamera', 'x(:,5)');
XModelText = funs.ModelConvertSymbol(XModelText, 'Pitch', 'x(:,7)');
XModelOpt = strcat('xModel = @(b, x)', XModelText)

YModelText = char(vpa(YModel,6));
YModelText = funs.ModelConvertSymbol(YModelText, 'YSonar', YSonarModelText);
YModelText = funs.ModelConvertSymbol(YModelText, 'LY', 'b(7)');
YModelText = funs.ModelConvertSymbol(YModelText, 'LZ', 'b(8)');
YModelText = funs.ModelConvertDegree(YModelText);
YModelText = funs.ModelConvertRT(YModelText, 'CTS'); % b1-6 used
YModelText = funs.ModelConvertSymbol(YModelText, '^', '.^');
YModelText = funs.ModelConvertSymbol(YModelText, '*', '.*');
YModelText = funs.ModelConvertSymbol(YModelText, '/', './');
YModelText = funs.ModelConvertSymbol(YModelText, 'FX', '735.4809');
YModelText = funs.ModelConvertSymbol(YModelText, 'FY', '733.6047');
YModelText = funs.ModelConvertSymbol(YModelText, 'UO', '388.9476');
YModelText = funs.ModelConvertSymbol(YModelText, 'VO', '292.0895');
YModelDone = strcat('yModel = @(b, RSonar, ThetaSonar, YVehicle, UCamera, VCamera, Pitch)', YModelText);
YModelText = funs.ModelConvertSymbol(YModelText, 'RSonar', 'x(:,1)');
YModelText = funs.ModelConvertSymbol(YModelText, 'ThetaSonar', 'x(:,2)');
YModelText = funs.ModelConvertSymbol(YModelText, 'YVehicle', 'x(:,3)');
YModelText = funs.ModelConvertSymbol(YModelText, 'UCamera', 'x(:,4)');
YModelText = funs.ModelConvertSymbol(YModelText, 'VCamera', 'x(:,5)');
YModelText = funs.ModelConvertSymbol(YModelText, 'Pitch', 'x(:,7)');
YModelOpt = strcat('yModel = @(b, x)', YModelText)

% Y Convert
YSonarThroughV = solve(UV(2) - VCamera,YSonar);
YSonarThroughV = vpa(YSonarThroughV,6);
YSonarThroughV = YSonarThroughV(1);
YSonarThroughVText = char(YSonarThroughV);
YSonarThroughVText = funs.ModelConvertDegree(YSonarThroughVText);
YSonarThroughVText = funs.ModelConvertMatrixInput(YSonarThroughVText, 'CTS');
YSonarThroughVText = funs.ModelConvertSymbol(YSonarThroughVText, 'FX', '735.4809');
YSonarThroughVText = funs.ModelConvertSymbol(YSonarThroughVText, 'FY', '733.6047');
YSonarThroughVText = funs.ModelConvertSymbol(YSonarThroughVText, 'UO', '388.9476');
YSonarThroughVText = funs.ModelConvertSymbol(YSonarThroughVText, 'VO', '292.0895');
YSonarThroughVOpt = strcat('ySonarThroughV = @(RSonar, ThetaSonar, CTS, VCamera)', YSonarThroughVText)

YSonarThroughU = solve(UV(1) - UCamera,YSonar);
YSonarThroughU = vpa(YSonarThroughU,6);
YSonarThroughU = YSonarThroughU(1);
YSonarThroughUText = char(YSonarThroughU);
YSonarThroughUText = funs.ModelConvertDegree(YSonarThroughUText);
YSonarThroughUText = funs.ModelConvertMatrixInput(YSonarThroughUText, 'CTS');
YSonarThroughUText = funs.ModelConvertSymbol(YSonarThroughUText, 'FX', '735.4809');
YSonarThroughUText = funs.ModelConvertSymbol(YSonarThroughUText, 'FY', '733.6047');
YSonarThroughUText = funs.ModelConvertSymbol(YSonarThroughUText, 'UO', '388.9476');
YSonarThroughUText = funs.ModelConvertSymbol(YSonarThroughUText, 'VO', '292.0895');
YSonarThroughUOpt = strcat('ySonarThroughU = @(RSonar, ThetaSonar, CTS, UCamera)', YSonarThroughVText)

eval('clearvars -except XModelOpt YModelOpt FullModelOpt YSonarThroughVOpt YSonarThroughUOpt YVehicleModelOpt YSonarModelOpt')
eval(XModelOpt);
eval(YModelOpt);
eval(FullModelOpt);
eval(YSonarThroughVOpt);
eval(YSonarThroughUOpt);
eval(YVehicleModelOpt);
eval(YSonarModelOpt);

% Test if correct
% PointSonarSolved = subs(PointSonar, RSonar, 5);
% PointSonarSolved = subs(PointSonarSolved, ThetaSonar, degtorad(20));
% PointSonarSolved = subs(PointSonarSolved, YSonar, 1);
% PointSonarSolved = vpa(PointSonarSolved,6);
% pix = F*funs.GetRT(0,20,0,0,0,0)*PointSonarSolved;
% UV = [pix(1)/pix(3) pix(2)/pix(3)];
% vpa(UV,6)
