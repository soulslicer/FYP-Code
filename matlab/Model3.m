RSonar = sym('RSonar');
MValue = sym('MValue');
YVehicle = sym('YVehicle');
YSonar = sym('YSonar');
Pitch = sym('Pitch');
LZ = sym('LZ');
LY = sym('LY');
conv = sym('pi')/180;


% m = solve(sqrt(RSonar.^2-YDepth.^2) - RSonar*cos(Pitch+MValue), MValue);
% x = RSonar*sin(m);
% 
% xSolved = subs(x, RSonar, 5);
% xSolved = subs(xSolved, YDepth, 2);
% xSolved = subs(xSolved, Pitch, conv*5);
% vpa(xSolved,4)

%%%%%%%%%%%%%%%%%%%%

% YDepth = RSonar*sin(Pitch+MValue) + LY*cos(Pitch) + LZ*sin(Pitch);
m = solve(RSonar*sin(Pitch+MValue) + LY*cos(Pitch) + LZ*sin(Pitch) - YVehicle, MValue);
x = RSonar*sin(m);
PointSonar = x(2);

% Inverse
% solve(RSonar*sin(Pitch - asin((LY*cos(Pitch) - YVehicle + LZ*sin(Pitch))/RSonar)) - YSonar, YVehicle);

y = RSonar*sin(Pitch+asin(YSonar/RSonar)) + LY*cos(Pitch) + LZ*sin(Pitch);

% x = YSonar
Pitching = -2;
xSolved = subs(x, RSonar, 10);
xSolved = subs(xSolved, YVehicle, 1.5);
xSolved = subs(xSolved, Pitch, conv*Pitching);
xSolved = subs(xSolved, LY, 0.5);
xSolved = subs(xSolved, LZ, 0.5);
YSonarCalc = vpa(xSolved,4)

% y = YVehicle
ySolved = subs(y, RSonar, 10);
ySolved = subs(ySolved, YSonar, YSonarCalc);
ySolved = subs(ySolved, Pitch, conv*Pitching);
ySolved = subs(ySolved, LY, 0.5);
ySolved = subs(ySolved, LZ, 0.5);
YVehicleCalc = vpa(ySolved,4)
%%%%%%%%%%%%%%%%%%%%

