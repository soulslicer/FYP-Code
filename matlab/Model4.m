% MAXRANGE = 15
% SPACE = [MAXRANGE*cosd(7.5)*sind(22.5) MAXRANGE*sind(7.5) MAXRANGE*cosd(7.5)*cosd(22.5)];
% Xs = SPACE(1); Ys = SPACE(2); Zs = SPACE(3); R = sqrt(SPACE(1).^2 + SPACE(2).^2 + SPACE(3).^2)
% ActualAng = rad2deg(atan2(Xs,sqrt(Ys.^2 + Zs.^2)))
% EstimatedAng = rad2deg(atan2(Xs,sqrt(0 + Zs.^2)))
% ErrR = R*cosd(ActualAng) - R*cosd(EstimatedAng)

functions = Functions
data=functions.pf_import('pftest.txt');
MAP = data('mapPoint');
PF = data('pfPoint');

figure(1);
subplot(1,2,2);
set(gcf,'color','w');
functions.Draw3DSingle(0,0,0,'red','*');
functions.Draw3DSingle(0,0,0,'blue','.');
title('Particle Filter Pool Conditions');
legend('Mapped Point','Particle Filter Point', 'Location','north');

Data = [];
count = size(data('mapPoint'),1);
err = [];
MPA = [];
PFA = [];
for i = 1:count,
    MAP_POINT = MAP(i,:);
    PF_POINT = PF(i,:);
    MPA = [MAP_POINT; MPA];
    PFA = [PF_POINT; PFA];

    if MAP_POINT(1) ~= 0
        functions.Draw3DSingle(MAP_POINT(1),MAP_POINT(2),MAP_POINT(3),'red','*');
        err = [MAP_POINT - PF_POINT; err];
    end
    functions.Draw3DSingle(PF_POINT(1),PF_POINT(2),PF_POINT(3),'blue','.');
    drawnow;
end

subplot(1,2,1);
title('Particle Filter Pool Conditions');
xlabel('Timestep');
ylabel('Distance in Metres');
MPA(MPA == 0) = NaN;
MPA = flipud(MPA);
PFA = flipud(PFA);
hold on;
h = plot(linspace(1,size(PFA,1),size(PFA,1)), MPA(:,1),'marker','*','color','red');
plot(linspace(1,size(PFA,1),size(PFA,1)), PFA(:,1),'-o','marker','.','color','blue');
uistack(h, 'top');
h = plot(linspace(1,size(PFA,1),size(PFA,1)), MPA(:,2),'marker','*','color','red');
plot(linspace(1,size(PFA,1),size(PFA,1)), PFA(:,2),'-o','marker','.','color','blue');
uistack(h, 'top');
h = plot(linspace(1,size(PFA,1),size(PFA,1)), MPA(:,3),'marker','*','color','red');
plot(linspace(1,size(PFA,1),size(PFA,1)), PFA(:,3),'-o','marker','.','color','blue');
uistack(h, 'top');
legend('Particle Filter Point','Mapped Point', 'Location','north');
hold off;