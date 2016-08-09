clear all
close all

% Mission configuration
nnn = [2];
tic;
map = Map('map_mh370.bmp','resolution',10,'hieght',200);
% map.show('border')
represent(map)
lawn_mower(map)
userConfig.xy=map.mission_location*.6; % for MH370
userConfig.minTour=floor(size(userConfig.xy,1)/nnn);
userConfig.popSize=1200;
userConfig.nSalesmen= nnn;
userConfig.batteryLife = 30;
userConfig.numIter = 5e3;

length_charging = .1;

% Run the simulation
a = position_ga(userConfig);
computation_time = toc/60

% 
route = a.optRoute;
breaks = a.optBreak;
N = length(route);
rng = [[1 breaks+1];[breaks N]]';
minTime = 15;
indBreaks = a.optRoute(a.optBreak);
ends = [a.xy(a.optStations,1),a.xy(a.optStations,2)];


%% Scale the map and set initial points
figure
[cc hh] = contour(map.matrix,1,'black','linewidth',2);
cc = cc (:,2:end)*.6;
close figure 3 
 
% initial_boat = [a.xy(a.optRoute(1),1) a.xy(a.optRoute(1),2); ...
%     a.xy(a.optRoute(breaks(1)+1),1) a.xy(a.optRoute(breaks(1)+1),2);...
%     a.xy(a.optRoute(breaks(2)+1),1) a.xy(a.optRoute(breaks(2)+1),2)];
initial_boat = [a.xy(a.optRoute(1),1) a.xy(a.optRoute(1),2); ...
    a.xy(a.optRoute(breaks(1)+1),1) a.xy(a.optRoute(breaks(1)+1),2)];
ends = [initial_boat;ends]; % the trajectory of charger
xz = 1:21;
xz = [1 11 2 12 3 13 4 14 5 15 6 16 7 17 8 18 9 19 10 20]+2;
xz = [1 4 2 5 3 6]+2;
% xz = [1 7 13 2 8 14 3 9 15 4 10 16 5 11 17 6 12 18]+3;
% xz = [1 2 3 xz];
xz = [1 xz];

figure (1)
hold on
plot(ends(xz,1),ends(xz,2),'k--','LineWidth',2)

xlabel('X(km)')
ylabel('Y(km)')
plot (cc (1,:),cc(2,:),'k','Linewidth',2)

initial_point = a.xy([a.optRoute(1) a.optRoute(breaks+1)],:);
plot(initial_point(:,1),initial_point(:,2),'g*','Linewidth',3)

legend('AUV','AUV 2','Charger','Rendevous location','Mission border','Inital location')
% legend('AUV','Charger','Rendevous location','Mission border','Inital location')
% legend('AUV','AUV 2','AUV 3','Charger','Rendevous location','Mission border','Inital location')
legend('boxoff')
% axis([0 50 0 40])
