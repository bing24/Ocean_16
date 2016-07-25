clear all
close all

mmm = 0;
% for nnn = [1 1 1 1 1 2 2 2 2 2 3 3 3 3 3 4 4 4 4 4 5 5 5 5 5]
 for nnn = [3]
mmm = mmm+1;
tic;
map = Map('map_mh370.bmp','resolution',3,'hieght',200);

% map.show('border')
represent(map)
% lawn_mower(map)
userConfig.xy=map.mission_location;
userConfig.minTour=floor(size(userConfig.xy,1)/nnn);
userConfig.popSize=8000;
userConfig.nSalesmen= nnn;
userConfig.batteryLife = 116;
userConfig.numIter = 5e3*4;
% a = opti_stations(userConfig);
length_charging = .1;
a = position_ga(userConfig);
computation_time = toc
figure (1)
hold on 

contour(map.matrix,1,'black','linewidth',5)


route = a.optRoute;
breaks = a.optBreak;
N = length(route);
rng = [[1 breaks+1];[breaks N]]';
minTime = 15;
indBreaks = a.optRoute(a.optBreak);
ends = [a.xy(a.optStations,1),a.xy(a.optStations,2)];

 end

initial_boat = [12,2];
xz = [1 7 13 2 8 14 3 9 15 4 10 16 5 11 17 6 12 18];
% xz = [1 11 2 12 3 13 4 14 5 15 6 16 7 17 8 18 9 19 10 20];
% xz = 1:20

plot([initial_boat(1),ends(1,1)],[initial_boat(2),ends(1,2)], 'k--','LineWidth',2)
plot(ends(xz,1),ends(xz,2),'k--','LineWidth',2)
%%
D_c = 0;
for i = 1:length(xz)-1
    D_c = D_c + a.dmat(a.optStations(xz(i)),a.optStations(xz(i+1)));
end
D_c
% %% Add working robots and charging robots 
% sim=Simulation();
% 
% sim.addOperatingRobots(nnn);
% sim.addChargingRobots(1);
% sim.setImageScale(.01);
% sim.setSimulationTime(400); % switch to 200 for circle scenario
% 
% % Give working robots trajectories
% 
% for s = 1:a.nSalesmen
% 	rte = route(rng(s,1):rng(s,2));
% 	sim.list_of_operating_robots(s).trajectory_x = a.xy(rte,1);
% 	sim.list_of_operating_robots(s).trajectory_y = a.xy(rte,2);
% end
% 
% %% Set initial positions and speed of charging robots using ChargingRobot.m 
% 
% setpos(sim.list_of_charging_robots(1),2,2);
%       setspeed(sim.list_of_charging_robots(1),10);
% 
% % setpos(sim.list_of_charging_robots(3),13,2);
% %       setspeed(sim.list_of_charging_robots(3),10);
% 
% setTimeStep(sim,1);
% setChargingTime(sim,length_charging); %switch to 15 for circle
% aw=sim.list_of_operating_robots(1);
% % Calculate charging positions time
% % Calculate length of each route
% lbreaks=diff([1 breaks N]);
% 
% nStations(1)= floor(lbreaks(1)/userConfig.batteryLife-0.000001);
% charging_period = [0 ones(1,nStations(1)-1)*length_charging];
% charging_period = cumsum(charging_period);
% templm=ones(1,nStations(1))*userConfig.batteryLife;
% if isempty(templm) == 1
%     indexStations=[];
% else
%     templm(1)=0+userConfig.batteryLife;
%     time = cumsum(templm)+charging_period;
%     
% end
% for i = 2 : length(lbreaks)
%     % count how many stations for each salesmen
%     
%     nStations(i)= floor(lbreaks(i)/userConfig.batteryLife-0.000001);
%     charging_period = [0 ones(1,nStations(i)-1)*length_charging];
% 	% charging_period = cumsum(charging_period);
%     templm=ones(1,nStations(i))*userConfig.batteryLife+ charging_period;
%     if isempty(templm) == 1
%         time = time;
%     else
%         time = [time cumsum(templm)] ;
%                                
%     end
% end
% 
% % Add "charging" to each operating robot
% tem = cumsum(nStations);
% sim.list_of_operating_robots(1).charging(1,:) = ends(1:tem(1),1);
% sim.list_of_operating_robots(1).charging(2,:) = ends(1:tem(1),2);
% sim.list_of_operating_robots(1).charging(3,:) = time(1:tem(1));
% for s = 2:a.nSalesmen
% 	sim.list_of_operating_robots(s).charging(1,:) = ends(tem(s-1)+1:tem(s),1);
% 	sim.list_of_operating_robots(s).charging(2,:) = ends(tem(s-1)+1:tem(s),2);
% 	sim.list_of_operating_robots(s).charging(3,:) = time(tem(s-1)+1:tem(s));
% end
% 
% 
% plan(sim,'LKH','Distance');
% 
% 
% sim.plot()
% % set(gca,'XTick',[])
% % set(gca,'YTick',[])
%  % pause (3)
%  % sim.simulate()