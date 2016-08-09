clear 
close all

cd 2000\static
load '2434_116_8000.mat'
cd ..
cd ..
map = Map('map_mh370.bmp','resolution',3,'hieght',200);
a.xy = .6*a.xy;

% Get distance matrix for end points

nPoints = size(ends,1);
ab = meshgrid(1:nPoints);
dmat = reshape(sqrt(sum((ends(ab,:)-ends(ab',:)).^2,2)),nPoints,nPoints);

% Find number of neighbor points of charging stations

numNeighbor = sum(dmat<=sqrt(68));

% Find isolated stations

isoStations = a.optStations(find(numNeighbor==1));
% Find biggest neighborhood

maxNeighbor = max(numNeighbor);

% Select locations for group stations
Stations =isoStations;
indEmpty = [];
for i = maxNeighbor:-1:2
	

	% empty neighbor locations
	
	while sum(numNeighbor == i) ~=0
		temp = find(numNeighbor == i );
		thisStation = a.optStations(temp(1));
		tempp = find( dmat(temp(1),:) <=sqrt(68) & dmat(temp(1),:) > 0);
		
		thisEmpty = a.optStations(tempp); 
		for k = 1: length(thisEmpty)
			route = [route(1:find (route == thisEmpty(k))-1) thisStation route(find (route == thisEmpty(k)):end)]; %Modify get charged before or after
		end

		% Eliminate assigned locations by puting large number to the distance matrix
		numNeighbor(temp(1)) = 0;
		numNeighbor(tempp) = 0;
		dmat(:,tempp) = 99;
		dmat(tempp,:) = 99;
		% indEmpty = [ indEmpty tempp];
		Stations = [Stations thisStation];
	end
	% indGroups = [indGroups temp];
end
% Modify breaks
newBreak = [];
for i = 1:length(breaks)
	newBreak(i) = find(route == indBreaks(i));
end

% Calculate cost functions

 
% % Select locations for pair stations

% numPNeighbor = sum(tril(dmat)<=sqrt(2) & tril(dmat)>0);

% indStations = [find(numPNeighbor == 1)];
% Plot 
N = length(route);
rng = [[1 newBreak+1];[newBreak N]]';
clr = [1 0 0; 0 0 1; 0.67 0 1; 0 1 0; 1 0.5 0];
figure,
hAx = gca;
for s = 1:a.nSalesmen
		 d = 0;
                
        for k = rng(s,1):rng(s,2)-1
            d = d + a.dmat(route(k),route(k+1));
        end

        tempt(s)=d;
        totalDist = sum(tempt);
        totalTime = max(tempt);
        rte = route(rng(s,1):rng(s,2));
                    
        plot(hAx,a.xy(rte,1),a.xy(rte,2),'.-','linewidth',2,'Color',clr(s,:)); 
        hold(hAx,'on');
end
% Calculate Multiobjective cost
fun = 1/4*(totalDist/size(a.xy,1))+1/4*(totalTime/userConfig.batteryLife)+1/4*(a.nSalesmen/1)+1/4*(length(Stations)/1);
plot(a.xy(Stations,1),a.xy(Stations,2),'k^', 'linewidth', 2,'MarkerSize',10,'MarkerFaceColor','k')
% xlabel('X(km)')
% ylabel('Y(km)')

figure
 [cc hh] = contour(map.matrix,1,'black','linewidth',2);
 cc = cc (:,2:end)*.6;
% legend('Robot #1 Trajectory','Robot #2 Trajectory','Robot #3 Trajectory','Robot #4 Trajectory','Robot #5 Trajectory','Charging Stations')
% legend('boxoff')
%  title(hAx,sprintf('Total Distance = %1.2f, Time = %1.2f, N_w = %d, N_c = %d, fun = %1.2f',totalDist,totalTime,a.nSalesmen,length(Stations),fun ));
hold(hAx,'off');
figure (1)
 hold on 
 plot (cc (1,:),cc(2,:),'k','Linewidth',2)
 initial_point = a.xy([a.optRoute(1) a.optRoute(breaks+1)],:);
plot(initial_point(:,1),initial_point(:,2),'g*','Linewidth',3)
legend('AUV 1','AUV 2','AUV 3','Charging station','Mission border','Inital location')
 legend('boxoff')
 axis([0 50 0 40])