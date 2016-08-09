clear 
% close all

cd 2000\1auv
load '2434_116_8000.mat'
cd ..
cd ..
map = Map('map_mh370.bmp','resolution',3,'hieght',200);
figure
 [cc hh] = contour(map.matrix,1,'black','linewidth',2);
 cc = cc (:,2:end)*.6;

initial_boat = [a.xy(a.optRoute(1),1) a.xy(a.optRoute(1),2)];
a.xy = .6*a.xy;
ends = [initial_boat;ends]*.6;
xz = 1:21;
% xz = [1 11 2 12 3 13 4 14 5 15 6 16 7 17 8 18 9 19 10 20]+1;
% xz = [1 7 13 2 8 14 3 9 15 4 10 16 5 11 17 6 12 18]+1;
% xz = [1 xz];
N = length(route);
rng = [[1 breaks+1];[breaks N]]';
clr = [1 0 0; 0 0 1; 1 0 1; 0 1 0; 1 0.5 0];
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

hold on
plot(ends(xz,1),ends(xz,2),'k--','LineWidth',2)
plot(a.xy(a.optStations,1),a.xy(a.optStations,2),'kx', 'linewidth', 2,'MarkerSize',10,'MarkerFaceColor','k')
% xlabel('X(km)')
% ylabel('Y(km)')
plot (cc (1,:),cc(2,:),'k','Linewidth',2)

initial_point = a.xy([a.optRoute(1) a.optRoute(breaks+1)],:);
plot(initial_point(:,1),initial_point(:,2),'g*','Linewidth',3)

% legend('AUV 1','AUV 2','Charger','Rendevous location','Mission border','Inital location')
% legend('AUV','Charger','Rendevous location','Mission border','Inital location')
legend('AUV 1','AUV 2','AUV 3','Charger','Rendevous location','Mission border','Inital location')
legend('boxoff')
axis([0 50 0 40])