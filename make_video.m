clear
close all

load ('sample_result')
figure
[cc hh] = contour(map.matrix,1,'black','linewidth',2);
cc = cc (:,2:end)*.6;
close
sim=Simulation();
sim.addOperatingRobots(a.nSalesmen);
sim.addChargingRobots(1);
sim.setImageScale(.002);
sim.setSimulationTime(740);

sim.list_of_operating_robots(1).trajectory_x=a.xy(a.optRoute(1:a.optBreak(1)),1);
sim.list_of_operating_robots(1).trajectory_y=a.xy(a.optRoute(1:a.optBreak(1)),2);

sim.list_of_operating_robots(2).trajectory_x=a.xy(a.optRoute(a.optBreak(1)+1:end),1);
sim.list_of_operating_robots(2).trajectory_y=a.xy(a.optRoute(a.optBreak(1)+1:end),2);

set_speed(sim.list_of_operating_robots(1), 0.086);
set_speed(sim.list_of_operating_robots(2), 0.086);
set_color(sim.list_of_operating_robots(1), [1 0 0]);
set_color(sim.list_of_operating_robots(2), [0 0 1]);

setpos(sim.list_of_charging_robots(1),a.xy(1,1),a.xy(1,2));
setspeed(sim.list_of_charging_robots(1),100);

setTimeStep(sim,1);
setChargingTime(sim,20); 




%% segment trajectory of AUV

for i = 1: a.nSalesmen
    AUV=sim.list_of_operating_robots(i);
    AUV.charging=[];
    position_matrix = [AUV.trajectory_x AUV.trajectory_y];
    distance_between_points = diff(position_matrix,1);
    dist_from_vertex_to_vertex = hypot(distance_between_points(:,1), distance_between_points(:,2));
    cumulative_dist_along_path = [0; cumsum(dist_from_vertex_to_vertex,1)];
    dist_steps = 0:AUV.max_speed:AUV.max_speed*AUV.simulation_time; %linspace(0, travel_length, number_of_timesteps);
    
    if dist_steps(end)>cumulative_dist_along_path(end)
        error('error working robot speed is too fast')
        
    else
        iter=1;
        dist_can_reach=[];
        position_can_reach=[];
        while dist_steps(end)-cumulative_dist_along_path(iter)>= 0
            dist_can_reach(iter)=cumulative_dist_along_path(iter);
            position_can_reach(iter,:)=position_matrix(iter,:);
            
            iter=iter+1;
        end
        dist_can_reach=[dist_can_reach cumulative_dist_along_path(iter)];
        position_can_reach=[position_can_reach; position_matrix(iter,:)];
    end
    
    
    new_points = interp1(dist_can_reach, position_can_reach, dist_steps);
    AUV.trajectory_x=new_points(:,1)';
    AUV.trajectory_y=new_points(:,2)';
    AUV.power_level=1-mod([0:AUV.simulation_time],AUV.battery_life)/AUV.battery_life;
    alert=(AUV.power_level<AUV.alert_level & AUV.power_level>=AUV.critical_level);
    for ii=1:length(find(AUV.power_level==1)) % for each charging window that exists
        temp_alert=find(alert);
        temp_alert=temp_alert(find(temp_alert<=AUV.battery_life*ii & temp_alert>AUV.battery_life*(ii-1)));
        indeces_to_plot=temp_alert;
        AUV.charging=[AUV.charging [AUV.trajectory_x(indeces_to_plot); AUV.trajectory_y(indeces_to_plot); indeces_to_plot]]; % TODO: put into seperate function
    end
    
end





%% segment the boat
% initial_boat = [a.xy(a.optRoute(1),1) a.xy(a.optRoute(1),2)];
% ends = [initial_boat;ends];
xz = [1 4 2 5 3 6]+2;
xz = [1 xz];
boat = sim.list_of_charging_robots;

boat.path_x = ends(xz,1);
boat.path_y = ends(xz,2);
boat.path_x = [boat.path_x; a.xy(a.optRoute(end),1)];
boat.path_y = [boat.path_y; a.xy(a.optRoute(end),2)];
boat.meeting_times = [ 0 100 209 315 422 537 640 740];
number_of_meeting_points=length(boat.path_x);
            temp_trajectory_x=[];
            temp_trajectory_y=[];
            for i=2:number_of_meeting_points
                position_matrix=[boat.path_x(i-1:i), boat.path_y(i-1:i)];
                distance_between_points = diff(position_matrix,1);
                dist_from_vertex_to_vertex = hypot(distance_between_points(:,1), distance_between_points(:,2));
                cumulative_dist_along_path = [0; cumsum(dist_from_vertex_to_vertex,1)];
                if boat.meeting_times(i-1)==0
                    speed=dist_from_vertex_to_vertex/boat.meeting_times(2);
                    trajectory_indexes_to_fill=[1:boat.meeting_times(2)];
                else
                    speed=dist_from_vertex_to_vertex/(boat.meeting_times(i)-boat.meeting_times(i-1)-boat.charging_period_time);
                    trajectory_indexes_to_fill=[(boat.meeting_times(i-1)+boat.charging_period_time+1):boat.meeting_times(i)];
                end
                dist_steps = speed:speed:dist_from_vertex_to_vertex;
                new_points = interp1(cumulative_dist_along_path, position_matrix, dist_steps);
                boat.trajectory_x(trajectory_indexes_to_fill)=new_points(:,1)';
                boat.trajectory_y(trajectory_indexes_to_fill)=new_points(:,2)';
                trajectory_indexes_to_fill=[trajectory_indexes_to_fill(end)+[1:boat.charging_period_time]];
                boat.trajectory_x(trajectory_indexes_to_fill)=new_points(end,1)*ones(1,boat.charging_period_time);
                boat.trajectory_y(trajectory_indexes_to_fill)=new_points(end,2)*ones(1,boat.charging_period_time);
            end

%% finalize trajectory


sim.list_of_operating_robots(1).meeting_times = [100 315 537];
sim.list_of_operating_robots(1).finalizeTrajectory()
sim.list_of_operating_robots(2).meeting_times = [209 422 640];
sim.list_of_operating_robots(2).finalizeTrajectory()
 figure 
 plot (cc (1,:),cc(2,:),'k','Linewidth',2)
 sim.plot
%  sim.simulate
 sim.simulate('record')
 