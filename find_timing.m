for i = [2 4 6 ]
    find(round(sim.list_of_operating_robots(1).trajectory_x,1) == round(boat.path_x(i),1))
    find(round(sim.list_of_operating_robots(1).trajectory_y,1) == round(boat.path_y(i),1))
    
end
%%
for i = [3 5 7 ]
    
    find(round(sim.list_of_operating_robots(2).trajectory_x,1) == round(boat.path_x(i),1))
    find(round(sim.list_of_operating_robots(2).trajectory_y,1) == round(boat.path_y(i),1))
end

%%
find(round(sim.list_of_operating_robots(1).trajectory_x,1) == 4.8)
find(round(sim.list_of_operating_robots(1).trajectory_y,1) == 2.4)

%%
find(round(sim.list_of_operating_robots(2).trajectory_x,1) == 9)
find(round(sim.list_of_operating_robots(2).trajectory_y,1) == 5.4)
