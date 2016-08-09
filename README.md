# Ocean_16

1. Run "work_positioning" and "work_mobile" to get result;
2. "make_video_static" and "make_video" are for making videos;
3. "test" and "test_mobile" are used to make figures;
4. 

To run "work_positioning" and "work_mobile":
1. Choose number of robots "nnn";
2. Choose the resolution by changing the parameter of "map"; "10" for 204 grids and "3" for 2344 grids;
3. Choose the "popSize" and "numIter" based on the resolution to get good results;
4. Choose "batteryLife";
5. After the simulation, set the initial points and order of charger (trajectory of charger);
6. Plot the figure

Important:
In "position_ga" and "position_ga_static", the weights of cost function need to be adjust; The cost function of charger need to be changed.

To run "make_video" and "make_video_static":
1. Load the data;
2. Choose SimulationTime and set operating_robots speed so the area is fully covered;
3. Set charging time;
4. Run "find_timing" to adjust the meeting time.