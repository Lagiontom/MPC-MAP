start_position = [2,7,pi/2]; % (x, y, theta)


public_vars.gnss_init_samples = 10;% Pocet vzorku pro inicializaci Kalmanova filtru z GNSS dat


public_vars.slow_start_duration = 100;% Počet iterací, po které bude robot jezdit pomalu na začátku (pro stabilizaci odhadu pozice)


public_vars.path_planning_clearance = 0.7; % ZVYSENO pro vetsi vzdalenost od sten

public_vars.lidar_history = [];
% definice mapy
%map_name = 'maps/outdoor_2.txt';
map_name = 'maps/mixed_1.txt';
%map_name = 'maps/indoor_1.txt';


%map_name = 'algorithms/mapy/1.txt';
%map_name = 'algorithms/mapy/2.txt';
%map_name = 'algorithms/mapy/3.txt';
%map_name = 'algorithms/mapy/4.txt';
%map_name = 'algorithms/mapy/5.txt';