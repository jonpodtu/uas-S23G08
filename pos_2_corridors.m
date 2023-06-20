function corridors = pos_2_corridors(start_pos, end_pos, coord1, coord2, coord3, coord4, box_sizes, box_dist, end_time)
    box_size_waypoints = box_sizes(1);
    box_size_center = box_sizes(2);
    box_size_ends = box_sizes(3);

    last_cor = start_pos;

    time = 1;

    % Initialize corridors with start position
    corridors.times = time;
    corridors.x_lower = start_pos(1)-box_size_waypoints;
    corridors.x_upper = start_pos(1)+box_size_waypoints;
    corridors.y_lower = start_pos(2)-box_size_waypoints;
    corridors.y_upper = start_pos(2)+box_size_waypoints;
    corridors.z_lower = start_pos(3)-box_size_waypoints;
    corridors.z_upper = start_pos(3)+box_size_waypoints;

    coords = [coord1; coord2; coord3; coord4];

    % Loop over 
    for i=1:4
        % Calculate axis of rotation:
        omega = 2*acos(coords(i,7));

        % Using axis angle representations, first make unit vector parralel
        % to x axis in world coords:
        unit_dir = [1 0 0];

        % Then rotate unit vector first around y with angle beta and then
        % around z with angle alpha:
        unit_dir = unit_dir*[cos(omega) sin(omega) 0; -sin(omega) cos(omega) 0; 0 0 1];
        % We now have unit vector in one direction, we invert it and make
        % two boxes. First scale:
        unit_dir = unit_dir*box_dist;

        % Get coords:
        center1 = coords(i,1:3)+abs(unit_dir);
        center2 = coords(i,1:3)-abs(unit_dir);

        % This part calculates which box to go through first:
        % Calculate distance from last waypoint to center_1:
        vec1 = center1-last_cor;
        dist1 = sqrt(vec1*transpose(vec1)); % Euclidean dist

        % Calculate distance from last waypoint to center_1:
        vec2 = center2-last_cor;
        dist2 = sqrt(vec2*transpose(vec2)); % Euclidean dist

        if dist1 > dist2 % if the distance to the "first" box to last waypoint is longer, switch order of the boxes
            [center1, center2] = deal(center2, center1);
            [dist1, ~] = deal(dist2, dist1);
        end

        % Update time for center 1
        time = time+dist1;
        corridors.times = [corridors.times time];
        corridors.x_lower = [corridors.x_lower center1(1)-box_size_ends];
        corridors.x_upper = [corridors.x_upper center1(1)+box_size_ends];
        corridors.y_lower = [corridors.y_lower center1(2)-box_size_ends];
        corridors.y_upper = [corridors.y_upper center1(2)+box_size_ends];
        corridors.z_lower = [corridors.z_lower center1(3)-box_size_ends];
        corridors.z_upper = [corridors.z_upper center1(3)+box_size_ends];

        % Make box in middle:
        vec = coords(i,1:3)-center1;
        dist = sqrt(vec*transpose(vec)); % Euclidean dist

        time = time+dist;
        corridors.times = [corridors.times time];
        corridors.x_lower = [corridors.x_lower coords(i,1)-box_size_center];
        corridors.x_upper = [corridors.x_upper coords(i,1)+box_size_center];
        corridors.y_lower = [corridors.y_lower coords(i,2)-box_size_center];
        corridors.y_upper = [corridors.y_upper coords(i,2)+box_size_center];
        corridors.z_lower = [corridors.z_lower coords(i,3)-box_size_center];
        corridors.z_upper = [corridors.z_upper coords(i,3)+box_size_center];

        % Do same for center 2:
        vec = center2-coords(i,1:3);
        dist = sqrt(vec*transpose(vec)); % Euclidean dist
        
        % Update time for center 2
        time = time+dist;
        corridors.times = [corridors.times time];
        corridors.x_lower = [corridors.x_lower center2(1)-box_size_ends];
        corridors.x_upper = [corridors.x_upper center2(1)+box_size_ends];
        corridors.y_lower = [corridors.y_lower center2(2)-box_size_ends];
        corridors.y_upper = [corridors.y_upper center2(2)+box_size_ends];
        corridors.z_lower = [corridors.z_lower center2(3)-box_size_ends];
        corridors.z_upper = [corridors.z_upper center2(3)+box_size_ends];

        last_cor = center2;
    end

    % Explicit fix of end pose:
    vec = end_pos-last_cor;
    dist = sqrt(vec*transpose(vec)); % Euclidean dist
    
    time = time+dist;

    corridors.times = [corridors.times time];
    corridors.x_lower = [corridors.x_lower end_pos(1)-box_size_waypoints];
    corridors.x_upper = [corridors.x_upper end_pos(1)+box_size_waypoints];
    corridors.y_lower = [corridors.y_lower end_pos(2)-box_size_waypoints];
    corridors.y_upper = [corridors.y_upper end_pos(2)+box_size_waypoints];
    corridors.z_lower = [corridors.z_lower end_pos(3)-box_size_waypoints];
    corridors.z_upper = [corridors.z_upper end_pos(3)+box_size_waypoints];

    % Finally scale the time:
    corridors.times = rescale(corridors.times, 1, end_time);
end