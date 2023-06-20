function corridors = pos_2_corridors(start_pos, end_pos, coord1, coord2, coord3, coord4, box_size, end_time)
    last_cor = start_pos;

    time = 1;

    % Initialize corridors
    corridors.times = time;
    corridors.x_lower = start_pos(1)-box_size;
    corridors.x_upper = start_pos(1)+box_size;
    corridors.y_lower = start_pos(2)-box_size;
    corridors.y_upper = start_pos(2)+box_size;
    corridors.z_lower = start_pos(3)-box_size;
    corridors.z_upper = start_pos(3)+box_size;

    coords = [coord1; coord2; coord3; coord4; end_pos];

    % Loop over 
    for i=1:5
        % Calculate distance from last waypoint:
        vec = coords(i,:)-last_cor;
        dist = sqrt(vec*transpose(vec)) % Euclidean dist

        % Update time
        time = time+dist;

        corridors.times = [corridors.times time];
        corridors.x_lower = [corridors.x_lower coords(i,1)-box_size];
        corridors.x_upper = [corridors.x_upper coords(i,1)+box_size];
        corridors.y_lower = [corridors.y_lower coords(i,2)-box_size];
        corridors.y_upper = [corridors.y_upper coords(i,2)+box_size];
        corridors.z_lower = [corridors.z_lower coords(i,3)-box_size];
        corridors.z_upper = [corridors.z_upper coords(i,3)+box_size];

        last_cor = coords(i, :);
    end

    % Finally scale the time:
    corridors.times = rescale(corridors.times, 1, end_time);
end
