%% Trajectory corridors
%for 2d
%map1 =rot90(map)
%map2 =rot90(map1)
%map3 =rot90(map2)
[corridors, corridors_scaled] = get_traj_corridors(map, route, box_size, 10)

function obstruction = check_obstruction(omap, cylinder)
    npOpts = occupancyMap3DCollisionOptions(CheckNarrowPhase=true,ReturnDistance=true,ReturnVoxels=true);
    [npIsColliding,bpResults] = checkMapCollision(omap,cylinder,npOpts);
    obstruction = npIsColliding;    
end

function omap = convert_to_occupancymap(map)
    omap = occupancyMap3D();
    s = size(map);
    for level=1:s(3) % loop over each level
        for row=1:s(1) % loop over x
            for col=1:s(2)
                element = map(row, col, level);
                if element == 1
                    % Calculate coords:
                    x_coord = row + 0.5;
                    y_coord = col + 0.5;
                    z_coord = level + 0.5;
                    updateOccupancy(omap,[x_coord y_coord z_coord], 1);
                end
            end
        end
    end
    show(omap)
end

function [cyl, len] = get_cylinder(start, end_)
    % First get direction unit vector
    dir_vec = end_ - start;
    len = norm(dir_vec);

    % Then get midpoint between points
    midpoint = [(end_(1)+start(1))/2 (end_(2)+start(2))/2 (end_(3)+start(3))/2];

    % Then get cylinder:
    cyl = collisionCylinder(0.1,norm(dir_vec));
    
    % Normalize the vector
    dir_vec = dir_vec/norm(dir_vec);

    % Align
    rx = dir_vec(1);
    ry = dir_vec(2);
    rz = dir_vec(3);
    sin_alpha = ry/(sqrt(rx^2+ry^2));
    cos_alpha = rx/(sqrt(rx^2+ry^2));
    sin_beta = sqrt(rx^2+ry^2);
    cos_beta = rz;
    alpha = atan2(sin_alpha, cos_alpha);
    alpha(isnan(alpha))=0; % Edge case
    beta = atan2(sin_beta, cos_beta);
    beta(isnan(beta))=0; % Edge case
    % beta around y and then alpha around z
    maty = axang2tform([0 1 0 beta]);
    matz = axang2tform([0 0 1 alpha]);
    mat_t = trvec2tform(midpoint);
    % Reverse order, first rotate around y, then z then translate
    mat = mat_t*matz*maty;
    cyl.Pose = mat;
end

function [corridors, corridors_scaled] = get_traj_corridors(map, route, box_size, end_time)
    % Add offsets
    route(:,1) = route(:,1) + 0.5;
    route(:,2) = route(:,2) + 0.5;
    route(:,3) = route(:,3) + 0.5;

    omap = convert_to_occupancymap(map);
    no_waypoints = length(route);
    start_waypoint = route(1,:);
    prev_waypoint = route(1,:);
    
    % Initialize corridors
    corridors.times = [1];
    corridors.x_lower = [route(1,1)-box_size-0.5];
    corridors.x_upper = [route(1,1)+box_size-0.5];
    corridors.y_lower = [route(1,2)-box_size-0.5];
    corridors.y_upper = [route(1,2)+box_size-0.5];
    corridors.z_lower = [route(1,3)-box_size-0.5];
    corridors.z_upper = [route(1,3)+box_size-0.5];

    time = 1;
    
    for wayp_idx = 2:no_waypoints
        current_waypoint = route(wayp_idx, :);
    
        % Check for blockage between start waypoint and current waypoint:
        [cylinder, len] = get_cylinder(start_waypoint, current_waypoint);
        obstruction = check_obstruction(omap, cylinder);
    
        if obstruction == 1
            % Idea - get the halfway point betweeen and check there
            % further decreasing the time
            midpoint = [(current_waypoint(1)+prev_waypoint(1))/2 (current_waypoint(2)+prev_waypoint(2))/2 (current_waypoint(3)+prev_waypoint(3))/2];
            
            % Check obstruction:
            [cylinder_midpoint, len_midpoint] = get_cylinder(start_waypoint, midpoint);
            obstruction_midpoint = check_obstruction(omap, cylinder_midpoint);

            if obstruction_midpoint == 0 % If there is no obstruction here, construct corridor in this space:
                % Calculate time:
                corridors.times = [corridors.times time+len_midpoint];
                corridors.x_lower = [corridors.x_lower midpoint(1)-box_size-0.5];
                corridors.x_upper = [corridors.x_upper midpoint(1)+box_size-0.5];
                corridors.y_lower = [corridors.y_lower midpoint(2)-box_size-0.5];
                corridors.y_upper = [corridors.y_upper midpoint(2)+box_size-0.5];
                corridors.z_lower = [corridors.z_lower midpoint(3)-box_size-0.5];
                corridors.z_upper = [corridors.z_upper midpoint(3)+box_size-0.5];

                % Update time:
                time = time + len_midpoint;
                start_waypoint = midpoint;
            
            else % If there is an obstruction, then just go back to previous marker
                % Calculate time:
                corridors.times = [corridors.times time+len];
                corridors.x_lower = [corridors.x_lower prev_waypoint(1)-box_size-0.5];
                corridors.x_upper = [corridors.x_upper prev_waypoint(1)+box_size-0.5];
                corridors.y_lower = [corridors.y_lower prev_waypoint(2)-box_size-0.5];
                corridors.y_upper = [corridors.y_upper prev_waypoint(2)+box_size-0.5];
                corridors.z_lower = [corridors.z_lower prev_waypoint(3)-box_size-0.5];
                corridors.z_upper = [corridors.z_upper prev_waypoint(3)+box_size-0.5];
            
                % Update time:
                time = time + len;
                start_waypoint = prev_waypoint;
            end
        end
        prev_waypoint = current_waypoint;
    end
    
    % End time
    % Get len between end box and penultimate box:
    dir_vec = current_waypoint - start_waypoint; % Recall that prev waypoint is current here
    len = norm(dir_vec);

    corridors.times = [corridors.times time+len];
    corridors.x_lower = [corridors.x_lower prev_waypoint(1)-box_size-0.5];
    corridors.x_upper = [corridors.x_upper prev_waypoint(1)+box_size-0.5];
    corridors.y_lower = [corridors.y_lower prev_waypoint(2)-box_size-0.5];
    corridors.y_upper = [corridors.y_upper prev_waypoint(2)+box_size-0.5];
    corridors.z_lower = [corridors.z_lower prev_waypoint(3)-box_size-0.5];
    corridors.z_upper = [corridors.z_upper prev_waypoint(3)+box_size-0.5];

    % Reshape time:
    corridors.times = rescale(corridors.times, 1, end_time);
    
    % Scale the route
    x_scale = 0.65;
    y_scale = 0.55;
    z_scale = 0.75;
    
    x_offset = 0.3;
    y_offset = 0.5;
    z_offset = 0.25;
    
    % Make a copy of the route
    corridors_scaled = corridors;
    
    % Scale the copy
    corridors_scaled.x_lower = (corridors_scaled.x_lower-1)  * x_scale + x_offset;
    corridors_scaled.x_upper = (corridors_scaled.x_upper-1) * x_scale + x_offset;

    corridors_scaled.y_lower = (corridors_scaled.y_lower-1) * y_scale + y_offset;
    corridors_scaled.y_upper = (corridors_scaled.y_upper-1) * y_scale + y_offset;
    
    corridors_scaled.z_lower = (corridors_scaled.z_lower-1) * z_scale + z_offset;
    corridors_scaled.z_upper = (corridors_scaled.z_upper-1) * z_scale + z_offset;
end