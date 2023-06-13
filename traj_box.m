function corridors = calculate_trajectory_boxes(waypoints)
    no_waypoints = length(waypoints);

	%for wayp_idx = 1:no_waypoints
    %    current_waypoint = 
    %end
    % Run to first waypoint
    
	% Check for blockage

	% If false then continue to next waypoint

	% If true then go to previous waypoint and draw a trajectory box
end

function obstruction = check_obstruction(map, cylinder)
    npOpts = occupancyMap3DCollisionOptions(CheckNarrowPhase=true,ReturnDistance=true,ReturnVoxels=true);
    [npIsColliding,bpResults] = checkMapCollision(map,cylinder,npOpts);
    obstruction = npIsColliding;    
end

function cyl = get_cylinder(start, end_, plot)
    % First get direction unit vector
    dir_vec = end_ - start;
    dir_vec = dir_vec/norm(dir_vec);

    % Then get midpoint between points
    midpoint = dir_vec*0.5;
    
    % Then get cylinder:
    cyl = collisionCylinder(0.1,norm(dir_vec));

    % Align
    rx = dir_vec(1);
    ry = dir_vec(2);
    rz = dir_vec(3);
    sin_alpha = ry/(sqrt(rx^2+ry^2));
    cos_alpha = rx/(sqrt(rx^2+ry^2));
    sin_beta = sqrt(rx^2+ry^2);
    cos_beta = rz;
    alpha = atan2(sin_alpha, cos_alpha);
    beta = atan2(sin_beta, cos_beta);
    % beta around y and then alpha around z
    maty = axang2tform([0 1 0 beta]);
    matz = axang2tform([0 0 1 alpha]);
    mat = matz*maty;
    cyl.Pose = mat;

    if plot==true
        show(cyl)
    end
end
