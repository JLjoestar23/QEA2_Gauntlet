function total_coords = single_LIDAR_Frame_Change(data, angle, position)

    ranges = data.ranges;
    theta = data.thetasInRadians;
    phi = deg2rad(angle);
    
    % Cleaning data of zeros
    clean_idx = (ranges ~= 0);
    ranges_clean = ranges(clean_idx);
    theta_clean = theta(clean_idx);

    % figure();
    % plot(ranges_clean, theta_clean);
    
    % LIDAR data in LIDAR frame
    L_frame = [ranges_clean ; ranges_clean] .* [cos(theta_clean) ; sin(theta_clean)];
    
    % LIDAR data in Neato frame
    N_frame = [L_frame - [0.08 ; 0] ; ones(1, size(L_frame, 2))];
    
    % Rotation matrix for ground frame transformation
    R = [cos(phi) -sin(phi) 0 ; sin(phi) cos(phi) 0 ; 0 0 1];
    
    % Translation matrix for ground frame transformation
    T = [1 0 position(1, 1) ; 0 1 position(1, 2) ; 0 0 1];
    
    % LIDAR data in Ground frame
    G_frame = T * R * N_frame;
    
    total_coords = G_frame';

    figure;
    scatter(G_frame(1,:), G_frame(2,:), '.');
    xlabel('X');
    ylabel('Y');
    title('LIDAR Scan');
    xlim([-1 2]);
    ylim([-1 2.5]);
    axis equal;
end