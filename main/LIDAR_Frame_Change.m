function total = LIDAR_Frame_Change(file_name)
    load(file_name, 'angles', 'positions', 'scans')
    total = cell(size(scans));
    for i=1:size(scans)
        ranges = scans{i}.ranges;
        theta = scans{i}.thetasInRadians;
        phi = deg2rad(angles(i));
        
        % Cleaning data of zeros
        clean_idx = (ranges ~= 0);
        ranges_clean = ranges(clean_idx);
        theta_clean = theta(clean_idx);
        
        % LIDAR data in LIDAR frame
        L_frame = [ranges_clean ; ranges_clean] .* [cos(theta_clean) ; sin(theta_clean)];
        
        % LIDAR data in Neato frame
        N_frame = [L_frame - [0.08 ; 0] ; ones(1, size(L_frame, 2))];
        
        % Rotation matrix for ground frame transformation
        R = [cos(phi) -sin(phi) 0 ; sin(phi) cos(phi) 0 ; 0 0 1];
        
        % Translation matrix for ground frame transformation
        T = [1 0 positions(i, 1) ; 0 1 positions(i, 2) ; 0 0 1];
        
        % LIDAR data in Ground frame
        G_frame = T * R * N_frame;
        
        total{i} = G_frame;

        % figure;
        % scatter(G_frame(1,:), G_frame(2,:), '.');
        % axis("equal");
        % 
        % figure(super_imposed);
        % scatter(G_frame(1,:), G_frame(2,:), '.');
        % hold on;
        % axis("equal");
        
    end