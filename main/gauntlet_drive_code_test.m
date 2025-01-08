function gauntlet_drive_code_test(ip)
    
    % Initializing Neato connection
    neatov2.connect(ip);
    neatov2.testConnection();
    disp('Connected')

    % we put these for your convenience.  These would track position and
    % orientation of your Neato.
    position = [0.0 ; 0.0];
    heading = [1 ; 0];
    d = 0.245;
    r = position;
    travel_distance = [0 ; 0];
    total_travel_distance = 0;
    current_theta = atan2(heading(2), heading(1));
    lambda = 0.0125;
    delta = 0.999;
    reach_destination = 0;
    i = 1;

    while reach_destination == 0 

        disp('Scanning')
        
        % Initiates a single LIDAR scan
        single_scan = cell(1,1);
    
        single_scan = neatov2.receive();

        disp('Frame Changing Data')
    
        % Transforming data from LIDAR frame to Ground Frame
        total_coords = single_LIDAR_Frame_Change(single_scan, rad2deg(current_theta), r(:, i)');
    
        disp('Detecting Circle')
        
        % Looking for circle of best fit

        [r_avg_best, x_c_best, y_c_best, detected] = fit_circle(total_coords, 0.115, 3);
        
    
        % if circle is detected...
        if detected == 1
            disp('Circle Detected')
            % code to change potential field based on circle location
    
            % Removing data points that are detected to be on the fitted circle
            circle_coords = [];
            for j = 1:size(total_coords, 1)
                distance_to_center = sqrt((total_coords(j, 1) - x_c_best)^2 + (total_coords(j, 2) - y_c_best)^2);
                
                % Check if the distance is within a certain threshold of the radius
                if abs(distance_to_center - r_avg_best) <= 0.075
                    circle_coords = [circle_coords; total_coords(j, :)];
                end
            end
    
            % Find the indices of points that are close to the fitted circle
            indices_to_remove = [];
            for i = j:size(total_coords, 1)
                distance_to_center = sqrt((total_coords(j, 1) - x_c_best)^2 + (total_coords(j, 2) - y_c_best)^2);
                if abs(distance_to_center - r_avg_best) <= 0.1
                    indices_to_remove = [indices_to_remove; j];
                end
            end
            

            equations = @(x, y) 0;
            % Remove the points from total_coords
            total_coords(indices_to_remove, :) = [];
            
            % for j = 1:10:size(total_coords, 1)
            %     equations = @(x, y) equations(x, y) + amplitude * exp(-((x - total_coords(j,1)).^2 + (y - total_coords(j,2)).^2) / (2 * sigma^2));
            % end

            % Applying sinks to fitted circle points
            for j = 1:2:size(circle_coords, 1)
                equations = @(x, y) equations(x, y) + 1 * log(sqrt((x - circle_coords(j,1))^2 + (y - circle_coords(j,2))^2));
            end
            
            [X, Y] = meshgrid(linspace(-2, 2, 100), linspace(-2, 2.5, 100));

            Z = zeros(size(X));
            for j = 1:numel(X)
                Z(j) = equations(X(j), Y(j));
            end

            figure;
            contour(X, Y, Z, 100);
            hold on;
            plot(total_coords(:,1), total_coords(:, 2), ".");
            plot(circle_coords(:,1), circle_coords(:,2), ".");
            title('Source Plots');
            xlabel('X');
            ylabel('Y');
            xlim([-1 2]);
            ylim([-1 2.5]);
            legend('Potential Field', 'Wall Points', 'Circle Points')
            axis equal;
            hold off;

            % calculating gradient of circle points
            grad = @(x, y) numerical_gradient(equations, x, y);


            % while loop for Neato movement
            i = 1;
            while norm(grad(r(1,i), r(2,i))) > 0.4
                r(:, i+1) = r(:,i) - lambda * grad(r(1,i), r(2,i));
                %disp("Actual Position: " + num2str(r(:, i)))
                lambda = lambda * delta;
                % magnitude of distance to be traveled
                travel_distance(:, i+1) = r(:, i+1) - r(:, i);
                travel_distance_mag = norm(travel_distance(:, i+1));
                %disp(travel_distance_mag);
                % how much turning to move
                delta_theta = current_theta - atan2(travel_distance(2, i+1), travel_distance(1, i+1));
                disp(rad2deg(delta_theta));
                
                % angular movement
                ang_V = 0.5;
                turn_time = abs(delta_theta) / ang_V;
                %disp(turn_time)
                tic;
                t = toc;    
                while t < turn_time
                    if delta_theta > 0
                        VL = -(ang_V * (d/2));
                        VR = (ang_V * (d/2));
                        t = toc;
                        neatov2.setVelocities(VR, VL);
                        pause(.05);
                        %neatov2.plotSim();
                    else
                        VL = (ang_V * (d/2));
                        VR = -(ang_V * (d/2));
                        t = toc;
                        neatov2.setVelocities(VR, VL);
                        pause(.05);
                        %neatov2.plotSim();
                    end
                end
                current_theta = atan2(travel_distance(2, i+1), travel_distance(1, i+1));
        
                % linear movement
                lin_V = 0.125;
                drive_time = travel_distance_mag / lin_V;
                %disp(drive_time)
                tic;
                t = toc;
                while t < drive_time
                    VL = lin_V;
                    VR = lin_V;
                    t = toc;
                    neatov2.setVelocities(VR, VL);
                    %neatov2.plotSim();
                end
                i = i+1;
                total_travel_distance = total_travel_distance + travel_distance_mag;
                disp('After Move')
                disp(total_travel_distance)
            end

            reach_destination = 1;

        else
            disp('No Circle Detected')
            % If no circle fitted, continue normal gradient descent behavior
            % Only apply sources to data points
            
            amplitude = 5;  % Amplitude of the Gaussian
            sigma = 0.15;    % Standard deviation, determines the width of the Gaussian
        
            equations = @(x, y) 0;
            for j = 1:10:size(total_coords, 1)
                equations = @(x, y) equations(x, y) + amplitude * exp(-((x - total_coords(j,1)).^2 + (y - total_coords(j,2)).^2) / (2 * sigma^2));
            end
            
            % Sets a general sink for when no circle is spotted
            equations = @(x, y) equations(x, y) + (y - 1).^4;

            [X, Y] = meshgrid(linspace(-2, 2, 100), linspace(-2, 2.5, 100));

            Z = zeros(size(X));
            for j = 1:numel(X)
                Z(j) = equations(X(j), Y(j));
            end

            figure;
            contour(X, Y, Z, 100);
            hold on;
            plot(total_coords(:,1), total_coords(:, 2), ".");
            title('Source Plots');
            xlabel('X');
            ylabel('Y');
            xlim([-1 2]);
            ylim([-1 2.5]);
            axis equal;
            legend('Potential Field', 'Wall Points')
            hold off;
            

            grad = @(x, y) numerical_gradient(equations, x, y);

            for i = i:(i + 40)
                % 5 movement steps after scan and processing LIDAR data
                if norm(grad(r(1,i), r(2,i))) > 0.4
                    r(:, i+1) = r(:,i) - lambda * grad(r(1,i), r(2,i));
                    %disp("Actual Position: " + num2str(r(:, i)))
                    lambda = lambda * delta;
                    % magnitude of distance to be traveled
                    travel_distance(:, i+1) = r(:, i+1) - r(:, i);
                    travel_distance_mag = norm(travel_distance(:, i+1));
                    %disp(travel_distance_mag);
                    % how much turning to move
                    delta_theta = current_theta - atan2(travel_distance(2, i+1), travel_distance(1, i+1));
                    disp(rad2deg(delta_theta));
                    
                    % angular movement
                    ang_V = 0.5;
                    turn_time = abs(delta_theta) / ang_V;
                    %disp(turn_time)
                    tic;
                    t = toc;    
                    while t < turn_time
                        if delta_theta > 0
                            VL = -(ang_V * (d/2));
                            VR = (ang_V * (d/2));
                            t = toc;
                            neatov2.setVelocities(VR, VL);
                            pause(.05);
                            %neatov2.plotSim();
                        else
                            VL = (ang_V * (d/2));
                            VR = -(ang_V * (d/2));
                            t = toc;
                            neatov2.setVelocities(VR, VL);
                            pause(.05);
                            %neatov2.plotSim();
                        end
                    end
                    current_theta = atan2(travel_distance(2, i+1), travel_distance(1, i+1));
            
                    % linear movement
                    lin_V = 0.125;
                    drive_time = travel_distance_mag / lin_V;
                    %disp(drive_time)
                    tic;
                    t = toc;
                    while t < drive_time
                        VL = lin_V;
                        VR = lin_V;
                        t = toc;
                        neatov2.setVelocities(VR, VL);
                        %neatov2.plotSim();
                    end
                    i = i+1;
                    total_travel_distance = total_travel_distance + travel_distance_mag;
                    disp('After Move')
                    disp(total_travel_distance)
                end
    
            end
        
        end
    
    end

    
    
end

% Function for calculating the numerical gradient
function grad = numerical_gradient(f, x, y)
    h = 1e-6;
    grad_x = (f(x + h, y) - f(x, y)) / h;
    grad_y = (f(x, y + h) - f(x, y)) / h;
    grad = [grad_x; grad_y];
end
