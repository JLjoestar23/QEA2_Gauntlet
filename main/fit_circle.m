function [r_avg_best, x_c_best, y_c_best, detected] = fit_circle(data_points, r, interval)
%   Creates and plots circle of best fit in accordance to data points.
%
%   Args:
%       data_points: A n x 2 matrix containing n data points consisting of
%       a x and y coordinate
%       r: an integer representing known radius
    

    data_points = data_points(~any(data_points==0, 2), :);

    r_avg_best = Inf;
    x_c_best = Inf;
    y_c_best = Inf;
    detected = 0;

    for idx = 1:size(data_points, 1) - interval

        A = zeros(interval + 1, 3);
        A(:,1) = 2 .* data_points(idx:idx + interval,1);
        A(:,2) = 2 .* data_points(idx:idx + interval,2);
        A(:,3) = -1;

        b = data_points(idx:idx + interval,1).^2 + data_points(idx:idx + interval,2).^2;

        w = A \ b;

        x_c = w(1);
        y_c = w(2);

        r_avg = mean(sqrt((data_points(idx:idx + interval, 1) - x_c).^2 + (data_points(idx:idx + interval, 2) - y_c).^2));

        if abs(r_avg - r) <= abs(r_avg_best - r) && x_c <= 0.5 && x_c >= -0.1 && y_c <= 1.4 && y_c >= 1
            r_avg_best = r_avg;
            x_c_best = x_c;
            y_c_best = y_c;
            % th = 0:pi/50:2*pi;
            % xunit = r_avg_best * cos(th) + x_c_best;
            % yunit = r_avg_best * sin(th) + y_c_best;
            % plot(xunit, yunit);
            % hold on;
            % scatter(x_c_best, y_c_best)
            % scatter(data_points(:, 1), data_points(:, 2), ".");
            % axis equal;
        end
        
        % th = 0:pi/50:2*pi;
        % xunit = r_avg_best * cos(th) + x_c_best;
        % yunit = r_avg_best * sin(th) + y_c_best;
        % plot(xunit, yunit);
        % hold on;
        % scatter(x_c_best, y_c_best)
        % scatter(data_points(:, 1), data_points(:, 2), ".");
        % axis equal;
        % hold off;

    end

    if abs(r_avg_best - r) <= 0.02
        detected = 1;
        disp(r_avg_best)
        th = 0:pi/50:2*pi;
        xunit = r_avg_best * cos(th) + x_c_best;
        yunit = r_avg_best * sin(th) + y_c_best;
        plot(xunit, yunit);
        hold on;
        % scatter(x_c_best, y_c_best)
        scatter(data_points(:, 1), data_points(:, 2), ".");
        xlabel('X')
        ylabel('Y')
        title('LIDAR Data with Circle Detection and Fitting')
        legend('Fitted Circle', 'LIDAR Data')
        axis equal;
        hold off;
    end

end
