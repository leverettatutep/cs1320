function [intersection, distance] = rayt(x, n, x_obstacle, n_obstacle, x_lim, y_lim)

nearest_distance = n_obstacle * x - dot(n_obstacle, x_obstacle, 2);
distances = -nearest_distance ./ (n_obstacle * n);

[~, i] = sort(distances);

for idx = i'
    if distances(idx) < 0
        continue
    end

    distance = distances(idx, 1);
    intersection = x + n * distance;
    
    x_in = intersection(1) > x_lim(idx, 1) && intersection(1) < x_lim(idx, 2);
    y_in = intersection(2) > y_lim(idx, 1) && intersection(2) < y_lim(idx, 2);
    
    if x_in && y_in
        return
    end
end

intersection = zeros(2, 1);
distance = inf;