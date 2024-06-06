function q_rand = samplePoint(shp,workspace,goalPoint)
    if rand<0.8
        q_rand = rand(1, 3) .* (workspace(:, 2) - workspace(:, 1))' + workspace(:, 1)';
        while ~inShape(shp, q_rand)
            q_rand = rand(1, 3) .* (workspace(:, 2) - workspace(:, 1))' + workspace(:, 1)';
        end
    else
        q_rand = goalPoint;  
    end
end