function fig = plotMaze(maze, solution, startCell, goalCell)
%PLOTMAZE Draw closed walls, boundary, and an optional solution.
%   SOLUTION contains graph indices returned by hexmaze.solveMaze.
    if nargin < 2, solution = []; end
    if nargin < 3, startCell = [maze.span, 0]; end
    if nargin < 4, goalCell = [maze.span, 2*maze.span]; end
    start = hexmaze.cellId(maze, startCell);
    goal = hexmaze.cellId(maze, goalCell);
    fig = figure('Color', 'w', 'Position', [100, 100, 850, 800]);
    hold on;
    segments = zeros(2, 0);
    % Internal walls are perpendicular bisectors between adjacent centers.
    for edge = 1:size(maze.walls, 1)
        a = maze.centers(maze.walls(edge, 1), :);
        b = maze.centers(maze.walls(edge, 2), :);
        segments = [segments, wallSegment(a, b, maze.hex_size), [NaN; NaN]]; %#ok<AGROW>
    end
    % A missing geometric neighbor identifies an outer boundary side.
    directions = (0:5)*pi/3;
    for id = 1:size(maze.cells, 1)
        center = maze.centers(id, :);
        adjacent = maze.centers(maze.neighbors{id}, :);
        for angle = directions
            other = center + sqrt(3)*maze.hex_size*[cos(angle), sin(angle)];
            distance = sqrt(sum(bsxfun(@minus, adjacent, other).^2, 2));
            if ~any(distance < 1e-6*maze.hex_size)
                segments = [segments, wallSegment(center, other, maze.hex_size), [NaN; NaN]]; %#ok<AGROW>
            end
        end
    end
    plot(segments(1, :), segments(2, :), '-', 'Color', [0.20, 0.26, 0.32], 'LineWidth', 1.2);
    if ~isempty(solution)
        points = maze.centers(solution, :);
        plot(points(:, 1), points(:, 2), '-', 'Color', [0.12, 0.48, 0.80], 'LineWidth', 2.5);
    end
    scatter(maze.centers(start, 1), maze.centers(start, 2), 90, [0.10, 0.65, 0.38], 'filled');
    scatter(maze.centers(goal, 1), maze.centers(goal, 2), 90, [0.90, 0.30, 0.20], 'filled');
    axis equal; axis off;
    title(sprintf('Hexagonal maze | span %d | %d cells', maze.span, size(maze.cells, 1)));
    hold off;
end

function segment = wallSegment(a, b, hexSize)
    midpoint = (a+b)/2;
    delta = b-a;
    perpendicular = [-delta(2), delta(1)] / norm(delta) * hexSize/2;
    segment = [midpoint-perpendicular; midpoint+perpendicular]';
end
