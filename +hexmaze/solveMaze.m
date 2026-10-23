function [solution, stats] = solveMaze(maze, startCell, goalCell, algorithm, beamWidth)
%SOLVEMAZE Search passages with 'bfs', 'dfs', 'astar', or 'beam'.
%   SOLUTION is a row of cell indices (including both endpoints), or []
%   when no route is found. STATS uses the same fields for every algorithm.
%   Beam search can discard the only route to the goal; default width is 5.
    if nargin < 4, algorithm = 'astar'; end
    if nargin < 5, beamWidth = 5; end
    algorithm = validatestring(algorithm, {'bfs', 'dfs', 'astar', 'beam'});
    validateattributes(beamWidth, {'numeric'}, {'scalar', 'integer', 'positive', 'finite'});
    start = hexmaze.cellId(maze, startCell);
    goal = hexmaze.cellId(maze, goalCell);
    count = size(maze.cells, 1);
    isAStar = strcmp(algorithm, 'astar');
    if isAStar || strcmp(algorithm, 'beam')
        % Euclidean distance in edge lengths is an admissible heuristic.
        delta = bsxfun(@minus, maze.centers, maze.centers(goal, :));
        heuristic = sqrt(sum(delta.^2, 2)) / (sqrt(3)*maze.hex_size);
    end
    parent = zeros(1, count);
    seen = false(1, count);
    seen(start) = true;
    frontier = start;
    if isAStar
        costs = inf(1, count);
        costs(start) = 0;
    end
    found = false;
    iterations = 0; explored = 0; maxFrontier = 1;

    if strcmp(algorithm, 'beam')
        while ~isempty(frontier)
            iterations = iterations + 1;
            maxFrontier = max(maxFrontier, numel(frontier));
            candidates = [];
            for current = frontier
                explored = explored + 1;
                if current == goal, found = true; break; end
                for neighbor = maze.paths{current}
                    if ~seen(neighbor)
                        seen(neighbor) = true;
                        parent(neighbor) = current;
                        candidates(end+1) = neighbor; %#ok<AGROW>
                    end
                end
            end
            if found, break; end
            [~, order] = sort(heuristic(candidates));
            frontier = candidates(order(1:min(beamWidth, numel(order))));
        end
    else
        while ~isempty(frontier)
            iterations = iterations + 1;
            maxFrontier = max(maxFrontier, numel(frontier));
            switch algorithm
                case 'dfs'
                    selected = numel(frontier);
                case 'astar'
                    [~, selected] = min(costs(frontier) + reshape(heuristic(frontier), 1, []));
                otherwise
                    selected = 1;
            end
            current = frontier(selected);
            frontier(selected) = [];
            explored = explored + 1;
            if current == goal, found = true; break; end
            for neighbor = maze.paths{current}
                if isAStar
                    improve = costs(current)+1 < costs(neighbor);
                else
                    improve = ~seen(neighbor);
                end
                if improve
                    seen(neighbor) = true;
                    parent(neighbor) = current;
                    if isAStar, costs(neighbor) = costs(current)+1; end
                    if ~isAStar || ~any(frontier == neighbor)
                        frontier(end+1) = neighbor; %#ok<AGROW>
                    end
                end
            end
        end
    end
    solution = [];
    pathLength = inf;
    if found
        solution = goal;
        while solution(1) ~= start
            solution = [parent(solution(1)), solution]; %#ok<AGROW>
        end
        pathLength = numel(solution)-1;
    end
    stats = struct('found', found, 'iterations', iterations, ...
        'nodes_explored', explored, ...
        'path_length', pathLength, 'max_frontier_size', maxFrontier);
end
