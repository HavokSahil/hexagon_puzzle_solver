clc; clear; close all;

% span = 10;
hex_size = 1;
% start_cell = [0, 0];
% goal_cell = [span, span];

% drawMazeSolution(span, hex_size, start_cell, goal_cell);

span_values = [2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30]; % Different maze sizes
hex_sizefunction = 1; % Hexagon side length
compareMazeSolvers(span_values, hex_size);

%% Function to Draw one Maze and its solution
function drawMazeSolution(span, hex_size, start_cell, goal_cell)
    % Generate hexagonal grid and walls
    [hexCenters, cellLabels] = generateHexGrid(span, hex_size);
    walls = generateWalls(span);
    walls = removeWallsKruskal(walls, span);
    paths = createPaths(walls, start_cell, goal_cell, span);
    [sol, stats] = solveMazeBeamSearch(span, paths, start_cell, goal_cell, 5, hexCenters);
    plotHexMaze(hexCenters, cellLabels, walls, hex_size, start_cell, goal_cell);
    drawPath(sol, hexCenters);
    
end

%% Function to Compare Different Algorithms for Maze Solving
function compareMazeSolvers(span_values, hex_size)
    % Initialize arrays to store statistics
    stats_a_star = [];
    stats_bfs = [];
    stats_dfs = [];
    stats_beam = [];

    % Iterate over different maze sizes
    for span = span_values
        % Set start and goal

        start_cell = [span, 0];
        goal_cell = [span, span*2];

        % Generate hexagonal grid and walls
        [hexCenters, cellLabels] = generateHexGrid(span, hex_size);
        walls = generateWalls(span);
        walls = removeWallsKruskal(walls, span);
        paths = createPaths(walls, start_cell, goal_cell, span);

        % Solve using different algorithms
        [~, stats_a] = solveMazeAStar(span, paths, start_cell, goal_cell, hexCenters);
        [~, stats_b] = solveMazeBFS(span, paths, start_cell, goal_cell);
        [~, stats_d] = solveMazeDFS(span, paths, start_cell, goal_cell);
        [~, stats_bs] = solveMazeBeamSearch(span, paths, start_cell, goal_cell, span, hexCenters);

        % Store statistics
        stats_a_star = [stats_a_star; span, stats_a.iterations, stats_a.nodes_explored, 0, stats_a.max_open_set_size];
        stats_bfs = [stats_bfs; span, stats_b.iterations, stats_b.nodes_explored, 0, stats_b.max_queue_size];
        stats_dfs = [stats_dfs; span, stats_d.iterations, stats_d.nodes_explored, 0, stats_d.max_stack_size];
        stats_beam = [stats_beam; span, stats_bs.iterations, stats_bs.nodes_explored, 0, stats_bs.max_level_size];
    end

    % Plot comparisons
    plotStatisticsComparison(span_values, stats_a_star, stats_bfs, stats_dfs, stats_beam);
end

%%  Function to Plot Performance Comparisons
function plotStatisticsComparison(span_values, stats_a_star, stats_bfs, stats_dfs, stats_beam)
    figure;

    % Subplot 1: Iterations
    subplot(1,3,1);
    plot(span_values, stats_a_star(:,2), 'ro-', 'LineWidth', 2, 'DisplayName', 'A*');
    hold on;
    plot(span_values, stats_bfs(:,2), 'bs-', 'LineWidth', 2, 'DisplayName', 'BFS');
    plot(span_values, stats_dfs(:,2), 'g^-', 'LineWidth', 2, 'DisplayName', 'DFS');
    plot(span_values, stats_beam(:,2), 'm*-', 'LineWidth', 2, 'DisplayName', 'Beam');
    hold off;
    xlabel('Span');
    ylabel('Iterations');
    title('Iterations Comparison');
    legend('show');
    grid on;

    % Subplot 2: Nodes Explored
    subplot(1,3,2);
    plot(span_values, stats_a_star(:,3), 'ro-', 'LineWidth', 2, 'DisplayName', 'A*');
    hold on;
    plot(span_values, stats_bfs(:,3), 'bs-', 'LineWidth', 2, 'DisplayName', 'BFS');
    plot(span_values, stats_dfs(:,3), 'g^-', 'LineWidth', 2, 'DisplayName', 'DFS');
    plot(span_values, stats_beam(:,3), 'm*-', 'LineWidth', 2, 'DisplayName', 'Beam');
    hold off;
    xlabel('Span');
    ylabel('Nodes Explored');
    title('Nodes Explored Comparison');
    legend('show');
    grid on;

    % Subplot 4: Max Queue Size
    subplot(1,3,3);
    plot(span_values, stats_a_star(:,5), 'ro-', 'LineWidth', 2, 'DisplayName', 'A*');
    hold on;
    plot(span_values, stats_bfs(:,5), 'bs-', 'LineWidth', 2, 'DisplayName', 'BFS');
    plot(span_values, stats_dfs(:,5), 'g^-', 'LineWidth', 2, 'DisplayName', 'DFS');
    plot(span_values, stats_beam(:,5), 'm*-', 'LineWidth', 2, 'DisplayName', 'Beam');
    hold off;
    xlabel('Span');
    ylabel('Max Open Set Size');
    title('Memory Usage Comparison');
    legend('show');
    grid on;

    % Show overall figure
    sgtitle('Comparison of Maze Solving Algorithms');
end


% Generate hexagonal grid coordinates
function [hexCenters, cellLabels] = generateHexGrid(span, hex_size)
    
    hexCenters = containers.Map('KeyType', 'char', 'ValueType', 'any'); 
    cellLabels = containers.Map('KeyType', 'char', 'ValueType', 'any'); 

    hex_width = sqrt(3) * hex_size;
    hex_height = 2 * hex_size;

    hex_xoffset = hex_width / 2;
    hex_yoffset = 3/2 * hex_size;
    
    hexCenters('0_0') = [0, 0]; 
    cellLabels('0_0') = [0, 0];

    for l = 1:span
        for m = 0:(6*l - 1)
            center = hexToCartesian(l, m);
            x = center(1); y = center(2);
            key = sprintf('%d_%d', l, m);
            hexCenters(key) = [x, y];
            cellLabels(key) = [l, m];
        end
    end
end

function walls = generateWalls(span)
    walls = containers.Map('KeyType', 'char', 'ValueType', 'any');
    for l = 0:span
        if l == 0
            walls('0_0') = {'1_0', '1_1', '1_2', '1_3', '1_4', '1_5'};
        end
        for m = 0:(6*l - 1)
            key = sprintf('%d_%d', l, m);
            neighbors = getNeighbors(l, m, span);
            wallList = {};
            for i = 1:size(neighbors, 1)
                n_key = sprintf('%d_%d', neighbors(i, 1), neighbors(i, 2));
                wallList{end+1} = n_key;
            end
            walls(key) = wallList;
        end
    end
end

% Randomly remove walls while ensuring connectivity
function walls = removeWalls(walls, span)
    all_keys = keys(walls);
    for i = 1:length(all_keys)
        key = all_keys{i};
        neighbors = walls(key);
        if ~isempty(neighbors)
            remove_idx = randi(length(neighbors));
            remove_key = neighbors{remove_idx};
            tmp = walls(key); %  First store the result in a variable
            tmp(remove_idx) = []; % Modify the variable
            walls(key) = tmp; % Assign back to the map

            if walls.isKey(remove_key)
                updated_neighbors = walls(remove_key);
                updated_neighbors = setdiff(updated_neighbors, {key}, 'stable');
                walls(remove_key) = updated_neighbors;
            end
        end
    end
end

function walls = removeWallsKruskal(walls, span)

    % Get all cell keys (each cell is a node)
    all_keys = keys(walls);
    
    % Initialize disjoint-set (each cell starts in its own set)
    parent = containers.Map(all_keys, all_keys);
    rank = containers.Map(all_keys, zeros(length(all_keys), 1));

    % Get all unique walls and shuffle them
    unique_walls = containers.Map('KeyType', 'char', 'ValueType', 'any');

    for i = 1:length(all_keys)
        key = all_keys{i};
        neighbors = walls(key);
        for j = 1:length(neighbors)
            edge = sort({key, neighbors{j}});
            edge_str = strjoin(edge, '-'); % Convert to string to ensure uniqueness
            if ~isKey(unique_walls, edge_str)
                unique_walls(edge_str) = edge;
            end
        end
    end
    
    % Convert map to cell array and shuffle walls
    all_walls = values(unique_walls);
    all_walls = all_walls(randperm(length(all_walls)));

    % Process walls with Kruskal's logic
    for i = 1:length(all_walls)
        wall = all_walls{i};
        cell1 = wall{1};
        cell2 = wall{2};
        
        % Find the root set of both cells
        [parent, root1] = findSet(parent, cell1);
        [parent, root2] = findSet(parent, cell2);
        
        % Only remove the wall if they belong to different sets (avoiding cycles)
        if ~strcmp(root1, root2)
            % Merge the sets (union operation by rank)
            [parent, rank] = unionSets(parent, rank, root1, root2);
            
            % Remove wall in both directions
            walls(cell1) = setdiff(walls(cell1), {cell2}, 'stable');
            walls(cell2) = setdiff(walls(cell2), {cell1}, 'stable');
        end
    end
end

% Disjoint-Set FIND operation with path compression
function [parent, root] = findSet(parent, node)
    if ~strcmp(parent(node), node) % If not root, find recursively
        [parent, root] = findSet(parent, parent(node)); % Recursive find
        parent(node) = root; % Path compression
    else
        root = node;
    end
end

% Disjoint-Set UNION operation with rank optimization
function [parent, rank] = unionSets(parent, rank, root1, root2)
    if rank(root1) > rank(root2)
        parent(root2) = root1;
    elseif rank(root1) < rank(root2)
        parent(root1) = root2;
    else
        parent(root2) = root1;
        rank(root1) = rank(root1) + 1;
    end
end



% Get neighboring cells
function neighbors = getNeighbors(l, m, span)
    if l == 0
        neighbors = [1, 0; 1, 1; 1, 2; 1, 3; 1, 4; 1, 5];
    else
        w = floor(m / l);
        i = mod(m, l);
        if i == 0
            neighbors = [l, mod(w * l - 1, 6*l);
                        l, mod(w * l + 1, 6*l); 
                        l-1, mod(w * (l-1), 6*(l-1))];
            if l < span
                neighbors = [neighbors;
                        l+1, mod(w * (l+1), 6*(l+1));
                        l+1, mod(w * (l+1) - 1, 6*(l+1));
                        l+1, mod(w * (l+1) + 1, 6*(l+1))];
            end
        else
            neighbors = [l, mod(w * l + i - 1, 6*l);
                        l, mod(w * l + i + 1, 6*l); 
                        l-1, mod(w * (l-1) + i - 1, 6*(l-1));
                        l-1, mod(w * (l-1) + i, 6*(l-1))];
            if l < span
                neighbors = [neighbors;
                            l+1, mod(w * (l+1) + i, 6*(l+1));
                            l+1, mod(w * (l+1) + i + 1, 6*(l+1))];
            end
        end
    end
end

% Plot hexagonal maze
function plotHexMaze(hexCenters, cellLabels, walls, hex_size, start_cell, goal_cell)
    hold on;
    for key = keys(hexCenters)
        key = key{1};
        center = hexCenters(key);
        drawHexagon(center, hex_size);
    end

    % Draw walls
    for key = keys(walls)
        key = key{1};
        center = hexCenters(key);
        neighbors = walls(key);
        key_coord = keyToCoord(key);
        all_neighbors = getNeighbors(key_coord(1), key_coord(2), 4);
        for i = 1:length(all_neighbors)
            n_key = sprintf('%d_%d', all_neighbors(i, 1), all_neighbors(i, 2));
            % Check if the neighbor is in the wall list
            if ismember(n_key, neighbors)
                continue;
            end
            if isKey(hexCenters, n_key)
                n_center = hexCenters(n_key);
                m_center = hexCenters(key);
                % Get the middle wall line
                line = getMiddleWall(m_center, n_center, 1);
                plot(line(1, :), line(2, :), 'w', 'LineWidth', 2);
            end
        end
    end

    % Mark start and goal cells
    startKey = sprintf('%d_%d', start_cell(1), start_cell(2));
    goalKey = sprintf('%d_%d', goal_cell(1), goal_cell(2));
    startPos = hexCenters(startKey); %  Store lookup result
    goalPos = hexCenters(goalKey);   %  Store lookup result

    scatter(startPos(1), startPos(2), 100, 'g', 'filled'); %  Corrected
    scatter(goalPos(1), goalPos(2), 100, 'r', 'filled');   %  Corrected


    axis equal;
    title('Hexagonal Maze');
    hold off;
end

% Draw a hexagon at a given center
function drawHexagon(center, size)
    angles = linspace(0, 2*pi, 7) + pi/2;
    x = center(1) + size * cos(angles);
    y = center(2) + size * sin(angles);
    plot(x, y, 'b', 'LineWidth', 2);
end

function center = hexToCartesian(l, m)
    w = floor(m/l); i = mod(m, l);
    rho = sqrt(3)*l;
    theta_0 = w*2*pi/6;
    theta_t = (w+1)*2*pi/6;
    x_0 = rho*cos(theta_0);
    y_0 = rho*sin(theta_0);
    x_t = rho*cos(theta_t);
    y_t = rho*sin(theta_t);
    x = x_0 + (x_t - x_0)*i/l;
    y = y_0 + (y_t - y_0)*i/l;
    center = [x, y];
end 

% Function to convert cell coordinate string to 2-tuple
function coord = keyToCoord(key)
    coord = cellfun(@str2num, strsplit(key, '_'));
end

function line = getMiddleWall(center1, center2, hex_size)
    % Get the middle point between two hexagon centers

    center1 = [center1(1); center1(2)];
    center2 = [center2(1); center2(2)];
    
    mid_point = (center1 + center2) / 2;
    p = center2 - center1;
    n = [0, -1; 1, 0] * p;
    N = n/norm(n);

    line = [mid_point - (hex_size/2)*N, mid_point + (hex_size/2)*N];
end

% Function to create paths based on wall checking
function paths = createPaths(walls, start_cell, goal_cell, span)
    % Get all cell keys (each cell is a node)
    all_keys = keys(walls);
    paths = containers.Map('KeyType', 'char', 'ValueType', 'any');

    % Get all unique walls and shuffle them
    unique_walls = containers.Map('KeyType', 'char', 'ValueType', 'any');

    for key = keys(walls)
        key = key{1};
        neighbors = walls(key);
        key_coord = keyToCoord(key);
        all_neighbors = getNeighbors(key_coord(1), key_coord(2), 4);
        for i = 1:length(all_neighbors)
            n_key = sprintf('%d_%d', all_neighbors(i, 1), all_neighbors(i, 2));
            % Check if the neighbor is in the wall list
            if ismember(n_key, neighbors)
                continue;
            end

            if (all_neighbors(i, 1) > span)
                continue;
            end
            
            % If the neighbor is not in the wall list, add it as a path
            if isKey(paths, key)
                paths(key) = [paths(key), n_key];
            else
                paths(key) = {n_key};
            end

            if isKey(paths, n_key)
                paths(n_key) = [paths(n_key), key];
            else
                paths(n_key) = {key};
            end
        end
    end
end

function drawPath(solved_path, hexCenters)
    hold on;
    for i = 1:length(solved_path) - 1
        key1 = solved_path{i};
        key2 = solved_path{i+1};
        center1 = hexCenters(key1);
        center2 = hexCenters(key2);
        % Draw a line from center1 to center2
        plot([center1(1), center2(1)], [center1(2), center2(2)], 'r', 'LineWidth', 2);
    end
end

% Algorithms to solve the maze

function [solution, stats] = solveMazeDFS(span, paths, start_cell, goal_cell)

    % Convert start and goal to string keys
    start_cell = sprintf('%d_%d', start_cell(1), start_cell(2));
    goal_cell = sprintf('%d_%d', goal_cell(1), goal_cell(2));

    % Initialize structures
    visited = containers.Map('KeyType', 'char', 'ValueType', 'logical');
    parent = containers.Map('KeyType', 'char', 'ValueType', 'char'); % Track the path

    for key = keys(paths)
        key = key{1};
        visited(key) = false;
    end

    % Statistics tracking
    iterations = 0;
    nodes_explored = 0;
    max_stack_size = 0;

    % Stack for DFS (LIFO)
    stack = {start_cell};
    visited(start_cell) = true;
    parent(start_cell) = ""; % Start has no parent

    % DFS Search
    while ~isempty(stack)
        iterations = iterations + 1;
        max_stack_size = max(max_stack_size, length(stack)); % Track max stack size
        current = stack{end};
        stack(end) = []; % Pop stack
        nodes_explored = nodes_explored + 1;

        if strcmp(current, goal_cell)
            break; % Goal reached
        end

        neighbors = paths(current);
        for i = 1:length(neighbors)
            neighbor = neighbors{i};
            if ~visited(neighbor)
                visited(neighbor) = true;
                parent(neighbor) = current; % Store where we came from
                stack{end+1} = neighbor; % Push to stack
            end
        end
    end

    % **Backtrack to reconstruct path**
    solution = {};
    if isKey(parent, goal_cell)
        step = goal_cell;
        while ~strcmp(step, "")
            solution = [step, solution]; % Prepend to path
            step = parent(step);
        end
    end

    % Path length
    path_length = length(solution) - 1; % Excluding the start node

    % Compute memory footprint
    memory_visited = length(keys(visited));
    memory_parent = length(keys(parent));
    
    % Store statistics
    stats.iterations = iterations;
    stats.nodes_explored = nodes_explored;
    stats.path_length = path_length;
    stats.memory_visited = memory_visited;
    stats.memory_parent = memory_parent;
    stats.max_stack_size = max_stack_size;

    % Print statistics
    fprintf('DFS Statistics:\n');
    fprintf('Iterations: %d\n', iterations);
    fprintf('Nodes Explored: %d\n', nodes_explored);
    fprintf('Path Length: %d\n', path_length);
    fprintf('Memory Footprint:\n');
    fprintf('  Visited Nodes Stored: %d\n', memory_visited);
    fprintf('  Parent Map Size: %d\n', memory_parent);
    fprintf('  Max Stack Size: %d\n', max_stack_size);
end

function [solution, stats] = solveMazeBFS(span, paths, start_cell, goal_cell)

    % Convert start and goal to string keys
    start_cell = sprintf('%d_%d', start_cell(1), start_cell(2));
    goal_cell = sprintf('%d_%d', goal_cell(1), goal_cell(2));

    % Initialize structures
    visited = containers.Map('KeyType', 'char', 'ValueType', 'logical');
    parent = containers.Map('KeyType', 'char', 'ValueType', 'char'); % Track path

    for key = keys(paths)
        key = key{1};
        visited(key) = false;
    end

    % Statistics tracking
    iterations = 0;
    nodes_explored = 0;
    max_queue_size = 0;

    % Queue for BFS (FIFO)
    queue = {start_cell};
    visited(start_cell) = true;
    parent(start_cell) = ""; % Start has no parent

    % BFS Search
    while ~isempty(queue)
        iterations = iterations + 1;
        max_queue_size = max(max_queue_size, length(queue)); % Track max queue size
        current = queue{1};
        queue(1) = []; % Dequeue
        nodes_explored = nodes_explored + 1;

        if strcmp(current, goal_cell)
            break; % Goal reached
        end

        neighbors = paths(current);
        for i = 1:length(neighbors)
            neighbor = neighbors{i};
            if ~visited(neighbor)
                visited(neighbor) = true;
                parent(neighbor) = current; % Store where we came from
                queue{end+1} = neighbor; % Enqueue
            end
        end
    end

    % **Backtrack to reconstruct path**
    solution = {};
    if isKey(parent, goal_cell)
        step = goal_cell;
        while ~strcmp(step, "")
            solution = [step, solution]; % Prepend to path
            step = parent(step);
        end
    end

    % Path length
    path_length = length(solution) - 1; % Excluding the start node

    % Compute memory footprint
    memory_visited = length(keys(visited));
    memory_parent = length(keys(parent));
    
    % Store statistics
    stats.iterations = iterations;
    stats.nodes_explored = nodes_explored;
    stats.path_length = path_length;
    stats.memory_visited = memory_visited;
    stats.memory_parent = memory_parent;
    stats.max_queue_size = max_queue_size;

    % Print statistics
    fprintf('BFS Statistics:\n');
    fprintf('Iterations: %d\n', iterations);
    fprintf('Nodes Explored: %d\n', nodes_explored);
    fprintf('Path Length: %d\n', path_length);
    fprintf('Memory Footprint:\n');
    fprintf('  Visited Nodes Stored: %d\n', memory_visited);
    fprintf('  Parent Map Size: %d\n', memory_parent);
    fprintf('  Max Queue Size: %d\n', max_queue_size);
end


function [solution, stats] = solveMazeAStar(span, paths, start_cell, goal_cell, hexCenters)

    % Convert start and goal to string keys
    start_cell = sprintf('%d_%d', start_cell(1), start_cell(2));
    goal_cell = sprintf('%d_%d', goal_cell(1), goal_cell(2));

    % Priority queue: {cost, node}
    openSet = containers.Map('KeyType', 'char', 'ValueType', 'double');
    openSet(start_cell) = 0; % Start node with cost 0

    % Cost maps
    gScore = containers.Map('KeyType', 'char', 'ValueType', 'double');
    fScore = containers.Map('KeyType', 'char', 'ValueType', 'double');
    parent = containers.Map('KeyType', 'char', 'ValueType', 'char'); % Path tracking

    parent(start_cell) = ""; % Start has no parent

    for key = keys(paths)
        key = key{1};
        gScore(key) = inf; % Infinite cost initially
        fScore(key) = inf;
    end
    gScore(start_cell) = 0;
    fScore(start_cell) = heuristic(start_cell, goal_cell, hexCenters);

    % Statistics tracking
    iterations = 0;
    nodes_explored = 0;
    max_open_set_size = 0;

    % A* Search
    while ~isempty(openSet)
        iterations = iterations + 1;
        max_open_set_size = max(max_open_set_size, length(openSet));

        % Get node with lowest fScore
        current = minKey(openSet, fScore);
        remove(openSet, current); % Remove from openSet
        nodes_explored = nodes_explored + 1;

        if strcmp(current, goal_cell)
            break; % Goal reached
        end
        
        neighbors = paths(current);
        for i = 1:length(neighbors)
            neighbor = neighbors{i};
            tentative_gScore = gScore(current) + 1; % Uniform cost (1 step per move)

            if tentative_gScore < gScore(neighbor)
                parent(neighbor) = current; % Track path
                gScore(neighbor) = tentative_gScore;
                fScore(neighbor) = gScore(neighbor) + heuristic(neighbor, goal_cell, hexCenters);
                openSet(neighbor) = fScore(neighbor);
            end
        end
    end

    % **Backtrack to reconstruct path**
    solution = {};
    if isKey(parent, goal_cell)
        step = goal_cell;
        while ~strcmp(step, "")
            solution = [step, solution]; % Prepend to path
            step = parent(step);
        end
    end

    % Path length
    path_length = length(solution) - 1; % Excluding the start node

    % Compute memory footprint
    memory_gScore = length(keys(gScore));
    memory_fScore = length(keys(fScore));
    memory_parent = length(keys(parent));

    % Store statistics
    stats.iterations = iterations;
    stats.nodes_explored = nodes_explored;
    stats.path_length = path_length;
    stats.memory_gScore = memory_gScore;
    stats.memory_fScore = memory_fScore;
    stats.memory_parent = memory_parent;
    stats.max_open_set_size = max_open_set_size;

    % Print statistics
    fprintf('A* Statistics:\n');
    fprintf('Iterations: %d\n', iterations);
    fprintf('Nodes Explored: %d\n', nodes_explored);
    fprintf('Path Length: %d\n', path_length);
    fprintf('Memory Footprint:\n');
    fprintf('  gScore Map Size: %d\n', memory_gScore);
    fprintf('  fScore Map Size: %d\n', memory_fScore);
    fprintf('  Parent Map Size: %d\n', memory_parent);
    fprintf('  Max Open Set Size: %d\n', max_open_set_size);
end

% **Helper function: Heuristic (Manhattan Distance)**
function h = heuristic(cell, goal, hexCenters)
    cell_coords = hexCenters(cell);
    goal_coords = hexCenters(goal);
    h = abs(cell_coords(1) - goal_coords(1)) + abs(cell_coords(2) - goal_coords(2));
end

% **Helper function: Get key with minimum fScore**
function minKey = minKey(openSet, fScore)
    keysList = keys(openSet);
    minKey = keysList{1};
    minVal = fScore(minKey);
    for i = 2:length(keysList)
        if fScore(keysList{i}) < minVal
            minKey = keysList{i};
            minVal = fScore(keysList{i});
        end
    end
end


function [solution, stats] = solveMazeBeamSearch(span, paths, start_cell, goal_cell, beam_width, hexCenters)

    % Convert start and goal to string keys
    start_cell = sprintf('%d_%d', start_cell(1), start_cell(2));
    goal_cell = sprintf('%d_%d', goal_cell(1), goal_cell(2));

    % Initialize the queue with the start cell
    current_level = {start_cell};
    visited = containers.Map('KeyType', 'char', 'ValueType', 'logical');
    visited(start_cell) = true;

    % Path tracking
    parent = containers.Map('KeyType', 'char', 'ValueType', 'char'); 

    parent(start_cell) = ""; % Start has no parent

    % Statistics tracking
    iterations = 0;
    nodes_explored = 0;
    max_level_size = 0;

    % Beam Search
    while ~isempty(current_level)
        iterations = iterations + 1;
        nodes_explored = nodes_explored + length(current_level);
        max_level_size = max(max_level_size, length(current_level));

        % Generate next level
        next_level = {};
        for i = 1:length(current_level)
            current = current_level{i};

            if strcmp(current, goal_cell)
                break;
            end

            neighbors = paths(current);
            for j = 1:length(neighbors)
                neighbor = neighbors{j};
                if ~visited.isKey(neighbor)
                    visited(neighbor) = true;
                    parent(neighbor) = current;
                    next_level = [next_level, neighbor];
                end
            end
        end

        % Check if the goal is reached
        if isKey(parent, goal_cell)
            break;
        end

        % Keep only the best "beam_width" nodes from next_level
        next_level = sortLevel(next_level, parent, goal_cell, hexCenters);
        current_level = next_level(1:min(beam_width, end));  % Keep only top "beam_width" nodes
    end

    % **Backtrack to reconstruct path**
    solution = {};
    if isKey(parent, goal_cell)
        step = goal_cell;
        while ~strcmp(step, "")
            solution = [step, solution]; % Prepend to path
            step = parent(step);
        end
    end

    % Path length
    path_length = length(solution) - 1; % Excluding the start node

    % Compute memory footprint
    memory_parent = length(keys(parent));

    % Store statistics
    stats.iterations = iterations;
    stats.nodes_explored = nodes_explored;
    stats.path_length = path_length;
    stats.memory_parent = memory_parent;
    stats.max_level_size = max_level_size;

    % Print statistics
    fprintf('Beam Search Statistics:\n');
    fprintf('Iterations: %d\n', iterations);
    fprintf('Nodes Explored: %d\n', nodes_explored);
    fprintf('Path Length: %d\n', path_length);
    fprintf('Memory Footprint:\n');
    fprintf('  Parent Map Size: %d\n', memory_parent);
    fprintf('  Max Level Size: %d\n', max_level_size);
end

% **Helper function: Sort next level by heuristic (Manhattan Distance)**
function sorted_level = sortLevel(next_level, parent, goal, hexCenters)
    % Sort next_level based on the heuristic (Manhattan distance to goal)
    scores = zeros(1, length(next_level));
    for i = 1:length(next_level)
        scores(i) = heuristic(next_level{i}, goal, hexCenters);
    end
    [~, idx] = sort(scores); % Sort by heuristic value
    sorted_level = next_level(idx);
end
