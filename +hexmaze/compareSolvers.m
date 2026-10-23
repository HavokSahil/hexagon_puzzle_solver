function results = compareSolvers(spans, hexSize)
%COMPARESOLVERS Run all four solvers on the same maze at each span.
%   RESULTS.stats{sizeIndex, algorithmIndex} contains a solver's statistics.
%   Beam width is max(1, span). Set rng before calling for reproducibility.
    if nargin < 1, spans = 2:2:30; end
    if nargin < 2, hexSize = 1; end
    validateattributes(spans, {'numeric'}, {'vector', 'nonempty', 'integer', 'nonnegative', 'finite'});
    algorithms = {'astar', 'bfs', 'dfs', 'beam'};
    stats = cell(numel(spans), numel(algorithms));
    for i = 1:numel(spans)
        span = spans(i);
        maze = hexmaze.generateMaze(span, hexSize);
        for j = 1:numel(algorithms)
            [~, stats{i, j}] = hexmaze.solveMaze(maze, [span, 0], ...
                [span, 2*span], algorithms{j}, max(1, span));
        end
    end
    results = struct('spans', spans(:)', 'algorithms', {algorithms}, 'stats', {stats});
end
