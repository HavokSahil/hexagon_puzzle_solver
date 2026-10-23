% Regenerate README figures using the actual MATLAB/Octave implementation.
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(repoRoot);
outputDir = fullfile(repoRoot, 'docs', 'images');
if ~exist(outputDir, 'dir'), mkdir(outputDir); end
previousVisibility = get(0, 'DefaultFigureVisible');
restoreVisibility = onCleanup(@() set(0, 'DefaultFigureVisible', previousVisibility));
set(0, 'DefaultFigureVisible', 'off');
rng(42, 'twister');
maze = hexmaze.generateMaze(6, 1);
[solution, stats] = hexmaze.solveMaze(maze, [6, 0], [6, 12], 'astar');
assert(stats.found);
fig = hexmaze.plotMaze(maze, solution, [6, 0], [6, 12]);
print(fig, fullfile(outputDir, 'solved-maze.png'), '-dpng', '-r150');
close(fig);
rng(42, 'twister');
results = hexmaze.compareSolvers(2:2:30, 1);
fig = hexmaze.plotComparison(results);
print(fig, fullfile(outputDir, 'solver-comparison.png'), '-dpng', '-r150');
close(fig);
clear restoreVisibility;
fprintf('Saved demo images to %s\n', outputDir);
