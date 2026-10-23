% Run from the repository root: run('examples/demo_maze.m')
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(repoRoot);
rng(42, 'twister');
maze = hexmaze.generateMaze(6, 1);
[solution, stats] = hexmaze.solveMaze(maze, [6, 0], [6, 12], 'astar');
hexmaze.plotMaze(maze, solution, [6, 0], [6, 12]);
disp(stats);
