% Run from the repository root in MATLAB or GNU Octave.
% For a single solved maze, see examples/demo_maze.m.
rng(42, 'twister');
results = hexmaze.compareSolvers(2:2:30, 1);
hexmaze.plotComparison(results);
