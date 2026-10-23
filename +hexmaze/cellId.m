function id = cellId(maze, coordinate)
%CELLID Convert a [ring, position] coordinate into a graph index.
    validateattributes(coordinate, {'numeric'}, {'size', [1, 2], 'integer', 'nonnegative', 'finite'});
    ring = coordinate(1); position = coordinate(2);
    if ring > maze.span || (ring == 0 && position ~= 0) || (ring > 0 && position >= 6*ring)
        error('hexmaze:InvalidCell', 'Cell [%d, %d] is outside this maze.', ring, position);
    end
    if ring == 0, id = 1; else, id = 2 + 3*ring*(ring-1) + position; end
end
