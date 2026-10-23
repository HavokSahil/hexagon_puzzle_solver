function maze = generateMaze(span, hexSize)
%GENERATEMAZE Build a perfect hexagonal maze using randomized Kruskal.
%   MAZE = hexmaze.generateMaze(SPAN, HEXSIZE) has 1+3*SPAN*(SPAN+1)
%   cells. Cells use [ring, position] coordinates, with positions 0:6*ring-1.
%   Set rng before calling to reproduce a maze. All graph indices are 1-based.
    if nargin < 2, hexSize = 1; end
    validateattributes(span, {'numeric'}, {'scalar', 'integer', 'nonnegative', 'finite'});
    validateattributes(hexSize, {'numeric'}, {'scalar', 'positive', 'finite'});
    count = 1 + 3 * span * (span + 1);
    cells = zeros(count, 2);
    centers = zeros(count, 2);
    neighbors = cell(count, 1);
    edges = zeros(0, 2);
    for ring = 0:span
        for position = 0:max(0, 6*ring-1)
            id = cellIndex(ring, position);
            cells(id, :) = [ring, position];
            if ring > 0
                sector = floor(position / ring);
                fraction = mod(position, ring) / ring;
                angles = [sector, sector+1] * pi/3;
                corners = sqrt(3)*ring*hexSize * [cos(angles); sin(angles)];
                centers(id, :) = ((1-fraction)*corners(:, 1) + fraction*corners(:, 2))';
            end
            coords = neighborCoordinates(ring, position, span);
            ids = zeros(1, size(coords, 1));
            for k = 1:numel(ids)
                ids(k) = cellIndex(coords(k, 1), coords(k, 2));
            end
            neighbors{id} = ids;
            other = ids(ids > id);
            edges = [edges; repmat(id, numel(other), 1), other(:)]; %#ok<AGROW>
        end
    end

    % Union by rank and path compression; each accepted edge opens a passage.
    parent = 1:count;
    ranks = zeros(1, count);
    open = false(size(edges, 1), 1);
    accepted = 0;
    for edge = randperm(size(edges, 1))
        [parent, a] = findRoot(parent, edges(edge, 1));
        [parent, b] = findRoot(parent, edges(edge, 2));
        if a == b, continue; end
        if ranks(a) < ranks(b)
            parent(a) = b;
        else
            parent(b) = a;
            if ranks(a) == ranks(b), ranks(a) = ranks(a) + 1; end
        end
        open(edge) = true;
        accepted = accepted + 1;
        if accepted == count-1, break; end
    end
    paths = cell(count, 1);
    for edge = find(open)'
        a = edges(edge, 1); b = edges(edge, 2);
        paths{a}(end+1) = b;
        paths{b}(end+1) = a;
    end
    maze = struct('span', span, 'hex_size', hexSize, 'cells', cells, ...
        'centers', centers, 'neighbors', {neighbors}, 'paths', {paths}, ...
        'walls', edges(~open, :));
end

function id = cellIndex(ring, position)
    if ring == 0, id = 1; else, id = 2 + 3*ring*(ring-1) + position; end
end

function coords = neighborCoordinates(ring, position, span)
    if ring == 0
        if span == 0, coords = zeros(0, 2); else, coords = [ones(6, 1), (0:5)']; end
        return;
    end
    sector = floor(position/ring);
    offset = mod(position, ring);
    coords = [ring, mod(position-1, 6*ring); ring, mod(position+1, 6*ring)];
    if ring == 1
        coords(end+1, :) = [0, 0];
    elseif offset == 0
        coords(end+1, :) = [ring-1, sector*(ring-1)];
    else
        coords = [coords; ring-1, sector*(ring-1)+offset-1; ...
            ring-1, mod(sector*(ring-1)+offset, 6*(ring-1))];
    end
    if ring < span
        positions = sector*(ring+1)+offset + [0, 1];
        if offset == 0, positions = [positions, sector*(ring+1)-1]; end
        coords = [coords; repmat(ring+1, numel(positions), 1), mod(positions(:), 6*(ring+1))];
    end
end

function [parent, root] = findRoot(parent, node)
    root = node;
    while parent(root) ~= root, root = parent(root); end
    while parent(node) ~= node
        next = parent(node);
        parent(node) = root;
        node = next;
    end
end
