function fig = plotComparison(results)
%PLOTCOMPARISON Plot search work and peak frontier counts, marking failures.
    fig = figure('Color', 'w', 'Position', [100, 100, 1200, 420]);
    fields = {'iterations', 'nodes_explored', 'max_frontier_size'};
    titles = {'Search iterations', 'Nodes explored', 'Peak frontier (nodes)'};
    labels = {'A*', 'BFS', 'DFS', 'Beam'};
    styles = {'-o', '-s', '-^', '-d'};
    colors = [0.13, 0.43, 0.68; 0.10, 0.58, 0.42; 0.82, 0.46, 0.12; 0.62, 0.30, 0.64];
    for panel = 1:3
        subplot(1, 3, panel); hold on;
        handles = cell(1, numel(results.algorithms));
        for algorithm = 1:numel(results.algorithms)
            measurements = zeros(size(results.spans));
            failed = false(size(results.spans));
            for i = 1:numel(results.spans)
                stats = results.stats{i, algorithm};
                measurements(i) = stats.(fields{panel});
                failed(i) = ~stats.found;
            end
            handles{algorithm} = plot(results.spans, measurements, styles{algorithm}, ...
                'Color', colors(algorithm, :), 'LineWidth', 1.5, 'MarkerSize', 4);
            plot(results.spans(failed), measurements(failed), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
        end
        title(titles{panel}); xlabel('Span'); ylabel('Count'); grid on;
        legend([handles{:}], labels, 'Location', 'northwest');
        hold off;
    end
end
