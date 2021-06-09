function initializePlotter(AX, grid_size)
% initializePlotter initalize axes and colorbar of plot
    hold(AX, 'on');
    grid(AX, 'on');

    % Initialize colormap of image
    % 0 - white
    % 1 - black
    mymap = Plotting.createGrayInverseColormap();
    colormap(AX, mymap);
    caxis(AX,[0,1]);
    c_bar = colorbar;
    c_bar.Label.String = 'Probability of occupancy';
    % set axes limits
    axis(AX, 'equal');
    xlim(AX,[0,grid_size]);
    ylim(AX,[0,grid_size]);
    xlabel(AX,'X, m');
    ylabel(AX,'Y, m');   
end

