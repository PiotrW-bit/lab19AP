function [mymap] = createGrayInverseColormap()
%createGrayInverseColormap Creates colormap from white (0) to black (1)
% Inverse of gray colormap
    % 0 - white
    % 1 - black
    mymap = [ linspace(1,0,101)', linspace(1,0,101)', linspace(1,0,101)' ];                   
end

