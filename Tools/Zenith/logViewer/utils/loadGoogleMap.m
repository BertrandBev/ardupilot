% Load Google map around the bounds
function [imagN, imagE, imag] = loadGoogleMap(initLatLong, boundN, boundE)
    % Check if the map exits in the cache
    filename = sprintf(['/map', repmat('_%f', 1, 6)], initLatLong(:), boundN(:), boundE(:));
    dir = fileparts(mfilename('fullpath'));
    path = [dir, filename, '.mat'];
    if exist(path, 'file')
        load(path);
        return;
    end    
    toRad = pi/180; toDeg = 1/toRad;
    [boxLat, boxLong] = ne2latlong(boundN, boundE, initLatLong(1)*toRad, initLatLong(2)*toRad);
    [boxLong, boxLat] = deal(boxLong*toDeg, boxLat*toDeg);
    figure('Name', 'Maps'); clf; plot(boxLong, boxLat);
    [lonVec, latVec, imag] = plot_google_map('MapType', 'satellite');

    % find the smallest square box that holds the desired bounds
    idx = find( (lonVec >= boxLong(1) & lonVec <= boxLong(2)) | (latVec >= boxLat(1) & latVec <= boxLat(2)));
    lonVec = lonVec(idx);
    latVec = latVec(idx);
    imag = imag(idx, idx, :);
    
    [imagN, imagE] = latlong2ne(latVec*toRad, lonVec*toRad, 0.*latVec,...
                                initLatLong(1)*toRad, initLatLong(2)*toRad, 0);
    
    close('Maps');
    % Save map to the cache
    if ~exist(dir, 'dir'); mkdir(dir); end
    save(path, 'imagN', 'imagE', 'imag');
end