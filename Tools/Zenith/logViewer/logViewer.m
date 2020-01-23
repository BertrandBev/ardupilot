% Z1 Logs viewer script
% Created Jan 17, 2020
% Bertrand Bevillard <bertrand@zenithaero.com>
%
% --------------------------------------------------------------------- #
% Dev log viewer
% --------------------------------------------------------------------- #
%
% Handy log plotting tool
% Setup Z1 path 
% --------------------------------------------------------------------- #

% Instanciate the core
if ~exist('core', 'var'); core = Core(); end

% Path settings ----------
core.ardupilotPath = getenv('ARDUPILOT_PATH');

% Time range presets -----
core.timeRange = {-1, -1}; % Plots time window
% ------------------------

% Figure preferences -----
core.mainTitle    = 'Small circle'; % Optional main title
core.mainTitleTags = false;         % Appends the plotName to the title
core.overrideFigs  = true;          % Override previous figure
core.dockFigs      = true;          % Docs all figures
core.saveFigs      = false;         % Save figures
% -----------------------

% Plot descriptor --------
core.logFiles     = {'log.mat'};    % {folder of logs, log files or empty for newlogs} 'gnc.zlink.mat'
core.cacheLogs    = false;  % Keep logs in memory
% ------------------------

% ------------------------
% 2d_pos       - 2d pos plot
% 2d_pos_map   - 2d pos plot with map
% 3d_pos       - 3d pos plot
% alt          - Alt plot
% body_long    - Body frame longitudinal plot
% body_lat     - Body frame lateral plot
% anomalies    - Anomalies plot
% ------------------------
core.plots = {'2d_pos'}; %, %{'3d_pos', '2d_pos', '2d_pos_map', 'alt', 'body_long', 'body_lat'};

core.init();
toDeg = 180/pi;

% Main loop
loopSize = [length(core.logFiles), length(core.plots)];
for k = 1:prod(loopSize)
    % Unpack index & init loop
    [l, p] = ind2sub(loopSize, k);
    log = core.getLog(l, p);
    
    % Dataframes shortcuts
    pos = log.LOCAL_POSITION_NED;
    
        
    % 2d pos --------------------------------------------------------------
    if core.isPlot('2d_pos')
        core.figureInit();
        rows = 1;
        
        [~, N] = core.trim(pos.timestamp, pos.x);
        [~, E] = core.trim(pos.timestamp, pos.y);
%         [~, legId] = core.trim(internal2, state_machine.legId);
%         [~, sublegId] = core.trim(internal2, state_machine.sublegId);
        
        core.subplotInit(rows, false);
%         % Leader track
%         cmap = hsv(length(lN));
%         scatter(lE, lN, [], cmap); hold on
%         h1 = plot(lE, lN, '--', 'linewidth', 2); 
%         % Waypoints
%         events = logical([0; diff(Ncmd) ~= 0 | diff(Ecmd) ~= 0]);
%         h2 = scatter(Ecmd(events), Ncmd(events), 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k');
%         
%         legId = legId(events);
%         sublegId = sublegId(events);
%         wpts = arrayfun(@(idx) sprintf('%d.%d', legId(idx), sublegId(idx)), 1:length(legId), 'UniformOutput', false);
%         text(double(Ecmd(events)) + 10, double(Ncmd(events)), wpts)
        
        % position
        h3 = plot(E, N, 'linewidth', 2);
        
        
%         legend([h1, h2, h3], {'Leader', 'Waypoints', 'Pos'});
        core.subplotFinalize('E', 'N', 'Position');
%         set(gca,'Ydir','reverse')
        axis equal
    end
    
    % 2d pos map ----------------------------------------------------------
    if core.isPlot('2d_pos_map')
        core.figureInit();
        rows = 1;
        
        initLatLon = [estimator.latitude(1)*toDeg, estimator.longitude(1)*toDeg];
        [~, Ncmd] = core.trim(internal2, auto_controller.NPosCmd);
        [~, Ecmd] = core.trim(internal2, auto_controller.EPosCmd);
        [~, NcmdFilt] = core.trim(internal2,auto_controller.NPosCmdFilt);
        [~, EcmdFilt] = core.trim(internal2,auto_controller.EPosCmdFilt);
        [~, N] = core.trim(internal2,estimator.ENU_N);
        [~, E] = core.trim(internal2,estimator.ENU_E);
        [~, lN] = core.trim(internal2,auto_controller.CTOL_leaderN);
        [~, lE] = core.trim(internal2,auto_controller.CTOL_leaderE);
        [~, agl] = core.trim(internal2,estimator.agl);
        
        % Load map
        mapPaddingPercent = 0.2; minLength = 20; % [m]
        [boundN, boundE] = deal([min(N), max(N)], [min(E), max(E)]);
        [padN, padE] = deal(max(diff(boundN)*mapPaddingPercent, minLength/2), max(diff(boundE)*mapPaddingPercent, minLength/2));
        [boundN, boundE] = deal(boundN + [-padN, padN], boundE + [-padE, padE]);
        [mapN, mapE, mapImg] = loadGoogleMap(initLatLon, boundN, boundE);
        
        % Plot map
        core.subplotInit(rows);
        image(mapE, mapN, mapImg); alpha(.6); axis xy; hold on
        plot(lE, lN, '*');
        plot(Ecmd, Ncmd, '-.', 'linewidth', 2); hold on
        plot(EcmdFilt, NcmdFilt, '--', 'linewidth', 2);
        plot3(E, N, agl, 'r', 'linewidth', 2);
        legend('LeaderPos', 'Cmd', 'CmdFilt', 'Pos');
        axis equal; xlim(sort([mapE(1), mapE(end)])); ylim(sort([mapN(1), mapN(end)]));
        xlabel('Easting [m]'); ylabel('Northing [m]'); zlabel('Altitude AGL [m]');
        grid on; view(0, 90);
    end
    
    % 3D pos --------------------------------------------------------------
    if core.isPlot('3d_pos')
        core.figureInit();
        rows = 1;
        
        % Load data
        initLatLon = [estimator.latitude(1)*toDeg, estimator.longitude(1)*toDeg];
        [~, N] = core.trim(internal2,estimator.ENU_N);
        [~, E]  = core.trim(internal2,estimator.ENU_E);
        [~, agl] = core.trim(internal2,estimator.agl);
        
        % Load map
        mapPaddingPercent = 0.2; minLength = 20; % [m]
        [boundN, boundE] = deal([min(N), max(N)], [min(E), max(E)]);
        [padN, padE] = deal(max(diff(boundN)*mapPaddingPercent, minLength/2), max(diff(boundE)*mapPaddingPercent, minLength/2));
        [boundN, boundE] = deal(boundN + [-padN, padN], boundE + [-padE, padE]);
        [mapN, mapE, mapImg] = loadGoogleMap(initLatLon, boundN, boundE);
        
        % Plot map
        core.subplotInit(rows);
        image(mapE, mapN, mapImg); alpha(.6); axis xy; hold on
        plot3(E, N, agl, 'r', 'linewidth', 2);
        axis equal; xlim(sort([mapE(1), mapE(end)])); ylim(sort([mapN(1), mapN(end)]));
        xlabel('Easting [m]'); ylabel('Northing [m]'); zlabel('Altitude AGL [m]');
        grid on; view(-30, 30);
    end
    
    % Altitude -----------------------------------------------------------
    if core.isPlot('alt')
        core.figureInit();
        rows = [1 2];
        
        core.subplotInit(rows);
        core.plotCurve(internal2, auto_controller.altCmd, 'altCmd', ':');
        core.plotCurve(internal2, auto_controller.altCmdRaw, 'altCmdRaw', '-.');
        core.plotCurve(internal2, auto_controller.altCmdFilt, 'altCmdFilt', '--');
        core.plotCurve(internal2, estimator.altFilt, 'altFilt', '-');
        core.plotCurve(internal2, estimator.agl, 'agl', '-');
        core.subplotFinalize('', 'm', 'Altitude', true);
        
        core.subplotInit(rows);
        core.plotCurve(internal2, auto_controller.blendedhDotCmd, 'hDotCmdBlend', '--');
        core.plotCurve(internal2, auto_controller.altRateFF, 'altRateFF', '-.');
        core.plotCurve(internal2, estimator.altRateFilt, 'altRateFilt', '-');
        core.subplotFinalize('', 'm/s', 'Alt rate', true);

        core.subplotInit(rows);
        core.plotCurve(internal2, auto_controller.fzCmd, 'fzCmd', '--');
        core.plotCurve(internal1, internal1.actuation.common.fzReqClip, 'fzRet', '-');
        core.subplotFinalize('time (s)', 'N', 'Fz', true);
    end

    % Body longitudinal ---------------------------------------------------
    if core.isPlot('body_long')
        core.figureInit();
        rows = [2, 3]; % [column 1, column 2]...
        
        [Ncmd, Ecmd] = NEtoXY(auto_controller.NPosCmd, auto_controller.EPosCmd, estimator.yaw);
        [NcmdFilt, EcmdFilt] = NEtoXY(auto_controller.NPosCmdFilt, auto_controller.EPosCmdFilt, estimator.yaw);
        [N, E] = NEtoXY(estimator.ENU_N, estimator.ENU_E, estimator.yaw);

        % Position
        core.subplotInit(rows);
        core.plotCurve(internal2, Ncmd, 'xCmd', '-.');
        core.plotCurve(internal2, NcmdFilt, 'xCmdFilt', '--');
        core.plotCurve(internal2, N, 'x', '-', 2);
        core.subplotFinalize('', 'm', 'Position', true);
        
        % Velocity
        core.subplotInit(rows);
        core.plotCurve(internal2, auto_controller.airspeedCmdFilt, 'iasCmd', '-.');
        core.plotCurve(internal2, estimator.iasCFLP, 'iasFilt_{est}', [], [], 10);
        core.plotCurve(internal2, auto_controller.uCmd, 'uCmd', '--');
        core.plotCurve(internal2, estimator.groundForwardSpeed, 'u_{est}');
        core.subplotFinalize('time (s)', 'm/s', 'Body U & IAS', true);
        
        % Pitch
        core.subplotInit(rows);
        core.plotCurve(internal2, auto_controller.pitchCmd*toDeg, 'pitchCmdVTOL', '-.');
        core.plotCurve(internal2, auto_controller.blendedThetaCmd*toDeg, 'pitchBlendedCmd', '--');
        core.plotCurve(internal2, auto_controller.blendedThetaCmdFilt*toDeg, 'pitchBlendedCmd_{filt}');
        core.plotCurve(internal2, estimator.pitch*toDeg, 'pitch_{est}');
        core.subplotFinalize('', 'deg', 'Pitch', true);
        
        % Pitch rate
        core.subplotInit(rows);
        core.plotCurve(internal2, auto_controller.CTOL_qCmd*toDeg, 'CTOL qCmd', '-.');
        core.plotCurve(internal2, auto_controller.qCmd*toDeg, 'VTOL qCmd', '--');
        core.plotCurve(internal2, estimator.q*toDeg, 'q_{est}', [], [], 10);
        core.subplotFinalize('', 'deg/s', 'Pitch rate', true);
        
        % Moment & elevator
        core.subplotInit(rows);
        core.plotCurve(internal2, auto_controller.CTOL_aileronCmd, 'aileronCmd');
        core.plotCurve(internal2, auto_controller.myCmd, 'myCmd');
        core.subplotFinalize('time (s)', 'Nm, rad', 'Actuation', true);
    end
    
    % Body lateral --------------------------------------------------------
    if core.isPlot('body_lat')
        core.figureInit();
        rows = [3];
        
        [Ncmd, Ecmd] = NEtoXY(auto_controller.NPosCmd, auto_controller.EPosCmd, estimator.yaw);
        [NcmdFilt, EcmdFilt] = NEtoXY(auto_controller.NPosCmdFilt, auto_controller.EPosCmdFilt, estimator.yaw);
        [N, E] = NEtoXY(estimator.ENU_N, estimator.ENU_E, estimator.yaw);
        
        % Roll
        core.subplotInit(rows);
        core.plotCurve(internal2, auto_controller.rollCmd*toDeg, 'VTOL rollCmd', '-.');
        core.plotCurve(internal2, auto_controller.CTOL_rollCmd*toDeg, 'CTOL rollCmd', '-.');
        core.plotCurve(internal2, auto_controller.blendedRollCmd*toDeg, 'rollCmdBlended', '-.');
        core.plotCurve(internal2, auto_controller.blendedRollCmdFilt*toDeg, 'rollBlended_{filt}', '--');
        core.plotCurve(internal2, estimator.roll*toDeg, 'roll_{est}');
        core.subplotFinalize('time (s)', 'deg', 'Roll', true);
        
        % Roll rate
        core.subplotInit(rows);
        core.plotCurve(internal2, auto_controller.pCmd*toDeg, 'pCmd', '--', [], 10);
        core.plotCurve(internal2, estimator.p*toDeg, 'p_{est}', [], [], 10);
        core.subplotFinalize('time (s)', 'deg/s', 'Roll rate', true);
        
        % Mx
        core.subplotInit(rows);
        core.plotCurve(internal2, auto_controller.mxCmd, 'MxCmd', [], [], 3);
        core.subplotFinalize('time (s)', 'Nm', 'Mx', true);
    end
    
    % Yaw -----------------------------------------------------------------
    if core.isPlot('yaw')
        core.figureInit();
        rows = 5;
        
        yawRate = core.smooth(estimator.r*toDeg, 50);
        yawAccel = [0; diff(yawRate)]/0.01;
        yawAccel = core.smooth(yawAccel, 50);
        yawJerk = [0; diff(yawAccel)]/0.01;

        % Yaw
        core.subplotInit(rows);
        core.plotCurve(internal2, auto_controller.yawCmd*toDeg, 'yawCmd', ':');
        core.plotCurve(internal2, auto_controller.yawCmdFilt*toDeg, 'yawCmdFilt', '--');
        core.plotCurve(internal2, estimator.yaw*toDeg, 'yaw_{est}', '-');
        core.subplotFinalize('', 'deg', 'yaw', true);
        
        % Yaw rate
        core.subplotInit(rows);
        core.plotCurve(internal2, auto_controller.rCmd*toDeg, 'rCmd', '--', [], 10);
        core.plotCurve(internal2, yawRate, 'r_{est}', [], []);
        core.subplotFinalize('time (s)', 'deg/s', 'Yaw rate', true);
        
        % Yaw accel
        core.subplotInit(rows);
        core.plotCurve(internal2, yawAccel, 'rDot', '--', []);
        core.subplotFinalize('time (s)', 'deg/s^2', 'Yaw accel', true);  
        
        % Yaw jerk
        core.subplotInit(rows);
        core.plotCurve(internal2, yawJerk, 'rDot2', '--', [], 50);
        core.subplotFinalize('time (s)', 'deg/s^3', 'Yaw jerk', true); 
        
        % Mx
        core.subplotInit(rows);
        core.plotCurve(internal2, auto_controller.mzCmd, 'MzCmd', [], [], 3);
        core.subplotFinalize('time (s)', 'Nm', 'Mz', true);
    end
    
    
    % 2d anim -------------------------------------------------------------
    if core.isPlot('2d_anim')
        core.figureInit();
        rows = 1; % [column 1, column 2]...
        
        % Settings
        speed = 60;
        saveAnim = true;
        frameRate = 20;
        
        [~, Ncmd] = core.trim(internal2, auto_controller.NPosCmd);
        [~, Ecmd] = core.trim(internal2, auto_controller.EPosCmd);
        [~, NcmdFilt] = core.trim(internal2,auto_controller.NPosCmdFilt);
        [~, EcmdFilt] = core.trim(internal2,auto_controller.EPosCmdFilt);
        [~, N] = core.trim(internal2,estimator.ENU_N);
        [~, E] = core.trim(internal2,estimator.ENU_E);
        [~, lN] = core.trim(internal2,auto_controller.CTOL_leaderN);
        [~, lE] = core.trim(internal2,auto_controller.CTOL_leaderE);
        [~, yaw] = core.trim(internal2,estimator.yaw);
       
        % Plot constant info
        events = logical([0; diff(Ncmd) ~= 0 | diff(Ecmd) ~= 0]);
        scatter(Ecmd(events), Ncmd(events), 'MarkerFaceColor', 'k'); hold on
        plot(lE, lN, '--', 'LineWidth', 2);
        xlabel('Easting [m]'); ylabel('Northing [m]');
        axis equal; grid on
        
        % Create the plots
        leaderLine =  plot([0, 0], [0, 0], '-');
        cmd = plot(0, 0, 'Marker', 'square', 'MarkerSize', 15, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'g');
        leader = plot(0, 0, 'Marker', 'o', 'MarkerSize', 10, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'g');
        posTrack = plot(0, 0, ':', 'LineWidth', 2);
        
        % Set bounds
        nBounds = [min(N), max(N)];
        eBounds = [min(E), max(E)];
        padding = 0.1*max(nBounds(2) - nBounds(1), eBounds(2) - eBounds(1));
        xlim([eBounds(1) - padding, eBounds(2) + padding]);
        ylim([nBounds(1) - padding, nBounds(2) + padding]);

        vehicleLength = padding/3;
        vehicle = plotVehicle([], 0, 0, 0, vehicleLength);
        
        % Setup videoWriter
        if saveAnim
            v = VideoWriter(strrep(core.mainTitle, ' ', '_'), 'MPEG-4'); %#ok<TNMLP>
            v.FrameRate = frameRate;
            open(v);
        end
        
        for z = 1:speed:length(Ncmd)
            t = tic;
            cmd.XData = Ecmd(z);
            cmd.YData = Ncmd(z);
            leader.XData = lE(z);
            leader.YData = lN(z);
            d = sqrt((Ncmd(z) - N(z))^2 + (Ecmd(z) - E(z))^2);
            leaderLine.XData = [E(z), lE(z)];
            leaderLine.YData = [N(z), lN(z)];
            posTrack.XData(end + 1) = E(z);
            posTrack.YData(end + 1) = N(z);
            plotVehicle(vehicle,  E(z), N(z),yaw(z), vehicleLength);
            drawnow;
            if saveAnim; writeVideo(v, getframe(gcf)); end
            pause(max(1/frameRate - toc(t), 0));
        end
        
        if saveAnim; close(v); end
    end
    
    core.figureFinalize();
end

% Helpers -----------------------------------------------------------------
% TODO: add to core
function h = plotVehicle(h, east, north, yaw, length)
    if isempty(h)
        h = fill(east, north, 'r');
    end
    h.XData = east - length*[0; sin(yaw + pi/6); sin(yaw - pi/6)];
    h.YData = north - length*[0; cos(yaw + pi/6); cos(yaw - pi/6)];
end
        
function [x, y] = NEtoXY(N, E, yaw)
    x = N.*cos(yaw) + E.*sin(yaw);
    y = E.*cos(yaw) - N.*sin(yaw);
end