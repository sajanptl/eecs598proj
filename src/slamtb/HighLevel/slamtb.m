% SLAMTB  An EKF-SLAM algorithm with simulator and graphics.
%
%   This script performs multi-robot, multi-sensor, multi-landmark 6DOF
%   EKF-SLAM with simulation and graphics capabilities.
%
%   Please read slamToolbox.pdf in the root directory thoroughly before
%   using this toolbox.
%
%   - Beginners should not modify this file, just edit USERDATA.M and enter
%   and/or modify the data you wish to simulate.
%
%   - More advanced users should be able to create new landmark models, new
%   initialization methods, and possibly extensions to multi-map SLAM. Good
%   luck!
%
%   - Expert users may want to add code for real-data experiments. 
%
%   See also USERDATA, USERDATAPNT, USERDATALIN.
%
%   Also consult slamToolbox.pdf in the root directory.

%   Created and maintained by
%   Copyright 2008, 2009, 2010 Joan Sola @ LAAS-CNRS.
%   Copyright 2011, 2012, 2013 Joan Sola.
%   Programmers (for parts of the toolbox):
%   Copyright David Marquez and Jean-Marie Codol @ LAAS-CNRS
%   Copyright Teresa Vidal-Calleja @ ACFR.
%   See COPYING.TXT for full copyright license.

%% OK we start here

% clear workspace and declare globals
clear
global Map    
global PinholeProj
global numLocalFrames

% No of frames to keep in QSOM
numLocalFrames = 10;
%% I. Specify user-defined options - EDIT USER DATA FILE userData.m

userData;           % user-defined data. SCRIPT.
% userDataPnt;        % user-defined data for points. SCRIPT.
% userDataLin;        % user-defined data for lines. SCRIPT.


%% II. Initialize all data structures from user-defined data in userData.m
% SLAM data
[Rob,Sen,Raw,Lmk,Obs,Tim]     = createSlamStructures(...
    Robot,...
    Sensor,...      % all user data
    Time,...
    Opt);

% Simulation data
[SimRob,SimSen,SimLmk,SimOpt] = createSimStructures(...
    Robot,...
    Sensor,...      % all user data
    World,...
    SimOpt);

% Graphics handles
[MapFig,SenFig]               = createGraphicsStructures(...
    Rob, Sen, Lmk, Obs,...      % SLAM data
    SimRob, SimSen, SimLmk,...  % Simulator data
    FigOpt);                    % User-defined graphic options


%% III. Initialize data logging
% TODO: Create source and/or destination files and paths for data input and
% logs.
% TODO: do something here to collect data for post-processing or
% plotting. Think about collecting data in files using fopen, fwrite,
% etc., instead of creating large Matlab variables for data logging.

% Clear user data - not needed anymore
clear Robot Sensor World Time   % clear all user data

q=0;
q2=0;

w = 0;
w2 = 0;
dt = Tim.dt;

Nf = 72;
Na = 0;

n = Na + Nf;

h = zeros(Nf,2);
h2 = zeros(Nf,2);
p = 0;
p2 = 0;
r2 = 0;

currentFeatures = [];

% fh = figure;


%% IV. Main loop
for currentFrame = Tim.firstFrame : Tim.lastFrame
    
    % 1. SIMULATION
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Simulate robots
    for rob = [SimRob.rob]

        % Robot motion
        SimRob(rob) = simMotion(SimRob(rob),Tim);
        
        % Simulate sensor observations
        for sen = SimRob(rob).sensors

            % Observe simulated landmarks
            Raw(sen) = simObservation(SimRob(rob), SimSen(sen), SimLmk, SimOpt) ;

        end % end process sensors

    end % end process robots
    
    
    % 1.5 REDUCE NUMBER OF FEATURES NEEDED
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     figure(fh);
%     surf(PinholeProj(:,Raw(sen).data.points.app));
%     drawnow

    if currentFrame == 1
        q_0 = SimRob.state.x(4:end);
        euls_0 = q2e(q_0);
        
    elseif currentFrame == 2
        q_1 = SimRob.state.x(4:end);
        euls_1 = q2e(q_1);
        
        
    elseif currentFrame == 3
        q_2 = SimRob.state.x(4:end);
        euls_2 = q2e(q_2);
        
        domega_0 = (euls_1 - euls_0)/dt;
        domega_1 = (euls_2 - euls_1)/dt;
        
        p = PinholeProj';
        r = SimRob.state.x(1:3);
        
        currentFeatures = Raw(sen).data.points.app;
        tempH = Raw(sen).data.points.coord'; 
        h(currentFeatures,:) = tempH;
        
        % No of rows per feature %%%%%%%%%%%%%%%%%%%%%%%%%%%
        % PUT THIS IN A BETTER PLACE
        RowsPerFeature = 146; %size(Q_SOM,1)/n;
        
        
    else
        %% The General case
        euls_0 = euls_1;
        euls_1 = euls_2;
        q_1 = q_2;
        q_2 = SimRob.state.x(4:end);
        euls_2 = q2e(q_2);
        
        p = p2;
        p2 = PinholeProj';
        
        r = r2;
        r2 = SimRob.state.x(1:3);
        
        h = h2;
        currentFeatures = Raw(sen).data.points.app;
        tempH = Raw(sen).data.points.coord'; 
        h2(currentFeatures, :) = tempH;       
        
        domega_0 = (euls_1 - euls_0)/dt;
        domega_1 = (euls_2 - euls_1)/dt;       
        
        
        [H, F, Q, OMEGA] = SysObsMeasure(q_1,  q_2, domega_0, domega_1, dt, Nf, Na, h, h2, r, r2, p, p2);
        Q_SOM = (formSOM(F, H, n, currentFrame, Q, OMEGA, dt));
        
%         size(Q_SOM)
        
        %% USE QSOM TO RANK CURRENT FEATURES
        
        % Compute Observability score
        TAUS = ComputeObsScoreFromQSOM(Q_SOM, currentFeatures, RowsPerFeature, max(1,currentFrame-5), currentFrame);
        [~,ind] = sort(TAUS,'descend');
        

        
        %% Replace Raw data for localization
%         
        NumFeaturesToUse = 5;

        currentFeatures;
        rankedFeatures = currentFeatures(ind(1:NumFeaturesToUse));
        
        Raw(sen).data.points.app = rankedFeatures;
        Raw(sen).data.points.coord = Raw(sen).data.points.coord(:,ind(1:NumFeaturesToUse)); 
        
    end
    

    % 2. ESTIMATION
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Process robots
    for rob = [Rob.rob]

        % Robot motion
        % NOTE: in a regular, non-simulated SLAM, this line is not here and
        % noise just comes from the real world. Here, the estimated robot
        % is noised so that the simulated trajectory can be made perfect
        % and act as a clear reference. The noise is additive to the
        % control input 'u'.
        Rob(rob).con.u = SimRob(rob).con.u + Rob(rob).con.uStd.*randn(size(Rob(rob).con.uStd));
        Rob(rob) = motion(Rob(rob),Tim);
        
        Map.t = Map.t + Tim.dt;                


        % Process sensor observations
        for sen = Rob(rob).sensors

            % Observe knowm landmarks
            [Rob(rob),Sen(sen),Lmk,Obs(sen,:)] = correctKnownLmks( ...
                Rob(rob),   ...
                Sen(sen),   ...
                Raw(sen),   ...
                Lmk,        ...   
                Obs(sen,:), ...
                Opt) ;

            % Initialize new landmarks
            ninits = Opt.init.nbrInits(1 + (currentFrame ~= Tim.firstFrame));
            for i = 1:ninits
                [Lmk,Obs(sen,:)] = initNewLmk(...
                    Rob(rob),   ...
                    Sen(sen),   ...
                    Raw(sen),   ...
                    Lmk,        ...
                    Obs(sen,:), ...
                    Opt) ;
            end

        end % end process sensors

    end % end process robots


    % 3. VISUALIZATION
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if currentFrame == Tim.firstFrame ...
            || currentFrame == Tim.lastFrame ...
            || mod(currentFrame,FigOpt.rendPeriod) == 0
        
        % Figure of the Map:
        MapFig = drawMapFig(MapFig,  ...
            Rob, Sen, Lmk,  ...
            SimRob, SimSen, ...
            FigOpt);
        
        if FigOpt.createVideo
            makeVideoFrame(MapFig, ...
                sprintf('map-%04d.png',currentFrame), ...
                FigOpt, ExpOpt);
        end
        
        % Figures for all sensors
        for sen = [Sen.sen]
            SenFig(sen) = drawSenFig(SenFig(sen), ...
                Sen(sen), Raw(sen), Obs(sen,:), ...
                FigOpt);
            
            if FigOpt.createVideo
                makeVideoFrame(SenFig(sen), ...
                    sprintf('sen%02d-%04d.png', sen, currentFrame),...
                    FigOpt, ExpOpt);
            end
            
        end

        % Do draw all objects
        drawnow;
    end
    

    % 4. DATA LOGGING
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % TODO: do something here to collect data for post-processing or
    % plotting. Think about collecting data in files using fopen, fwrite,
    % etc., instead of creating large Matlab variables for data logging.
    

end

%% V. Post-processing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Enter post-processing code here



% ========== End of function - Start GPL license ==========


%   # START GPL LICENSE

%---------------------------------------------------------------------
%
%   This file is part of SLAMTB, a SLAM toolbox for Matlab.
%
%   SLAMTB is free software: you can redistribute it and/or modify
%   it under the terms of the GNU General Public License as published by
%   the Free Software Foundation, either version 3 of the License, or
%   (at your option) any later version.
%
%   SLAMTB is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU General Public License for more details.
%
%   You should have received a copy of the GNU General Public License
%   along with SLAMTB.  If not, see <http://www.gnu.org/licenses/>.
%
%---------------------------------------------------------------------

%   SLAMTB is Copyright:
%   Copyright (c) 2008-2010, Joan Sola @ LAAS-CNRS,
%   Copyright (c) 2010-2013, Joan Sola,
%   Copyright (c) 2014-    , Joan Sola @ IRI-UPC-CSIC,
%   SLAMTB is Copyright 2009 
%   by Joan Sola, Teresa Vidal-Calleja, David Marquez and Jean Marie Codol
%   @ LAAS-CNRS.
%   See on top of this file for its particular copyright.

%   # END GPL LICENSE