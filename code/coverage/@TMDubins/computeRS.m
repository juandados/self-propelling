function computeRS(obj, type)
% computeRS(obj, type)
%
% Computes a reachable set specified by its type. Currently, the available
% types are as follows:
%
%   qr_qr_safeV
%
% Mo Chen, 2015-11-03
% Updated by Mahesh Vashishtha, 2015-12-03

% For now, always assume we have a reconstructed set; deal with
% reconstruction on the fly later...
visualize = true;
save_flag = false;

% Computes base reachable sets
switch type
    case 'db_db_safe_V_circle'
        % Safety between two quadrotors square domain
        %filename = [fileparts(mfilename('fullpath')) ...
        %    '/../RS_core/saved/db_db_safe_V_circle_Radius_' ...
        %    num2str(obj.cr) '_Speed_' num2str(obj.speedLimit) ...
        %    '_SafetyTime_' num2str(obj.safetyTime) '.mat'];
        filename = [fileparts(mfilename('fullpath')) ...
            '/../RS_core/saved/optimized_dubins_dubins_safe_V_circle_Radius_2_Speed_10_SafetyTime_15.mat'];
        if exist(filename, 'file')
            load(filename)
        else
            error(strcat("Run optimized_dp with the sistem parameters an save at ", filename))
        end

        if ~exist('g','var')
            save_flag = true;
            grid_min = [-20; -20; -pi; -10; -10];% Bounds on computational domain
            grid_max = [20; 20; pi; 10; 10];
            N = [40; 40; 20; 20; 20];
            pdDims = [3];
            g = createGrid(grid_min, grid_max, N, pdDims);
        end

        if ~exist('grad', 'var')
            save_flag = true;
            grad = computeGradients(g, dataV);
        end

        obj.qr_qr_safe_V.g = g;
        obj.qr_qr_safe_V.data = dataV;
        obj.qr_qr_safe_V.grad = grad;

        if save_flag
            save(filename, 'g', 'dataV', 'grad', '-v7.3');
        end
    otherwise
        error('Undefined reachable set type.')
end
end % end function
