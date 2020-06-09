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
fourD = true;
visualize = true;

% Computes base reachable sets
switch type
    case 'qr_qr_safe_optimized_dp'
        % Safety between two quadrotors square domain
        %filename = [fileparts(mfilename('fullpath')) ...
        %    '/../RS_core/saved/db_db_safe_V_circle_Radius_' ...
        %    num2str(obj.cr) '_Speed_' num2str(obj.speedLimit) ...
        %    '_SafetyTime_' num2str(obj.safetyTime) '.mat'];
        filename = [fileparts(mfilename('fullpath')) ...
            '/../RS_core/saved/optimized_quad_quad_safe_V_circle_Radius_2_Speed_10_SafetyTime_3.mat'];
        if exist(filename, 'file')
            load(filename)
        else
            error(strcat("Run optimized_dp with the sistem parameters an save at ", filename))
        end

        save_flag = false;

        if ~exist('g','var')
            save_flag = true;
            grid_min = [-30; -10; -30; -10];% Bounds on computational domain
            grid_max = [30; 10; 30; 10];
            N = [81; 31; 81; 31];
            pdDims = [];
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
