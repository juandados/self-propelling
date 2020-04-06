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
    case 'qr_qr_safe_V_circle'
        % Safety between two quadrotors square domain
        filename = [fileparts(mfilename('fullpath')) ...
            '/../RS_core/saved/db_db_safe_V_circle_Radius_' ...
            num2str(obj.cr) '_Speed_' num2str(obj.speedLimit) ...
            '_SafetyTime_' num2str(obj.safetyTime) '.mat'];
        if exist(filename, 'file')
            load(filename)
        else
            [ttr, ttr_grad, g, dataV] = ...
                dubins_dubins_collision(obj.cr, obj.speedLimit, obj.safetyTime, visualize);
            % Is the grad properly computed? or should it computed using
            % computecostate?
            save(filename, 'g', 'ttr', 'ttr_grad', 'dataV', '-v7.3');
        end
        obj.db_db_safe_V.g = g;
        obj.db_db_safe_V.ttr = ttr;
        obj.db_db_safe_V.ttr_grad = ttr_grad;
        %obj.qr_qr_safe_V.tau = tau;
        %obj.qr_qr_safe_V.ttr = dataV;
        %obj.qr_qr_safe_V.ttr_grad = computeGradients(g, dataV);
    otherwise
        error('Undefined reachable set type.')
end
end % end function
