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
            '/../RS_core/saved/qr_qr_safe_V_circle_Radius_' ...
            num2str(obj.cr) '_Speed_' num2str(obj.speedLimit) ...
            '_SafetyTime_' num2str(obj.safetyTime) '.mat'];
        if exist(filename, 'file')
            load(filename)
        else
            [data, grad, g, tau] = ...
                quad_quad_collision_4D(obj.cr, obj.speedLimit, obj.safetyTime, visualize);
            % Is the grad properly computed? or should it computed using
            % computecostate?
            save(filename, 'g', 'data', 'grad', 'tau', '-v7.3');
        end
        obj.qr_qr_safe_V.g = g;
        obj.qr_qr_safe_V.data = data;
        obj.qr_qr_safe_V.grad = grad;
        obj.qr_qr_safe_V.tau = tau;        
    case 'qr_qr_safe_V'
        % Safety between two quadrotors square domain
        % TO DO: Change the filename when working with projections
        filename = [fileparts(mfilename('fullpath')) ...
          '/../RS_core/saved/qr_qr_safe_V_' ...
          num2str(obj.cr) '_' num2str(obj.speed_limit) '.mat'];

        if exist(filename, 'file')
            load(filename)
        else
            [data, g, tau] = ...
                quad_quad_collision_2D(obj.cr, obj.speed_limit, visualize);
            if fourD
                % Reconstruct the base reachable set
                gridLim = [g.min-1 g.max+1; g.min-1 g.max+1];
                [~, ~, TTR_out] = ...
                  recon2x2D(tau, {g; g}, {data; data}, gridLim, tau(end));
                g = TTR_out.g;
                data = TTR_out.value;
                grad = TTR_out.grad;
                save(filename, 'g', 'data', 'grad', 'tau', '-v7.3');
            else
                grad = [];
            end
        end
        obj.qr_qr_safe_V.g = g;
        obj.qr_qr_safe_V.data = data;
        obj.qr_qr_safe_V.grad = grad;
        obj.qr_qr_safe_V.tau = tau;  
    otherwise
        error('Undefined reachable set type.')
end
end % end function
