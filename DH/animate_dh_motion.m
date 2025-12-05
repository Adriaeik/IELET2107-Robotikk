function animate_dh_motion(dh_table, trajectory, T)
    if nargin < 3
        T = linspace(0, size(trajectory,1)/30, size(trajectory,1)); % fallback-tid
    end

    fig = figure('Name','SCARA-animert DH-visualisering');
    ax = axes('Parent',fig); grid on; axis equal; view(30,30);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('SCARA-robot – DH-basert animasjon');
    hold on;

    param_fields = {'theta','d','a','alpha'};

    % Predefiner joint types for rask oppdatering
    jt = repmat('R',1,dh_table.num_joints);
    for k = 1:dh_table.num_variables
        jidx = dh_table.param_info.indices(k,1);
        if strcmp(dh_table.param_info.types{k}, 'translation')
            jt(jidx) = 'P';
        end
    end

    for k = 1:size(trajectory,1)
        % Oppdater leddvariablar
        idx = 1;
        for j = 1:dh_table.num_joints
            if jt(j) == 'R'
                dh_table.joints(j).theta.value = trajectory(k,idx); idx=idx+1;
            else
                dh_table.joints(j).d.value = trajectory(k,idx); idx=idx+1;
            end
        end

        % Teikn robot
        cla(ax);
        T_all = eye(4);
        O = zeros(3, dh_table.num_joints+1);
        O(:,1) = [0;0;0];
        for j = 1:dh_table.num_joints
            jn = dh_table.joints(j);
            A = dh_transform(jn.theta.value, jn.d.value, jn.a.value, jn.alpha.value);
            T_all = T_all * A;
            O(:,j+1) = T_all(1:3,4);
        end
        plot3(ax, O(1,:),O(2,:),O(3,:), 'k-o','LineWidth',2);
        axis(ax, 'equal');
        axis(ax,[-2 2 -2 2 -0.5 1]);
        title(ax,sprintf('t = %.2f s', T(k)));
        drawnow;
        pause(0.02);
    end

    function A = dh_transform(theta,d,a,alpha)
        ct = cos(theta); st = sin(theta);
        ca = cos(alpha); sa = sin(alpha);
        A = [ct, -st*ca,  st*sa, a*ct;
             st,  ct*ca, -ct*sa, a*st;
             0,   sa,     ca,    d;
             0,   0,      0,     1];
    end
end
