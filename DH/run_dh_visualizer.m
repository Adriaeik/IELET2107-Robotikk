function run_dh_visualizer(dh_table,verbose)
% DH-visualisering med automatisk skalering, orienterte leddmarkørar,
% og aksevising: X=raud, Z=blå ved ALLE rammer, Y=gul KUN ved base og EE.

    if dh_table.num_variables == 0
        error('No variable parameters found in DH table!');
    end
    if nargin < 2
        verbose = false;
    end

    % ======= TUNABLE STØRRELSESFAKTORAR =======
    cfg.cyl_r      = 0.08;   % radius for revolutt-sylinder ( * base_scale)
    cfg.cyl_h      = 0.38;   % høgd for revolutt-sylinder ( * base_scale)
    cfg.prism_xy   = 0.22;   % side for kvadratisk prisme ( * base_scale)
    cfg.prism_z    = 0.40;   % høgd langs lokal z ( * base_scale)
    cfg.axis_len   = 0.90;   % lengde på akser ( * base_scale)
    cfg.axis_lw    = 3;      % line width på akser
    cfg.link_lw    = 4;      % line width på lenkjer
    cfg.pt_ms      = 10;     % markørstorleik for ramme-origin
    cfg.base_ms    = 18;     % markørstorleik for base

    % ---------- FIGUR OG AKSAR ----------
    fig = figure('Name', sprintf('DH Visualizer - %d DOF Robot', dh_table.num_joints), ...
                 'Position', [100, 100, 1200, 700], 'NumberTitle', 'off');

    ax_robot = subplot(1,2,1, 'Parent', fig);
    hold(ax_robot, 'on'); grid(ax_robot, 'on'); axis(ax_robot, 'equal'); axis(ax_robot, 'vis3d');
    xlabel(ax_robot,'X'); ylabel(ax_robot,'Y'); zlabel(ax_robot,'Z');
    title(ax_robot, sprintf('%d-DOF Manipulator - DH Frames', dh_table.num_joints));
    view(ax_robot, 25, 30);

    % ---------- KONTROLLPANEL ----------
    panel = uipanel('Parent', fig, 'Title', 'Controls', 'Position', [0.56, 0.05, 0.42, 0.9]);

    num_sliders = dh_table.num_variables;
    slider_h = 0.025; label_h = 0.028;
    spacing = min(0.08, 0.72/max(1,num_sliders+2));
    start_y = 0.93 - 0.02;

    sliders = gobjects(num_sliders,1);
    texts   = gobjects(num_sliders,1);
    unit_map = containers.Map({'rotation','translation'}, {'(rad)','(m)'});
    param_fields = {'theta','d','a','alpha'};

    for i = 1:num_sliders
        y = start_y - (i-1)*spacing;
        pname = dh_table.param_info.names{i};
        prange = dh_table.param_info.ranges(i,:);
        ptype  = dh_table.param_info.types{i};
        jidx   = dh_table.param_info.indices(i,1);
        pidx   = dh_table.param_info.indices(i,2);
        curval = dh_table.joints(jidx).(param_fields{pidx}).value;

        uicontrol(panel,'Style','text','String',sprintf('%s %s:',pname,unit_map(ptype)),...
            'Units','normalized','Position',[0.05 y 0.40 label_h],'HorizontalAlignment','left');

        sliders(i) = uicontrol(panel,'Style','slider','Min',prange(1),'Max',prange(2),'Value',curval,...
            'Units','normalized','Position',[0.05 y-label_h 0.62 slider_h]);

        texts(i) = uicontrol(panel,'Style','text','String',sprintf('%.3f',curval),...
            'Units','normalized','Position',[0.70 y-label_h 0.25 slider_h],'HorizontalAlignment','center');

        addlistener(sliders(i), 'ContinuousValueChange', @update_plot);
    end

    reset_y = start_y - num_sliders*spacing - 0.035;
    uicontrol(panel,'Style','pushbutton','String','Reset to Defaults','Units','normalized',...
        'Position',[0.18 reset_y 0.64 0.055],'Callback',@reset_sliders,'FontWeight','bold');

    pose_y = reset_y - 0.08;
    uicontrol(panel,'Style','text','String','End-Effector Pose:','Units','normalized',...
        'Position',[0.08 pose_y 0.84 0.035],'FontWeight','bold');
    text_pose = uicontrol(panel,'Style','text','String','','Units','normalized',...
        'Position',[0.05 0.02 0.9 pose_y-0.06],'HorizontalAlignment','left','FontName','FixedWidth');

    % ---------- LEDDTYPAR OG ARBEIDSROM ----------
    joint_types = classify_joint_types(dh_table); % 'R'/'P'
    workspace_limits = estimate_workspace(dh_table, 350);

    use_workspace_limits = false;  % 1) stram inn for autosave
    update_plot();
    use_workspace_limits = true;   % 2) lås til arbeidsrom
    update_plot();

    % ===================== FUNKSJONAR =====================
    function A = dh_transform(theta,d,a,alpha)
        ct = cos(theta); st = sin(theta); ca = cos(alpha); sa = sin(alpha);
        A = [ct, -st*ca,  st*sa, a*ct;
             st,  ct*ca, -ct*sa, a*st;
             0,   sa,     ca,    d;
             0,   0,      0,     1];
    end

    function draw_axes(T, len, drawY, label, idx)
        o = T(1:3,4);
        x = T(1:3,1)*len;
        y = T(1:3,2)*len;
        z = T(1:3,3)*len;
    
        % X-akse (raud)
        plot3(ax_robot, ...
            [o(1) o(1)+x(1)], ...
            [o(2) o(2)+x(2)], ...
            [o(3) o(3)+x(3)], ...
            'r-', 'LineWidth', cfg.axis_lw);
    
        % Z-akse (blå)
        plot3(ax_robot, ...
            [o(1) o(1)+z(1)], ...
            [o(2) o(2)+z(2)], ...
            [o(3) o(3)+z(3)], ...
            'b-', 'LineWidth', cfg.axis_lw);
    
        % Y-akse (gul) berre når drawY = true
        if drawY
            plot3(ax_robot, ...
                [o(1) o(1)+y(1)], ...
                [o(2) o(2)+y(2)], ...
                [o(3) o(3)+y(3)], ...
                'y-', 'LineWidth', cfg.axis_lw);
        end
    
        % Label (base) / (EE)
        if ~isempty(label)
            text(o(1)+0.04, o(2)+0.04, o(3)+0.04, ...
                 label, 'FontSize',11,'FontWeight','bold','Parent',ax_robot);
        end
    
        % Nummer på aksane (X_i, Y_i, Z_i)
        if ~verbose
            text(o(1)+x(1)*1.1, o(2)+x(2)*1.1, o(3)+x(3)*1.1, ...
                 sprintf('x%d', idx), 'Color','r','FontSize',10,'Parent',ax_robot);
        
            if drawY
                text(o(1)+y(1)*1.1, o(2)+y(2)*1.1, o(3)+y(3)*1.1, ...
                     sprintf('y%d', idx), 'Color','y','FontSize',10,'Parent',ax_robot);
            end
        
            text(o(1)+z(1)*1.1, o(2)+z(2)*1.1, o(3)+z(3)*1.1, ...
                 sprintf('z%d', idx), 'Color','b','FontSize',10,'Parent',ax_robot);
        end
    end


    function current_dh = get_current_dh_values()
        current_dh = dh_table;
        for k = 1:num_sliders
            jidx = dh_table.param_info.indices(k,1);
            pidx = dh_table.param_info.indices(k,2);
            val  = get(sliders(k),'Value');
            current_dh.joints(jidx).(param_fields{pidx}).value = val;
        end
    end

    function update_plot(~,~)
        cur = get_current_dh_values();
        for k=1:num_sliders, set(texts(k),'String',sprintf('%.3f',get(sliders(k),'Value'))); end

        N = dh_table.num_joints;
        T_all = zeros(4,4,N+1); T = eye(4); T_all(:,:,1) = T;
        for i = 1:N
            j = cur.joints(i);
            A = dh_transform(j.theta.value, j.d.value, j.a.value, j.alpha.value);
            T = T * A; 
            T_all(:,:,i+1) = T;
        end

        cla(ax_robot); hold(ax_robot,'on');

        % --- posisjonar og skala ---
        O = squeeze(T_all(1:3,4,:));
        % base_scale = local_scale(O);
        base_scale = 1;
        axis_len = cfg.axis_len * base_scale;

        % --- lenkjer ---
        plot3(ax_robot, O(1,:), O(2,:), O(3,:), 'k-','LineWidth',cfg.link_lw);

        % --- base-markør ---
        plot3(ax_robot, O(1,1), O(2,1), O(3,1), 'p', 'MarkerSize',cfg.base_ms, ...
              'MarkerFaceColor',[0.2 0.7 0.9], 'MarkerEdgeColor','k');

        % --- leddmarkørar ---
        for i = 1:N
            Tjoint = T_all(:,:,i);
            switch joint_types(i)
                case 'P'
                    draw_square_prism(Tjoint, cfg.prism_xy*base_scale, cfg.prism_xy*base_scale, cfg.prism_z*base_scale, [0.95 0.8 0.2]);
                otherwise
                    draw_cylinder_oriented(Tjoint, cfg.cyl_r*base_scale, cfg.cyl_h*base_scale, [0.8 0.2 0.2]);
            end
        end

        % --- ramme-origin ---
        plot3(ax_robot, O(1,:), O(2,:), O(3,:), 'ko', 'MarkerSize',cfg.pt_ms, 'MarkerFaceColor','y');

        % --- AKSAR: X/Z for alle, Y berre på base og EE ---
        for i = 1:(N+1)
            drawY = (i==1) || (i==(N+1));
            if i==1
                lbl = '{0} (base)';
            elseif i==(N+1)
                lbl = sprintf('{%d} (EE)',N);
            else
                lbl = ''; % ingen label for mellomrammer for å unngå rot
            end
            draw_axes(T_all(:,:,i), axis_len/2, drawY, lbl, i-1);
        end

        % --- akselimiter ---
        if use_workspace_limits
            xlim(ax_robot, workspace_limits.x); ylim(ax_robot, workspace_limits.y); zlim(ax_robot, workspace_limits.z);
        else
            [xl,yl,zl] = tight_limits(O, 0.08);
            xlim(ax_robot, xl); ylim(ax_robot, yl); zlim(ax_robot, zl);
        end

        % --- EE-pose ---
        Tend = T_all(:,:,end);
        p = Tend(1:3,4); R = Tend(1:3,1:3);
        sy = sqrt(R(1,1)^2 + R(2,1)^2); sing = sy < 1e-6;
        if ~sing
            roll = atan2(R(3,2), R(3,3));
            pitch = atan2(-R(3,1), sy);
            yaw = atan2(R(2,1), R(1,1));
        else
            roll = atan2(-R(2,3), R(2,2)); pitch = atan2(-R(3,1), sy); yaw = 0;
        end
        set(text_pose,'String',sprintf(['Position (m):\n x = %.3f\n y = %.3f\n z = %.3f\n\n' ...
                                        'Orientation (rad):\n roll  = %.3f\n pitch = %.3f\n yaw   = %.3f'], ...
                                        p(1),p(2),p(3), roll,pitch,yaw));
    end

    function reset_sliders(~,~)
        for k=1:num_sliders
            jidx = dh_table.param_info.indices(k,1);
            pidx = dh_table.param_info.indices(k,2);
            defv = dh_table.joints(jidx).(param_fields{pidx}).value;
            set(sliders(k),'Value',defv);
        end
        update_plot();
    end

    % ===================== HJELPEFUNKSJONAR =====================
    function jt = classify_joint_types(tbl)
        N = tbl.num_joints;
        jt = repmat('R',1,N);
        has_rot = false(1,N); has_pri = false(1,N);
        for k = 1:tbl.num_variables
            jidx = tbl.param_info.indices(k,1);
            typ = tbl.param_info.types{k};
            if strcmpi(typ,'rotation'),    has_rot(jidx) = true; end
            if strcmpi(typ,'translation'), has_pri(jidx) = true; end
        end
        for i=1:N
            if has_pri(i) && ~has_rot(i), jt(i)='P'; else, jt(i)='R'; end
        end
    end

    function lim = estimate_workspace(tbl, nsamp)
        nsamp = max(150, nsamp);
        M = tbl.num_variables; rng('default');
        S = zeros(M, nsamp);
        param_fields = {'theta','d','a','alpha'};
        for m = 1:M
            r = tbl.param_info.ranges(m,:); S(m,:) = r(1) + (r(2)-r(1))*rand(1,nsamp);
        end
        mins = [ inf;  inf;  inf]; maxs = [-inf; -inf; -inf];

        corner_ct = min(M, 10); C = dec2bin(0:(2^corner_ct-1)) - '0';
        if ~isempty(C)
            for c = 1:size(C,1)
                v = NaN(M,1);
                for m = 1:corner_ct
                    rr = tbl.param_info.ranges(m,:); v(m) = rr(1 + C(c,m));
                end
                for m = corner_ct+1:M
                    rr = tbl.param_info.ranges(m,:); v(m) = mean(rr);
                end
                S = [S, v]; %#ok<AGROW>
            end
        end

        for s = 1:size(S,2)
            cur = tbl;
            for m = 1:M
                jidx = tbl.param_info.indices(m,1);
                pidx = tbl.param_info.indices(m,2);
                cur.joints(jidx).(param_fields{pidx}).value = S(m,s);
            end
            T = eye(4); O = T(1:3,4);
            mins = min(mins, O); maxs = max(maxs, O);
            for i=1:tbl.num_joints
                j = cur.joints(i);
                A = dh_transform(j.theta.value, j.d.value, j.a.value, j.alpha.value);
                T = T*A; O = T(1:3,4);
                mins = min(mins, O); maxs = max(maxs, O);
            end
        end
        span = max(maxs - mins, 1e-3);
        margin = 0.12 * span;
        lim.x = [mins(1)-margin(1), maxs(1)+margin(1)];
        lim.y = [mins(2)-margin(2), maxs(2)+margin(2)];
        lim.z = [mins(3)-margin(3), maxs(3)+margin(3)];
    end

    function [xl,yl,zl] = tight_limits(O, frac_margin)
        mn = min(O,[],2); mx = max(O,[],2);
        span = max(mx-mn, 1e-3);
        m = frac_margin * span;
        xl = [mn(1)-m(1), mx(1)+m(1)];
        yl = [mn(2)-m(2), mx(2)+m(2)];
        zl = [mn(3)-m(3), mx(3)+m(3)];
    end

    function s = local_scale(O)
        if size(O,2) >= 2
            d = vecnorm(diff(O,1,2));
            s = max(mean(d), 0.3);
        else
            s = 0.5;
        end
    end

    function draw_cylinder_oriented(T, r, h, col)
        [xc,yc,zc] = cylinder(r, 24);
        zc = zc*h - h/2;
        P = [xc(:)'; yc(:)'; zc(:)'];
        P2 = T(1:3,1:3) * P + T(1:3,4);
        X = reshape(P2(1,:), size(xc)); Y = reshape(P2(2,:), size(yc)); Z = reshape(P2(3,:), size(zc));
        surf(ax_robot, X, Y, Z, 'FaceColor', col, 'EdgeColor','none', 'FaceAlpha', 0.9);
        th = linspace(0,2*pi,24);
        circ = [r*cos(th); r*sin(th); zeros(size(th))];
        for z0 = [-h/2, h/2]
            Pc = T(1:3,1:3) * [circ(1,:); circ(2,:); circ(3,:)+z0] + T(1:3,4);
            patch(ax_robot, Pc(1,:), Pc(2,:), Pc(3,:), col, 'EdgeColor','none','FaceAlpha',0.9);
        end
    end

    function draw_square_prism(T, lx, ly, lz, col)
        hx = lx/2; hy = ly/2; hz = lz/2;
        V = [ -hx -hy -hz;
               hx -hy -hz;
               hx  hy -hz;
              -hx  hy -hz;
              -hx -hy  hz;
               hx -hy  hz;
               hx  hy  hz;
              -hx  hy  hz]';
        Vw = T(1:3,1:3) * V + T(1:3,4);
        Vw = Vw';
        F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
        patch(ax_robot, 'Vertices', Vw, 'Faces', F, 'FaceColor', col, 'EdgeColor','none', 'FaceAlpha',0.9);
    end
end
