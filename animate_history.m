function animate_history(save_video)
% ANIMATE_HISTORY 将闭环仿真结果渲染为动画
%
% 输入:
%   save_video: bool, 是否保存为 MP4 (默认为 false)

    if nargin < 1, save_video = false; end

    % 1. 加载数据
    if ~isfile('sim_results.mat')
        error('Result file not found. Run simulation first.');
    end
    load('sim_results.mat'); % 加载 sim_log, scenario, constraints_init, dt
    
    N_sim = length(sim_log);
    
    % 2. 准备图形窗口
    figure('Name', 'Closed-Loop Simulation Playback', 'Color', 'w', ...
           'Position', [100, 100, 1200, 600]);
    
    % 布局: 上面是大图(轨迹)，下面是仪表盘(速度/控制)
    % subplot 布局: [2行, 4列]
    % 大图占用: 行1-2, 列1-3 (3/4 宽度)
    % 仪表盘占用: 行1-2, 列4 (1/4 宽度)
    
    ax_main = subplot(1, 2, [1, 2]);
    hold(ax_main, 'on'); axis(ax_main, 'equal'); box(ax_main, 'on');
    xlabel('Global X [m]'); ylabel('Global Y [m]');
    title(ax_main, 'Autonomous Driving Simulation');
    
    % 绘制静态场景 (车道线)
    draw_road_network(ax_main, scenario);
    
    % 初始化动态对象的句柄 (Handles)
    h_ego_body = [];      % 自车方块
    h_obs_bodies = [];    % 障碍物方块数组
    h_obs_preds = [];     % 障碍物预测线数组
    h_cands = [];         % 候选轨迹线数组
    h_plan = [];          % 最优规划轨迹线
    h_title_text = [];    % 顶部状态文字
    
    % 3. 视频录制设置
    if save_video
        v_writer = VideoWriter('simulation_video.mp4', 'MPEG-4');
        v_writer.FrameRate = 10; % 1/dt
        open(v_writer);
    end
    
    % ==========================================
    % 4. 动画循环
    % ==========================================
    for k = 1:N_sim
        frame_data = sim_log(k);
        
        % --- A. 更新自车 (Ego Vehicle) ---
        x_ego = frame_data.ego_state; % [x, y, theta, v]
        
        if isempty(h_ego_body)
            % 初始化
            h_ego_body = draw_car_patch(ax_main, x_ego, scenario.ego_size, 'g', 0.8);
        else
            % 更新
            update_car_patch(h_ego_body, x_ego, scenario.ego_size);
        end
        
        % --- B. 更新障碍物 (Obstacles) ---
        obs_list = frame_data.obstacles;
        num_obs = length(obs_list);
        
        % 动态扩展句柄数组 (如果障碍物数量变化)
        if length(h_obs_bodies) < num_obs
            for j = length(h_obs_bodies)+1 : num_obs
                h_obs_bodies(j) = draw_car_patch(ax_main, [0;0;0;0], struct('length',4.5,'width',2), [1 0.7 0.7], 0.6);
                h_obs_preds(j) = plot(ax_main, 0, 0, 'r:', 'LineWidth', 1.0); % 预测线
            end
        end
        
        for j = 1:num_obs
            obs = obs_list(j);
            % 构造状态向量 [x, y, theta, v]
            obs_state = [obs.x; obs.y; obs.theta; 0]; 
            obs_size = struct('length', obs.length, 'width', obs.width);
            update_car_patch(h_obs_bodies(j), obs_state, obs_size);
            
            % 更新预测线
            if isfield(obs, 'prediction') && ~isempty(obs.prediction)
                set(h_obs_preds(j), 'XData', obs.prediction(1,:), 'YData', obs.prediction(2,:));
            end
        end
        
        % --- C. 更新规划结果 (Planning Results) ---
        plan = frame_data.plan_info;
        
        % 1. 候选轨迹 (灰色)
        results = plan.results;
        num_cands = length(results);
        
        % 动态管理句柄
        if length(h_cands) < num_cands
            for j = length(h_cands)+1 : num_cands
                h_cands(j) = plot(ax_main, 0, 0, '-', 'Color', [0.6 0.6 0.6, 0.4], 'LineWidth', 1.0);
            end
        end
        
        for j = 1:length(h_cands)
            if j <= num_cands && ~isempty(results(j).X)
                set(h_cands(j), 'XData', results(j).X(1,:), 'YData', results(j).X(2,:), 'Visible', 'on');
            else
                set(h_cands(j), 'Visible', 'off');
            end
        end
        
        % 2. 最优轨迹 (绿色粗线)
        best_idx = plan.best_idx;
        if best_idx ~= -1
            best_traj = results(best_idx).X;
            if isempty(h_plan)
                h_plan = plot(ax_main, best_traj(1,:), best_traj(2,:), 'g-', 'LineWidth', 2.5);
            else
                set(h_plan, 'XData', best_traj(1,:), 'YData', best_traj(2,:), 'Visible', 'on');
            end
        else
            set(h_plan, 'Visible', 'off');
        end
        
        % --- D. 视口跟随 (Camera Follow) ---
        % 保持自车在中心偏左
        view_range_x = [-20, 120]; % 前后可视范围
        view_range_y = [-10, 10]; % 左右可视范围
        
        xlim(ax_main, x_ego(1) + view_range_x);
        ylim(ax_main, 0 + view_range_y); % Y轴可以固定，也可以跟随 y_ego(2)
        
        % --- E. 更新文字信息 ---
        info_str = {
            sprintf('Time: %.1f s', frame_data.time),
            sprintf('Speed: %.1f m/s', x_ego(4)),
            sprintf('Intent: %s', strrep(plan.best_cand_name, '_', '\_')),
            sprintf('Cost: %.2f', plan.best_score)
        };
        
        if isempty(h_title_text)
            % 固定在视口左上角，需要用相对坐标或者 update position
            title(ax_main, info_str, 'FontSize', 12, 'Interpreter', 'tex');
        else
            title(ax_main, info_str);
        end
        
        drawnow limitrate; % 限制刷新率以保证流畅
        
        if save_video
            frame = getframe(gcf);
            writeVideo(v_writer, frame);
        else
            pause(0.04); % 手动延时，方便观看
        end
    end
    
    if save_video
        close(v_writer);
        fprintf('Video saved to simulation_video.mp4\n');
    end
end

% ==========================================
% 绘图辅助函数
% ==========================================

function draw_road_network(ax, scenario)
    % 绘制车道线
    centers = scenario.lane_centers;
    width = 4.0;
    if isfield(scenario, 'lane_width'), width = scenario.lane_width; end
    
    x_range = [-100, 2000]; % 假设很长的路
    
    % 道路边界
    y_min = min(centers) - width/2;
    y_max = max(centers) + width/2;
    plot(ax, x_range, [y_min, y_min], 'k-', 'LineWidth', 2);
    plot(ax, x_range, [y_max, y_max], 'k-', 'LineWidth', 2);
    
    % 分隔线
    for i = 1:length(centers)-1
        y_div = (centers(i) + centers(i+1)) / 2;
        plot(ax, x_range, [y_div, y_div], 'k--', 'LineWidth', 1);
    end
end

function h_patch = draw_car_patch(ax, x_state, size_struct, color, alpha)
    % 创建车辆图形对象
    len = size_struct.length;
    wid = size_struct.width;
    
    % 初始坐标 (0,0)
    p_corners = [-len/2, len/2, len/2, -len/2;
                 -wid/2, -wid/2, wid/2, wid/2];
                 
    h_patch = patch(ax, p_corners(1,:), p_corners(2,:), color, ...
        'FaceAlpha', alpha, 'EdgeColor', 'k');
        
    update_car_patch(h_patch, x_state, size_struct);
end

function update_car_patch(h_patch, x_state, size_struct)
    % 更新车辆位置
    cx = x_state(1); cy = x_state(2); phi = x_state(3);
    len = size_struct.length; wid = size_struct.width;
    
    p_corners = [-len/2, len/2, len/2, -len/2;
                 -wid/2, -wid/2, wid/2, wid/2];
                 
    R = [cos(phi), -sin(phi); sin(phi), cos(phi)];
    p_global = R * p_corners + [cx; cy];
    
    set(h_patch, 'XData', p_global(1,:), 'YData', p_global(2,:));
end