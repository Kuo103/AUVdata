close all;
% clear;
load('AUV_Right20250604_191002.mat'); % 加配重的数据
% load('AUV_Right20250507_204001.mat');
% load('AUV_Right20250605_185317.mat');   % 右转的
% load('AUV_Right20250605_181551.mat');
% load('AUV_Right20250605_183209.mat'); % 左转的
% loadfile = 'AUV_Right20250711_161111';
 % load(loadfile);
% 提取数据
pwm_raw = userDataStruct.PWM_True_Store(1:end,1);
time_raw = userDataStruct.IMUdata(1:end,1);
phi_raw = userDataStruct.IMUdata(1:end,10);
weizhi_raw = userDataStruct.UWB_val_store(1:end,:); 
force_caiji = userDataStruct.Force_store(1:end,:); 

%% 时间戳处理（异常值+跳变修复）
w_time = 68;
time_med = movmedian(time_raw, w_time);
time_thresh = 3;

time_proc = time_raw;
for i = ceil(w_time/2):length(time_raw)-floor(w_time/2)
    if abs(time_raw(i) - time_med(i)) > time_thresh
        idx = max(1, i-60):min(length(time_raw), i+60);
        time_proc(i) = median(time_raw(idx));
    end
end

% 边界修正
time_proc(1:ceil(w_time/2)-1) = time_proc(ceil(w_time/2));
time_proc(end-floor(w_time/2)+1:end) = time_proc(end-floor(w_time/2));

% 限幅
time_proc = min(max(time_proc, 0), 200);

% 跳变检测与累加修复
offset = 0;
time_continuous = time_proc;
for i = 2:length(time_proc)
    if (time_proc(i-1) > 100) && (time_proc(i) < 70)
        offset = offset + 200;
    end
    time_continuous(i) = time_proc(i) + offset;
end
% 平滑处理
time_filtered = medfilt1(medfilt1(time_continuous, 15), 15);

% 可视化
figure('Name', '时间戳处理');
subplot(2,1,1); plot(time_raw); title('原始时间戳'); grid on;
subplot(2,1,2); plot(time_filtered); title('处理后时间戳'); grid on;
%%
%% PWM信号异常值处理 + 双中值滤波
w_pwm = 41; 
pwm_med = movmedian(pwm_raw, w_pwm);
pwm_thresh = 3;  % 异常值判据阈值

pwm_proc = pwm_raw;
for i = ceil(w_pwm/2):length(pwm_raw)-floor(w_pwm/2)
    if abs(pwm_raw(i) - pwm_med(i)) > pwm_thresh
        idx = max(1, i-7):min(length(pwm_raw), i+60);
        pwm_proc(i) = median(pwm_raw(idx));
    end
end

% 边界填补
pwm_proc(1:ceil(w_pwm/2)-1) = pwm_proc(ceil(w_pwm/2));
pwm_proc(end-floor(w_pwm/2)+1:end) = pwm_proc(end-floor(w_pwm/2));

% 限幅 [1400, 1600]
pwm_proc = max(min(pwm_proc, 1600), 1400);
pwm_filtered = medfilt1(medfilt1(pwm_proc, 15), 15);

% 可视化
figure('Name', 'PWM处理');
subplot(2,1,1); plot(time_filtered,pwm_raw); title('原始PWM'); grid on;
subplot(2,1,2); plot(time_filtered,pwm_filtered); title('处理后PWM'); grid on;
% 
% figure
% time_filtered = time_filtered - time_filtered(534);
% plot(time_filtered(534:5137),pwm_filtered(534:5137));  grid on;
%% 姿态角(phi)异常值 + SG滤波
w_phi = 30;
phi_raw = max(min(phi_raw, 180), -180); % 限幅 [-180, 180]
phi_med = movmedian(phi_raw, w_phi);
phi_proc = phi_raw;
for i = ceil(w_phi/2):length(phi_raw)-floor(w_phi/2)
    if abs(phi_raw(i) - phi_med(i)) > 20
        idx = max(1, i-7):min(length(phi_raw), i+50);
        phi_proc(i) = median(phi_raw(idx));
    end
end

% 边界修正
phi_proc(1:ceil(w_phi/2)-1) = phi_proc(ceil(w_phi/2));
phi_proc(end-floor(w_phi/2)+1:end) = phi_proc(end-floor(w_phi/2));

% 去相位跳变
phi_proc = unwrap(phi_proc, 179);

% 双中值 + SG滤波
phi_filtered = medfilt1(medfilt1(phi_proc, 15), 15);
phi_sg = sgolayfilt(phi_filtered, 5, 51);

% 可视化
figure('Name', '角度数据phi处理');
subplot(2,1,1); plot(time_filtered,phi_raw); title('原始phi'); grid on;
subplot(2,1,2); plot(time_filtered,phi_sg); title('处理后phi'); grid on;hold on;
% subplot(2,1,2); plot(time_filtered,userDataStruct.IMU_referstore); title('处理后phi'); grid on;
% 
% figure
% plot(time_filtered(534:5137),phi_sg(534:5137)/180*pi);  grid on;
% 
% 
% figure
% plot(time_filtered(534:5137),phi_raw(534:5137)/180*pi);  grid on;
%% 传感器的力
w_force = 40;
force_caiji = max(min(force_caiji, 20), -20); % 限幅 [-180, 180]
force_med = movmedian(force_caiji, w_force);
force_proc = force_caiji;
for i = ceil(w_force/2):length(force_caiji)-floor(w_force/2)
    if abs(force_caiji(i) - force_med(i)) > 20
        idx = max(1, i-7):min(length(force_caiji), i+50);
        force_proc(i) = median(force_caiji(idx));
    end
end

% 边界修正
force_proc(1:ceil(w_force/2)-1) = force_proc(ceil(w_force/2));
force_proc(end-floor(w_force/2)+1:end) = force_proc(end-floor(w_force/2));

% 去相位跳变
force_proc = unwrap(force_proc, 179);

% 双中值 + SG滤波
force_filtered = medfilt1(medfilt1(force_proc, 15), 15);
force_sg = sgolayfilt(force_filtered, 5, 51);

% 可视化
figure('Name', '力学数据force处理');

% 原始力
subplot(2,1,1);
plot(time_filtered, force_caiji);
title('原始force');
grid on;

% 处理后力
subplot(2,1,2);
plot(time_filtered, force_sg(:,1), 'b-', 'DisplayName', '方向1'); hold on;
plot(time_filtered, force_sg(:,3), 'g-', 'DisplayName', '方向2'); grid on;
title('处理后force');

%% UWB位置数据处理（逐维处理）
weizhi_filtered = zeros(size(weizhi_raw));  % 初始化输出矩阵
sg_order = 3;       % SG滤波多项式阶数
sg_window = 31;     % SG滤波窗口大小
outlier_thresh = 1; % 离群点判据（可调）
% weizhi_raw(:,4) = 0;
for dim = 1:4
    raw = weizhi_raw(:, dim);
    
    % 限幅（可选，根据数据特点设置）
    raw(raw > 40) = 0;  
    raw(raw < -40) = 0;

    % 局部滑动中位数窗口
    w_uwb = 60;
    mov_med = movmedian(raw, w_uwb);
    
    % 离群点替换
    proc = raw;
    for i = ceil(w_uwb/2):length(raw)-floor(w_uwb/2)
        if abs(raw(i) - mov_med(i)) > outlier_thresh
            idx = max(1, i-7):min(length(raw), i+60);
            proc(i) = median(raw(idx));
        end
    end

    % 边界值镜像处理
    proc(1:ceil(w_uwb/2)-1) = proc(ceil(w_uwb/2));
    proc(end-floor(w_uwb/2)+1:end) = proc(end-floor(w_uwb/2));

    % 双重中值滤波
    median_filt = medfilt1(medfilt1(proc, 15), 15);

    % SG平滑（可选）
    smooth_filt = sgolayfilt(median_filt, sg_order, sg_window);

    % 保存
    weizhi_filtered(:, dim) = smooth_filt;
end
% weizhi_filtered(:,4) = 0;
h=3;
A3 = [25.5,0,h];
A2 = [25.5,15.5,h];
A1 = [0,15.5,h];
A0 = [0,0,h];
anchor_pos = [A0; A1; A2; A3]';
direct_pos_store = zeros(length(weizhi_filtered),3);
for i=2:length(weizhi_filtered)
    direct_pos =direct_pos_store(i-1,:)';
    direct_pos_store(i,:) = ch_multilateration(anchor_pos, direct_pos, weizhi_filtered(i,:), 2)';
end
      
% 提取坐标
x_data = direct_pos_store(:, 1);
y_data = direct_pos_store(:, 2);
n_points = length(x_data);
% 构造颜色映射：用序号表示时间（渐变）
color_index = linspace(1, 256, n_points);  % 颜色索引
cmap = jet(256);  % 使用 jet 渐变色谱

% 创建图窗
figure('Name', 'AUV轨迹与坐标随时间变化', 'Position', [100, 100, 1200, 800]);

% 子图1：XY 平面轨迹图
subplot(2,1,1);
scatter(x_data, y_data, 18, cmap(round(color_index),:), 'filled');
xlabel('X 位置 (m)');
ylabel('Y 位置 (m)');
title('AUV 平面运动轨迹（颜色随时间渐变）');
axis equal;
grid on;
colormap(cmap);
cb = colorbar;
cb.Label.String = '时间步';
cb.Ticks = [0 0.5 1];
cb.TickLabels = {'起始', '中间', '终止'};


subplot(2,1,2);
plot(time_filtered,x_data, 'b-', 'LineWidth', 1.2); hold on;
plot(time_filtered, y_data, 'g-', 'LineWidth', 1.2);
plot(time_filtered, userDataStruct.P_referstore, 'k-', 'LineWidth', 1.2);hold on
plot(time_filtered,userDataStruct.P_refer_qianhou*ones(1,length(time_filtered)), 'k-', 'LineWidth', 1.2);
xlabel('时间步');
ylabel('坐标值 (m)');
title('AUV X/Y 坐标随时间变化');
legend('X 位置', 'Y 位置');
grid on;
