function trace = make_bandlimited_disturbance(t, seed, cfg)
%% 非周期随机扰动生成器：AR(1) 低通白噪声
%
% 用途：为 TIMC 返修的"非周期随机扰动"实验生成带限随机加速度轨迹，替代原有
%       4 个正弦叠加的确定性扰动（见 load_lifting_payload_dynamics.m）。
%
% 用法：
%   trace = make_bandlimited_disturbance(t, 101, cfg);
%
% 输入：
%   t    : 仿真时间列向量（0:dt:Tf），必须与主循环同一网格、严格单调递增
%   seed : 整数随机种子（101/202/303/404/505）
%   cfg  : 结构体，字段（缺省值见下）
%          bandwidth_hz : 一阶低通截止频率（Hz），默认 1.0
%          amp_pos      : 平移三轴目标 RMS（m/s^2），默认 2.15
%          amp_att      : 姿态三轴目标 RMS（rad/s^2），默认 0.3
%
% 输出：
%   trace : 结构体
%          t   (N×1)  时间列向量（与输入 t 相同）
%          a_p (3×N)  世界系平移加速度扰动，三轴独立
%          a_R (3×N)  机体系角加速度扰动，三轴独立
%          seed / bandwidth_hz / amp_pos / amp_att / dt / N
%          rms_pos / rms_att       归一化后的实测 RMS（应等于目标值）
%          max_abs_pos / max_abs_att
%          hash                    轨迹 SHA-256 前 16 位（复核用）
%
% 生成方法（TIMC_随机扰动实验说明.pdf 第 3.1 节）：
%   rng(seed,'twister');          % 每个 case 只在此处重置一次随机流
%   a = exp(-2*pi*fc*dt);
%   w = randn(3,N);               % 每个通道独立白噪声
%   x(:,1) = w(:,1); x(:,k) = a*x(:,k-1) + sqrt(1-a^2)*w(:,k);
%   x = x - mean(x,2);            % 去均值
%   x = x ./ sqrt(mean(x.^2,2)) * amp;   % 归一化到目标 RMS
%
% 注意：1 kHz 采样下 randn 的能量集中在高频，必须经低通 + 去均值 + 归一化，
%       否则会把分配器与内环打崩。归一化按行（每个通道）独立进行。

if nargin < 3 || isempty(cfg)
    cfg = struct();
end
if nargin < 2 || isempty(seed)
    seed = 101;
end

validateattributes(t, {'numeric'}, {'vector', 'real', 'finite', 'nonempty'}, mfilename, 't');
validateattributes(seed, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'}, mfilename, 'seed');
if ~isstruct(cfg)
    error('TIMC:RandomDisturbance:BadConfig', 'cfg must be a struct.');
end

t = double(t(:));
N = numel(t);
if N < 2
    error('TIMC:RandomDisturbance:TooShort', 't must contain at least two samples.');
end
dt = t(2) - t(1);
if ~(dt > 0)
    error('TIMC:RandomDisturbance:BadGrid', 't must be strictly increasing.');
end
grid_err = max(abs(diff(t) - dt));
if grid_err > 1e-9 * max(1, abs(dt))
    error('TIMC:RandomDisturbance:NonUniformGrid', ...
        't must be a uniform grid (max deviation %.3e s).', grid_err);
end

fc = get_cfg_value(cfg, 'bandwidth_hz', 1.0);
amp_pos = get_cfg_value(cfg, 'amp_pos', 2.15);
amp_att = get_cfg_value(cfg, 'amp_att', 0.3);
validateattributes(fc, {'numeric'}, {'scalar', 'real', 'finite', 'positive'}, mfilename, 'cfg.bandwidth_hz');
validateattributes(amp_pos, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'}, mfilename, 'cfg.amp_pos');
validateattributes(amp_att, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'}, mfilename, 'cfg.amp_att');
if amp_pos == 0 && amp_att == 0
    error('TIMC:RandomDisturbance:ZeroAmplitude', 'amp_pos and amp_att are both zero.');
end

% 唯一一次 rng 重置：每个 case 的状态只由 seed 决定，不依赖上一轮
rng(seed, 'twister');

a = exp(-2 * pi * fc * dt);
w_pos = randn(3, N);
w_att = randn(3, N);

a_p = ar1_lowpass(w_pos, a);
a_R = ar1_lowpass(w_att, a);

a_p = normalize_rms(a_p, amp_pos);
a_R = normalize_rms(a_R, amp_att);

trace = struct();
trace.t = t;
trace.a_p = a_p;
trace.a_R = a_R;
trace.seed = double(seed);
trace.bandwidth_hz = double(fc);
trace.amp_pos = double(amp_pos);
trace.amp_att = double(amp_att);
trace.dt = dt;
trace.N = N;
trace.generator = 'ar1_lowpass_bandlimited';
trace.rms_pos = sqrt(mean(a_p .^ 2, 2));
trace.rms_att = sqrt(mean(a_R .^ 2, 2));
trace.max_abs_pos = max(abs(a_p), [], 2);
trace.max_abs_att = max(abs(a_R), [], 2);
trace.hash = hash_trace(trace);
end

% ---------------------------------------------------------------- 局部函数

function x = ar1_lowpass(w, a)
% 一阶低通（AR(1)）：x_k = a*x_{k-1} + sqrt(1-a^2)*w_k
N = size(w, 2);
x = zeros(size(w));
x(:, 1) = w(:, 1);
b = sqrt(1 - a ^ 2);
for k = 2:N
    x(:, k) = a * x(:, k - 1) + b * w(:, k);
end
end

function x = normalize_rms(x, amp)
% 去均值后逐通道归一化到目标 RMS
x = x - mean(x, 2);
rms_now = sqrt(mean(x .^ 2, 2));
if any(rms_now <= 0)
    error('TIMC:RandomDisturbance:DegenerateTrace', 'A disturbance channel has zero RMS.');
end
x = x ./ rms_now * amp;
end

function value = get_cfg_value(cfg, name, default_value)
if isfield(cfg, name) && ~isempty(cfg.(name))
    value = cfg.(name);
else
    value = default_value;
end
end

function hex = hash_trace(trace)
% SHA-256（取前 16 位十六进制），输入为 t / a_p / a_R 的原始 IEEE-754 字节
raw = typecast([trace.t(:); trace.a_p(:); trace.a_R(:)], 'uint8');
try
    digest = java.security.MessageDigest.getInstance('SHA-256');
    digest.update(raw);
    hex_full = lower(reshape(dec2hex(typecast(digest.digest(), 'uint8'), 2)', 1, []));
catch
    % 无 JVM 时退化为纯 MATLAB 校验和（仍对轨迹逐位敏感）
    hex_full = lower(reshape(dec2hex(mod(cumsum(double(raw)) * 2654435761, 2 ^ 32), 8)', 1, []));
    hex_full = [repmat('n', 1, 0), hex_full];
end
hex = hex_full(1:min(16, numel(hex_full)));
end
