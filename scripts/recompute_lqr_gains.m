function result = recompute_lqr_gains()
%RECOMPUTE_LQR_GAINS Recompute Ascento LQR gains from src/app_config.h.
%
% Run from MATLAB:
%   cd C:\workplace\wheel_leg\zephyr_ascento_f407_wheel_leg\scripts
%   result = recompute_lqr_gains();
%
% The plant model uses a per-wheel sagittal equivalent because the firmware
% converts balance_torque_nm directly into each wheel's current command.

this_dir = fileparts(mfilename('fullpath'));
zephyr_root = fileparts(this_dir);
cfg_path = fullfile(zephyr_root, 'src', 'app_config.h');

p = read_zephyr_params(cfg_path);
Q = diag([1500 1 1200 100]);
Rcost = 100;

leg = p.leg_min:0.005:p.leg_max;
if leg(end) < p.leg_max
    leg = [leg p.leg_max];
end

K = zeros(numel(leg), 4);
model = repmat(struct('L', 0, 'body_h', 0, 'leg_h', 0, ...
    'mp', 0, 'mw', 0, 'Iw', 0, 'Ip', 0), numel(leg), 1);

for i = 1:numel(leg)
    [K(i, :), model(i)] = zephyr_get_k1_length(leg(i), p, Q, Rcost);
end

a11 = polyfit(leg, K(:, 1).', 2);
a12 = polyfit(leg, K(:, 2).', 2);
a13 = polyfit(leg, K(:, 3).', 0);
a14 = polyfit(leg, K(:, 4).', 2);
[K_stand, model_stand] = zephyr_get_k1_length(p.leg_stand, p, Q, Rcost);

result = struct('source', cfg_path, 'leg', leg, 'K', K, ...
    'a11', a11, 'a12', a12, 'a13', a13, 'a14', a14, ...
    'K_stand', K_stand, 'model', model, ...
    'model_stand', model_stand, 'params', p);

save(fullfile(this_dir, 'zephyr_lqr_gains.mat'), '-struct', 'result');

fprintf('Zephyr app_config: %s\n', cfg_path);
fprintf('Model convention: per-wheel sagittal LQR input torque.\n');
fprintf('Q=diag([1500 1 1200 100]), R=100\n');
fprintf('a11=[%.10f %.10f %.10f]\n', a11);
fprintf('a12=[%.10f %.10f %.10f]\n', a12);
fprintf('a13=[%.10f]\n', a13);
fprintf('a14=[%.10f %.10f %.10f]\n', a14);
fprintf('stand_L=%.6f K=[%.10f %.10f %.10f %.10f]\n', ...
    p.leg_stand, K_stand);
fprintf('stand_model L_com=%.6f body_h=%.6f leg_h=%.6f mp=%.6f Ip=%.8f\n', ...
    model_stand.L, model_stand.body_h, model_stand.leg_h, ...
    model_stand.mp, model_stand.Ip);

end

function p = read_zephyr_params(cfg_path)
txt = fileread(cfg_path);
p.R = macro_value(txt, 'APP_ASCENTO_WHEEL_RADIUS_M');
p.body_mass = macro_value(txt, 'APP_ASCENTO_BODY_MASS_KG');
p.wheel_mass = macro_value(txt, 'APP_ASCENTO_SINGLE_WHEEL_MASS_KG');
p.leg_mass = macro_value(txt, 'APP_ASCENTO_SINGLE_LEG_MASS_KG');
p.body_com_height_stand = macro_value(txt, 'APP_ASCENTO_BODY_COM_HEIGHT_M');
p.body_pitch_inertia = macro_value(txt, 'APP_ASCENTO_BODY_PITCH_INERTIA_KG_M2');
p.wheel_inertia = macro_value(txt, 'APP_ASCENTO_WHEEL_INERTIA_KG_M2');
p.leg_min = macro_value(txt, 'APP_ASCENTO_LEG_LENGTH_MIN_M');
p.leg_max = macro_value(txt, 'APP_ASCENTO_LEG_LENGTH_MAX_M');
p.leg_stand = macro_value(txt, 'APP_ASCENTO_LEG_LENGTH_STAND_M');
p.fb_L1 = macro_value(txt, 'APP_ASCENTO_FB_L1');
p.fb_L2 = macro_value(txt, 'APP_ASCENTO_FB_L2');
p.fb_L3 = macro_value(txt, 'APP_ASCENTO_FB_L3');
p.fb_L4 = macro_value(txt, 'APP_ASCENTO_FB_L4');
p.fb_L23 = macro_value(txt, 'APP_ASCENTO_FB_L23');
end

function v = macro_value(txt, name)
pat = ['#define\s+' name '\s+\(?\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)f?\s*\)?'];
m = regexp(txt, pat, 'tokens', 'once');
if isempty(m)
    error('Could not read macro %s', name);
end
v = str2double(m{1});
end

function [K, m] = zephyr_get_k1_length(leg_length, p, Q, Rcost)
body_mass_side = 0.5 * p.body_mass;
body_inertia_side = 0.5 * p.body_pitch_inertia;
leg_mass_side = p.leg_mass;

body_h = p.body_com_height_stand + (leg_length - p.leg_stand);
leg_h = zephyr_leg_com_height(leg_length, p);

mp = body_mass_side + leg_mass_side;
mw = p.wheel_mass;
Iw = p.wheel_inertia;
L = (body_mass_side * body_h + leg_mass_side * leg_h) / mp;
Ip = body_inertia_side + ...
     body_mass_side * (body_h - L)^2 + ...
     leg_mass_side * (leg_h - L)^2;

[A, B] = sagittal_linear_model(p.R, Iw, mw, L, mp, Ip, 9.80665);
K = lqr(A, B, Q, Rcost);

m = struct('L', L, 'body_h', body_h, 'leg_h', leg_h, ...
    'mp', mp, 'mw', mw, 'Iw', Iw, 'Ip', Ip);
end

function [A, B] = sagittal_linear_model(R, Iw, mw, L, mp, Ip, g)
den_x = Iw / R + mw * R;
M = [R * mp * L, den_x + R * mp;
     Ip + mp * L^2, mp * L];
theta_gain = M \ [0; mp * g * L];
torque_gain = M \ [1; -1];
A = [0, 1, 0, 0;
     theta_gain(1), 0, 0, 0;
     0, 0, 0, 1;
     theta_gain(2), 0, 0, 0];
B = [0; torque_gain(1); 0; torque_gain(2)];
end

function leg_h = zephyr_leg_com_height(target_leg_length, p)
obj = @(q) (fourbar_leg_length(q, p) - target_leg_length).^2;
q = fminbnd(obj, 1.5, 3.2);
[leg_length, leg_h] = fourbar_leg_length(q, p);
if abs(leg_length - target_leg_length) > 0.002
    warning('Leg-length solve mismatch: target %.4f got %.4f', ...
        target_leg_length, leg_length);
end
end

function [leg_length, leg_h] = fourbar_leg_length(qC, p)
C = [0, 0];
B = [p.fb_L4 / sqrt(2), p.fb_L4 / sqrt(2)];
D = [-p.fb_L2 * cos(qC), -p.fb_L2 * sin(qC)];
[A1, A2, count] = circle_intersections(B, p.fb_L3, D, p.fb_L23);
if count == 0
    leg_length = NaN;
    leg_h = NaN;
    return;
end
if count == 1 || A1(1) >= A2(1)
    A = A1;
else
    A = A2;
end
E = D + p.fb_L1 * (D - A) / p.fb_L23;
leg_length = -E(2);

mid_L2 = (C + D) * 0.5;
mid_L23 = (D + A) * 0.5;
mid_L3 = (B + A) * 0.5;
mid_L1 = (D + E) * 0.5;
leg_com = (mid_L1 + mid_L2 + mid_L3 + mid_L23) * 0.25;
leg_h = leg_com(2) - E(2);
end

function [p1, p2, count] = circle_intersections(c1, r1, c2, r2)
dvec = c2 - c1;
d = norm(dvec);
p1 = [NaN, NaN];
p2 = [NaN, NaN];
count = 0;
if d < 1e-9 || d > r1 + r2 + 1e-9 || d < abs(r1 - r2) - 1e-9
    return;
end
ex = dvec / d;
a = (r1^2 - r2^2 + d^2) / (2 * d);
h = sqrt(max(r1^2 - a^2, 0));
p0 = c1 + a * ex;
ey = [-ex(2), ex(1)];
p1 = p0 + h * ey;
p2 = p0 - h * ey;
count = 1 + (h > 1e-9);
end
