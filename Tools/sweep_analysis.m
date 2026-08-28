clear;
clc;
close all;

% Paste the complete UART sweep output between the two markers below.
% Both the legacy 6-column format and the new 9-column format are accepted.
%{
SWEEP_LOG_BEGIN
index,frequency_hz,iq_amplitude_a,position_amplitude_rad,gain_rad_per_a,phase_deg,window_count,gain_std_percent,phase_std_deg
0,0.488281,0.139660,6.898400,49.396000000,-125.570,2,0.000,0.000
1,0.683594,0.139780,4.277100,30.598000000,-126.540,2,0.000,0.000
2,0.976562,0.139290,2.362800,16.963000000,-129.540,2,0.000,0.000
3,1.464844,0.139270,1.228100,8.818500000,-131.820,2,0.000,0.000
4,1.953125,0.139220,0.753870,5.415100000,-133.740,2,0.000,0.000
5,2.929688,0.144170,0.393090,2.726600000,-137.590,2,0.000,0.000
6,4.882812,0.148740,0.167130,1.123700000,-141.330,2,0.000,0.000
7,7.080078,0.153390,0.092737,0.604590000,-145.050,2,0.000,0.000
8,10.009766,0.158240,0.050914,0.321760000,-148.610,2,0.000,0.000
9,14.648438,0.167830,0.029978,0.178630000,-154.440,4,0.000,0.000
10,19.531250,0.177420,0.019698,0.111030000,-159.570,4,0.000,0.000
11,30.273438,0.185740,0.008997,0.048438000,-164.910,8,0.000,0.000
12,40.039062,0.194510,0.004381,0.022522000,-174.610,8,0.000,0.000
SWEEP_LOG_END
%}

%% Input
script_file = [mfilename('fullpath'), '.m'];
script_text = fileread(script_file);
log_block = regexp(script_text, ...
    '(?s)SWEEP_LOG_BEGIN\s*(.*?)\s*SWEEP_LOG_END', 'tokens', 'once');
if isempty(log_block)
    error('Sweep data markers were not found in this script.');
end
raw_log = log_block{1};

%% Parse UART records
number_pattern = '[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?';
record_pattern = ['(?m)^\s*\d+(?:,\s*' number_pattern ...
                  '){5,8}\s*$'];
record_lines = regexp(raw_log, record_pattern, 'match');
if isempty(record_lines)
    error('No valid numeric CSV sweep records found.');
end

data = nan(numel(record_lines), 9);
for k = 1:numel(record_lines)
    values = str2double(strsplit(strtrim(record_lines{k}), ','));
    if numel(values) == 6
        data(k, 1:6) = values;
    elseif numel(values) == 9
        data(k, :) = values;
    else
        error('Sweep row %u has %u columns; expected 6 or 9.', ...
              k, numel(values));
    end
end

% Keep the last record for a duplicated index, then restore sweep order.
[~, unique_rows] = unique(data(:, 1), 'last');
data = sortrows(data(unique_rows, :), 1);

index = data(:, 1);
frequency_hz = data(:, 2);
iq_amplitude_a = data(:, 3);
position_amplitude_rad = data(:, 4);
gain_rad_per_a = data(:, 5);
phase_deg = data(:, 6);
window_count = data(:, 7);
gain_std_percent = data(:, 8);
phase_std_deg = data(:, 9);
repeatability_available = all(isfinite(data(:, 7:9)), 2);

if any(~isfinite(data(:, 1:6)), 'all') || any(frequency_hz <= 0) || ...
        any(iq_amplitude_a <= 0) || any(position_amplitude_rad <= 0) || ...
        any(gain_rad_per_a <= 0)
    error('Sweep records contain invalid required numeric values.');
end
if any(any(isfinite(data(:, 7:9)), 2) & ~repeatability_available)
    error('Repeatability columns must either all be present or all be absent.');
end
if any(window_count(repeatability_available) < 2) || ...
        any(window_count(repeatability_available) ~= ...
            round(window_count(repeatability_available))) || ...
        any(gain_std_percent(repeatability_available) < 0) || ...
        any(phase_std_deg(repeatability_available) < 0)
    error('Sweep repeatability values are invalid.');
end

sweep_table = table(index, frequency_hz, iq_amplitude_a, ...
                    position_amplitude_rad, gain_rad_per_a, phase_deg, ...
                    window_count, gain_std_percent, phase_std_deg);
disp(sweep_table);

%% Confidence and frequency-balance weights
% A repeated frequency receives the same total weight as a single frequency.
% Window scatter changes that weight, but is capped to prevent overconfidence.
quality_weight = ones(size(frequency_hz));
if any(repeatability_available)
    relative_standard_error = sqrt((...
        gain_std_percent(repeatability_available)/100).^2 + ...
        deg2rad(phase_std_deg(repeatability_available)).^2) ./ ...
        sqrt(window_count(repeatability_available));
    raw_quality = 1 ./ max(relative_standard_error, 0.01).^2;
    reference_quality = median(raw_quality);
    quality_weight(repeatability_available) = min(max(...
        raw_quality/reference_quality, 0.25), 4.0);

    fprintf('Median window gain std = %.3f %%\n', ...
            median(gain_std_percent(repeatability_available)));
    fprintf('Median window phase std = %.3f deg\n', ...
            median(phase_std_deg(repeatability_available)));
end

[unique_frequency_hz, ~, frequency_group] = unique(frequency_hz, 'sorted');
records_per_frequency = accumarray(frequency_group, 1);
fit_weight = quality_weight ./ records_per_frequency(frequency_group);
fit_weight = fit_weight / mean(fit_weight);

%% Frequency response and staged bands
w = 2*pi*frequency_hz;
measured_response = gain_rad_per_a .* exp(1j*deg2rad(phase_deg));
fit_mask = true(size(frequency_hz));
low_mask = frequency_hz <= 5.0;
high_mask = frequency_hz >= 10.0;

if nnz(fit_mask) < 3 || nnz(low_mask) < 3 || nnz(high_mask) < 3
    error('All, low, and high identification bands require at least 3 records.');
end

options = optimset('Display', 'off', ...
                   'TolX', 1e-10, ...
                   'TolFun', 1e-10, ...
                   'MaxIter', 1500, ...
                   'MaxFunEvals', 5000);

% For a velocity-friction mechanical model, real(1/H) = -Ka*w^2.
% This initializes Ka from the high-frequency band without assigning the
% imaginary friction term to inertia.
inverse_high_response = 1 ./ measured_response(high_mask);
positive_ka = -real(inverse_high_response) ./ (w(high_mask).^2);
positive_ka = positive_ka(isfinite(positive_ka) & (positive_ka > 0));
if isempty(positive_ka)
    error('High-frequency records do not provide a positive Ka estimate.');
end
high_ka_initial = log(median(positive_ka));
high_ka_cost = @(log_ka) weighted_scaled_real_cost(...
    -exp(log_ka)*w(high_mask).^2, real(inverse_high_response), ...
    abs(inverse_high_response), fit_weight(high_mask));
log_ka_stage = fminsearch(high_ka_cost, high_ka_initial, options);
Ka_stage = exp(log_ka_stage);

%% Viscous-only Ka-Kv model
model_response = @(x, wq) 1 ./ (...
    exp(x(1))*(1j*wq).^2 + exp(x(2))*(1j*wq));
linear_cost = @(x) weighted_relative_cost(...
    model_response(x, w), measured_response, fit_weight);
linear_initial_guess = log([Ka_stage; 1.0e-2]);
x = fminsearch(linear_cost, linear_initial_guess, options);

Ka = exp(x(1));
Kv = exp(x(2));
corner_frequency_hz = Kv / (2*pi*Ka);
normalized_gain = 1 / Ka;
normalized_pole = Kv / Ka;
fitted_response = model_response(x, w);
fit_rms_percent = relative_rms_percent(fitted_response, measured_response);
fit_weighted_rms_percent = weighted_relative_rms_percent(...
    fitted_response, measured_response, fit_weight);

fprintf('\nStaged identification:\n');
fprintf('Low band: %.3f Hz to %.3f Hz (%u records)\n', ...
        min(frequency_hz(low_mask)), max(frequency_hz(low_mask)), ...
        nnz(low_mask));
fprintf('High band: %.3f Hz to %.3f Hz (%u records)\n', ...
        min(frequency_hz(high_mask)), max(frequency_hz(high_mask)), ...
        nnz(high_mask));
fprintf('High-band Ka initialization = %.9g A*s^2/rad\n', Ka_stage);

fprintf('\nViscous-only joint fit (%u records):\n', nnz(fit_mask));
fprintf('Ka = %.9g A*s^2/rad\n', Ka);
fprintf('Kv = %.9g A*s/rad\n', Kv);
fprintf('Mechanical corner frequency = %.6g Hz\n', corner_frequency_hz);
fprintf('Relative complex RMS error = %.2f %%\n', fit_rms_percent);
fprintf('Weighted relative complex RMS error = %.2f %%\n', ...
        fit_weighted_rms_percent);
fprintf('\nG(s) = 1 / (%.9g*s^2 + %.9g*s)\n', Ka, Kv);
fprintf('     = %.9g / (s^2 + %.9g*s)\n\n', ...
        normalized_gain, normalized_pole);

if exist('tf', 'file') == 2
    G_mech = tf(1, [Ka, Kv, 0]);
    disp('Viscous-only mechanical transfer function:');
    G_mech
else
    warning(['Control System Toolbox is unavailable. The numeric transfer ' ...
             'function coefficients were printed above.']);
end

%% Coulomb-friction describing-function model
% Ka*theta_ddot + Kv*theta_dot + Tc*sign(theta_dot) = Iq.
% First initialize Kv and Tc in the low band with Ka fixed, then jointly
% refine Ka, Kv, and Tc using every record.
friction_model_response = @(xq, wq, iq_q) ...
    coulomb_friction_response(xq, wq, iq_q);
low_friction_cost = @(y) weighted_relative_cost(...
    friction_model_response([log(Ka_stage); y(:)], ...
                            w(low_mask), iq_amplitude_a(low_mask)), ...
    measured_response(low_mask), fit_weight(low_mask));
low_initial_guess = log([max(0.5*Kv, 1e-4); 0.03]);
y_low = fminsearch(low_friction_cost, low_initial_guess, options);
Kv_stage = exp(y_low(1));
Tc_stage = exp(y_low(2));

friction_cost = @(xq) weighted_relative_cost(...
    friction_model_response(xq, w, iq_amplitude_a), ...
    measured_response, fit_weight);
friction_initial_guess = log([Ka_stage; Kv_stage; Tc_stage]);
x_friction = fminsearch(friction_cost, friction_initial_guess, options);

Ka_friction = exp(x_friction(1));
Kv_friction = exp(x_friction(2));
Tc_friction = exp(x_friction(3));
friction_corner_frequency_hz = Kv_friction / (2*pi*Ka_friction);
fitted_friction_response = friction_model_response(...
    x_friction, w, iq_amplitude_a);
friction_rms_percent = relative_rms_percent(...
    fitted_friction_response, measured_response);
friction_weighted_rms_percent = weighted_relative_rms_percent(...
    fitted_friction_response, measured_response, fit_weight);
friction_low_rms_percent = relative_rms_percent(...
    fitted_friction_response(low_mask), measured_response(low_mask));
friction_high_rms_percent = relative_rms_percent(...
    fitted_friction_response(high_mask), measured_response(high_mask));
friction_improvement_percent = fit_weighted_rms_percent - ...
    friction_weighted_rms_percent;

fprintf('\nLow-band friction initialization with fixed Ka:\n');
fprintf('Kv initialization = %.9g A*s/rad\n', Kv_stage);
fprintf('Tc initialization = %.9g A\n', Tc_stage);

fprintf('\nCoulomb-friction joint fit:\n');
fprintf('Ka = %.9g A*s^2/rad\n', Ka_friction);
fprintf('Kv = %.9g A*s/rad\n', Kv_friction);
fprintf('Tc = %.9g A (equivalent Coulomb-friction current)\n', ...
        Tc_friction);
fprintf('Viscous corner frequency = %.6g Hz\n', ...
        friction_corner_frequency_hz);
fprintf('Relative complex RMS error = %.2f %%\n', ...
        friction_rms_percent);
fprintf('Weighted relative complex RMS error = %.2f %%\n', ...
        friction_weighted_rms_percent);
fprintf('Low-band RMS error = %.2f %%\n', friction_low_rms_percent);
fprintf('High-band RMS error = %.2f %%\n', friction_high_rms_percent);
fprintf('Weighted RMS improvement over Ka-Kv model = %.2f points\n', ...
        friction_improvement_percent);
fprintf(['This amplitude-dependent friction model is nonlinear and does not ' ...
         'have one fixed transfer function.\n\n']);

%% Bode plot
f_plot = logspace(log10(0.8*min(frequency_hz)), ...
                  log10(1.2*max(frequency_hz)), 800).';
w_plot = 2*pi*f_plot;
viscous_response_plot = model_response(x, w_plot);
representative_iq = accumarray(frequency_group, iq_amplitude_a, [], @mean);
iq_amplitude_plot = interp1(log(unique_frequency_hz), representative_iq, ...
                            log(f_plot), 'linear', 'extrap');
friction_response_plot = friction_model_response(...
    x_friction, w_plot, iq_amplitude_plot);
measured_phase_unwrapped_deg = rad2deg(unwrap(deg2rad(phase_deg)));
viscous_phase_deg = rad2deg(unwrap(angle(viscous_response_plot)));
friction_phase_deg = rad2deg(unwrap(angle(friction_response_plot)));
fitted_friction_phase_deg = rad2deg(unwrap(angle(fitted_friction_response)));

figure('Color', 'w', 'Name', 'Iq to mechanical position sweep');

subplot(2, 1, 1);
if any(repeatability_available)
    magnitude_error_db = zeros(size(gain_std_percent));
    magnitude_error_db(repeatability_available) = ...
        (20/log(10))*(gain_std_percent(repeatability_available)/100);
    errorbar(frequency_hz, 20*log10(gain_rad_per_a), ...
             magnitude_error_db, 'Color', [0.65 0.65 0.65], ...
             'LineStyle', 'none', 'HandleVisibility', 'off');
    hold on;
end
semilogx(frequency_hz, 20*log10(gain_rad_per_a), 'ko', ...
         'MarkerFaceColor', 'k', 'LineStyle', 'none', ...
         'DisplayName', 'Measured');
hold on;
semilogx(f_plot, 20*log10(abs(viscous_response_plot)), 'r--', ...
         'LineWidth', 1.5, 'DisplayName', 'Ka-Kv joint fit');
semilogx(f_plot, 20*log10(abs(friction_response_plot)), 'b-', ...
         'LineWidth', 1.6, ...
         'DisplayName', 'Ka-Kv-Tc fit at mean Iq');
semilogx(frequency_hz, 20*log10(abs(fitted_friction_response)), 'bx', ...
         'LineStyle', 'none', 'HandleVisibility', 'off');
set(gca, 'XScale', 'log');
grid on;
ylabel('Magnitude (dB, rad/A)');
title('Actual Iq to mechanical position frequency response');
legend('Location', 'southwest');

subplot(2, 1, 2);
if any(repeatability_available)
    phase_error = zeros(size(phase_std_deg));
    phase_error(repeatability_available) = ...
        phase_std_deg(repeatability_available);
    errorbar(frequency_hz, measured_phase_unwrapped_deg, phase_error, ...
             'Color', [0.65 0.65 0.65], 'LineStyle', 'none', ...
             'HandleVisibility', 'off');
    hold on;
end
semilogx(frequency_hz, measured_phase_unwrapped_deg, 'ko', ...
         'MarkerFaceColor', 'k', 'LineStyle', 'none', ...
         'DisplayName', 'Measured');
hold on;
semilogx(f_plot, viscous_phase_deg, 'r--', ...
         'LineWidth', 1.5, 'DisplayName', 'Ka-Kv joint fit');
semilogx(f_plot, friction_phase_deg, 'b-', ...
         'LineWidth', 1.6, ...
         'DisplayName', 'Ka-Kv-Tc fit at mean Iq');
semilogx(frequency_hz, fitted_friction_phase_deg, 'bx', ...
         'LineStyle', 'none', 'HandleVisibility', 'off');
set(gca, 'XScale', 'log');
grid on;
xlabel('Frequency (Hz)');
ylabel('Phase (deg)');
legend('Location', 'southwest');

function cost = weighted_scaled_real_cost(model_value, measured_value, ...
                                          scale, weight)
    residual = (model_value(:) - measured_value(:)) ./ ...
        max(scale(:), eps);
    cost = sum(weight(:).*residual.^2) / sum(weight);
    if ~isfinite(cost)
        cost = realmax;
    end
end

function cost = weighted_relative_cost(model_value, measured_value, weight)
    if any(~isfinite(model_value)) || any(abs(model_value) < eps)
        cost = 1e6;
        return;
    end
    residual = (model_value(:) - measured_value(:)) ./ measured_value(:);
    cost = sum(weight(:).*abs(residual).^2) / sum(weight);
    if ~isfinite(cost)
        cost = 1e6;
    end
end

function value = relative_rms_percent(model_value, measured_value)
    residual = (model_value(:) - measured_value(:)) ./ measured_value(:);
    value = 100*sqrt(mean(abs(residual).^2));
end

function value = weighted_relative_rms_percent(model_value, ...
                                                measured_value, weight)
    residual = (model_value(:) - measured_value(:)) ./ measured_value(:);
    value = 100*sqrt(sum(weight(:).*abs(residual).^2) / sum(weight));
end

function response = coulomb_friction_response(x, w, iq_amplitude)
    Ka = exp(x(1));
    Kv = exp(x(2));
    Tc = exp(x(3));
    friction_fundamental = 4*Tc/pi;

    w = w(:);
    iq_amplitude = iq_amplitude(:);
    quadratic = (Ka*w.^2).^2 + (Kv*w).^2;
    radicand = (Kv*w*friction_fundamental).^2 + ...
        quadratic.*(iq_amplitude.^2 - friction_fundamental^2);
    valid = (iq_amplitude > friction_fundamental) & (radicand > 0);

    position_amplitude = zeros(size(w));
    position_amplitude(valid) = (...
        -Kv*w(valid)*friction_fundamental + sqrt(radicand(valid))) ./ ...
        quadratic(valid);
    valid = valid & (position_amplitude > 0);

    response = complex(zeros(size(w)));
    effective_denominator = -Ka*w(valid).^2 + 1j*(...
        Kv*w(valid) + friction_fundamental./position_amplitude(valid));
    response(valid) = 1./effective_denominator;
end
