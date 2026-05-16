%% ============================================================
%  HEADING CONTROL - DIFFERENTIAL DRIVE ROVER
%  Auto-Generate Model Simulink
%  Praktikum Teknik Kendali dan Otomasi - Universitas Diponegoro
%  Kelompok 28 | 2026
%% ============================================================
%  CARA PAKAI:
%    1. Buka MATLAB (R2021b ke atas disarankan)
%    2. Jalankan script: Run > Run (F5)
%    3. Model Simulink akan terbuka otomatis
%    4. Klik Run di Simulink untuk mulai simulasi
%    5. Double-click blok Scope untuk melihat grafik heading
%% ============================================================

clear; clc; close all;

%% ============================================================
%% [1] PARAMETER SISTEM
%% ============================================================

% --- Parameter Motor DC ---
K  = 0.01;     % Motor constant (Nm/A atau V·s/rad)
J  = 0.01;     % Momen inersia rotor (kg·m²)
b  = 0.1;      % Koefisien gesek viskos (N·m·s)
L  = 0.5;      % Induktansi armatur (H)
R  = 1.0;      % Resistansi armatur (Ohm)

% --- Parameter Kinematika Rover ---
r_wheel = 0.05;  % Jari-jari roda (m)
L_base  = 0.30;  % Jarak antar roda / wheelbase (m)

% --- Parameter Simulasi ---
Vbase    = 1.0;   % Kecepatan maju dasar (m/s, sebagai referensi tegangan)
sim_time = 15;    % Durasi simulasi (detik)
setpoint = 0;     % Target heading sudut (derajat) → 0° = lurus

% --- Nilai PID Awal (bisa di-tune lewat PID Tuner di Simulink) ---
Kp = 5.0;
Ki = 1.5;
Kd = 0.8;

% --- Gangguan (Disturbance) ---
% Simulasi gundukan: heading tiba-tiba bergeser pada t=3 detik
dist_time  = 3;    % Waktu gangguan (detik)
dist_amp   = 10;   % Amplitudo gangguan (derajat)

%% ============================================================
%% [2] TRANSFER FUNCTION MOTOR DC
%%     G(s) = K / [(Js+b)(Ls+R) + K^2]
%% ============================================================

num_motor = [K];
den_motor = [J*L, (J*R + b*L), (b*R + K^2)];

fprintf('=== Transfer Function Motor DC ===\n');
fprintf('Numerator  : %s\n', mat2str(num_motor));
fprintf('Denominator: %s\n', mat2str(den_motor));
TF_motor = tf(num_motor, den_motor);
disp(TF_motor);

% Simpan ke workspace untuk dipakai di Simulink via blok Transfer Fcn
assignin('base','num_motor', num_motor);
assignin('base','den_motor', den_motor);
assignin('base','Kp', Kp);
assignin('base','Ki', Ki);
assignin('base','Kd', Kd);
assignin('base','r_wheel', r_wheel);
assignin('base','L_base', L_base);
assignin('base','Vbase', Vbase);
assignin('base','setpoint', setpoint);
assignin('base','dist_time', dist_time);
assignin('base','dist_amp', dist_amp);
assignin('base','sim_time', sim_time);

%% ============================================================
%% [3] BUILD MODEL SIMULINK SECARA PROGRAMATIK
%% ============================================================

model_name = 'HeadingControl_DiffDriveRover';

% Hapus model lama jika ada
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

% Buat model baru
new_system(model_name);
open_system(model_name);

% Set waktu simulasi
set_param(model_name, 'StopTime', num2str(sim_time));
set_param(model_name, 'Solver', 'ode45');

%% ============================================================
%% [4] BANGUN MODEL — LAYOUT BERSIH ALA DIAGRAM BLOK KLASIK
%% ============================================================
%
%  Strategi: bundle semua dinamika rover (motor kiri+kanan, diff drive,
%  integrator heading) ke dalam SATU SUBSYSTEM bernama "Rover_Plant".
%  Diagram utama jadi sangat sederhana:
%
%   Setpoint ──►(+)── Σ ──► PID ──► Rover_Plant ──┬──► Scope
%               (-)│                              │
%               (+)│         ◄── Disturbance      │
%                  └──────────[ feedback ]────────┘
%
% ─────────────────────────────────────────────────────────────

% ── Grid layout untuk DIAGRAM UTAMA (top level) ───────────────
Y0 = 200;            % baris utama
add_block('simulink/Sources/Step',            [model_name '/Setpoint'], ...
    'Position', [40, Y0-15, 80, Y0+15], ...
    'Time','0','Before',num2str(setpoint),'After',num2str(setpoint));

add_block('simulink/Math Operations/Sum',     [model_name '/Sum_Error'], ...
    'Position', [150, Y0-15, 180, Y0+15], ...
    'Inputs','+-');

add_block('simulink/Continuous/PID Controller',[model_name '/PID_Controller'], ...
    'Position', [240, Y0-25, 340, Y0+25], ...
    'P','Kp','I','Ki','D','Kd');

add_block('simulink/Math Operations/Sum',     [model_name '/Sum_Disturb'], ...
    'Position', [400, Y0-15, 430, Y0+15], ...
    'Inputs','++');

add_block('simulink/Sources/Step',            [model_name '/Disturbance'], ...
    'Position', [380, Y0+90, 430, Y0+120], ...
    'Time',num2str(dist_time),'Before','0','After',num2str(dist_amp));

% ── SUBSYSTEM: Rover_Plant ────────────────────────────────────
% Buat subsystem kosong dengan cara yang aman
add_block('built-in/Subsystem', [model_name '/Rover_Plant'], ...
    'Position', [490, Y0-30, 620, Y0+30]);

% Hapus koneksi dan blok default di dalam subsystem secara aman
sub = [model_name '/Rover_Plant'];
default_lines = find_system(sub, 'FindAll','on','type','line');
for k = 1:length(default_lines)
    delete_line(default_lines(k));
end
default_blocks = find_system(sub, 'SearchDepth',1,'type','block');
for k = 1:length(default_blocks)
    bname = get_param(default_blocks{k},'Name');
    btype = get_param(default_blocks{k},'BlockType');
    if strcmp(btype,'Inport') || strcmp(btype,'Outport')
        delete_block(default_blocks{k});
    end
end

add_block('simulink/Sinks/Scope',             [model_name '/Scope_Heading'], ...
    'Position', [720, Y0-20, 770, Y0+20]);

add_block('simulink/Sinks/To Workspace',      [model_name '/ToWS_Heading'], ...
    'Position', [720, Y0+50, 800, Y0+80], ...
    'VariableName','heading_out','MaxDataPoints','inf','SaveFormat','Array');

add_block('simulink/Sinks/Scope',             [model_name '/Scope_Control'], ...
    'Position', [400, Y0-110, 450, Y0-70]);

%% ── ISI DALAM SUBSYSTEM Rover_Plant ───────────────────────────

% Input port (sinyal u dari luar)
add_block('simulink/Ports & Subsystems/In1', [sub '/u_in'], ...
    'Position', [30, 180, 60, 200]);

% Vbase
add_block('simulink/Sources/Constant',        [sub '/Vbase'], ...
    'Position', [30, 80, 90, 110], 'Value','Vbase');

% Sum_Right: Vbase + u
add_block('simulink/Math Operations/Sum',     [sub '/Sum_Right'], ...
    'Position', [140, 105, 170, 135], 'Inputs','++');

% Sum_Left:  Vbase - u
add_block('simulink/Math Operations/Sum',     [sub '/Sum_Left'], ...
    'Position', [140, 245, 170, 275], 'Inputs','+-');

% Motor kanan
add_block('simulink/Continuous/Transfer Fcn', [sub '/MotorRight_TF'], ...
    'Position', [230, 95, 340, 145], ...
    'Numerator','num_motor','Denominator','den_motor');

% Motor kiri
add_block('simulink/Continuous/Transfer Fcn', [sub '/MotorLeft_TF'], ...
    'Position', [230, 235, 340, 285], ...
    'Numerator','num_motor','Denominator','den_motor');

% Sum_DiffSpeed: vR - vL
add_block('simulink/Math Operations/Sum',     [sub '/Sum_DiffSpeed'], ...
    'Position', [400, 175, 430, 205], 'Inputs','+-');

% Gain r/L
add_block('simulink/Math Operations/Gain',    [sub '/Gain_rL'], ...
    'Position', [470, 175, 540, 205], 'Gain','r_wheel/L_base');

% Integrator
add_block('simulink/Continuous/Integrator',   [sub '/Integrator_Heading'], ...
    'Position', [580, 175, 620, 205], 'InitialCondition','0');

% Rad2Deg
add_block('simulink/Math Operations/Gain',    [sub '/Rad2Deg'], ...
    'Position', [660, 175, 730, 205], 'Gain','180/pi');

% Output port
add_block('simulink/Ports & Subsystems/Out1', [sub '/theta_out'], ...
    'Position', [780, 180, 810, 200]);

%% ── KONEKSI DI DALAM SUBSYSTEM ────────────────────────────────
% Input u → Sum_Right (+) port 2 dan Sum_Left (-) port 2
add_line(sub, 'u_in/1',              'Sum_Right/2', 'autorouting','smart');
add_line(sub, 'u_in/1',              'Sum_Left/2',  'autorouting','smart');

% Vbase → Sum_Right port 1 dan Sum_Left port 1
add_line(sub, 'Vbase/1',             'Sum_Right/1', 'autorouting','smart');
add_line(sub, 'Vbase/1',             'Sum_Left/1',  'autorouting','smart');

% Sums → Motor TF
add_line(sub, 'Sum_Right/1',         'MotorRight_TF/1');
add_line(sub, 'Sum_Left/1',          'MotorLeft_TF/1');

% Motor TF → Sum_DiffSpeed
add_line(sub, 'MotorRight_TF/1',     'Sum_DiffSpeed/1', 'autorouting','smart');
add_line(sub, 'MotorLeft_TF/1',      'Sum_DiffSpeed/2', 'autorouting','smart');

% Kinematika
add_line(sub, 'Sum_DiffSpeed/1',     'Gain_rL/1');
add_line(sub, 'Gain_rL/1',           'Integrator_Heading/1');
add_line(sub, 'Integrator_Heading/1','Rad2Deg/1');
add_line(sub, 'Rad2Deg/1',           'theta_out/1');

%% ── KONEKSI DI DIAGRAM UTAMA (top-level) ──────────────────────
add_line(model_name, 'Setpoint/1',       'Sum_Error/1');
add_line(model_name, 'Sum_Error/1',      'PID_Controller/1');
add_line(model_name, 'PID_Controller/1', 'Sum_Disturb/1');
add_line(model_name, 'Disturbance/1',    'Sum_Disturb/2', 'autorouting','smart');
add_line(model_name, 'Sum_Disturb/1',    'Rover_Plant/1');
add_line(model_name, 'Rover_Plant/1',    'Scope_Heading/1');
add_line(model_name, 'Rover_Plant/1',    'ToWS_Heading/1', 'autorouting','smart');
add_line(model_name, 'PID_Controller/1', 'Scope_Control/1','autorouting','smart');

% FEEDBACK — lewat bawah Rover_Plant lalu balik ke Sum_Error port 2
add_line(model_name, 'Rover_Plant/1',    'Sum_Error/2', 'autorouting','smart');

%% ============================================================
%% [6] AUTO-ARRANGE & KONFIGURASI
%% ============================================================

% Aktifkan auto-arrange Simulink agar layout dirapikan otomatis
try
    Simulink.BlockDiagram.arrangeSystem(model_name);
    Simulink.BlockDiagram.arrangeSystem([model_name '/Rover_Plant']);
catch
    % auto-arrange tidak tersedia di versi MATLAB lama, skip
end

set_param([model_name '/Scope_Heading'], 'TimeRange', num2str(sim_time));
set_param([model_name '/Scope_Control'], 'TimeRange', num2str(sim_time));

%% ============================================================
%% [7] TAMPILKAN INFO & SIMPAN MODEL
%% ============================================================

fprintf('\n=== MODEL SIMULINK BERHASIL DIBUAT ===\n');
fprintf('Nama Model  : %s\n', model_name);
fprintf('Durasi Sim  : %d detik\n', sim_time);
fprintf('Set-point   : %d derajat\n', setpoint);
fprintf('\n--- Parameter Motor DC ---\n');
fprintf('K = %.4f | J = %.4f | b = %.4f | L = %.4f | R = %.4f\n', K, J, b, L, R);
fprintf('\n--- Parameter Kinematika ---\n');
fprintf('r_wheel = %.3f m | L_base = %.3f m\n', r_wheel, L_base);
fprintf('\n--- Parameter PID ---\n');
fprintf('Kp = %.2f | Ki = %.2f | Kd = %.2f\n', Kp, Ki, Kd);
fprintf('\n--- Gangguan ---\n');
fprintf('Waktu disturbance = %d s | Amplitudo = %d derajat\n', dist_time, dist_amp);
fprintf('\n=== PETUNJUK ===\n');
fprintf('1. Klik tombol RUN (▶) di toolbar Simulink\n');
fprintf('2. Double-click "Scope_Heading" untuk melihat grafik heading\n');
fprintf('3. Untuk tuning PID: klik blok PID_Controller → Tune...\n');
fprintf('4. Ubah Kp, Ki, Kd di bagian atas script ini lalu jalankan ulang\n');
fprintf('================================================\n\n');

% Simpan model ke folder Documents (bukan System32)
save_folder = fullfile(userpath, 'HeadingControl');
if ~exist(save_folder, 'dir')
    mkdir(save_folder);
end
save_system(model_name, fullfile(save_folder, model_name));
fprintf('Model disimpan di: %s\n', fullfile(save_folder, [model_name '.slx']));

%% ============================================================
%% [8] PLOT ANALISIS OPEN-LOOP (PREVIEW SEBELUM SIMULASI)
%% ============================================================

fprintf('Menjalankan analisis open-loop motor...\n');

figure('Name', 'Analisis Sistem - Heading Control Rover', ...
    'Position', [100, 100, 1100, 700]);

% [8.1] Step Response Motor DC
subplot(2,3,1);
step(TF_motor, 0:0.01:3);
title('Step Response Motor DC');
xlabel('Waktu (s)'); ylabel('\omega (rad/s)');
grid on;

% [8.2] Bode Plot Motor DC
subplot(2,3,2);
bode(TF_motor);
title('Bode Plot Motor DC');
grid on;

% [8.3] Pole-Zero Map Motor DC
subplot(2,3,3);
pzmap(TF_motor);
title('Pole-Zero Map Motor DC');
grid on;

% [8.4] Closed-Loop dengan PID (analisis matematis sederhana)
% Plant: dari tegangan V → heading θ
% = (r/L) * (1/s) * [G_right(s) - G_left(s)] per unit u(t)
% Untuk diferensial: efek u(t) = 2 * (r/L_base) * (1/s) * G(s)
num_plant = 2 * (r_wheel/L_base) * num_motor;
den_plant = conv([1 0], den_motor);  % tambah integrator (1/s → kalikan s)

TF_plant = tf(num_plant, den_plant);

% PID Controller TF: Kp + Ki/s + Kd*s = (Kd*s^2 + Kp*s + Ki) / s
TF_pid = pid(Kp, Ki, Kd);

TF_OL = TF_pid * TF_plant;
TF_CL = feedback(TF_OL, 1);

subplot(2,3,4);
t_sim = 0:0.05:sim_time;
dist_signal = zeros(size(t_sim));
dist_signal(t_sim >= dist_time) = dist_amp;
[y_dist, ~] = lsim(feedback(TF_plant, TF_pid*1), dist_signal, t_sim);
step_ref = setpoint * ones(size(t_sim));
[y_cl, t_cl] = step(TF_CL * setpoint, t_sim);
plot(t_cl, y_cl, 'b', 'LineWidth', 2); hold on;
plot(t_sim, dist_signal * 0.1, 'r--', 'LineWidth', 1);
title(sprintf('Respons CL (Kp=%.1f, Ki=%.1f, Kd=%.1f)', Kp, Ki, Kd));
xlabel('Waktu (s)'); ylabel('Heading (derajat)');
legend('Heading', 'Disturbance (scaled)');
grid on;

% [8.5] Root Locus
subplot(2,3,5);
rlocus(TF_OL);
title('Root Locus Sistem CL');
grid on;

% [8.6] Respon terhadap disturbance
subplot(2,3,6);
[y_d, t_d] = step(feedback(TF_plant, TF_pid), 0:0.05:sim_time);
plot(t_d, y_d * dist_amp, 'r', 'LineWidth', 2); hold on;
yline(0, 'k--', 'Setpoint 0°');
yline(dist_amp, 'b:', sprintf('Disturbance %d°', dist_amp));
title('Respons terhadap Disturbance');
xlabel('Waktu (s)'); ylabel('Heading (derajat)');
legend('Respons heading', 'Setpoint', 'Level disturbance');
grid on;

sgtitle('Analisis Sistem Heading Control - Differential Drive Rover', ...
    'FontSize', 13, 'FontWeight', 'bold');

%% ============================================================
%% [9] CETAK KARAKTERISTIK RESPONS
%% ============================================================

try
    info = stepinfo(TF_CL);
    fprintf('\n=== KARAKTERISTIK RESPONS (Closed-Loop) ===\n');
    fprintf('Rise Time      : %.4f s\n', info.RiseTime);
    fprintf('Settling Time  : %.4f s\n', info.SettlingTime);
    fprintf('Overshoot      : %.2f %%\n', info.Overshoot);
    fprintf('Peak Value     : %.4f\n', info.Peak);
    fprintf('===========================================\n\n');
catch
    fprintf('(Info respons tidak tersedia untuk setpoint = 0)\n');
end

fprintf('✔ Model Simulink "%s" siap digunakan.\n', model_name);
fprintf('✔ Grafik analisis open-loop telah ditampilkan.\n');