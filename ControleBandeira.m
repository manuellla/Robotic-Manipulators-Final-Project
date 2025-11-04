clear; close all; clc;

% Colunas: [theta d a alpha]
% d e a em metros, angulos em radianos  
dh = [...
    0       -0.450   0.150   pi/2;     
   -pi/2     0        0.590   pi;       
    pi/2     0        0.130  -pi/2;    
    0       -0.6471   0      -pi/2;    
    0        0        0       pi/2;    
    pi      -0.095    0       pi];      

for i = 1:6
    L(i) = Link('d', dh(i,2), 'a', dh(i,3), 'alpha', dh(i,4), 'offset', dh(i,1));
end

COMAU_SmartSix = SerialLink(L, 'name', 'COMAU Smart Six');
COMAU_SmartSix.base = trotx(pi);

% Configuração inicial
q_inicial = [0; 0; -pi/2; 0; -pi/2; 0];

% Parâmetros do controle de regulação
K = 0.8;      % Ganho proporcional
epsilon = 1e-4;   % Critério de parada
max_iter = 1000;  % Número máximo de iterações
dt = 0.05;  % Passo de integração

% Orientação desejada
% Eixo Ze do efetuador (approach) na mesma direção do eixo Xb da base
% Eixo Xe do efetuador (normal) na direção contrária ao eixo Zb da base
Ze_desired = [1; 0; 0];  % approach na direção de Xb
Xe_desired = [0; 0; -1]; % normal na direção -Zb
Ye_desired = cross(Ze_desired, Xe_desired);
Rd = [Xe_desired, Ye_desired, Ze_desired];

% movimentação retângulo

% movimento 1: Inicial para P0 (regulação)
fprintf('ETAPA 1: RETÂNGULO\n');
fprintf('Controle de regulação até P0\n\n');
P0 = [700; 0; 650] / 1000;
[theta_P0, pos_P0, theta_traj_P0, err_P0, u_P0, err_pos_P0, err_rpy_P0, err_axang_P0] = ...
    controleRegulacao(COMAU_SmartSix, q_inicial, P0, Rd, K, epsilon, max_iter, dt);

% movimento 2: P0 para P1 (regulação)
fprintf('\nControle de regulação até P1\n\n');
P1 = [1000; -600; 1000] / 1000;
[theta_P1, pos_P1, theta_traj_P1, err_P1, u_P1, err_pos_P1, err_rpy_P1, err_axang_P1] = ...
    controleRegulacao(COMAU_SmartSix, theta_P0, P1, Rd, K, epsilon, max_iter, dt);

% movimentação retangulo (P1-P2-P3-P4-P1) (trajetória)
fprintf('\nControle de seguimento de trajetória: desenho do retângulo\n\n');
P2 = [1000; 600; 1000] / 1000;
P3 = [1000; 600; 300] / 1000;
P4 = [1000; -600; 300] / 1000;
traj_points_rect = {P1, P2, P3, P4, P1}; % sequência de pontos
theta_prev = theta_P1;
pos_traj_rect = [];
theta_traj_rect = [];
err_rect = [];
u_rect = [];
err_pos_rect = [];
err_rpy_rect = [];
err_axang_rect = [];
for k = 1:length(traj_points_rect)-1
    p_start = traj_points_rect{k};
    p_end = traj_points_rect{k+1};
    tf = 5; % duração (s)
    [theta_prev, pos_temp, theta_temp, err_temp, u_temp, err_pos_temp, err_rpy_temp, err_axang_temp] = ...
        controleTrajetoria(COMAU_SmartSix, theta_prev, p_start, p_end, Rd, K, dt, tf);
    pos_traj_rect = [pos_traj_rect, pos_temp];
    theta_traj_rect = [theta_traj_rect, theta_temp];
    err_rect = [err_rect, err_temp];
    u_rect = [u_rect, u_temp];
    err_pos_rect = [err_pos_rect, err_pos_temp];
    err_rpy_rect = [err_rpy_rect, err_rpy_temp];
    err_axang_rect = [err_axang_rect, err_axang_temp];
end

% movimento final do retangulo: P1 para P0 (regulação)
fprintf('\nControle de regulação de retorno (Retângulo) até P0\n\n');
[theta_return_R, pos_return_R, theta_traj_return_R, err_return_R, u_return_R, err_pos_return_R, err_rpy_return_R, err_axang_return_R] = ...
    controleRegulacao(COMAU_SmartSix, theta_prev, P0, Rd, K, epsilon, max_iter, dt);

% movimentação losango

fprintf('\nETAPA 2: LOSANGO\n');
% Pontos do Losango (em metros)
P5 = [1000; 0; 300] / 1000;
PL1 = [1000; -600; 650] / 1000;
P6_losango = [1000; 0; 1000] / 1000;
PL2 = [1000; 600; 650] / 1000;

% movimento 1: P0 para P5 (regulação)
fprintf('Controle de regulação até P5\n\n');
theta_prev = theta_return_R; % Começa de onde parou (P0)
[theta_P5, pos_P5, theta_traj_P5, err_P5, u_P5, err_pos_P5, err_rpy_P5, err_axang_P5] = ...
    controleRegulacao(COMAU_SmartSix, theta_prev, P5, Rd, K, epsilon, max_iter, dt);

% movimentação losango (P5-PL2-P6-PL2-P5) (trajetória)
fprintf('\nControle de seguimento de trajetória: desenho do losango (anti-horário)\n\n');
traj_points_losango = {P5, PL2, P6_losango, PL1, P5}; % sequência anti-horária
theta_prev = theta_P5; % Começa de P5
pos_traj_losango = [];
theta_traj_losango = [];
err_losango = [];
u_losango = [];
err_pos_losango = [];
err_rpy_losango = [];
err_axang_losango = [];

for k = 1:length(traj_points_losango)-1
    p_start = traj_points_losango{k};
    p_end = traj_points_losango{k+1};
    tf = 5; % duração (s)
    [theta_prev, pos_temp, theta_temp, err_temp, u_temp, err_pos_temp, err_rpy_temp, err_axang_temp] = ...
        controleTrajetoria(COMAU_SmartSix, theta_prev, p_start, p_end, Rd, K, dt, tf);
    pos_traj_losango = [pos_traj_losango, pos_temp];
    theta_traj_losango = [theta_traj_losango, theta_temp];
    err_losango = [err_losango, err_temp];
    u_losango = [u_losango, u_temp];
    err_pos_losango = [err_pos_losango, err_pos_temp];
    err_rpy_losango = [err_rpy_losango, err_rpy_temp];
    err_axang_losango = [err_axang_losango, err_axang_temp];
end

% movimento final do losango: P5 para P0 (regulação)
fprintf('\nControle de regulação de retorno (Losango) até P0\n\n');
[theta_return_L, pos_return_L, theta_traj_return_L, err_return_L, u_return_L, err_pos_return_L, err_rpy_return_L, err_axang_return_L] = ...
    controleRegulacao(COMAU_SmartSix, theta_prev, P0, Rd, K, epsilon, max_iter, dt);

% movimento círculo

fprintf('\nETAPA 3: CÍRCULO\n');
Xc_circ = 1000 / 1000;
yc_circ = 0 / 1000;
zc_circ = 650 / 1000;
r_circ = 302.32 / 1000;

P_C_Start = [Xc_circ; yc_circ; zc_circ + r_circ]; % [1000; 0; 952.32] / 1000
fprintf('Ponto inicial do círculo: [%.1f, %.1f, %.2f] mm\n', P_C_Start(1)*1000, P_C_Start(2)*1000, P_C_Start(3)*1000);

fprintf('Controle de regulação até Ponto Inicial do Círculo\n\n');
theta_prev = theta_return_L; % Começa de onde parou (P0)
[theta_C_Start, pos_C_Start, theta_traj_C_Start, err_C_Start, u_C_Start, err_pos_C_Start, err_rpy_C_Start, err_axang_C_Start] = ...
    controleRegulacao(COMAU_SmartSix, theta_prev, P_C_Start, Rd, K, epsilon, max_iter, dt);

fprintf('\nControle de seguimento de trajetória: desenho do círculo (horário)\n\n');
tf_circ = 10; % 10 segundos para completar o círculo
ang_start = pi/2; % Ponto inicial P6 (t=pi/2)
ang_end = pi/2 - 2*pi; % Uma volta completa no sentido horário

[theta_C_End, pos_traj_circ, theta_traj_circ, err_circ, u_circ, err_pos_circ, err_rpy_circ, err_axang_circ] = ...
    controleTrajetoriaCirculo(COMAU_SmartSix, theta_C_Start, Rd, K, dt, tf_circ, ...
                              Xc_circ, yc_circ, zc_circ, r_circ, ang_start, ang_end);

% movimento final do círculo: P_C_Start para P0 (regulação)
% O ponto final do círculo é o mesmo ponto inicial P_C_Start
fprintf('\nControle de regulação de retorno (Círculo) até P0\n\n');
[theta_return_C, pos_return_C, theta_traj_return_C, err_return_C, u_return_C, err_pos_return_C, err_rpy_return_C, err_axang_return_C] = ...
    controleRegulacao(COMAU_SmartSix, theta_C_End, P0, Rd, K, epsilon, max_iter, dt);


% Concatenar todos os dados
pos_traj = [pos_P0, pos_P1, pos_traj_rect, pos_return_R, ...
            pos_P5, pos_traj_losango, pos_return_L, ...
            pos_C_Start, pos_traj_circ, pos_return_C];
        
theta_traj = [theta_traj_P0, theta_traj_P1, theta_traj_rect, theta_traj_return_R, ...
              theta_traj_P5, theta_traj_losango, theta_traj_return_L, ...
              theta_traj_C_Start, theta_traj_circ, theta_traj_return_C];
          
err = [err_P0, err_P1, err_rect, err_return_R, ...
       err_P5, err_losango, err_return_L, ...
       err_C_Start, err_circ, err_return_C];
   
u_traj = [u_P0, u_P1, u_rect, u_return_R, ...
          u_P5, u_losango, u_return_L, ...
          u_C_Start, u_circ, u_return_C];
      
err_pos = [err_pos_P0, err_pos_P1, err_pos_rect, err_pos_return_R, ...
           err_pos_P5, err_pos_losango, err_pos_return_L, ...
           err_pos_C_Start, err_pos_circ, err_pos_return_C];
       
err_rpy = [err_rpy_P0, err_rpy_P1, err_rpy_rect, err_rpy_return_R, ...
           err_rpy_P5, err_rpy_losango, err_rpy_return_L, ...
           err_rpy_C_Start, err_rpy_circ, err_rpy_return_C];
       
err_axang = [err_axang_P0, err_axang_P1, err_axang_rect, err_axang_return_R, ...
             err_axang_P5, err_axang_losango, err_axang_return_L, ...
             err_axang_C_Start, err_axang_circ, err_axang_return_C];

% q_seq para CoppeliaSim
step_coppelia = 5;
indices = 1:step_coppelia:size(theta_traj, 2);
q_seq = rad2deg(theta_traj(:, indices));
q_seq = [rad2deg(q_inicial), q_seq];

fprintf('\nDados para o Coppelia:\n');
fprintf('Tamanho de q_seq: %d juntas x %d pontos\n', size(q_seq,1), size(q_seq,2));

% visualização
tempo = (0:size(theta_traj,2)-1) * dt; % Vetor de tempo para os gráficos

if ~exist('figs', 'dir')
    mkdir('figs');
end

% Figure 1: Animação 3D do manipulador
figure(1);
COMAU_SmartSix.plot(deg2rad(q_seq)', ...
    'workspace', [-2 2 -2 2 -0.5 2], 'scale', 0.5, 'trail', {'r', 'LineWidth', 2});
title('Trajetória completa');
saveas(gcf, fullfile('figs', 'fig1_animacao_3D.png'));

% Figure 2: Caminho 3D do efetuador
figure(2);
plot3(pos_traj(1,:), pos_traj(2,:), pos_traj(3,:), 'b-', 'LineWidth', 2);
hold on;
plot3(P0(1), P0(2), P0(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor','g');
plot3(P1(1), P1(2), P1(3), 'mo', 'MarkerSize', 10, 'MarkerFaceColor','m');
plot3(P2(1), P2(2), P2(3), 'co', 'MarkerSize', 10, 'MarkerFaceColor','c');
plot3(P3(1), P3(2), P3(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor','r');
plot3(P4(1), P4(2), P4(3), 'yo', 'MarkerSize', 10, 'MarkerFaceColor','y');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Caminho 3D percorrido pelo efetuador');
grid on; axis equal;
legend('Trajetória', 'P0', 'P1', 'P2', 'P3', 'P4');
view(3);
saveas(gcf, fullfile('figs', 'fig2_caminho_3D.png'));

% Figure 3: Posição das juntas ao longo do tempo
figure(3);
for i = 1:6
    subplot(3,2,i);
    plot(tempo, rad2deg(theta_traj(i,:)), 'LineWidth', 1.5);
    xlabel('Tempo (s)');
    ylabel(['Junta ' num2str(i) ' (°)']);
    title(['Posição da Junta ' num2str(i)]);
    grid on;
end
sgtitle('Posição das Juntas ao Longo do Tempo');
saveas(gcf, fullfile('figs', 'fig3_posicao_juntas.png'));

% Figure 4: Ação de controle (velocidades das juntas)
figure(4);
for i = 1:6
    subplot(3,2,i);
    plot(tempo, rad2deg(u_traj(i,:)), 'LineWidth', 1.5);
    xlabel('Tempo (s)');
    ylabel(['u_' num2str(i) ' (°/s)']);
    title(['Ação de Controle - Junta ' num2str(i)]);
    grid on;
end
sgtitle('Ação de Controle (Velocidades das Juntas)');
saveas(gcf, fullfile('figs', 'fig4_acao_controle.png'));

% Figure 5: Erro de posição (distância euclidiana)
figure(5);
plot(tempo, err_pos*1000, 'LineWidth', 2);
xlabel('Tempo (s)');
ylabel('Erro de Posição (mm)');
title('Erro de Posição (Distância Euclidiana)');
grid on;
saveas(gcf, fullfile('figs', 'fig5_erro_posicao.png'));

% Figure 6: Erro de orientação - Roll, Pitch, Yaw
figure(6);
subplot(3,1,1);
plot(tempo, rad2deg(err_rpy(1,:)), 'r', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('Roll (°)');
title('Erro de Roll');
grid on;

subplot(3,1,2);
plot(tempo, rad2deg(err_rpy(2,:)), 'g', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('Pitch (°)');
title('Erro de Pitch');
grid on;

subplot(3,1,3);
plot(tempo, rad2deg(err_rpy(3,:)), 'b', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('Yaw (°)');
title('Erro de Yaw');
grid on;

sgtitle('Erro de Orientação (Roll, Pitch, Yaw)');
saveas(gcf, fullfile('figs', 'fig6_erro_orientacao_rpy.png'));

% Figure 7: Erro de orientação - Representação Eixo-Ângulo (usado no controle)
figure(7);
subplot(4,1,1);
plot(tempo, rad2deg(err_axang(1,:)), 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('n_x φ (°)');
title('Erro Eixo-Ângulo - Componente X');
grid on;

subplot(4,1,2);
plot(tempo, rad2deg(err_axang(2,:)), 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('n_y φ (°)');
title('Erro Eixo-Ângulo - Componente Y');
grid on;

subplot(4,1,3);
plot(tempo, rad2deg(err_axang(3,:)), 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('n_z φ (°)');
title('Erro Eixo-Ângulo - Componente Z');
grid on;

subplot(4,1,4);
plot(tempo, vecnorm(err_axang), 'k', 'LineWidth', 2);
xlabel('Tempo (s)'); ylabel('||nφ|| (rad)');
title('Norma do Erro de Orientação (Eixo-Ângulo)');
grid on;

sgtitle('Erro de Orientação - Representação Eixo-Ângulo (Usado no Controle)');
saveas(gcf, fullfile('figs', 'fig7_erro_eixo_angulo.png'));

% Funções:
% Função de controle de regulação
function [theta_final, pos_traj, theta_traj, err, u_traj, err_pos, err_rpy, err_axang] = ...
    controleRegulacao(robot, theta_init, pd, Rd, K, epsilon, max_iter, dt)

theta = theta_init;
e_ant = ones(6,1);
e = zeros(6,1);
i = 0;

err = [];
pos_traj = [];
theta_traj = [];
u_traj = [];
err_pos = [];
err_rpy = [];
err_axang = [];

while (norm(e - e_ant) > epsilon && i < max_iter)
    i = i + 1;
    
    J = robot.jacob0(theta');
    T_atual = robot.fkine(theta');
    T_atual = T_atual.T;
    
    p_atual = T_atual(1:3, 4);
    R_atual = T_atual(1:3, 1:3);
    
    % Erro de posição
    p_err = pd - p_atual;
    
    % Erro de orientação (eixo-ângulo)
    R_err = Rd * R_atual';
    axang = rotm2axang2(R_err);
    n = axang(1:3)';
    phi = axang(4);
    nphi_err = n * phi;
    
    % Erro de orientação (Roll, Pitch, Yaw)
    rpy_err = tr2rpy(R_err);
    %rpy_err = rotm2rpy(R_err)';
    %rpy_err = [rpy_err(1); rpy_err(2); rpy_err(3)];  % [roll; pitch; yaw]

    % Vetor de erro
    e_ant = e;
    e = [p_err; nphi_err];
    
    % Lei de controle
    u = pinv(J) * K * e;
    
    % Atualização das juntas
    theta = theta + dt * u;
    
    % Armazenando
    err(end+1) = norm(e);
    pos_traj(:,end+1) = p_atual;
    theta_traj(:,end+1) = theta;
    u_traj(:,end+1) = u;
    err_pos(end+1) = norm(p_err);
    err_rpy(:,end+1) = rpy_err;
    err_axang(:,end+1) = nphi_err;
end

theta_final = theta;
end

% Função de controle de trajetória (seguimento)
function [theta_final, pos_traj, theta_traj, err, u_traj, err_pos, err_rpy, err_axang] = ...
    controleTrajetoria(robot, theta_init, p_start, p_end, Rd, K, dt, tf)

theta = theta_init;
pos_traj = [];
theta_traj = [];
err = [];
u_traj = [];
err_pos = [];
err_rpy = [];
err_axang = [];

t = 0:dt:tf;

for i = 1:length(t)
    s = t(i)/tf;
    pd = p_start + (p_end - p_start)*s;
    pd_dot = (p_end - p_start)/tf;
    
    T = robot.fkine(theta').T;
    p = T(1:3,4);
    R = T(1:3,1:3);
    
    % Erro de posição
    p_err = pd - p;
    
    % Erro de orientação (eixo-ângulo)
    R_err = Rd * R';
    axang = rotm2axang2(R_err);
    n = axang(1:3)';
    phi = axang(4);
    nphi_err = n * phi;
    
    % Erro de orientação (Roll, Pitch, Yaw)
    
    % comando pela matriz R_err do Robotics Toolbox = descobrir
    rpy_err = tr2rpy(R_err);
    %rpy_err = rotm2rpy(R_err)';
    rpy_err = [rpy_err(1); rpy_err(2); rpy_err(3)];  % [roll; pitch; yaw]
    
    e = [p_err; nphi_err];
    
    % Lei de controle
    v = [pd_dot; zeros(3,1)] + K*e;
    J = robot.jacob0(theta');
    u = pinv(J) * v;
    
    % Integração
    theta = theta + dt * u;
    
    % Armazenando
    pos_traj(:,end+1) = p;
    theta_traj(:,end+1) = theta;
    err(end+1) = norm(e);
    u_traj(:,end+1) = u;
    err_pos(end+1) = norm(p_err);
    err_rpy(:,end+1) = rpy_err;
    err_axang(:,end+1) = nphi_err;
end

theta_final = theta;
end


% Função de controle de seguimento de trajetória para o círculo
function [theta_final, pos_traj, theta_traj, err, u_traj, err_pos, err_rpy, err_axang] = ...
    controleTrajetoriaCirculo(robot, theta_init, Rd, K, dt, tf, Xc, yc, zc, r, ang_start, ang_end)

theta = theta_init;
pos_traj = [];
theta_traj = [];
err = [];
u_traj = [];
err_pos = [];
err_rpy = [];
err_axang = [];
t_vec = 0:dt:tf;

ang_dot = (ang_end - ang_start) / tf;

for i = 1:length(t_vec)
    t = t_vec(i);
    s = t/tf;
    
    ang_atual = ang_start + (ang_end - ang_start) * s;
    
    pd = [Xc; ...
          yc + r * cos(ang_atual); ...
          zc + r * sin(ang_atual)];
      
    pd_dot = [0; ...
              -r * sin(ang_atual) * ang_dot; ...
               r * cos(ang_atual) * ang_dot];
    
    T = robot.fkine(theta').T;
    p = T(1:3,4);
    R = T(1:3,1:3);
    
    % Erro de posição
    p_err = pd - p;
    
    % Erro de orientação (eixo-ângulo)
    R_err = Rd * R';
    axang = rotm2axang2(R_err);
    n = axang(1:3)';
    phi = axang(4);
    nphi_err = n * phi;
    
    % erro de rpy
    rpy_err = tr2rpy(R_err);
    %rpy_err = rotm2rpy(R_err)';
    rpy_err = [rpy_err(1); rpy_err(2); rpy_err(3)];  % [roll; pitch; yaw]
    
    e = [p_err; nphi_err];
    
    v = [pd_dot; zeros(3,1)] + K*e;
    J = robot.jacob0(theta');
    u = pinv(J) * v;
    
    % Integração
    theta = theta + dt * u;
    
    pos_traj(:,end+1) = p;
    theta_traj(:,end+1) = theta;
    err(end+1) = norm(e);
    u_traj(:,end+1) = u;
    err_pos(end+1) = norm(p_err);
    err_rpy(:,end+1) = rpy_err;
    err_axang(:,end+1) = nphi_err;
end
theta_final = theta;
end
