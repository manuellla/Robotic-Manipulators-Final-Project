clear; close all; clc;

% Columns: [theta d a alpha]
% d and a in meters, angles in radians  
dh = [...
    0       -0.450   0.150   pi/2;     % Joint 1
   -pi/2     0        0.590   pi;      % Joint 2
    pi/2     0        0.130  -pi/2;    % Joint 3
    0       -0.6471   0      -pi/2;    % Joint 4
    0        0        0       pi/2;    % Joint 5
    pi      -0.095    0       pi];     % Joint 6

for i = 1:6
    L(i) = Link('d', dh(i,2), 'a', dh(i,3), 'alpha', dh(i,4), 'offset', dh(i,1));
end

COMAU_SmartSix = SerialLink(L, 'name', 'COMAU Smart Six');
COMAU_SmartSix.base = trotx(pi);

% Configuração inicial
q_inicial = [0; 0; -pi/2; 0; -pi/2; 0];

% Orientação desejada
% Eixo Ze do efetuador (approach) na mesma direção do eixo Xb da base
% Eixo Xe do efetuador (normal) na direção contrária ao eixo Zb da base
Ze_desired = [1; 0; 0];  % approach na direção de Xb
Xe_desired = [0; 0; -1]; % normal na direção -Zb
Ye_desired = cross(Ze_desired, Xe_desired);
Rd = [Xe_desired, Ye_desired, Ze_desired];

% Parâmetros do controle de regulação
K = 0.8;      % Ganho proporcional
epsilon = 1e-4;   % Critério de parada
max_iter = 1000;  % Número máximo de iterações
dt = 0.05;  % Passo de integração


% movimento 1: Inicial para P0 (regulação)
fprintf('Controle de regulação até P0\n\n');
P0_mm = [700; 0; 650];
pd0 = P0_mm / 1000;

[theta_P0, pos_traj_P0, theta_traj_P0, err_P0] = ...
    controleRegulacao(COMAU_SmartSix, q_inicial, pd0, Rd, K, epsilon, max_iter, dt);

% movimento 2: P0 para P1 (regulação)
fprintf('\nControle de regulação até P1\n\n');
P1_mm = [1000; -600; 1000];
pd1 = P1_mm / 1000;

[theta_P1, pos_traj_P1, theta_traj_P1, err_P1] = ...
    controleRegulacao(COMAU_SmartSix, theta_P0, pd1, Rd, K, epsilon, max_iter, dt);

% Preparar q_seq para CoppeliaSim
% q_seq deve ter 6 linhas (juntas) e n colunas (pontos no tempo)
% Valores em GRAUS
% q_seq está sendo salva na workspace para só rodar o script do Coppelia (n precisa mudar nada nele!)
% Reduzir a taxa de amostragem para o CoppeliaSim (a cada 5 pontos)
pos_traj = [pos_traj_P0, pos_traj_P1];
theta_traj = [theta_traj_P0, theta_traj_P1];
err = [err_P0, err_P1];

step_coppelia = 5;
indices = 1:step_coppelia:size(theta_traj, 2);

% Criar q_seq em GRAUS
q_seq = rad2deg(theta_traj(:, indices));

% Adicionar configuração inicial no começo
q_seq = [rad2deg(q_inicial), q_seq]; 

fprintf('\nDados para o Coppelia:\n');
fprintf('Tamanho de q_seq: %d juntas x %d pontos\n', size(q_seq,1), size(q_seq,2));
fprintf('Configuração inicial (graus): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', q_seq(:,1));
fprintf('Configuração final (graus):   [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', q_seq(:,end));

% Visualização no MATLAB
figure(1);
COMAU_SmartSix.plot(deg2rad(q_seq)', ...
    'workspace', [-2 2 -2 2 -0.5 2], 'scale', 0.5, 'trail', {'r', 'LineWidth', 2});
title('Trajetória completa: Inicial → P0 → P1');

figure(2);
plot3(pos_traj(1,:), pos_traj(2,:), pos_traj(3,:), 'b-', 'LineWidth', 2);
hold on;
plot3(pos_traj(1,1), pos_traj(2,1), pos_traj(3,1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot3(pd0(1), pd0(2), pd0(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(pd1(1), pd1(2), pd1(3), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Trajetória do Efetuador: Inicial → P0 → P1');
legend('Trajetória', 'Início', 'P0', 'P1');
grid on; axis equal;

figure(3);
plot(err, 'LineWidth', 2);
xlabel('Iterações'); ylabel('Norma do erro |e|');
title('Evolução do Erro Total');
grid on;


% Função de controle de regulação

function [theta_final, pos_traj, theta_traj, err] = controleRegulacao(robot, theta_init, pd, Rd, K, epsilon, max_iter, dt)

theta = theta_init;
e_ant = ones(6,1);
e = zeros(6,1);
i = 0;

control_sig = [];
err = [];
pos_traj = [];
theta_traj = [];

while (norm(e - e_ant) > epsilon && i < max_iter)
    i = i + 1;
    
    J = robot.jacob0(theta');     % Jacobiana geométrica
    
    T_atual = robot.fkine(theta');     % Cinemática direta
    T_atual = T_atual.T;
    
    % Posição e rotação atuais
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
    
    % Vetor de erro
    e_ant = e;
    e = [p_err; nphi_err];
    
    % Lei de controle
    u = pinv(J) * K * e;
    
    % Atualização das juntas
    theta = theta + dt * u;
    
    % Armazenando
    control_sig(:,i) = u;
    err(i) = norm(e);
    pos_traj(:,i) = p_atual;
    theta_traj(:,i) = theta;
    
end

theta_final = theta;
end
