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

% movimento 1: Inicial para P0 (regulação)
fprintf('Controle de regulação até P0\n\n');
P0 = [700; 0; 650] / 1000;
[theta_P0, pos_P0, theta_traj_P0, err_P0] = ...
    controleRegulacao(COMAU_SmartSix, q_inicial, P0, Rd, K, epsilon, max_iter, dt);

% movimento 2: P0 para P1 (regulação)
fprintf('\nControle de regulação até P1\n\n');
P1 = [1000; -600; 1000] / 1000;
[theta_P1, pos_P1, theta_traj_P1, err_P1] = ...
    controleRegulacao(COMAU_SmartSix, theta_P0, P1, Rd, K, epsilon, max_iter, dt);

% movimentação retangulo (P1-P2-P3-P4-P1) (trajetória)
fprintf('\nControle de seguimento de trajetória: desenho do retângulo\n\n');
P2 = [1000; 600; 1000] / 1000;
P3 = [1000; 600; 300] / 1000;
P4 = [1000; -600; 300] / 1000;

traj_points = {P1, P2, P3, P4, P1}; % sequência de pontos
theta_prev = theta_P1;
pos_traj_rect = []; theta_traj_rect = []; err_rect = [];

for k = 1:length(traj_points)-1
    p_start = traj_points{k};
    p_end = traj_points{k+1};
    tf = 5; % duração (s)
    [theta_prev, pos_temp, theta_temp, err_temp] = ...
        controleTrajetoria(COMAU_SmartSix, theta_prev, p_start, p_end, Rd, K, dt, tf);
    pos_traj_rect = [pos_traj_rect, pos_temp];
    theta_traj_rect = [theta_traj_rect, theta_temp];
    err_rect = [err_rect, err_temp];
end


% movimento final do retangulo: P1 para P0 (regulação)
fprintf('\nControle de regulação de retorno até P0\n\n');
[theta_return, pos_return, theta_traj_return, err_return] = ...
    controleRegulacao(COMAU_SmartSix, theta_prev, P0, Rd, K, epsilon, max_iter, dt);

% q_seq deve ter 6 linhas (juntas) e n colunas (pontos no tempo)
% Valores em GRAUS
% q_seq está sendo salva na workspace para só rodar o script do Coppelia (n precisa mudar nada nele!)

pos_traj = [pos_P0, pos_P1, pos_traj_rect, pos_return];
theta_traj = [theta_traj_P0, theta_traj_P1, theta_traj_rect, theta_traj_return];
err = [err_P0, err_P1, err_rect, err_return];

step_coppelia = 5; % Reduzir a taxa de amostragem para o CoppeliaSim (a cada 5 pontos)
indices = 1:step_coppelia:size(theta_traj, 2);
q_seq = rad2deg(theta_traj(:, indices)); % Criar q_seq em GRAUS
q_seq = [rad2deg(q_inicial), q_seq]; % Adicionar configuração inicial no começo

fprintf('\nDados para o Coppelia:\n');
fprintf('Tamanho de q_seq: %d juntas x %d pontos\n', size(q_seq,1), size(q_seq,2));

% Visualização no MATLAB
figure(1);
COMAU_SmartSix.plot(deg2rad(q_seq)', ...
    'workspace', [-2 2 -2 2 -0.5 2], 'scale', 0.5, 'trail', {'r', 'LineWidth', 2});
title('Trajetória completa: Inicial - P0 - P1 - Retângulo - P0');

figure(2);
plot3(pos_traj(1,:), pos_traj(2,:), pos_traj(3,:), 'b-', 'LineWidth', 2);
hold on;
plot3(P0(1), P0(2), P0(3), 'go', 'MarkerFaceColor','g');
plot3(P1(1), P1(2), P1(3), 'mo', 'MarkerFaceColor','m');
plot3(P2(1), P2(2), P2(3), 'co', 'MarkerFaceColor','c');
plot3(P3(1), P3(2), P3(3), 'ro', 'MarkerFaceColor','r');
plot3(P4(1), P4(2), P4(3), 'yo', 'MarkerFaceColor','y');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Trajetória do efetuador: Inicial - P0 - P1 - Retângulo - P0');
grid on; axis equal;
legend('Trajetória', 'P0', 'P1', 'P2', 'P3', 'P4');

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
    err(end+1) = norm(e);
end
theta_final = theta;
end

% Função de controle de trajetória (seguimento)
function [theta_final, pos_traj, theta_traj, err] = controleTrajetoria(robot, theta_init, p_start, p_end, Rd, K, dt, tf)

% Inicialização
theta = theta_init;  % Configuração inicial das juntas
pos_traj = [];  % Histórico de posições do efetuador
theta_traj = []; % Histórico de configurações das juntas
err = [];  % Histórico da norma do erro

t = 0:dt:tf;  % Vetor de tempo discretizado

for i = 1:length(t)
    % Interpolação linear da trajetória: s varia de 0 a 1
    s = t(i)/tf;
    
    % Posição desejada interpolada entre p_start e p_end
    pd = p_start + (p_end - p_start)*s;
    
    % Velocidade desejada constante (derivada da interpolação linear)
    pd_dot = (p_end - p_start)/tf;
    
    T = robot.fkine(theta').T;     % transformação homogênea atual
    p = T(1:3,4);   % posição atual (3x1)
    R = T(1:3,1:3);  % rotação atual (3x3)
    
    % Erro de posição = diferença entre desejada e atual
    p_err = pd - p;
    
    % Erro de orientação = rotação entre Rd e R
    R_err = Rd * R';  % Matriz de erro de rotação
    axang = rotm2axang2(R_err); % Converte para eixo-ângulo
    n = axang(1:3)'; % Vetor unitário do eixo (3x1)
    phi = axang(4);    % Ângulo de rotação (escalar)
    nphi_err = n * phi;   % Vetor erro de orientação (3x1)
    
    e = [p_err; nphi_err];   % Vetor de erro completo = posição + orientação (6x1)
    
    % Lei de controle = feedforward (xdot.d) + feedback proporcional (Ke)
    % v = [velocidade linear desejada; velocidade angular desejada=0] + correção
    v = [pd_dot; zeros(3,1)] + K*e;
    
    % Jacobiana geométrica
    J = robot.jacob0(theta');
    % disp(J);
    
    % Ação de controle: inverte jacobiana para obter velocidades das juntas
    u = pinv(J) * v;  % u = Jinv.v (pseudo-inversa)
    
    % Integração numérica (Euler): atualiza configuração das juntas
    theta = theta + dt * u;
    
    % Armazena dados da iteração
    pos_traj(:,end+1) = p; 
    theta_traj(:,end+1) = theta; 
    err(end+1) = norm(e);
end

theta_final = theta;    % Retorna configuração final
end
