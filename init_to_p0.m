% codigo que leva da config inicial ate p0

clear; close all; clc;

% Columns: [theta d a alpha]
% d and a in meters, angles in radians  
dh = [...
    0       -0.450   0.150   pi/2;     % Joint 1
   -pi/2     0        0.590   pi;       % Joint 2
    pi/2     0        0.130  -pi/2;     % Joint 3
    0       -0.6471   0      -pi/2;     % Joint 4
    0        0        0       pi/2;     % Joint 5
    pi      -0.095    0       pi];      % Joint 6

L(1) = Link('d', dh(1,2), 'a', dh(1,3), 'alpha', dh(1,4), 'offset', 0);
L(2) = Link('d', dh(2,2), 'a', dh(2,3), 'alpha', dh(2,4), 'offset', dh(2,1));
L(3) = Link('d', dh(3,2), 'a', dh(3,3), 'alpha', dh(3,4), 'offset', dh(3,1));
L(4) = Link('d', dh(4,2), 'a', dh(4,3), 'alpha', dh(4,4), 'offset', dh(4,1));
L(5) = Link('d', dh(5,2), 'a', dh(5,3), 'alpha', dh(5,4), 'offset', dh(5,1));
L(6) = Link('d', dh(6,2), 'a', dh(6,3), 'alpha', dh(6,4), 'offset', dh(6,1));

COMAU_SmartSix = SerialLink(L, 'name', 'COMAU Smart Six');

T0fix = trotz(0) * troty(0) * trotx(pi);
COMAU_SmartSix.base = T0fix;

% Configuração inicial
q_inicial = [0; 0; -pi/2; 0; -pi/2; 0];

fprintf(' controle de regulação até P0 \n\n');

% Posição desejadas
% Ponto P0 em milímetros = converter para metros
P0_mm = [700; 0; 650];
pd = P0_mm / 1000; % Posição desejada em metros

% Orientação desejada
% Eixo Ze do efetuador (approach) na mesma direção do eixo Xb da base
% Eixo Xe do efetuador (normal) na direção contrária ao eixo Zb da base
Ze_desired = [1; 0; 0];  % approach na direção de Xb
Xe_desired = [0; 0; -1]; % normal na direção -Zb
Ye_desired = cross(Ze_desired, Xe_desired); 

Rd = [Xe_desired, Ye_desired, Ze_desired];
Td = [Rd, pd; 0 0 0 1];

% Parâmetros do controle de regulação
K = 0.8;      % Ganho proporcional
epsilon = 1e-4;   % Critério de parada
max_iter = 1000;  % Número máximo de iterações
dt = 0.05;  % Passo de integração

% Inicializando variáveis e vetores
theta = q_inicial;
e_ant = ones(6,1);
e = zeros(6,1);
i = 0;

control_sig = [];
err = [];
pos_traj = [];
theta_traj = [];

% Loop de controle de regulação
while (norm(e - e_ant) > epsilon && i < max_iter)
    i = i + 1;
    
    J = COMAU_SmartSix.jacob0(theta');     % Jacobiana geométrica
    
    T_atual = COMAU_SmartSix.fkine(theta');     % Cinemática direta
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
    
    if mod(i, 50) == 0
        fprintf('Iteração %d: Erro = %.6f\n', i, norm(e));
    end
end

fprintf('\nControle finalizado!\n');
fprintf('Iterações: %d\n', i);
fprintf('Erro final: %.6f m\n', norm(e));

% Preparar q_seq para CoppeliaSim
% q_seq deve ter 6 linhas (juntas) e n colunas (pontos no tempo)
% Valores em GRAUS
% q_seq está sendo salva na workspace para só rodar o script do Coppelia (n precisa mudar nada nele!)
% Reduzir a taxa de amostragem para o CoppeliaSim (a cada 5 pontos)
step_coppelia = 5;
indices = 1:step_coppelia:size(theta_traj, 2);

% Criar q_seq em GRAUS
q_seq = rad2deg(theta_traj(:, indices));

% Adicionar configuração inicial no começo
q_seq = [rad2deg(q_inicial), q_seq];

fprintf('\n Dados pro Coppelia \n');
fprintf('Tamanho de q_seq: %d juntas x %d pontos\n', size(q_seq,1), size(q_seq,2));
fprintf('Configuração inicial (graus): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', ...
        q_seq(:,1)');
fprintf('Configuração final (graus):   [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', ...
        q_seq(:,end)');

% Visualização no MATLAB
figure(1);
COMAU_SmartSix.plot(deg2rad(q_seq)', 'workspace', [-2 2 -2 2 -0.5 2], 'scale', 0.5, 'jaxes');

% Plot da trajetória 3D
figure(2);
plot3(pos_traj(1,:), pos_traj(2,:), pos_traj(3,:), 'b-', 'LineWidth', 2);
hold on;
plot3(pos_traj(1,1), pos_traj(2,1), pos_traj(3,1), 'ro', ...
      'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot3(pd(1), pd(2), pd(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
hold off;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Trajetória do Efetuador até P0');
legend('Trajetória', 'Início', 'P0');
grid on; axis equal;

% Plot do erro
figure(3);
plot(err, 'LineWidth', 2);
xlabel('Iterações'); ylabel('Norma do erro |e|');
title('Evolução do Erro');
grid on;

% Verificação da posição final
T_final = COMAU_SmartSix.fkine(deg2rad(q_seq(:,end)'));
T_final = T_final.T;
p_final = T_final(1:3, 4);

fprintf('\n verificação \n');
fprintf('Posição final alcançada (m): [%.4f, %.4f, %.4f]\n', p_final');
fprintf('Posição desejada P0 (m):     [%.4f, %.4f, %.4f]\n', pd');
fprintf('Erro de posição (mm):        [%.2f, %.2f, %.2f]\n', (p_final - pd)*1000);
