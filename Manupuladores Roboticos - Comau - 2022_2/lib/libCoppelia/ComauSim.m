classdef ComauSim
    % Esta classe gera uma interface com o simulador do comau no
    % coppeliasim
    
    properties
        %% interface com o simulador
        client;     % objeto cliente da comunicao
        sim;        % objeto da regular api do coppelia
        h_smartsix; % objeto do modelo do comau
        h_joints;   % handler das juntas do comau
        h_f_base;   % handler do frame da base
        h_f_tcp;    % handler do frame tcp

        %% parametros do robo
        n_joints = 6;
    end
    
    %% ==== COMMON METHODS ====
    methods

        %% Metodo construtor
        function obj = ComauSim()

            % cria o objeto de comunicacao com o CoppeliaSim
            obj.client = RemoteAPIClient();

            % recebe o objeto de simulacao
            obj.sim = obj.client.getObject('sim');

            % When simulation is not running, ZMQ message handling could be a bit
            % slow, since the idle loop runs at 8 Hz by default. So let's make
            % sure that the idle loop runs at full speed for this program:
            defaultIdleFps = obj.sim.getInt32Param(obj.sim.intparam_idle_fps);
            obj.sim.setInt32Param(obj.sim.intparam_idle_fps, 0);

            %% obtencao dos handlers

            % handler do modelo do comau
            obj.h_smartsix = obj.sim.getObject('/Smartsix');

            % handler das juntas
            obj.h_joints = zeros(1,obj.n_joints);
            for i = 1:obj.n_joints
                obj.h_joints(i) = obj.sim.getObject(strcat('/Smartsix/junta',int2str(i),''));
            end

            % handler do frame base
            obj.h_f_base = obj.sim.getObject('/Smartsix/frame_base');

            % handler do frame tcp
            obj.h_f_tcp = obj.sim.getObject('/Smartsix/frame_tcp');

            
        end

        %% Inicia a simulacao
        function startSimulation(obj)
            obj.sim.startSimulation();
        end

        %% Finaliza a simulacao
        function stopSimulation(obj)
            % finaliza a simulacao
            obj.sim.stopSimulation();
        end

        %% Recebe o tempo de simulacao
        function ret = getSimulationTime(obj)
            ret = obj.sim.getSimulationTime();
        end

        %% Envia um target position para as juntas do Comau
        function setJointTargetPosition(obj, qd)
            for i = 1:obj.n_joints
                obj.sim.setJointTargetPosition(obj.h_joints(i), qd(i));
            end

        end

        %% Define o stepping de comunicacao com o simulador
        function setStepping(obj, flag_in)
            obj.client.setStepping(flag_in);
        end

    end % -- fim dos metodos comuns
        
%     %% ==== STATIC METHODS ====
%     methods (Static)
% 
% 
%     end % fim dos metodos staticos

    

end

















    

