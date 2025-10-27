classdef ComauComm < handle
    
    properties (SetAccess = private)
        host
        port
        type_connection
    end
    
    % Properties of Socket
    
    properties
        socket
    end
    
    properties
        input_stream
        d_input_stream
        bytes_available
    end
    
    properties
        % retry
    end
    % Properties of TCPIP
    
    properties
        output_stream
        d_output_stream
    end
    
    properties (Access = private)
        retryIfError
        resetToolRelPose
    end
    
    properties
        tcpcomm
    end
    
    methods
        
        % This function is used to assign the parameters of the connection
        function self = ComauComm(connection, hostValue, portValue)
            % properties of connection like host, port and type of 
            % connection : socket or tcpip
            % These numbers of protocol are already defined
            % Example: ComauComm('tcpip', '172.22.121.2', 4121)
            
            self.type_connection = connection;
            self.host = hostValue;
            self.port = portValue;
            
            if(strcmp(self.type_connection,'socket'))
                self.Comm();
            elseif(strcmp(self.type_connection,'tcpip'))
                self.Comm();
            end
        end
        
        function Comm(self)
            % Connects to the server by socket or tcpip
            if(strcmp(self.type_connection,'socket'))
                self.open();  
                
            elseif(strcmp(self.type_connection,'tcpip'))
                % The communication class uses the class tcpip
                % (which depends on the instrumentation toolbox, though...)
                self.tcpcomm = tcpip(self.host, self.port);
                self.retryIfError = false;
                self.resetToolRelPose = false;
                self.open();
                self.waitResponse;
                
                % Change to initial Tool and Frame
                self.changeFrame(0);
                self.changeTool(0);
                
            end
        end
        
        function open(self)
            % Open a conection with server
            % Needs to import some packages from java.io and java.net
            if(strcmp(self.type_connection,'socket'))
                % Import the packages from java to make a constructor
                %
                % You can find java.net.Socket in:
                % http://docs.oracle.com/javase/1.4.2/docs/api/java/net/Socket.html
                %
                % and description for java.io in:
                % http://docs.oracle.com/javase/1.4.2/docs/api/java/io/package-summary.html
                import java.net.Socket
                import java.io.*
                
                % "number_of_retries" is the number of attempt and
                % "retry" is a default value that count the attemp to
                % conection
                number_of_retries = 3;
                retry = 0;
                
                % Try to establish a connection with server
                while true
                    retry = retry + 1;
                    
                    % testing whether attempts have ended
                    if ((number_of_retries > 0) && (retry > number_of_retries))
                        fprintf(1, 'Too many retries\n');
                        break;
                    end

                    try
                        fprintf(1, 'Retry %d connecting to %s:%d\n', ...
                                retry, self.host, self.port);

                        % throws if unable to connect
                        self.socket = Socket(self.host, self.port);

                        % get a buffered data input stream from the socket
                        self.input_stream   = self.socket.getInputStream;
                        self.d_input_stream = DataInputStream(self.input_stream);
                        
                        fprintf(1, 'Connected to server\n');
                        
                        % read data from the socket - wait a short time first
                        pause(0.5);

                        % Bytes that is avalliable to read from socket
                        tam = self.input_stream.available;

                        % allocate space in "message" store data from
                        % socket
                        message = zeros(1, tam);

                        % read each data from socket
                        for i = 1:tam
                            s = self.input_stream.read;
                            message(i) = (s);
                        end

                        native2unicode(message)
                        self.bytes_available = self.input_stream.available;

                        % get a buffered data output stream from the socket
                        self.output_stream   = self.socket.getOutputStream;
                        self.d_output_stream = DataOutputStream(self.output_stream);
                        break;

                    catch
                        if ~isempty(self.socket)
                            self.socket.close;
                        end

                        % pause before retrying
                        pause(1);
                    end
                end

            elseif(strcmp(self.type_connection,'tcpip'))
                % Open a new connection and create a object
                if ~ self.retryIfError
                    % teste kinect
                    fopen(self.tcpcomm);
                    disp('Connection established');
                    return
                end

                while ~strcmp(self.tcpcomm.status, 'open')
                    disp('I could not start the communication with COMAU. I will insist a little more...');
                    fopen(self.tcpcomm);
                    pause(1);
                end
            end
        end
        
        function close(self)
            % Close the conexion and delete the object
            
            if(strcmp(self.type_connection,'socket'))
                self.socket.close
                disp (['Connection ended: ' datestr(now)]);
                
            elseif(strcmp(self.type_connection,'tcpip'))

                fclose(self.tcpcomm);
                delete(self.tcpcomm);
            end
            self.delete;
        end
        
        
        function message = waitResponse(self, messageId)
            %WAITRESPONSE Get a response from the server
            %   obj.waitResponse() waits for a response from the server in the form of
            %   one or more lines where the first character is either '+' or '-'. Only the
            %   last line contains just one character: all others must also contain a space
            %   as the second character.
            %
            %   The text from the third character on composes a message that will be
            %   displayed to the user. If the prefix is '+', then the message is just
            %   printed; if the prefix is '-', then it will be thrown as an exception.
            %comau
            %   obj.waitResponse(messageId) specifies the message ID for throwing the
            %   exception; the default is 'waitResponse:error'.
            
            
            if(strcmp(self.type_connection,'socket'))
                % I have to make this function
                
            elseif(strcmp(self.type_connection,'tcpip'))
                type_message = '';
                message = '';

                if nargin < 2
                    messageId = 'waitResponse';
                end
                messageId = sprintf('%s:%s', messageId, 'responseError');

                while true
                    line = fscanf(self.tcpcomm); %Read data from buffer

                    if numel(line) >= 2 && isempty(type_message)
                        type_message = line(1);                    
                    end

                    if numel(line) >= 3                   
                        message = [message, line(3:end)]; %Store the message (it starts from the third character...)                  
                    else
                        break
                    end
                end

                if type_message == '+'
                    return
                end
                if type_message == '-'                
                    if numel(message) > 0 && strcmp(message(end), char(10)) %Verify if the last character in the message corresponds to a line feed
                        message = message(1:(end - 1));
                    end
                    MException(messageId, message).throwAsCaller();
                end

                error('waitResponse:invalidFormat', 'Invalid response from the server.');
            end
        end
        % Functions exclusive of socket
        function sendS(self, str)
            import java.io.*
            self.output_stream.write(uint8([str 10]))
        end
        
        function value = readS(self)
            % This function read data from buffer
            % n is the number of messages, b is the
            % buffer size = 500 characters and the 
            % response of the server is the variable
            % value
            
            n = self.input_stream.available;

            % Buffer size = 100 characters
            % b = zeros(1, 100);
            for i = 1:n
                %b(i) = self.input_stream.read;
            end

            if (b(1) ~= 0)
                value = (char(b));
                %value = str2num(value);
            end
            
        end
        
        % Functions to send the command to the Comau
        
        function ping(self)
            fprintf(self.tcpcomm,'ping');
            line = self.waitResponse(); %Read data from buffer
            disp(line);
        end
        
        function setTimeout(self, timeout)
            % You have to send the number of the Frame that you want
            % to change. 
            % example : 
            % In Matlab you have : self.changeFrame(i) 
            % Is equivalente in PDL $FRAME(i)
            if(strcmp(self.type_connection,'socket'))
                self.sendS('setTimeout');

            elseif(strcmp(self.type_connection,'tcpip'))      
                fprintf(self.tcpcomm,'setTimeout');
                fprintf(self.tcpcomm, sprintf('%d', timeout));
                self.waitResponse();
            end
        end

	  function abreGarra(self)
			% Seta o bit DOUT[2] = OFF;
            
            if(strcmp(self.type_connection,'socket'))
                self.tcpcomm.printf('abreGarra\n');
               % self.waitResponse();
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'abreGarra');
              %  self.waitResponse();
            end
	  end

	function fechaGarra(self)
			% Seta o bit DOUT[2] = ON;
            
            if(strcmp(self.type_connection,'socket'))
                self.tcpcomm.printf('fechaGarra\n');
             %   self.waitResponse();
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'fechaGarra');
             %   self.waitResponse();
            end
	  end

	
        
        function teste = changeFrame(self, n)
            % You have to send the number of the Frame that you want
            % to change. 
            % example : 
            % In Matlab you have : self.changeFrame(i) 
            % Is equivalente in PDL $FRAME(i)
            if(strcmp(self.type_connection,'socket'))
                self.sendS('changeFrame');

            elseif(strcmp(self.type_connection,'tcpip'))      
                fprintf(self.tcpcomm,'changeFrame');
                fprintf(self.tcpcomm, sprintf('%d', n));
                teste = self.waitResponse();
            end
        end
        
        function teste = changeTool(self, n)
            % You have to send the number of the Tool that you want
            % to change. 
            % example : 
            % In Matlab you have : self.changetool(i) 
            % Is equivalente in PDL $TOOL(i)
            if(strcmp(self.type_connection,'socket'))
                self.sendS('changeTool');

            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm,'changeTool');
                fprintf(self.tcpcomm, sprintf('%d', n));
                teste = self.waitResponse();
                % message = self.waitResponse();
            end
        end
        
        function curr = getCurrentsVector(self)
            % curr = getCurrentsVector(self) It represents the actual
            % motor current expressed in amperes
            % This is Attributes that can read-only and 
            % the data type is array of real of one dimension
            if(strcmp(self.type_connection,'socket'))
                self.sendS('getCurrentsVector');
                %curr = waitingforMessagesFromServer(self);
                curr = readS(self);
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm,'getCurrentsVector');
                curr = self.waitResponse();
                curr = sscanf(curr, '%f');
            end
        end
        
        function acel = getArmAccelerationOverride(self)
			% It represents the aceleration ovrerride percentage
			% for motions to default arm.
            % The limits of this  value its 1 to 100
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('getArmAccelerationOverride');
                acel = self.readS();

            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'getArmAccelerationOverride');
                acel = self.waitResponse();
                acel = sscanf(acel, '%d');
            end
		end
		
		function decel = getArmDecelerationOverride(self)
			% It represents the deceleration ovrerride percentage
			% for motions to default arm.
            % The limits of this  value its 1 to 100
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('getArmDecelerationOverride');
                decel = self.readS();

            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'getArmDecelerationOverride');
                decel = self.waitResponse();
                decel = sscanf(decel, '%d');
            end
        end

		function arm_over = getArmOverride(self)
			% Scale the speed, aceleration and deceleration, 
			% so that the trajetory remains constant with 
			% changes in its value. The type is a interger
            % The limits of this  value its 1 to 100 
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('getArmOverride');
                arm_over = self.readS();

            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'getArmOverride');
                arm_over = self.waitResponse();
                arm_over = sscanf(arm_over, '%d');
            end
        end
        
        function setArmAccelerationTotal(self)
			% It represents the aceleration ovrerride percentage
			% for motions to default arm
            
            if(strcmp(self.type_connection,'socket'))
                self.tcpcomm.printf('setArmAccelerationTotal\n');
                self.waitResponse();
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'setArmAccelerationTotal');
                self.waitResponse();
            end
		end
		
		function setArmDecelerationTotal(self)
			% It represents the aceleration ovrerride percentage
			% for motions to default arm
            
            if(strcmp(self.type_connection,'socket'))
                self.tcpcomm.printf('setArmDecelerationTotal\n');
                self.waitResponse();
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'setArmDecelerationTotal');
                self.waitResponse();
            end
        end
        
        function setArmAcceleration(self, arm_accel)
			% Scale the speed, aceleration and deceleration, 
			% so that the trajetory remains constant with 
			% changes in its value. The type is a interger
            % The limits of this  value its 1 to 100
            
            if(strcmp(self.type_connection,'socket'))
                self.tcpcomm.printf('setArmAcceleration');
                self.tcpcomm.printf('%d', arm_accel);
                self.readS();
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'setArmAcceleration');
                fprintf(self.tcpcomm, sprintf('%d', arm_accel));
                self.waitResponse();
            end
        end
        
        function setArmDeceleration(self, arm_deccel)
			% Scale the speed, aceleration and deceleration, 
			% so that the trajetory remains constant with 
			% changes in its value. The type is a interger
            % The limits of this  value its 1 to 100
            
            if(strcmp(self.type_connection,'socket'))
                self.tcpcomm.printf('setArmDeceleration');
                self.tcpcomm.printf('%d', arm_deccel);
                self.readS();
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'setArmDeceleration');
                fprintf(self.tcpcomm, sprintf('%d', arm_deccel));
                self.waitResponse();
            end
		end
        
        function dq = getDualPosition(self)
            % Return a vector with in dual quaternion form
            
            [T,Q] = self.getArmPose();
            p = DQ([0;T]);
            r = DQ(Q);
            dq = r + DQ.E * (1/2)*p*r;
        end

        
        function setDualPosition(self,dq)
            aux = vec4(translation(dq));
            T = aux(2:4);
            X = T(1);
            Y = T(2);
            Z = T(3);
            
            Q = vec4(P(dq));
            [A, E, R] = ComauComm.QuatToAER(Q);

            fprintf(self.tcpcomm, 'setArmPose');
            fprintf(self.tcpcomm, sprintf('%f %f %f %f %f %f',...
                X, Y, Z, A, E, R));
        end
        
        function setDualPositionFly(self,dq)
            % This function its only a fly movent in one point
            % This function receive a dual quaternion
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('flyMovement')
                
            elseif(strcmp(self.type_connection,'tcpip'))
            aux = vec4(translation(dq));
            T = aux(2:4);
            X = T(1);
            Y = T(2);
            Z = T(3);
            
            Q = vec4(P(dq));
            [A, E, R] = ComauComm.QuatToAER(Q);

            fprintf(self.tcpcomm, 'flyMovement');
            fprintf(self.tcpcomm, sprintf('%f %f %f %f %f %f',...
                X, Y, Z, A, E, R));
            self.waitResponse();
            end
        end
        
        function setArmPoseAERFly(self, vec)
            % This function its only a fly movent in one point
            % i have to explain more
            % This function receive a vector with positions in X, Y, Z and
            % orientation as Euler angles A,E,R of the COMAU arm.
            
            X = vec(1);
            Y = vec(2);
            Z = vec(3);
            A = vec(4);
            E = vec(5);
            R = vec(6);
            if(strcmp(self.type_connection,'socket'))
                self.sendS('flyMovement')
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'flyMovement');
                fprintf(self.tcpcomm, sprintf('%f %f %f %f %f %f',...
                    X, Y, Z, A, E, R));
                self.waitResponse();
            end
        end
        
        function joint = getJointPositions(self)
            %GETJOINTPOSITIONS Get the pose of joints according to Denavit Hartenberg parameters
            % joint = getJointPositions(self)
            % return the pose of joints in array
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('getJointPositions')
                joint = readS(self);
                %joint = str2num(joint);
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'getJointPositions');
                joint = self.waitResponse();
                joint = sscanf(joint, '%f');
            end

        end
         
         function setJointPositions(self, joint)
            % setJointPositions
            % message = setJointPositions(self,joint) 
            % sets the pose with array of joints
            if(strcmp(self.type_connection,'socket'))
                self.sendS('setJointPositions')
                %message = readS(self);
                str = num2str(joint);
                self.sendS(str);
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'setJointPositions');
                fprintf(self.tcpcomm, sprintf('%f %f %f %f %f %f', ...
                    joint(1), joint(2), joint(3), joint(4), joint(5), joint(6)));
                self.waitResponse();
            end
         end
         
         function setJointPositionsFly(self, joint, flag_response)
            % setJointPositions
            % message = setJointPositions(self,joint) 
            % sets the pose with array of joints
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('setJointPositionsFly')
                str = num2str(joint);
                self.sendS(str);
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'setJointPositionsFly');
                fprintf(self.tcpcomm, sprintf('%f %f %f %f %f %f', ...
                    joint(1), joint(2), joint(3), joint(4), joint(5), joint(6)));
                if flag_response
                    self.waitResponse();
                end
            end
         end
         
        function AER = getArmPoseAER(self)
            %GETARMPOSAER Get the arm pose: position coordinates and AER Euler angles.
            %  [X, Y, Z, A, E, R] = obj.getArmPoseAER() returns the current position
            %  X,Y,Z and the orientation as Euler angles A,E,R of the COMAU arm.
            %
            %  The AER rotation convention is defined in the "Comau Robotics Instruction
            %  Handbook" (CR00757604_en-00/1109), p. 3-7. Essentially, A, E and R are angles
            %  in degrees that describe a 3D rotation in the Z-Y'-Z'' Euler convention
            %  (first the rotation A around the Z axis, then the rotation E around the
            %  resulting Y axis, then the rotation R around the resulting Z axis).

            if(strcmp(self.type_connection,'socket'))
                self.sendS('getArmPose')
                AER = readS(self);
                %[X, Y, Z, A, E, R] = str2num(aux);
                %AER = str2num(aux);
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm,'getArmPose');
                AER = self.waitResponse();
                AER = sscanf(AER, '%f');
            end
        end
        
        function RotMatriz = getArmPoseRotMatrix(self)
            % getArmPoseRotMatrix Get the arm pose: position and rotation matrix.
            %  RotMatriz = obj.getArmPoseRotMatrix() returns the current position T and the
            %  orientation as a rotation matrix RM of the COMAU arm.
            
            resp = self.getArmPoseAER();
            resp = num2cell(resp);
            [X, Y, Z, A, E, R] = deal(resp{:});
            
            T = [X; Y; Z];
            RM = ComauComm.AERToRotMatrix(A, E, R);
            RotMatriz = [T, RM];
        end
        
        function [T, Q] = getArmPose(self)
            % getArmPose Get the arm pose: position and quaternion.
            %  [T, Q] = obj.getArmPose() returns the current position T and the orientation
            %  quaternion Q of the COMAU arm.
            
            resp = self.getArmPoseAER();
            resp = num2cell(resp);
            [X, Y, Z, A, E, R] = deal(resp{:});
            
            T = [X; Y; Z];
            Q = ComauComm.AERToQuat(A, E, R);
            if nargout <= 1
                T = [T; Q];
            end
        end
        
        
        function setArmPoseAER(self, vec)
            % setArmPoseAER sets the pose with
            %  position X,Y,Z and orientation given by the Euler angles
            %  A,E,R and returns the message sent by the server
            
            X = vec(1);
            Y = vec(2);
            Z = vec(3);
            A = vec(4);
            E = vec(5);
            R = vec(6);
            if(strcmp(self.type_connection,'socket'))
                self.sendS('setArmPose')
                readS(self);
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'setArmPose');
                fprintf(self.tcpcomm, sprintf('%f %f %f %f %f %f',...
                    X, Y, Z, A, E, R));
                self.waitResponse();
            end

        end
        
        function setArmPoseRotMatrix(self, T, R)
            % setArmPoseRotMatrix sets the position T and the
            %  orientation as a rotation matrix RM of the COMAU arm.
            

            self.setArmPose(T, Quat.FromRot(R));
        end
        
        function setArmPose(self, vec)
            T = [vec(1), vec(2), vec(3)];
            Q = [vec(4), vec(5), vec(6), vec(7)];
            
%             assert(ndims(T) == 2, 'ComauComm:setArmPose:T', 'Invalid size for argument T.');
%             
%             if all(size(T) == [7, 1])
%                 Q = T(4:7);
%                 T = T(1:3);
%             end
%             
%             assert(isfloat(T), 'ComauComm:setArmPose:T', 'Invalid type for argument T.');
%             assert(all(size(T) == [3, 1]), 'ComauComm:setArmPose:T', ...
%                 'Invalid size for argument T.');
%             
%             assert(ndims(Q) == 2, 'ComauComm:setArmPose:Q', 'Invalid size for argument Q.');
%             assert(isfloat(Q), 'ComauComm:setArmPose:Q', 'Invalid type for argument Q.');
%             assert(all(size(Q) == [4, 1]), 'ComauComm:setArmPose:Q', ...
%                 'Invalid size for argument Q.');
            
            [A, E, R] = self.QuatToAER(Q);
            arm_pose = [T(1), T(2), T(3), A, E, R];
            self.setArmPoseAER(arm_pose);
        end
        
        function TPose = getToolPoseAER(self)
            % getToolPoseAER Get the tool pose: position coordinates and AER Euler angles.
            %  [X, Y, Z, A, E, R] = obj.getToolRelPoseAER() returns the relative position
            %  X,Y,Z and the orientation as Euler angles A,E,R of the COMAU tool.
            %
            %  The AER rotation convention is defined in the "Comau Robotics Instruction
            %  Handbook" (CR00757604_en-00/1109), p. 3-7. Essentially, A, E and R are angles
            %  in degrees that describe a 3D rotation in the Z-Y'-Z'' Euler convention
            %  (first the rotation A about the Z axis, then the rotation E about the
            %  resulting Y axis, then the rotation R about the resulting Z axis).
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('getToolPose')
                TPose = self.readS();
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'getToolPose');
                TPose = self.waitResponse();
                TPose = sscanf(TPose, '%f');
            end

        end
        
        function RotMatriz = getToolPoseRotMatrix(self)
            % getToolPoseRotMatrix Get the relative tool pose: position and rotation matrix.
            %  RotMatriz = obj.getToolRelPoseRotMatrix() returns the relative position T and the
            %  orientation as a rotation matrix RM of the COMAU tool.
                       
            resp = self.getToolPoseAER();
            resp = num2cell(resp);
            [X, Y, Z, A, E, R] = deal(resp{:});
            
            T = [X; Y; Z];
            RM = ComauComm.AERToRotMatrix(A, E, R);
            RotMatriz = [T, RM];
        end
        
        function [T, Q] = getToolPose(self)
            % getToolPose Get the relative tool pose: position and quaternion.
            %  [T, Q] = obj.getToolRelPose() returns the relative position T and the
            %  orientation quaternion Q of the COMAU tool.
            
            resp = self.getToolPoseAER();
            resp = num2cell(resp);
            [X, Y, Z, A, E, R] = deal(resp{:});
            
            T = [X; Y; Z];
            Q = ComauComm.AERToQuat(A, E, R);
            if nargout <= 1
                T = [T; Q];
            end
        end
        
        function message = setToolPoseAER(self, vec)            
            % setToolPoseAER(self, X, Y, Z, A, E, R) sets the 
            %  pose of Comau tool with position X,Y,Z and orientation given 
            %  by the Euler angles A,E,R and returns the message sent by the server
            
            X = vec(1);
            Y = vec(2);
            Z = vec(3);
            A = vec(4);
            E = vec(5);
            R = vec(6);
            if(strcmp(self.type_connection,'socket'))
                self.sendS('setToolPose')
                str = num2str(AER);
                self.sendS(str);
                message = self.readS();
                
            elseif(strcmp(self.type_connection,'tcpip'))
	            fprintf(self.tcpcomm, 'setToolPose');
                fprintf(self.tcpcomm, sprintf('%f %f %f %f %f %f', X, Y, Z, A, E, R));
                message = self.waitResponse();  
            end
        end
        
        function setToolPoseRotMatrix(self, T, R)
            % setToolRelPoseRotMatrix(self, T, R) sets the position T and the
            % orientation as a rotation matrix RM of the COMAU tool.

            self.setToolRelPose(T, Quat.FromRot(R));
        end
        
        function setToolRelPose(self, T, Q)
            assert(ndims(T) == 2, 'ComauComm:setToolRelPose:T', ...
                'Invalid size for argument T.');
            
            if all(size(T) == [7, 1]) && (nargin == 2)
                Q = T(4:7);
                T = T(1:3);
            end
            
            assert(isfloat(T), 'ComauComm:setToolRelPose:T', 'Invalid type for argument T.');
            assert(all(size(T) == [3, 1]), 'ComauComm:setToolRelPose:T', ...
                'Invalid size for argument T.');
            
            assert(ndims(Q) == 2, 'ComauComm:setToolRelPose:Q', ...
                'Invalid size for argument Q.');
            assert(isfloat(Q), 'ComauComm:setToolRelPose:Q', 'Invalid type for argument Q.');
            assert(all(size(Q) == [4, 1]), 'ComauComm:setToolRelPose:Q', ...
                'Invalid size for argument Q.');
            
            [A, E, R] = ComauComm.QuatToAER(Q);
            self.setToolRelPoseAER(T(1), T(2), T(3), A, E, R);
        end
        
        function setToolPoseHcam2marker(self, Hcam2marker)
            %SETTOOLRELPOSHCAM2MARKER Set the relative tool position based on calibration data
            %  obj.setToolRelPoseHcam2marker(Hcam2marker) sets the relative tool position
            %  based on the results of the Christian Wengert's handeye.m function, see
            %  http://www.vision.ee.ethz.ch/software/calibration_toolbox/
            %  calibration_toolbox.php

            self.setToolRelPoseRotMatrix(Hcam2marker(1:3, 4), Hcam2marker(1:3, 1:3));
        end

        function UPoseAER = getUframePoseAER(self)
            %GETUFRAMERELPOSE Get the pose of user frame linked to 
            % the workpiece: position coordinates and AER Euler angles.
            %  [X, Y, Z, A, E, R] = getUframeRelPoseAER(self) returns the relative position
            %  X,Y,Z and the orientation as Euler angles A,E,R of the COMAU tool.
            %
            %  The AER rotation convention is defined in the "Comau Robotics Instruction
            %  Handbook" (CR00757604_en-00/1109), p. 3-7. Essentially, A, E and R are angles
            %  in degrees that describe a 3D rotation in the Z-Y'-Z'' Euler convention
            %  (first the rotation A about the Z axis, then the rotation E about the
            %  resulting Y axis, then the rotation R about the resulting Z
            %  axis).
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('getUframePose');
                UPoseAER = readS(self);
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'getUframePose');
                UPoseAER = self.waitResponse();
                UPoseAER = sscanf(UPoseAER, '%f');
            end

        end
        
        function RotMatriz = getUframePoseRotMatrix(self)
            % getUframePoseRotMatrix Get the relative pose
            % of user frame linked to the workpiece: position and rotation matrix.
            % RotMatriz = obj.getToolRelPoseRotMatrix() returns the relative position T and the
            % orientation as a rotation matrix RM of the COMAU user frame.
            % Its OK
            resp = self.getUframePoseAER();
            resp = num2cell(resp);
            [X, Y, Z, A, E, R] = deal(resp{:});
            
            T = [X; Y; Z];
            RM = ComauComm.AERToRotMatrix(A, E, R);
            RotMatriz = [T, RM];
        end
         
        function [T, Q] = getUframePose(self)
            % getUframePose Get the relative pose of user frame: position and quaternion.
            %  [T, Q] = getUframeRelPose(self) returns the relative position T and the
            %  orientation quaternion Q of the COMAU user frame.
            %
            %  M = obj.getUframeRelPose() returns M = [T; Q].
            % Its Ok
            resp = self.getUframePoseAER();
            resp = num2cell(resp);
            [X, Y, Z, A, E, R] = deal(resp{:});
            
            T = [X; Y; Z];
            Q = ComauComm.AERToQuat(A, E, R);
            if nargout <= 1
                T = [T; Q];
            end
        end
        
        function setUframePoseAER(self, vec)            
            % setUframePoseAER(self, X, Y, Z, A, E, R) sets the 
            % pose of Comau tool with position X,Y,Z and orientation given 
            % by the Euler angles A,E,R and returns the message sent by the server

            X = vec(1);
            Y = vec(2);
            Z = vec(3);
            A = vec(4);
            E = vec(5);
            R = vec(6);
            if(strcmp(self.type_connection,'socket'))
                self.sendS('setUframePose')
                str = num2str(vec);
                self.sendS(str);
                self.readS();
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'setUframePose');
                fprintf(self.tcpcomm, sprintf('%f %f %f %f %f %f', X, Y, Z, A, E, R));
                self.waitResponse();
            end
		end
        
        function setUframePoseRotMatrix(self, T, R)
            % setUframePoseRotMatrix(self, T, R) sets the position T and the
            % orientation as a rotation matrix RM of the COMAU tool.
            % I have to test
            self.setUframeRelPose(T, Quat.FromRot(R));
        end
        
        function setUframeRelPose(self, T, Q)
            assert(ndims(T) == 2, 'ComauComm:setUframePose:T', ...
                'Invalid size for argument T.');
            
            if all(size(T) == [7, 1]) && (nargin == 2)
                Q = T(4:7);
                T = T(1:3);
            end
            
            assert(isfloat(T), 'ComauComm:setUframePose:T', 'Invalid type for argument T.');
            assert(all(size(T) == [3, 1]), 'ComauComm:setUframePose:T', ...
                'Invalid size for argument T.');
            
            assert(ndims(Q) == 2, 'ComauComm:setUframePose:Q', ...
                'Invalid size for argument Q.');
            assert(isfloat(Q), 'ComauComm:setUframePose:Q', 'Invalid type for argument Q.');
            assert(all(size(Q) == [4, 1]), 'ComauComm:setUframePose:Q', ...
                'Invalid size for argument Q.');
            
            [A, E, R] = ComauComm.QuatToAER(Q);
            self.setUframeRelPoseAER(T(1), T(2), T(3), A, E, R);
        end
        
        function BPoseAER = getBasePoseAER(self)
            % getBasePoseAER Get the base pose: position coordinates and
            %  AER Euler angles.
            %  [X, Y, Z, A, E, R] = getBaseRelPoseAER(self) returns the relative position
            %  X,Y,Z and the orientation as Euler angles A,E,R of the COMAU tool.
            %
            %  The AER rotation convention is defined in the "Comau Robotics Instruction
            %  Handbook" (CR00757604_en-00/1109), p. 3-7. Essentially, A, E and R are angles
            %  in degrees that describe a 3D rotation in the Z-Y'-Z'' Euler convention
            %  (first the rotation A about the Z axis, then the rotation E about the
            %  resulting Y axis, then the rotation R about the resulting Z
            %  axis).
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('getBasePose');
                BPoseAER = readS(self);
                % BPoseAER = str2num(aux);
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'getBasePose');
                BPoseAER = self.waitResponse();
                BPoseAER = sscanf(BPoseAER, '%f');

            end
        end

         function RotMatriz = getBasePoseRotMatrix(self)
            % getBasePoseRotMatrix Get the relative tool pose: position and rotation matrix.
            %  RotMatriz = getBaseRelPoseRotMatrix(self) returns the relative position T and the
            %  orientation as a rotation matrix RM of the COMAU base.

            resp = self.getBasePoseAER();
            resp = num2cell(resp);
            [X, Y, Z, A, E, R] = deal(resp{:});
            
            T = [X; Y; Z];
            RM = ComauComm.AERToRotMatrix(A, E, R);
            RotMatriz = [T, RM];
         end
        
         function [T, Q] = getBasePose(self)
            % getBasePose Get the relative base pose: position and quaternion.
            %  [T, Q] = getBaseRelPose(self) returns the relative position T and the
            %  orientation quaternion Q of the COMAU base.
            
            resp = self.getBasePoseAER();
            resp = num2cell(resp);
            [X, Y, Z, A, E, R] = deal(resp{:});
            
            T = [X; Y; Z];
            Q = ComauComm.AERToQuat(A, E, R);
            if nargout <= 1
                T = [T; Q];
            end
         end
		
		function setBasePoseAER(self, vec)            
            % setBasePoseAER(self, X, Y, Z, A, E, R) sets the 
            % pose of Comau tool with position X,Y,Z and orientation given 
            % by the Euler angles A,E,R and returns the message sent by the
            % server
             
            X = vec(1);
            Y = vec(2);
            Z = vec(3);
            A = vec(4);
            E = vec(5);
            R = vec(6);
            if(strcmp(self.type_connection,'socket'))
                self.sendS('setBasePose')
                self.readS();
                str = num2str(vec);
                self.sendS(str);
                self.readS();
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'setBasePose');
                fprintf(self.tcpcomm, sprintf('%f %f %f %f %f %f', X, Y, Z, A, E, R));
                self.waitResponse();
            end

		end
        
        function setBasePoseRotMatrix(self, T, R)
            % setBasePoseRotMatrix(self, T, R) sets the position T and the
            % orientation as a rotation matrix RM of the COMAU tool
            
            self.setBaseRelPose(T, Quat.FromRot(R));
        end
        
        function setBaseRelPose(self, T, Q)
            % I have to test
            assert(ndims(T) == 2, 'ComauComm:setBasePose:T', ...
                'Invalid size for argument T.');
            
            if all(size(T) == [7, 1]) && (nargin == 2)
                Q = T(4:7);
                T = T(1:3);
            end
            
            assert(isfloat(T), 'ComauComm:setBasePose:T', 'Invalid type for argument T.');
            assert(all(size(T) == [3, 1]), 'ComauComm:setBasePose:T', ...
                'Invalid size for argument T.');
            
            assert(ndims(Q) == 2, 'ComauComm:setBasePose:Q', ...
                'Invalid size for argument Q.');
            assert(isfloat(Q), 'ComauComm:setBasePose:Q', 'Invalid type for argument Q.');
            assert(all(size(Q) == [4, 1]), 'ComauComm:setBasePose:Q', ...
                'Invalid size for argument Q.');
            
            [A, E, R] = ComauComm.QuatToAER(Q);
            self.setBaseRelPoseAER(T(1), T(2), T(3), A, E, R);
        end
         
         function setPoseCircMovAER(self,dest, via)
            % message = setPoseCircMov(self,dest, via) set the pose in
            % dest, where dest = [X, Y, Z, A, E, R], doing a movement
            % of pre-defined arm that joins the starting 
            % point with a circumference that passes through via, 
            % where via = [X, Y, Z, A, E, R]
            % position X,Y,Z and orientation given by the Euler angles
            % A,E,R and returns the message sent by the server
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('setPoseCircMov')
                self.readS();
                str1 = num2str(dest);
                str2 = num2str(via);
                self.sendS(str1, str2);
                self.readS();
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'setPoseCircMov');
                fprintf(self.tcpcomm, sprintf('%f %f %f %f %f %f %f %f %f %f %f %f', dest(1), dest(2), dest(3),...
                    dest(4), dest(5), dest(6), via(1), via(2), via(3), via(4), via(5), via(6)));
                self.waitResponse();
            end

         end
         
         function moveNearDestinationAER(self, dest, value)
            % moveNearDestinationAER(self, obj, value) allows the programmer to specify a 
            % destination along the tool approach vector that is within 
            % a specified distance from a position. The distance, specified as a
            % real expression, is measured in millimeters along the
            % negative tool approach vector. 
            % the destination is obj [X, Y, Z, A, E, R], and the is distance 
            % is value
            % position X,Y,Z and orientation given by the Euler angles
            % A,E,R and returns the message sent by the server
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('moveNearDestination')
                self.readS();
                str1 = num2str(obj);
                str2 = num2str(value);
                self.sendS(str1, str2);
                self.readS();
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'moveNearDestination');
                fprintf(self.tcpcomm, sprintf('%f %f %f %f %f %f', dest(1), ...
                    dest(2), dest(3), dest(4), dest(5), dest(6), value));
                self.waitResponse();
            end

         end
         
         function moveAwayDestination(self, obj, value)
            % moveAwayDestination(self, obj, value) allows the programmer to specify a 
            % destination along the tool approach vector that is within 
            % a specified distance from a position. The distance, specified as a
            % real expression, is measured in millimeters along the
            % negative tool approach vector. 
            % the destination is obj [X, Y, Z, A, E, R], and the is distance 

            if(strcmp(self.type_connection,'socket'))
                self.sendS('moveAwayDestination')
                self.readS();
                str1 = num2str(obj);
                str2 = num2str(value);
                self.sendS(str1, str2);
                self.readS();
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'moveAwayDestination');
                fprintf(self.tcpcomm, sprintf('%f %f %f %f %f %f', obj(1), ...
                    obj(2), obj(3), obj(4), obj(5), obj(6), value));
                self.waitResponse();
            end
         end
         
          function moveAway(self, value)
            % moveAway(self, value) allows to
            % set the tool away to actual distance
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('moveAwayDestination')
                self.readS();
                str = num2str(value);
                self.sendS(str);
                self.readS();
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'moveAwayDestination');
                fprintf(self.tcpcomm, sprintf('%f', value));
                self.waitResponse();
            end

          end
         
         function moveRelativeTool(self, vec)
            % moveRelativeTool(self, vec) allows the 
            % programmer to specify a destination 
            % relative to the current location of the arm. 
            % The destination is indicated by a vector expression,
            % measured in millimeters, using the Tool coordinate frame
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('moveRelativeTool')
                self.readS();
                str = num2str(vec);
                self.sendS(str);
                self.readS();

            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'moveRelativeTool');
                fprintf(self.tcpcomm, sprintf('%f %f %f', vec));
                self.waitResponse();
            end
         end
         
         function moveRelativeBase(self, vec)
            % moveRelativeBase(self, vec) allows the 
            % programmer to specify a destination 
            % relative to the current location of the arm. 
            % The destination is indicated by a vector expression,
            % measured in millimeters, using the Base coordinate frame

            if(strcmp(self.type_connection,'socket'))
                self.sendS('moveRelativeBase')
                self.readS();
                str = num2str(vec);
                self.sendS(str);
                self.readS();
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'moveRelativeBase');
                fprintf(self.tcpcomm, sprintf('%f %f %f', vec));
                self.waitResponse();
            end
         end
         
        function moveRelativeUframe(self, vec)
            % moveRelativeUframe(self, vec) allows the 
            % programmer to specify a destination 
            % relative to the current location of the arm. 
            % The destination is indicated by a vector expression,
            % measured in millimeters, using the Uframe coordinate frame
            % Its OK
            if(strcmp(self.type_connection,'socket'))
                self.sendS('moveRelativeUframe')
                self.readS();
                str = num2str(vec);
                self.sendS(str);
                self.readS();
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'moveRelativeUframe');
                fprintf(self.tcpcomm, sprintf('%f %f %f', vec));
                self.waitResponse();
            end
        end
                           
        function moveAboutTool(self, vec, angle)
            % moveAboutTool(self, vec, angle) allows the 
            % programmer to specify a destination that is reached by
            % rotating the tool an angular distance about a specified vector from 
            % the current position.The angle, a real expression, represents the
            % rotation in degrees about the vector, using the Tool coordinate frame

            if(strcmp(self.type_connection,'socket'))
                self.sendS('moveAboutTool')
                self.readS();
                str1 = num2str(vec);
                str2 = num2str(angle);
                self.sendS(str1, str2);
                self.readS();
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'moveAboutTool');
                fprintf(self.tcpcomm, sprintf('%f %f %f %f', vec(1), vec(2), vec(3), angle));
                self.waitResponse();
            end

        end
         
        function moveAboutBase(self, vec, angle)
            % moveAboutBase(self, vec, angle) allows the 
            % programmer to specify a destination that is reached by
            % rotating the tool an angular distance about a specified vector from 
            % the current position.The angle, a real expression, represents the
            % rotation in degrees about the vector, using the Base coordinate frame

            if(strcmp(self.type_connection,'socket'))
                self.sendS('moveAboutBase')
                self.readS();
                str1 = num2str(vec);
                str2 = num2str(angle);
                self.sendS(str1, str2);
                self.readS();
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'moveAboutBase');
                fprintf(self.tcpcomm, sprintf('%f %f %f %f', vec(1), vec(2), vec(3), angle));
                self.waitResponse();
            end
        end
                            
        function moveAboutUframe(self, vec, angle)
            % moveAboutUframe(self, vec, angle) allows the 
            % programmer to specify a destination that is reached by
            % rotating the tool an angular distance about a specified vector from 
            % the current position.The angle, a real expression, represents the
            % rotation in degrees about the vector, using the Uframe coordinate frame

            if(strcmp(self.type_connection,'socket'))
                self.sendS('moveAboutUframe')
                self.readS();
                str1 = num2str(vec);
                str2 = num2str(angle);
                self.sendS(str1, str2);
                self.readS();
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'moveAboutUframe');
                fprintf(self.tcpcomm, sprintf('%f %f %f %f', vec(1), vec(2), vec(3), angle));
                self.waitResponse();
            end
        end
         
        function moveJointsBy(self, incremental_joints)
            % MOVE BY allows the programmer to specify a destination as a 
            % list of REAL expressions, with each item corresponding to an 
            % incremental move for the joint of an arm. For rotational axes, 
            % the units are degrees, and for transitional, they are millimeters

            if(strcmp(self.type_connection,'socket'))
            self.sendS('moveJointsBy')
            self.readS();
            str = num2str(incremental_joints);
            self.sendS(str);
            self.readS();
            
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'moveJointsBy');
                fprintf(self.tcpcomm, sprintf('%f %f %f %f %f %f',incremental_joints));
                self.waitResponse();
            end
        end
        
        function moveForDestAER(self, dest, move)
            % moveForDestAER(self, dest, move) allows the 
            % programmer to specify a partial move along the 
            % trajectory toward a theoretical destination.
            % The orientation of the tool changes in proportion 
            % to the distance.
            
            % The destination is used position X,Y,Z and orientation given 
            % by the Euler angles A,E,R.
            
            % The distance is represented by a real 
            % expression. If the value is positive, 
            % the move is measured toward the destination.
            % If the value is negative, the move is
            % measuredin the opposite direction. The distance 
            % is measuredin millimeters.
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('moveForDest')
                self.readS();
                str1 = num2str(dest);
                str2 = num2str(move);
                self.sendS(str1, str2);
                self.readS();
            
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'moveForDest');
                fprintf(self.tcpcomm, sprintf('%f %f %f %f %f %f %f',dest(1),...
                dest(2), dest(3), dest(4), dest(5), dest(6), move));
                self.waitResponse();
            end
        end
         
        function moveJointsForDest(self, joint, move)
            % moveJointsForDest(self, dest, move) allows the 
            % programmer to specify a partial move along the 
            % trajectory toward a theoretical destination.
            % The orientation of the tool changes in proportion 
            % to the distance.
            % The destination is used in the joint space and the distance
            % of movemen is a value in millimeters
           
            if(strcmp(self.type_connection,'socket'))
                self.sendS('moveJointsForDest')
                self.readS();
                str1 = num2str(joint);
                str2 = num2str(move);
                self.sendS(str1, str2);
                self.readS();
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'moveJointsForDest');
                fprintf(self.tcpcomm, sprintf('%f %f %f %f %f %f %f',...
                    joint(1), joint(2), joint(3), joint(4), joint(5), joint(6), move));
                self.waitResponse();
            end
        end
        
        function moveFly(self, vec_a, vec_b, vec_c)
            % function message = moveFly(self, vec_a)
            if(strcmp(self.type_connection,'socket'))
                self.sendS('moveFly')
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'moveFly');
                fprintf(self.tcpcomm, sprintf('%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f',...
                    vec_a(1), vec_a(2), vec_a(3), vec_a(4), vec_a(5), vec_a(6),... 
                    vec_b(1), vec_b(2), vec_b(3), vec_b(4), vec_b(5), vec_b(6),...
                    vec_c(1), vec_c(2), vec_c(3), vec_c(4), vec_c(5), vec_c(6)));
                self.waitResponse();
            end
        end
        
        function lockDefaultArmMovement(self)
            if(strcmp(self.type_connection,'socket'))
                self.sendS('lockDefaultArmMovement')
                self.readS();
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'lockDefaultArmMovement');
                self.waitResponse();
            end
        end
        
        function LockAllArmMovement(self)
            if(strcmp(self.type_connection,'socket'))
                self.sendS('LockAllArmMovement')
                self.readS();

            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'lockAllArmMovement');
                self.waitResponse();
            end

        end
        
        function LockArmMovement(self)
            if(strcmp(self.type_connection,'socket'))
                self.sendS('LockArmMovement');
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'lockArmMovement');
                self.waitResponse();
            end
        end
        
        function unlockAllArmMovement(self)
            if(strcmp(self.type_connection,'socket'))
                self.sendS('unlockAllArmMovement')
                self.readS();
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'unlockAllArmMovement');
                self.waitResponse();
            end
        end
        
        function pauseProgram(self)
            % pauseProgram() pausing a program that is running puts that program 
            % in a paused state. Pausing a program that is in a ready 
            % state puts that program in a paused-ready state.
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('pauseProgram')
                self.readS();
            
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'pauseProgram')
                self.waitResponse();
            end
        end
        
        function pauseAllPrograms(self)
            % pauseAllPrograms() pause all the program that is running
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('pauseAllPrograms')
                self.readS();

            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'pauseAllPrograms');
                self.waitResponse();
            end
        end
        
        function unpauseProgram(self)
            % unpauseProgram() unpauses paused programs. 
            % The effect of unpausing a program depends on the
            % holdable/non-holdable program attribute. If the 
            % statement is issued from a non-holdable program, 
            % holdable programs are placed in the ready state and
            % non-holdable programs are placed in the running state.
            % If the statement is issued from a holdable program, 
            % the programs are placed in the running state
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('unpauseProgram')
                self.readS();

            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'unpauseProgram');
                self.waitResponse();
            end
        end
        
        function unpauseAllProgram(self)
            % unpauseAllProgram() unpause all the program that is running
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('unpauseAllProgram')
                self.readS();

            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'unpauseAllProgram');
                self.waitResponse();
            end
        end
        
        function activatesProgram(self)
            % activatesProgram() actives a programa and depends 
            % on the holdable/non-holdable attribute of the program
            % issuing the statement and the program being activated
            
            if(strcmp(self.type_connection,'socket'))
            self.sendS('activatesProgram')
            self.readS();

            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'activatesProgram');
                self.waitResponse();
            end

        end
        
        function activatesAllProgram(self)
            % activatesAllProgram() allows active all programs that are
            % deactives
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('activatesAllProgram')
                self.readS();

            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'activatesAllProgram');
                self.waitResponse();
            end
        end
        
        function deactivatesProgram(self)
            % deactivatesProgram() deactivates programs that are in running, ready, paused,
            % or paused-ready states. Deactivated programs remain loaded, but do not continue to
            % cycle and cannot be resumed
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('deactivatesProgram')
                self.readS();

            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'deactivatesProgram');
                self.waitResponse();
            end
        end
        
        function deactivatesAllProgram(self)
            % deactivatesAllProgram()activates all the programs that are in
            % running, ready, paused, or paused-ready states.
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('deactivatesAllProgram')
                self.readS();

            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'deactivatesAllProgram');
                self.waitResponse();
            end
        end
		
		function getApplicationIdentifier(self)
			% Contain the name of an application
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('getApplicationIdentifier');
                self.readS();

            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'getApplicationIdentifier');
                self.waitResponse();
            end

		end
		
		function identifier = getApplicationIdentifiers(self)
            % Contain Applications

            if(strcmp(self.type_connection,'socket'))
                self.sendS('getApplicationIdentifiers');
                identifier = self.readS();

            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'getApplicationIdentifiers');
                resp = self.waitResponse();
                resp = num2cell(sscanf(resp, '%s'));
                identifier = deal(resp{:});
            end

		end
		
		function options = getApplicationOptions(self)
            % Contain options
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('getApplicationOptions');
                options = self.readS();

            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'getApplicationOptions');
                resp = self.waitResponse();
                resp = num2cell(sscanf(resp, '%s'));
                options = deal(resp{:});
            end

        end
        
		function setRobotArmData(self, X, Y, Z, A, E, R)            
			% setRobotArmData(self, X, Y, Z, A, E, R) sets the 
			% arm-related data of each arm. In this case, its arm1 and arm2
			% by the Euler angles A,E,R and returns the message sent by the server
            if(strcmp(self.type_connection,'socket'))
                self.tcpcomm.printf('setRobotArmData\n');
                self.tcpcomm.printf('%f %f %f %f %f %f\n', X, Y, Z, A, E, R);
                self.waitResponse();
                
            elseif(strcmp(self.type_connection,'tcpip'))
                self.tcpcomm.printf('setRobotArmData');
                self.tcpcomm.printf('%f %f %f %f %f %f\n', X, Y, Z, A, E, R);
                self.waitResponse();
            end
		end
		
		function bool = getArmDisableFlags(self)
			% Return a boolean that represents the 
			% current state of the arm and wheter it is
			% enable for motions and with the DRIVE ON, or its disable
            
            if(strcmp(self.type_connection,'socket'))
                fprintf(self.tcpcomm, 'getArmDisableFlags');
                resp = self.waitResponse();
                resp = num2cell(sscanf(resp, '%b'));
                bool = deal(resp{:});
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'getArmDisableFlags');
                resp = self.waitResponse();
                resp = num2cell(sscanf(resp, '%b'));
                bool = deal(resp{:});
            end
        end
		
		function bool = getEnableDisableArmCoupling(self)
			% Return a boolean and its used in configurations in which
			% an arm is mechanically linked to another
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('getEnableDisableArmCoupling');
                bool = self.readS();
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'getEnableDisableArmCoupling');
                resp = self.waitResponse();
                resp = num2cell(sscanf(resp, '%b'));
                bool = deal(resp{:});
            end
        end
		
		function setArmOverride(self, arm_over)
			% Scale the speed, aceleration and deceleration, 
			% so that the trajetory remains constant with 
			% changes in its value. The type is a interger
            % The limits of this  value its 1 to 100
            
            if(strcmp(self.type_connection,'socket'))
                self.tcpcomm.printf('setArmOverride\n');
                self.tcpcomm.printf('%f\n', arm_over);
                self.readS();
                
            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'setArmOverride');
                fprintf(self.tcpcomm, sprintf('%f', arm_over));
                self.waitResponse();
            end
		end
		
		function statusArm = getArmSimulateFlag(self)
			% Return a boolean and its represents the current 
			% state of the arm and wheter it is in simulate mode 
			% or not. Useful for testing the timing and flow of a 
			% program without causing any motion on the arm
			% Whem TRUE means the arm is in the simulated state
            
            if(strcmp(self.type_connection,'socket'))
                self.sendS('getArmSimulateFlag');
                statusArm = self.readS();

            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'getArmSimulateFlag');
                statusArm = self.waitResponse();
                statusArm = sscanf(statusArm, '%b');
            end
        end
		
		function curr_state = getCurrentArmSpace(self)
		    % It represents the current arm space. It is only 
			% used in cartesian motions for indicating the 
			% average TCP space expressed in millimeters.
			
            if(strcmp(self.type_connection,'socket'))
                self.sendS('getCurrentArmSpace');
                curr_state = self.readS();

            elseif(strcmp(self.type_connection,'tcpip'))
                fprintf(self.tcpcomm, 'getCurrentArmSpace');
                resp = self.waitResponse();
                resp = num2cell(sscanf(resp, '%f'));
                curr_state = deal(resp{:});
            end
        end
		
		function setConfigurationCare(self, conf_flag)
            % Possible to execute the movement even if the flags do not correspond
            % predefined variable to FALSE to make the flag of the final point
            % match that of the starting point.

            % Here the meaniing of each flag:
            %  S indicates that the WCP (Wrist Center Point) is in the zone lying behind the plane
            % defined by the first and the second axes.
            % Motion Control
            %  E indicates that the WCP is in the zonelying behind the extension of the second
            % axis;
            %  W indicates that the value of the fifth axis is negative.
            %  A indicates that the TCP (Tool Center Point) is in the zone lying behind the
            % extension of the second axis;
            %  B indicates that the TCP (Tool Center Point) is in the zone lying behind the plane
            % defined by the first and the second axes.
            
            if(strcmp(self.type_connection,'socket'))
                self.tcpcomm.printf('setConfigurationCare\n');
                self.tcpcomm.printf('%f\n', conf_flag);
                self.readS();
                
            elseif(strcmp(self.type_connection,'tcpip'))
                self.tcpcomm.printf('setConfigurationCare\n');
                self.tcpcomm.printf('%f\n', conf_flag);
                self.waitResponse();
            end
        end
    end
        
    
    % end of programs that I have to test
    
    methods (Static = true)
        function RM = AERToRotMatrix(A, E, R)
            cA = cosd(A); sA = sind(A);
            cE = cosd(E); sE = sind(E);
            cR = cosd(R); sR = sind(R);

            RA = [cA, -sA, 0; sA, cA, 0; 0, 0, 1];
            RE = [cE, 0, sE; 0, 1, 0; -sE, 0, cE];
            RR = [cR, -sR, 0; sR, cR, 0; 0, 0, 1];

            % If the rotations were about fixed axes, then the multiplication order would
            % be reversed. Since they are about relative axes, then the multiplication
            % order must be swapped in respect to fixed representation, so they are
            % straightforward.
            RM = RA * RE * RR;
        end

        function Q = AERToQuat(A, E, R)
            Q = [ ...
                cosd(A/2) * cosd(E/2) * cosd(R/2) - sind(A/2) * cosd(E/2) * sind(R/2);
                cosd(A/2) * sind(E/2) * sind(R/2) - sind(A/2) * sind(E/2) * cosd(R/2);
                cosd(A/2) * sind(E/2) * cosd(R/2) + sind(A/2) * sind(E/2) * sind(R/2);
                cosd(A/2) * cosd(E/2) * sind(R/2) + sind(A/2) * cosd(E/2) * cosd(R/2)];
        end
        
        % Leonardo - begin test 1
        
        function Q = AERToQuatLeo(A, E, R)
            Q = [ ...
                cos(A/2)*cosd(E/2)*cosd(R/2)+sind(A/2)*sind(E/2)*sind(R/2)
                sind(A/2)*cosd(E/2)*cosd(R/2)-cosd(A/2)*sind(E/2)*sind(R/2)
                cosd(A/2)*sind(E/2)*cosd(R/2)+sind(A/2)*cosd(E/2)*sind(R/2)
                cosd(A/2)*cosd(E/2)*sind(R/2)-sind(A/2)*sind(E/2)*cosd(R/2)];
        end
        
        function Q = AERToQuatLeo1(A, E, R)
            Q = [ ...
                cos(R/2)*cosd(E/2)*cosd(A/2)+sind(R/2)*sind(E/2)*sind(A/2)
                sind(R/2)*cosd(E/2)*cosd(A/2)-cosd(R/2)*sind(E/2)*sind(A/2)
                cosd(R/2)*sind(E/2)*cosd(A/2)+sind(R/2)*cosd(E/2)*sind(A/2)
                cosd(R/2)*cosd(E/2)*sind(A/2)-sind(R/2)*sind(E/2)*cosd(A/2)];
        end
        
        function Q = AERToQuatTestLeo2(A, E, R)
            cA = cosd(A/2); sA = sind(A/2);
            cE = cosd(E/2); sE = sind(E/2);
            cR = cosd(R/2); sR = sind(R/2);
            
            Q = [cR+DQ.i*sA cE+DQ.j*sE cA+DQ.k*sR];
        end
        
        function Q = AERToQuatTestLeo3(A, E, R)
            % test that I saw in wikipedia
            q1 = -cosd((A-R)/2)*sind(E/2);
            q2 = -sind((A-R)/2)*sind(E/2);
            q3 = -sind((A+R)/2)*cosd(E/2);
            q4 = sind((A+R)/2)*cosd(E/2);
            Q = [q1 q2 q3 q4];
        end

        % end  of test 1
        
        function [A, E, R] = QuatToAER(Q)
            RM = Quat.ToRot(Q);
            A = atan2(RM(2, 3), RM(1, 3)) * 180 / pi;
            E = atan2(cosd(A) * RM(1, 3) + sind(A) * RM(2, 3), RM(3, 3)) * 180 / pi;
            R = atan2(-sind(A) * RM(1, 1) + cosd(A) * RM(2, 1), ...
                -sind(A) * RM(1, 2) + cosd(A) * RM(2, 2)) * 180 / pi;
        end
        
        
        % Begin of teste 2
        
        function [A, E, R] = QuatToAERLeo1(Q)
            
            A = atan2(2*( Q(1)*Q(2)+Q(3)*Q(4) ), ( 1-2*(Q(2)*Q(2)+Q(3)*Q(3))));
            E = asin(2*(Q(1)*Q(3)-Q(4)*Q(2)));
            R = atan2(2*( Q(1)*Q(4)+Q(2)*Q(3) ), ( 1-2*( Q(3)*Q(3)+Q(4)*Q(4) )));
        end
        
        % End of test 2

        function TestQuatToAER(NTests)
            if nargin < 1, NTests = 1000; end

            for n = 1:NTests
                a = rand * 360 - 180;
                e = rand * 180;
                r = rand*360-180;
                [al, el, rl] = ComauComm.QuatToAER(ComauComm.AERToQuat(a, e, r));
                d = [al, el, rl] - [a, e, r];
                if any(abs(d) > 0.001)
                    fprintf('*** Error detected!\n');
                    fprintf('    Input values were:  A=%.4f, E=%.4f, R=%.4f\n', a, e, r);
                    fprintf('    Output values were: A=%.4f, E=%.4f, R=%.4f\n', al, el, rl);
                    fprintf('    Differences are:    A=%.4f, E=%.4f, R=%.4f\n', ...
                        al - a, el - e, rl - r);
                    return
                end
            end

            fprintf('Tests OK!\n');
        end
    end
end