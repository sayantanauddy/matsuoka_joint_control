% Modified by Sayantan Auddy from the original file provided by Coppelia 
% Robotics
%
% This script does the following:
% 1. Connects to V-REP
% 2. Retrieves names of all objects in the scene
% 3. Retreives absolute position of the NAO robot in the scene
% 4. Resets the NAO by first removing the model and then adding it again
%
%
% Make sure to have the server side running in V-REP: 
% in a child script of a V-REP scene, add following command
% to be executed just once, at simulation start:
%
% simExtRemoteApiStart(19999)
%
% then start simulation, and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!

function simpleTest()
    disp('Program started');
    % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1); % just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');
            
        % Now try to retrieve data in a blocking fashion (i.e. a service call):
        [res,objs]=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking);
        if (res==vrep.simx_return_ok)
            fprintf('Number of objects in the scene: %d\n',length(objs));
        else
            fprintf('Remote API function call returned with error code: %d\n',res);
        end
        
        % Try to retrieve all the object names
        [retcode, handles, intData, floatData, stringData]=vrep.simxGetObjectGroupData(clientID,vrep.sim_appobj_object_type,0,vrep.simx_opmode_blocking);
        if (retcode==vrep.simx_return_ok)
            for index = 1:length(stringData)
                fprintf('Name: %s\n',char(stringData(index)));
            end
        else
            fprintf('Remote API function call returned with error code: %d\n',res);
        end
        
        % Try to retrieve the position of the object named "NAO"
        % Refer to the API for details about parameters and return values
        % API: http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsMatlab.htm
        % First retrieve the object handle
        % The object name 'NAO' is case sensitive
        [retcode, handle]=vrep.simxGetObjectHandle(clientID,'NAO',vrep.simx_opmode_blocking);
        if (retcode==vrep.simx_return_ok)
            % Try to retrieve the object position by using the handle
            % The parameter relativeToObjectHandle=-1 is used to get the
            % absolute position. Use vrep.sim_handle_parent to retrieve the 
            % position relative to the object's parent, or an object handle
            % relative to whose reference frame you want the position
            % It is possible to have non-blocking calls also. Check the API
            [retcode, position]=vrep.simxGetObjectPosition(clientID,handle,-1,vrep.simx_opmode_blocking);
            if (retcode==vrep.simx_return_ok)
                fprintf('The absolute position of Nao [x y z]: [%s]\n',num2str(position));
            else
                fprintf('Remote API function call returned with error code: %d\n',res);
            end
        else
            fprintf('Remote API function call returned with error code: %d\n',res);
        end
        
        %Pause for 5 seconds so that we can switch to VREP and see the
        %model being removed and then added again
        pause(5);
    
        % Try to reset the position of the robot
        % For doing this first we will remove the model from the scene
        [retcode]=vrep.simxRemoveModel(clientID, handle, vrep.simx_opmode_blocking);
        if (retcode==vrep.simx_return_ok)
            fprintf('Model removed successfully\n');
        else
            fprintf('Remote API function call returned with error code: %d\n',retcode);
        end
        % Now try to load the model
        % Use the file robot_walking/model/nao.ttm
        [retcode, handle]=vrep.simxLoadModel(clientID,'/home/sayantan/Knowledge/Independant Studies/Robot Walking/code/robot_walking/model/nao.ttm', 1, vrep.simx_opmode_blocking);
        if (retcode==vrep.simx_return_ok)
            fprintf('Model loaded successfully\n');
        else
            fprintf('Remote API function call returned with error code: %d\n',retcode);
        end
        
        % Now retrieve streaming data (i.e. in a non-blocking fashion):
        t=clock;
        startTime=t(6);
        currentTime=t(6);
        vrep.simxGetIntegerParameter(clientID,vrep.sim_intparam_mouse_x,vrep.simx_opmode_streaming); % Initialize streaming
%         while (currentTime-startTime < 5)   
%             [returnCode,data]=vrep.simxGetIntegerParameter(clientID,vrep.sim_intparam_mouse_x,vrep.simx_opmode_buffer); % Try to retrieve the streamed data
%             if (returnCode==vrep.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
%                 fprintf('Mouse position x: %d\n',data); % Mouse position x is actualized when the cursor is over V-REP's window
%             end
%             t=clock;
%             currentTime=t(6);
%         end
%             
        % Now send some data to V-REP in a non-blocking fashion:
        vrep.simxAddStatusbarMessage(clientID,'Hello again V-REP!',vrep.simx_opmode_oneshot);

        % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        vrep.simxGetPingTime(clientID);

        % Now close the connection to V-REP:    
        vrep.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    vrep.delete(); % call the destructor!
    
    disp('Program ended');
end
