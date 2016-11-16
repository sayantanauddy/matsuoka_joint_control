% Example to call python functions from matlab

% Setup VREP and Naoqi as described in the README.md

% Setup the python module
if count(py.sys.path,'') == 0
    insert(py.sys.path,int32(0),'');
end
mod = py.importlib.import_module('nao');
py.reload(mod);

% Approach for setting and getting the joint angle from the matsuoka
% function.
% The angle received (q) should be used to set the robot angle
% The proprioceptive feedback should be obtained from the robot's
% actual angle.

% Initialize the joint (in degrees)
py.nao.setJointAngle('HeadYaw',-30.0,1.0);

% Get the sensed angle
pySensedAngle = py.nao.getJointAngle('HeadYaw');

% Convert to Matlab type
cp = cell(pySensedAngle);
matlabSensedAngle = cellfun(@double,cp);

disp(matlabSensedAngle);
