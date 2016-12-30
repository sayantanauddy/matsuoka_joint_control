function [return_angle] = validate_angle( joint_name, joint_angle )
%Validates the value of a joint angle
%   Checks if the value of an angle is withing the specified range. 
%   If yes, the input argument is returned
%   If not, then the lower or upper limit is returned based on which limit 
%   is exceeded.
    
    % Read the XML config
    x_doc = read_config();
    
    % Get the Joints tag
    joints = x_doc.getElementsByTagName('Joint');
    
    % Get the correct joint and the required attributes
    for k = 0:joints.getLength-1
        joint = joints.item(k);
        if strcmp(char(joint.getElementsByTagName('Name').item(0).getFirstChild.getData),joint_name)
           minval = str2double(joint.getElementsByTagName('MinVal').item(0).getFirstChild.getData);
           maxval = str2double(joint.getElementsByTagName('MaxVal').item(0).getFirstChild.getData);
        end 
    end
    
    % Check if the angle is within the limits
    if joint_angle>minval &  joint_angle<maxval
        return_angle = joint_angle;
    elseif joint_angle<minval
        return_angle = minval;
    elseif joint_angle>maxval
        return_angle = maxval;
    else
        return_angle = [];
    end

end

