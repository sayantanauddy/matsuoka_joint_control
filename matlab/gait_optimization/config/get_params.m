function [ minval, maxval ] = get_params( param_name )
%Reads the config XML and returns the parameter asked for 
%   example: get_params('TLHipPitch')
%   returns: [minval, maxval]

    % Read the XML config
    x_doc = read_config();
    
    % Get the Param tag
    params = x_doc.getElementsByTagName('Param');
    
    minval = [];
    maxval = [];
    
    % Get the correct param and the required attributes
    for k = 0:params.getLength-1
        param = params.item(k);
        if strcmp(char(param.getElementsByTagName('Name').item(0).getFirstChild.getData),param_name)
           minval = str2double(param.getElementsByTagName('MinVal').item(0).getFirstChild.getData);
           maxval = str2double(param.getElementsByTagName('MaxVal').item(0).getFirstChild.getData);
        end 
    end
    
end

