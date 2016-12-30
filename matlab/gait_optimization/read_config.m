function [ x_doc ] = read_config()
%Reads the config file and returns the DOM object
    
    xml_file = 'config/nao_parameters.xml';
    x_doc = xmlread(xml_file);

end

