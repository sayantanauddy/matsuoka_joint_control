function [ x_doc ] = read_config()
%Reads the config file and returns the DOM object
    
    global xml_file;
    x_doc = xmlread(xml_file);

end

