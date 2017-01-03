function log_write( log_str )
%LOG_WRITE Writes the input string in the log file
%   Adds the current timestamp infront of every line

    global log_file;
    
    fileID = fopen(log_file,'a+');
    fprintf(fileID,'%s\n',['[',datestr(now,'yyyy-mm-dd HH:MM:SS:FFF'),'] ',log_str]);
    fclose(fileID);

end

