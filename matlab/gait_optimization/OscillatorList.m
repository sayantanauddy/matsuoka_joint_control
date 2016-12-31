classdef OscillatorList
    %OSCILLATORLIST Class to store a list of oscillators and also to
    %provide some utility functions which operate on the list
    %   Detailed explanation goes here
    
    properties
        oscillator_list; % List of oscillators
    end
    
    methods (Static)
        % Constructor
        function obj=OscillatorList(oscillator_list)
            obj.oscillator_list = oscillator_list;
        end
        
        % Function to retrieve an Oscillator object by name from a list
        function ret_oscillator=find_oscillator(oscillator_list, name)
            ret_oscillator = [];
            for index=1:length(oscillator_list)
                if oscillator_list(index).name == name
                    ret_oscillator = oscillator_list(index);
                end
            end
        end
        
        % Function to retrieve the coupled Oscillator objects of a given
        % oscillator by name
        function [coupled_oscillators]=find_couplings(oscillator_list, name)
            coupled_oscillators = [];
            
            % First locate the oscillator
            oscillator = OscillatorList.find_oscillator(oscillator_list, name);
            
            % Then identify all the coupled oscillator names
            
            for index=1:length(oscillator.couplings)
                new_oscillator = OscillatorList.find_oscillator(oscillator_list, oscillator.couplings(index))];
                coupled_oscillators = [coupled_oscillators new_oscillator];                    
            end
        end
        
    end
    
end

