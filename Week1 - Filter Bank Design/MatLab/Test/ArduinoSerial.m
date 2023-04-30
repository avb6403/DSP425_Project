classdef ArduinoSerial < handle
    % ArduinoSerial Class to interface to the Arduino Microprocessor
    %   through the serial port
    %
    %   Input Arguments
    %
    %   ComPort -- Com port of the serial interface to the Arduino -- 3
    %   BaudRate -- Baudrate of the Comm port Default 9600
    %   ReadyString -- The string expected from the Arduino to indicate
    %       that it is up and running -- Default '%Arduino Ready'
    %   PortTimeOut -- Time out on the serial port (seconds) -- Default 1
    %       second
    %
    %
    
    
    properties
        comPort = [];
        baudRate = [];
        
        numReadSamples = [];
        
        readStr = [];
        writeStr = [];
        
        readyString = [];
        
        arduinoPort = [];
        portTimeOut = [];
        
    end
    
    methods
        function obj = ArduinoSerial(varargin)
            % ArduinoSerial -- Construct an instance of this class
            %
            p = inputParser;
            defaultComPort = 3;
            defaultBaudRate = 9600;
%             defaultNumReadSamples = 2400;
            defaultReadyString = '%Arduino Ready';
            defaultPortTimeOut = 1;  %  1 second port time out
            
            
            p.addParameter('ComPort', defaultComPort, @isnumeric);
            p.addParameter('BaudRate', defaultBaudRate, @isnumeric);
%             p.addParameter('NumReadSamples', defaultNumReadSamples, @isnumeric);
            p.addParameter('ReadyString', defaultReadyString, @ischar);
            p.addParameter('PortTimeOut', defaultPortTimeOut, @isnumeric);
            
            p.parse(varargin{:});
            inputArgs = p.Results;
            
            obj.comPort = inputArgs.ComPort;
            obj.baudRate = inputArgs.BaudRate;
%             obj.numReadSamples = inputArgs.NumReadSamples;
            obj.readyString = inputArgs.ReadyString;
            obj.portTimeOut = inputArgs.PortTimeOut;
            
        end
        
        %  method OpenSerial
        function returnValue = OpenSerial(obj)
            %
            %
            
            %  Close all existing COM ports
            if ~isempty(instrfind)
                fclose(instrfind);
            end
            
            obj.arduinoPort = serial(sprintf('com%1d',obj.comPort),...
                'baudrate', obj.baudRate);
            fopen(obj.arduinoPort);
            
            returnValue = true;
            %  If the port didn't open correctly returnValue will be -1
            
        end
        
        %  method ClosePort
        function returnValue = ClosePort(obj)
            %  CloseSerial
            %
            %  Close the arduino serial port
            fclose( obj.arduinoPort );
        end % ClosePort
        
        
        
        %  method WaitForReady
        function readyFlag = WaitForReady(obj)
            %  WaitForReady
            %
            %  Arduino will look for a g character to start.  Read the port and see if
            %  the correct string has been sent.
            
            readyFlag = false;
            while ~readyFlag
                
                %  Idle while waiting for data in the buffer
                while ~obj.arduinoPort.BytesAvailable
                end
                rxString = obj.ReadArduino;
                
                %  Echo the string to the command window anyway, but don't
                %  capture the data in an array just yet
                fprintf('%s\n',rxString)
                
                %  Check if it is the correct ready string
                if regexp( rxString, obj.readyString )
                    readyFlag = true;
                end
                
            end
        end
        
        %  method WriteString
        function WriteString( obj, strValue )
            %  WriteString
            %
            %  Sends the string argument over the arduino serial port
            fprintf(obj.arduinoPort, '%s\n',strValue);
            
        end %  WriteString
        
        %  method WriteNumeric
        function WriteNumeric( obj, numValue )
            %  WriteNumeric
            %
            %  WriteNumeric first convers numeric argument to a string then
            %  calls the WriteString method to send the string to the
            %  Arduino serial port
            
            %  Convert the numeric value to a string then send it as a
            %  string
            
            numericalString = num2str( numValue );
            obj.WriteString( numericalString );
            
            
        end  %  Write Numeric
        
        
        %%  method ReadArduino
        function readString = ReadArduino(obj)
            %  ReadArduino
            %
            %  ReadArdino reads a string value from the Arduino serial port
            %  object
            readString = fgetl( obj.arduinoPort );
            
        end
        
        %% method WaitForData
        function dataAvailable = WaitForData( obj )
            %  WaitForData
            %
            %  This method checks whether data is available on the port.
            %  If data is not available before the timeout period the
            %  method returns false.  If there is data in the buffer then
            %  the method returns true
            
            %  Start the timer
            dataAvailable = true;
            tic
            while ~obj.arduinoPort.BytesAvailable
                if toc > obj.portTimeOut
                    dataAvailable = false;
                    break
                end
            end
        end
        
    end
    
end




