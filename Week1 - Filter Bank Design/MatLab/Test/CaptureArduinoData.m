function outData = CaptureArduinoData(varargin)
%% CaptureArduinoData
%
%  Function to interface to the serial port to capture data from the
%  Arduino.
%
%  Input Arguments
%
%  Input arguments are in Name/Value pairs
%
% 'ComPort' -- Serial Comm port to communicate with the Arduino
%           Default 3
%
% 'BaudRate' -- Baud rate of the Comm Port  Default 9600 bps
%
% 'ReadyString' -- Char array of the string from the Arduino letting MATLAB
%           know that it is ready -- Object waits for that ready string
%           before sending back the response string to tell the Arduino
%           to being execution -- Default '%Arduino Ready'
%
% 'PortTimeOut' -- Value in seconds that MATLAB will wait for the serial
%           port for a character before timing out -- Default 1 sec
%
%
% 'EchoData' -- Logical value telling MATLAB to echo each string sent from
%           the Arduino to the command window -- Default -- true
%
% 'NumActivePlots' -- The number of active plot objects to be created.  Each
%           plot will plot a column of data versus the first column of data
%           Default -- [] No plots are created
%
% 'DataFile' -- Name of the data file to use to send to the Arduino over
%           the serial port.  If a file is specified then it is assumed
%           that data will be sent to the Arduino, nothing else is required
%
%
%  Output Arguments
%
%  outData -- A matrix of the numerical values sent from the Arduino to the
%           Serial Port
%
%  Written by Mark Thompson 8/2020
%  ECTET Department
%  College of Engineering Technology
%  Rochester Institute of Technology
%
%  Revisions A
%
%  9/8/2020 Modified code to improve receiving a NaN in the data
%



p = inputParser;
defaultComPort = 3;
defaultBaudRate = 9600;
defaultNumReadSamples = [];
defaultReadyString = '%Arduino Ready';
defaultPortTimeOut = 1;  %  1 second port time out
defaultEchoData = true;  %  Default to echo the RX String
defaultActivePlot = [];  %  Default active plotting to false
defaultGraphDelay = 1;  %  Default delay between updates to the graphs
defaultDataFile = '';  %  Default file name


p.addParameter('ComPort', defaultComPort, @isnumeric);
p.addParameter('BaudRate', defaultBaudRate, @isnumeric);
p.addParameter('NumReadSamples', defaultNumReadSamples, @isnumeric);
p.addParameter('ReadyString', defaultReadyString, @ischar);
p.addParameter('PortTimeOut', defaultPortTimeOut, @isnumeric);
p.addParameter('EchoData', defaultEchoData, @islogical);
p.addParameter('NumActivePlots', defaultActivePlot, @isnumeric);
p.addParameter('GraphDelay', defaultGraphDelay, @isnumeric);
p.addParameter('DataFile', defaultDataFile, @ischar);


p.parse(varargin{:});
inputArgs = p.Results;

comPort = inputArgs.ComPort;
baudRate = inputArgs.BaudRate;
numReadSamples = inputArgs.NumReadSamples;
readyString = inputArgs.ReadyString;
portTimeOut = inputArgs.PortTimeOut;
echoData = inputArgs.EchoData;
nActivePlots = inputArgs.NumActivePlots;
graphDelay = inputArgs.GraphDelay;
sendDataFileName = inputArgs.DataFile;


TAB = char(9);

%  If the active plot is enabled, instantiate and object and create the
%  figure

if ~isempty( nActivePlots )
    %
    %  Create the number of active plots and place in an array of handles
    %
    %  Create the figures
    %
    
    for iPlot = 1:nActivePlots
        hPlot(iPlot) = ActivePlot('GraphTitle',sprintf('Graph %2d',iPlot) );
        hPlot(iPlot).CreateFigure;
        hPlot(iPlot).xLabelStr = 'Sample';
    end
end

%  Instantiate the Arduino Serial Port Object
hArd = ArduinoSerial('ComPort', comPort,...
    'BaudRate',baudRate,...
    'ReadyString', readyString,...
    'PortTimeOut', portTimeOut);

%  Open the serial port to the Arduino
hArd.OpenSerial;

%  Arduino will look for a g character to start.  Read the port and see if
%  the correct string has been sent.
hArd.WaitForReady;

%  Send the 'g' character to start reading data
hArd.WriteString('g');


%  Read data from the serial portr

dataAvailable = true;   %  Set data available flag to true
contReadSamples = true;  %  Set to continue to read samples
firstLine = true;
rowIndex = 1;
outData = [];
graphPointCount = 0;
sendDataFlag = false;   %  Flag to indicate if there is data to be sent
sendDataCounter = 1;

%  If there is a file to be sent then load the file 

if ~isempty( sendDataFileName )
    
    %  Load the file
    sendSerialData = load( sendDataFileName );
    
    %  The file has a variable named breathingData which has at least two columns
    %  The first column is the sample number.  The second column breathing  data
    %
    %  There may be multiple columns of data?
    %
    dataMx = sendSerialData.breathingData;

    %  Reshape the breathing data so that it is in one column only.  Don't
    %  use the sample number column
    
    [nSendSamples,nColumns] = size(dataMx);
    inputData = reshape( dataMx(:,2:end), nSendSamples * (nColumns-1) , 1 );
    
    sendDataFlag = true;
    
end

while dataAvailable && contReadSamples
    
    %  If the send data flag is true then grab the next value and send it
    %  over the port
    
    if sendDataFlag
        
        %  Get the next value from the input matrix and send it
        sendValue = inputData( sendDataCounter );
        hArd.WriteNumeric( sendValue );
        
        sendDataCounter = sendDataCounter + 1;
        
    end
    
    
    %  Idle while waiting for data in the buffer
    
    %  Start the timer
    dataAvailable = hArd.WaitForData;
    
    if dataAvailable
        
        
        rxString = hArd.ReadArduino;
        
        if ~isempty( rxString )
            
            %  Echo the serial data to the MATLAB command window
            if echoData
                fprintf('%s\n',rxString)
            end
            
            %  Split the string by TABs
            dataString = strsplit( rxString, TAB );
            
            %  Convert each cell in the split to a double.  If it is the
            %  first data received and is a NaN then assume it is a header
            %  and don't put it in the data array.  If its a NaN but not
            %  the first row then put a NaN in the plot array
            
            
            for iCell = 1:length(dataString)
                
                %  Flag to indicate if the row has any data in it.  If it
                %  does the row index will be incremented
                dataWrittenInRow = false;
                
                %  Convert the string in the cell to a numerical value.  Check
                %  if it is NaN
                numericValue = str2double( dataString{iCell} );
                
                
                %  If this is the first string of data and is a NaN then assume it is a header
                %  and don't put it in the data array.  If its a NaN but not
                %  the first row then put a NaN in the plot array
                
                if ~(  isnan(  numericValue ) && firstLine  )
                    outData( rowIndex, iCell ) = numericValue;  % Its a number put a number in the array
                    dataWrittenInRow = true;
                end
                
                
            end
            
            %  Check if any valid data written in the row.  If so then
            %  increment the row index value.  Clear the firstData flag
            if dataWrittenInRow
                rowIndex = rowIndex + 1;
                firstLine = false;
            end
            
            %  Compare the the number of rows of data written to the input
            %  argument NumReadSamples.  If it is equal then stop reading
            %  from the port.  The NumReadSamples is empty keep reading
            %  indefinitely
            
            if ~isempty( numReadSamples )
                if rowIndex > numReadSamples
                    contReadSamples = false;
                end
            end
            
            %  If the active plot is enabled then update the trace
            
            if ~isempty( nActivePlots )
                
                if ~isempty( outData )
                    
                    %  Check the GraphDisplay parameter to see if it is
                    %  time to update the display
                    
                    graphPointCount = graphPointCount + 1;
                    
                    if graphPointCount >= graphDelay
                        
                        %  Update the display
                        for iPlot = 1:nActivePlots
                            hPlot(iPlot).UpdatePlot( [outData(:,1), outData(:,iPlot+1)] );
                        end
                        
                        graphPointCount = 0;
                        
                    end
                    
                end
                
            end
            
            %  Clear the firstLine flag
            firstLine = false;
            
        end
        
        
        
    end
    
    
    
end

%  Close the serial port
hArd.ClosePort;

%  Prompt user whether to save data


defaultAnswer = {'N'};

response = inputdlg('Save Data to File? [N]','Save File',1,defaultAnswer);

if ~isempty(response)  %  Was CANCEL pressed?
if strcmpi( response(1), 'y')
    
    [fileName, pathName ] = uiputfile({'*.mat'},'Select File Name');
  
        if fileName ~= 0
            save(fullfile(pathName, fileName),'outData');

        end
end
    
end




