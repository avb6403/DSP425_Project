function impulseResponse = FIR_Designer( varargin )
%
%  FIR_Designer
%
%  This script returns the impulse response of a windowed SINC filter.  The
%  filter has various input parameters in Name, Value pairs
%
%  The routine will print out a set of C-header lines for use on the Arduino
%  platform.
%
%
%  Input Arguments (Required)
%
%  'nOrder' -- Order of the filter, that is the length of the impulse reponse
%  'cutBPM' -- The corner frequency of the filter in BPM.  Assumes that the
%              sample rate is 600 BPM.  The "corner frequency" is the 1/2 voltage point
%
%  Optional Input Arguments
%
%  'Type' -- 'HPF' or 'LPF'
%  'PrintHeader' -- [true, false]  to print C header format
%  'FxdPoint' -- [true, false] to Convert to a fixed point coefficients (default true)
%  'FxdPointScale' -- Manual scale for the value to convert to fixed point
%  'SampleRate' -- Sample rate of the filter -- Default to 600 BPM
%  'Window' -- Window selection, Hamming or Blackman or 'none' -- Default Hamming
%  'PlotResponses' -- [true, false] Turn on/off graphing the responses --  Default true
%
%
%  Advanced Parameters
%
%  'ZeroTolerance'  -- Value below which is considered to be zero
%
%  Output Arguments
%
%  impulseReponse -- The filter impulse response
%
%
%  Written by Mark Thompson 11/2020
%  ECTET Department
%  College of Engineering Technology
%  Rochester Institute of Technology
%
%  Revisions A
%
%

p = inputParser;
defaultN = 31;          %  Default number of samples in impulse response
defaultFCorner = 50;    %  Default corner frequency
defaultType = 'LOW';    %  Lowpass filter
defaultHeader = true;  %  Print the header
defaultFxdPoint = true; %  Flag to automatically scale for fixed point numbers
defaultFxdPointScale = [];  %  Manually set fixed point scale value
defaultZeroTolerance = 1/4096;  %  Value below which the point is considered zero
defaultSampleRate = 600;  %  Sample rate in BPM
defaultWindow = 'hamming';  %  Default window hamming
defaultPlots = true;  %  Default window hamming
defaultMATLAB = true;  %  Default value for printing in MATLAB format for easy copying



p.addParameter('CutBPM', defaultFCorner, @isnumeric);
p.addParameter('nOrder', defaultN, @isnumeric);
p.addParameter('Type', defaultType, @ischar);
p.addParameter('PrintHeader', defaultHeader, @islogical);
p.addParameter('FxdPoint', defaultFxdPoint, @islogical);
p.addParameter('FxdPointScale', defaultFxdPointScale, @isnumeric);
p.addParameter('ZeroTolerance', defaultZeroTolerance, @isnumeric);
p.addParameter('SampleRate', defaultSampleRate, @isnumeric);
p.addParameter('Window', defaultWindow, @ischar);
p.addParameter('PlotResponses', defaultPlots, @islogical);
p.addParameter('PrintMATLAB',defaultMATLAB, @islogical);


p.parse(varargin{:});
inputArgs = p.Results;

filterLength = inputArgs.nOrder;
fCornerBPM = inputArgs.CutBPM;
filterType = inputArgs.Type;
printHeaderFlag = inputArgs.PrintHeader;
fxdPointFlag = inputArgs.FxdPoint;
fxdPointScale = inputArgs.FxdPointScale;
zeroTolerance = inputArgs.ZeroTolerance;
sampleRateBPM = inputArgs.SampleRate;
windowType = inputArgs.Window;
plotResponses = inputArgs.PlotResponses;
printMATLABFlag = inputArgs.PrintMATLAB;





%%  Create a windowed SINC filter with a Hamming window

impulseResponse = [];
winValue = [];
fCornerNorm = fCornerBPM / sampleRateBPM;

%  Iterate through the points of the impulse response.  Computing the SINC
%  response and the window

for i = 0:filterLength-1
    phi = 2*pi * fCornerNorm * (i-(filterLength-1)/2);
    
    %  Check for the middle of the filter to avoid division by zero.
    %  Otherwise compute the sinc function
    
    if i == (filterLength-1) / 2
        impulseResponse(i+1) = 1;  % MATLAB indexing
        
    else
        impulseResponse(i+1) = sin(phi)/phi; %  MATLAB indexing
        
    end
    
    %  Apply a window value
    
    
    switch lower(windowType)
        
        case {'hamming', 'hamm', 'h'}
            winSample = ( 0.54 -.46*cos(2*pi*i/(filterLength-1)));
            
        case {'blackman','black','b'}
            winSample = 0.42 - 0.5 * cos( 2*pi*i/(filterLength-1)) + 0.08*cos(4*pi*i/(filterLength-1));
            
        case {'none','off',''}
            winSample = 1;
            
        otherwise
            winSample = 1;
    end
    
    
    
    
    
    impulseResponse(i+1) = impulseResponse(i+1) * winSample ;
    
    
end

%  Normalize the DC gain of the LPF to 1.0
impulseResponse = impulseResponse/sum(impulseResponse);
filterTypeStr = 'LPF';

%  If the filter type is a highpass filter then convert the low pass
%  impulse response to a HPF.  Otherwise leave it as a LPF (don't do
%  anything)

switch lower(filterType)
    case {'high','hpf','h'}
        
        deltaFcn = zeros(1,filterLength);
        deltaFcn( (filterLength-1) /2 + 1 ) = 1;
        impulseResponse = deltaFcn - impulseResponse;
        
        filterTypeStr = 'HPF';
        
    otherwise
end

% %  Round the impulse response values to 7 digits
%
% impulseResponse = round(impulseResponse / sampleTolerance )*sampleTolerance;


%  If the filter is selected to be a fixed point filter find the
%  fxdPointScale value if it is not specified in the calling function

if fxdPointFlag
    if isempty(fxdPointScale)
        
        %  Find the smallest non-zero coefficient of the impulse response
        smallestCoeff = min(abs(impulseResponse(abs(impulseResponse) >= zeroTolerance ) ) );
        HFXPT = 2^ceil( log2( 1/smallestCoeff ) );
        
    else
        HFXPT = fxdPointScale;
    end
    
end

%  If the printHeaderFlag is true then output C-header code in the command
%  window

if printHeaderFlag
    
    numColumns = 14;  %  Number of columns in each row
    
    fprintf('\n\t// %s FIR Filter Coefficients MFILT = %d, Fc = %d\n',filterTypeStr, filterLength, fCornerBPM);
    
    %      const int HFXPT = 1, MFILT = 4;
    fprintf( '\tconst int HFXPT = %d, MFILT = %d;\n',HFXPT, filterLength );
    
    for iCoeff = 1:filterLength-1
        
        hCoeffValue = round(impulseResponse(iCoeff) * HFXPT);
        
        if iCoeff == 1
            fprintf('\tint h[] = {');
        end
        
        fprintf( '%d', hCoeffValue );
        
        if mod(iCoeff,numColumns) == 0
            fprintf( ',\n\t');
        else
            fprintf(', ');
        end
    end
    hCoeffValue = round( impulseResponse(filterLength) * HFXPT );
    
    fprintf( '%d};\n', hCoeffValue )
end

if printMATLABFlag
    
    %  If the printMATLABFlag is true then output the impulse response so
    %  that it can easily be copied into MATLAB
     
    if fxdPointFlag
        
        fprintf('\n Fixed Point scale and FIR Filter Coefficients for easy copy to MATLAB\n\n');        
        fprintf('HFXPT = %d;\n\n', HFXPT);

        numColumns = 14;  %  Number of columns in each row
   
        for iCoeff = 1:filterLength-1

            hCoeffValue = round(impulseResponse(iCoeff) * HFXPT);

            if iCoeff == 1
                fprintf('h = [');
            end
            
            fprintf( '%d', hCoeffValue );
            
            if mod(iCoeff,numColumns) == 0
                fprintf( ',...\n');
            else
                fprintf(', ');
            end
        end

        hCoeffValue = round(impulseResponse(filterLength) * HFXPT);

        fprintf( '%d];\n', hCoeffValue )
        
    else
        
        %  Not fixed point
         
        numColumns = 14;  %  Number of columns in each row
         
        for iCoeff = 1:filterLength-1
            
 
            hCoeffValue = impulseResponse(iCoeff);

            if iCoeff == 1
                fprintf('h = [');
            end
            
            fprintf( '%f', hCoeffValue );
            
            if mod(iCoeff,numColumns) == 0
                fprintf( ',...\n');
            else
                fprintf(', ');
            end
        end
        
        
        hCoeffValue = impulseResponse(filterLength);

        fprintf( '%f];\n', hCoeffValue )

    end
end



%%  Compute the frequency response

[mag,phase] = freqz( impulseResponse, 1, 512 );



%%  Plot the impulse and frequency response if selected

if plotResponses
    
    figure
    plot(impulseResponse, 'LineWidth',2);
    grid on
    title('Filter Impulse Response');
    xlabel('Samples');
    ylabel('Amplitude');
    
    
    figure
    plot(phase/(2*pi)*sampleRateBPM, abs(mag), 'LineWidth',2);
    grid on
    title('Filter Frequency Response');
    xlabel('Frequency');
    ylabel('Amplitude');
    
end






