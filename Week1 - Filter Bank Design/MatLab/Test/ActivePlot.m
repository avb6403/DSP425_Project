classdef ActivePlot < handle
    % ActivePlot Class to create a plot that can be dynamically written to
    %   while data is being captured.
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
        
        %  Set initial values for the graph scale.  Allow y-scale to autoscale
        
        maxX = [];  %  Autoscale
        maxY = [];  %  Autoscale
        graphTitleStr = []';
        xLabelStr = [];
        yLabelStr = [];
        gridOn = true;
        lineWidth = [];
        lineColor = [];
        lineStyle = [];
        traceData = [];
        
        hFig = [];   %  Handle to the figure
        hAxis = [];  %  Handle to the axis
        
        
    end
    
    methods
        function obj = ActivePlot(varargin)
            % ActivePlot -- Construct an instance of this class
            %
            p = inputParser;
            defaultMaxX = [];   %  Autoscale X
            defaultMaxY = [];   %  Autoscale Y
            defaultTitle = 'Graph Title';
            defaultXlabel = 'Sample Number';
            defaultYlabel = 'Value';
            defaultGridOn = 'True';
            defaultLineWidth = 1;
            defaultLineColor = 'b';
            defaultLineStyle = '-';
            
            p.addParameter('MaxX', defaultMaxX, @isnumeric);
            p.addParameter('MaxY', defaultMaxY, @isnumeric);
            p.addParameter('GraphTitle', defaultTitle, @ischar);
            p.addParameter('XLabel', defaultXlabel, @ischar);
            p.addParameter('YLabel', defaultYlabel, @ischar);
            p.addParameter('GridOn', defaultGridOn, @islogical);
            p.addParameter('LineWidth', defaultLineWidth, @isnumeric);
            p.addParameter('LineStyle', defaultLineStyle, @ischar);
            p.addParameter('Color', defaultLineColor, @ischar);
            
            
            p.parse(varargin{:});
            inputArgs = p.Results;
            
            obj.maxX = inputArgs.MaxX;
            obj.maxY = inputArgs.MaxY;
            obj.graphTitleStr = inputArgs.GraphTitle;
            obj.xLabelStr = inputArgs.XLabel;
            obj.yLabelStr = inputArgs.YLabel;
            obj.gridOn = inputArgs.GridOn;
            obj.lineWidth = inputArgs.LineWidth;
            obj.lineColor = inputArgs.Color;
            obj.lineStyle = inputArgs.LineStyle;
            
            
        end  %  ActivePlot Constructor
        
        
        %  CreateFigure method
        function CreateFigure(obj)
            %  Create a new figure.  Save the handles for the figure and the axis
            obj.hFig = figure;
            obj.hAxis = gca;
            
            %  Add title and Axes labels
            
            obj.TitleGraph;
            obj.LabelXAxis;
            obj.LabelYAxis;
            
            if obj.gridOn
                grid( obj.hAxis, 'on')
            end
            
            
        end
        
        %  TitleGraph method
        function TitleGraph(obj)
            %  Title the graph and label the Axes
            title( obj.hAxis, obj.graphTitleStr );
        end  % TitleGraph
        
        %  LabelXAxis method
        function LabelXAxis( obj )
            xlabel( obj.hAxis, obj.xLabelStr );
        end % LabelXAxis
        
        %  LabelYAxis
        function LabelYAxis( obj )
            ylabel( obj.hAxis, obj.yLabelStr );
        end % LabelYAxis
        
        
        
        %  Update the active plot
        % method UpdatePlot
        function UpdatePlot(obj, plotData)
            %  UpdatePlot
            %
            %  Update the traces on the plot with the data contained in plotData
            %  array.  Right now,the first column of plotData is the x-axis and
            %  each column is a trace data
            %
            %  If there are no traces then create them
            
            [~,nColumns] = size(plotData);
            if isempty( obj.traceData )
                
                %  Create a trace for each column of data beyond the first
                %  column which is the x-axis
                
                for iColumn = 1:nColumns-1
                    obj.traceData(iColumn) = line(obj.hAxis, 0,0,'LineWidth',obj.lineWidth,...
                        'Color',obj.lineColor,...
                        'LineStyle',obj.lineStyle);
                    
                end
            end
            
            %  Update the trace with new data
            nTraces = nColumns - 1;
            for iTrace = 1:nTraces
                set(obj.traceData(iTrace),'XData', plotData(:,1) );
                set(obj.traceData(iTrace),'YData', plotData(:,iTrace + 1) );
            end
            
            %  Rescale the axes if necessary
            
            %  Set the X and Y limits.  If the max values are empty then
            %  let MATLAB autoscale
            
            if ~isempty( obj.maxY )
                %  If the data goes beyond the original max X and Y settings
                %  then readjust the plot manually
                obj.maxY = max(  obj.maxY, max(plotData(:,iTrace + 1)) );
                obj.hAxis.YLim = [0, obj.maxY];
                
            end
            
            %  Set the Y limits.  If the max values are empty then let
            %  MATLAB autoscale
            if ~isempty( obj.maxX )
                obj.maxX = max( numSamples, max(plotData(:,1)) );
                obj.hAxis.XLim = [0, obj.maxX];
            end
            
            %  Set the grid
            if obj.gridOn
                grid( obj.hAxis, 'on')
            end
            
            
            drawnow
            
            
        end
    end
    
    
end