%PathSmootherSpline Smooth path using cubic spline interpolation
%
%   This class is for internal use only. It may be removed in the future.
%
%   ps = driving.internal.planning.PathSmootherSpline creates a path
%   smoother system object, ps, that generates poses and driving
%   directions along the smoothed path.
%
%   ps = driving.internal.planning.PathSmootherSpline(Name,Value) creates a 
%   path smoother system object, ps, with each specified property set to 
%   the specified value.
%
%   NumSmoothPoses          - A positive scalar value specifies the number
%                             of poses in the output poses
%   
%   MinSeparation           - A positive scalar used to filter poses in the 
%                             input refPoses. If the Euclidean [x, y] 
%                             distance between two poses is less than this 
%                             value, one of the poses will be excluded in 
%                             interpolation.
%
%   SampleTime              - Sample time
%
%   ShowOptionalOutports    - Flag indicating if enabling the optional
%                             outputs
%
%   PathSmootherSpline methods:
%   step                    - Smooth the reference path
%   release                 - Allow property value changes
%   clone                   - Create a copy of the object
%   isLocked                - Locked status (logical)
%
%   Step method syntax:
%   [poses, directions] = step(ps, refPoses, refDirections) generates a
%   smooth C^2-continuous path by interpolating the input reference path
%   points, refPoses, using a cubic spline.
%
%   [..., cumLengths, curvatures] = step(...) additionally returns the
%   cumulative path length and the signed path curvature along the smooth 
%   path specified by the output poses when the property
%   ShowOptionalOutports is set true.
%
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj) and y = obj() are equivalent.
%
%   Example 1: Smooth a path generated from path planning
%   -----------------------------------------------------
%   % Load a costmap for a parking lot
%   data = load('parkingLotCostmap.mat');
%   parkMap = data.parkingLotCostmap;
%
%   % Define a start and goal pose as [x,y,theta]
%   startPose = [6,  4, 90]; % [meters, meters, degrees]
%   goalPose  = [15, 14, 0];
%
%   % Create an RRT planner
%   planner = pathPlannerRRT(parkMap);
%
%   % Plan a route from start to goal pose
%   refPath = plan(planner, startPose, goalPose);
%
%   % Retrieve transition poses and directions
%   [refPoses, refDirections] = interpolate(refPath);
%
%   % Specify number of poses to return using a separation of
%   % approximately 0.1m
%   approxSeparation = 0.1; % meters
%   numSmoothPoses   = round(refPath.Length / approxSeparation);
%
%   % Create a path smoother
%   ps = driving.internal.planning.PathSmootherSpline( ...
%   'NumSmoothPoses', numSmoothPoses);
%
%   % Return discretized poses along the smooth path
%   [poses, directions] = ps(refPoses, refDirections);
%
%   % Compare the original and the smoothed path
%   figure
%   % Plot the parking lot
%   plot(parkMap)
%   hold on
%
%   % Plot the planned path
%   plot(refPath, 'Vehicle', 'off', 'DisplayName', 'Reference path')
%
%   % Plot the smoothed path
%   plot(poses(:,1), poses(:,2), 'LineWidth', 2, 'DisplayName', 'Smooth path')
%
%   Example 2: Automated Parking Valet in Simulink
%   ----------------------------------------------
%   % Steer a vehicle to the final spot in a parking lot.
%   % <a href="matlab:web(fullfile(docroot, 'driving/ug/automated-parking-valet-in-simulink.html'))">View example</a>
%
%   See also smoothPathSpline, pathPlannerRRT, driving.Path/interpolate, checkPathValidity, spline

%   References
%   ----------
%   [1] Lepetic, Marko, Gregor Klancar, Igor Skrjanc, Drago Matko, and Bostjan
%       Potocnik. "Time Optimal Path Planning Considering Acceleration Limits."
%       Robotics and Autonomous Systems. Vol 45, No. 3-4, 2003, pp. 199-210.
%   [2] Floater, Michael S. "On the deviation of a parametric cubic spline
%       interpolant from its data polygon." Computer Aided Geometric
%       Design. Vol 25, No. 3, 2008, pp. 148-156.
%
%   Copyright 2018-2020 The MathWorks, Inc.

%#codegen

classdef PathSmootherSpline < matlab.System
    properties (Nontunable)
        %NumSmoothPoses Number of output poses
        %   The output size is fixed during one simulation   
        %
        %   Default:         100
        NumSmoothPoses     = 100
        
        %MinSeparation Minimum separation between input poses
        %
        %   Default:         1e-3
        MinSeparation      = 1e-3
        
        
        %SampleTime Sample time
        %   The sample time for any variable-size signal must be discrete
        %
        %   Default:         -1 (inherit)
        SampleTime         = -1
    end
    
    properties(Access = private)
        % Properties store the inputs of the previous step
        
        %RefPosesInternal 
        RefPosesInternal
        
        %RefDirectionsInternal 
        RefDirectionsInternal
        
        % Properties store the outputs of the previous step
        
        %LastPosesOutput 
        LastPosesOutput
        
        %LastDirectionsOutput 
        LastDirectionsOutput
        
        %LastCumLengthsOutput 
        LastCumLengthsOutput
        
        %LastCurvaturesOutput 
        LastCurvaturesOutput
    end
    
    properties (Nontunable)
        %ShowOptionalOutports Show CumLength and Curvature output ports
        %
        %   Default:           false
        ShowOptionalOutports (1, 1) logical = false;
    end
    
    %----------------------------------------------------------------------
    % Setter and constructor
    %----------------------------------------------------------------------
    methods
        function set.NumSmoothPoses(obj, numSmoothPoses)
            driving.internal.validation.checkNumPoses(...
                numSmoothPoses, 'NumSmoothPoses', class(obj));
            
            obj.NumSmoothPoses = numSmoothPoses;
        end
        
        %------------------------------------------------------------------
        function set.MinSeparation(obj, minSeparation)
            driving.internal.validation.checkMinSeparation(...
                minSeparation, 'minSeparation', class(obj));
            
            obj.MinSeparation = minSeparation;
        end
        
        %------------------------------------------------------------------
        function set.SampleTime(obj, st)
            validateattributes( st, {'numeric'}, ...
                {'scalar', 'real', 'nonnan', 'finite', 'nonzero'}, ...
                class(obj), 'SampleTime');
            
            % Only inherited and discrete sample time are supported for
            % variable-size signal inputs
            coder.internal.errorIf((st < 0) && (st ~=-1), ...
                'driving:PathSmootherSpline:negativeSampleTime');
            
            obj.SampleTime = st;
        end
        
        %------------------------------------------------------------------
        function obj = PathSmootherSpline(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:}, 'NumSmoothPoses', 'SampleTime');
        end
    end
    
    %----------------------------------------------------------------------
    % Main algorithm
    %----------------------------------------------------------------------
    methods(Access = protected)
        %------------------------------------------------------------------
        function setupImpl(obj,refPoses,refDirections)
            %setupImpl Perform one-time calculations
            
            % Initialize 
            obj.RefPosesInternal      = nan(size(refPoses),        'like', refPoses);
            obj.RefDirectionsInternal = nan(size(refDirections),   'like', refDirections);  
            
            obj.LastPosesOutput       = nan(obj.NumSmoothPoses, 3, 'like', refPoses);
            obj.LastDirectionsOutput  = nan(obj.NumSmoothPoses, 1, 'like', refDirections);
            obj.LastCumLengthsOutput  = nan(obj.NumSmoothPoses, 1, 'like', refPoses);
            obj.LastCurvaturesOutput  = nan(obj.NumSmoothPoses, 1, 'like', refPoses);
        end
        
        %------------------------------------------------------------------
        function [poses, directions, varargout] = stepImpl(obj, refPoses, refDirections)
            %stepImpl Implement the algorithm of interpolating a spline
            
            % Check if refDirections contains invalid values
            hasInvalidDirection = any(refDirections ~=1 & refDirections ~= -1);
            coder.internal.errorIf(hasInvalidDirection, ...
                'driving:smoothPathSpline:invalidDirections', 'RefDirections');
            
            % If the input poses are not new, use the previous output
            if isequaln(refPoses, obj.RefPosesInternal) && ...
                    isequaln(refDirections, obj.RefDirectionsInternal)
                poses       = obj.LastPosesOutput;
                directions  = obj.LastDirectionsOutput;
                
                if obj.ShowOptionalOutports
                    varargout{1} = obj.LastCumLengthsOutput;
                    varargout{2} = obj.LastCurvaturesOutput;
                end
               
                return
            else
                obj.RefPosesInternal      = refPoses;
                obj.RefDirectionsInternal = refDirections;
            end
            
            % Smooth the path
            [poses, directions, cumLengths, curvatures] = ...
                smoothPathSpline(refPoses, refDirections, obj.NumSmoothPoses, obj.MinSeparation);
            
            % Optional outputs
            if obj.ShowOptionalOutports
                varargout{1} = cumLengths;
                varargout{2} = curvatures;
            end
            
            % Store the outputs
            obj.LastPosesOutput       = poses;
            obj.LastDirectionsOutput  = directions;
            obj.LastCumLengthsOutput   = cumLengths;
            obj.LastCurvaturesOutput  = curvatures;
        end
    end
    
    %----------------------------------------------------------------------
    % Common methods
    %----------------------------------------------------------------------
    methods(Access = protected)
        %------------------------------------------------------------------
        function validateInputsImpl(obj, refPoses, refDirections)
            % Validate inputs to the step method at initialization
            
            % Validate refPoses
            matlabshared.planning.internal.validation.checkPose( ...
                refPoses, 3, 'RefPoses', class(obj), true);
            
            % Validate refDirections
            validateattributes(refDirections, {'single', 'double'}, ...
                {'real', 'nonsparse', 'column', 'nrows', size(refPoses, 1)}, ...
                class(obj), 'RefDirections');
            
            % Input datatypes should be the same
            isDataTypeEqual = isequal(class(refPoses), class(refDirections));           
            coder.internal.errorIf(~isDataTypeEqual, ...
                'driving:PathSmootherSpline:dataTypeMismatch', ...
                class(refPoses), class(refDirections));
        end
        
        function flag = isInputSizeMutableImpl(~,~)
            % Return false if input size cannot change between calls
            flag = true;
        end
        
        function num = getNumInputsImpl(~)
            % Define total number of inputs
            num = 2;
        end
        
        function num = getNumOutputsImpl(obj)
            % Define total number of outputs
            if obj.ShowOptionalOutports
                num = 4;
            else
                num = 2;
            end
        end
        
        %------------------------------------------------------------------
        function s = saveObjectImpl(obj)
            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);
            
            % Set private and protected properties
            s.RefPosesInternal      = obj.RefPosesInternal;
            s.RefDirectionsInternal = obj.RefDirectionsInternal;
            s.LastPosesOutput       = obj.LastPosesOutput;
            s.LastDirectionsOutput  = obj.LastDirectionsOutput;
            s.LastCumLengthsOutput  = obj.LastCumLengthsOutput;
            s.LastCurvaturesOutput  = obj.LastCurvaturesOutput;
        end
        
        %------------------------------------------------------------------
        function loadObjectImpl(obj,s,wasLocked)
            % Set private and protected properties
            obj.RefPosesInternal      = s.RefPosesInternal;
            obj.RefDirectionsInternal = s.RefDirectionsInternal;
            obj.LastPosesOutput       = s.LastPosesOutput;
            obj.LastDirectionsOutput  = s.LastDirectionsOutput;
            obj.LastCumLengthsOutput  = s.LastCumLengthsOutput;
            obj.LastCurvaturesOutput  = s.LastCurvaturesOutput;
            
            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
    end
    
    %----------------------------------------------------------------------
    % Simulink-only methods
    %----------------------------------------------------------------------
    methods(Access = protected)
        %------------------------------------------------------------------
        function varargout = getOutputSizeImpl(obj)
            % Return size for each output port
            varargout{1} = [obj.NumSmoothPoses, 3]; % Poses
            varargout{2} = [obj.NumSmoothPoses, 1]; % Directions
            
            if obj.ShowOptionalOutports
                varargout{3} = [obj.NumSmoothPoses, 1]; % CumLengths
                varargout{4} = [obj.NumSmoothPoses, 1]; % Curvatures
            end
        end
        
        %------------------------------------------------------------------
        function varargout = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            varargout{1} = propagatedInputDataType(obj,1);
            varargout{2} = propagatedInputDataType(obj,2);
            
            if obj.ShowOptionalOutports
                varargout{3} = propagatedInputDataType(obj,2);
                varargout{4} = propagatedInputDataType(obj,2);
            end
        end
        
        %------------------------------------------------------------------
        function varargout = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            varargout{1} = false;
            varargout{2} = false;
            
            if obj.ShowOptionalOutports
                varargout{3} = false;
                varargout{4} = false;
            end
        end
        
        %------------------------------------------------------------------
        function varargout = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            varargout{1} = true;
            varargout{2} = true;
            
            if obj.ShowOptionalOutports
                varargout{3} = true;
                varargout{4} = true;
            end
        end
       
        
        function sts = getSampleTimeImpl(obj)
            if obj.SampleTime == -1
                sts = createSampleTime(obj,'Type','Inherited');
            else
                sts = createSampleTime(obj,'Type','Discrete',...
                    'SampleTime',obj.SampleTime);
            end
        end
        
        %------------------------------------------------------------------
        function icon = getIconImpl(~)
            % Define icon for System block
            icon = ["Path", "Smoother", "Spline"];
        end
        
        %------------------------------------------------------------------
        function [name1,name2] = getInputNamesImpl(~)
            % Return input port names for System block
            name1 = 'RefPoses';
            name2 = 'RefDirections';
        end
        
        %------------------------------------------------------------------
        function varargout = getOutputNamesImpl(obj)
            % Return output port names for System block
            varargout{1} = 'Poses';
            varargout{2} = 'Directions';
            
            if obj.ShowOptionalOutports
                varargout{3} = 'CumLengths';
                varargout{4} = 'Curvatures';
            end
        end
    end
    
    %----------------------------------------------------------------------
    % Simulink dialog
    %----------------------------------------------------------------------
    methods(Access = protected, Static)
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(...
                'Title','driving:block:PathSmootherSplineTitle',...
                'Text','driving:block:PathSmootherSplineDesc',...
                'ShowSourceLink', false);
        end
        
        function group = getPropertyGroupsImpl
            % Define property section for System block dialog
            propNames = {'NumSmoothPoses','MinSeparation', 'SampleTime', ...
                'ShowOptionalOutports'};
            group = matlabshared.tracking.internal.getDisplaySection(...
                'driving','PathSmootherSpline','',propNames);
        end
    end
end

% LocalWords:  costmap RRT
