%VelocityProfiler Generate velocity profile along the reference path
%
%   This class is for internal use only. It may be removed in the future.
%
%   Given a reference path, VelocityProfiler generates the corresponding
%   reference velocity at each point on the path satisfying the speed, 
%   acceleration and jerk constraints.
%
%   vp = driving.internal.planning.VelocityProfiler creates a velocity profile 
%   generator system object, vp, that generates a velocity profile for a  
%   given reference path.
%
%   vp = driving.internal.planning.VelocityProfiler(Name,Value) creates a 
%   velocity profile generator object, vp, with each specified property set  
%   to the specified value.
%   
%   VelocityProfiler properties:
%   MaxLongitudinalAccel     - Maximum longitudinal acceleration (m/s^2)
%   MaxLongitudinalDecel     - Maximum longitudinal deceleration (m/s^2)
%   MaxSpeed                 - Maximum speed along the path (m/s)
%   MaxLongitudinalJerk      - Maximum longitudinal jerk (m/s^3)
%   MaxLateralAccel          - Maximum lateral acceleration (m/s^2) 
%   SampleTime               - Sample time (s)
%   ShowOptionalOutport      - Flag indicating if times output is enabled                        
%
%   VelocityProfiler methods:
%   step                     - Generate velocity profile
%   release                  - Allow property value changes
%   clone                    - Create a copy of the object
%   isLocked                 - Locked status (logical)
%
%   Step method syntax:
%   velocities = step(vp, directions, cumLengths, curvatures, startVelocity, endVelocity) 
%   generates a velocity profile, velocities, given the cumulative path length,  
%   cumLengths, the path curvatures, curvatures, the driving directions  
%   at each point of the path, directions, the start velocity, startVelocity,
%   and the end velocity, endVelocity
%
%   [~, times] = step(...) also returns the corresponding time of arrival
%   at each velocity value.
%
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj) and y = obj() are equivalent.
%
%   Algorithms
%   ----------
%   - Generally the generated velocity profile contains seven intervals:
%     1: Accelerate from the starting velocity to reach the maximum
%        longitudinal acceleration. The initial acceleration is assumed zero
%     2: Continue accelerating at the maximum longitudinal acceleration 
%     3: Decrease the acceleration to zero and reach the maximum speed
%     4: Keep constant velocity
%     5: Decelerate to reach the maximum longitudinal deceleration
%     6: Continue decelerating at the maximum longitudinal deceleration 
%     7: Decrease the deceleration to zero and reach the ending velocity
%   - When the maximum velocity is too close to the starting and ending
%     velocities, the maximum longitudinal acceleration/deceleration may
%     not be reached in intervals 1 and 5, in this case intervals 2 and 6 
%     do not exist.
%   - When the path length is too short to reach the maximum speed, a new
%     maximum speed is calculate to satisfy the path length constraint.
%   - In all the intervals, the longitudinal jerk is constrained in the 
%     range of [-MaxLongitudinalJerk, MaxLongitudinalJerk].
%
%   Class Support
%   -------------
%   The inputs can be single or double. The output velocity profile have  
%   the same data type as the first input.
%
%   Example 1: Generate a velocity profile for a smooth path 
%   --------------------------------------------------------
%   % Load a costmap for a parking lot
%   data = load('parkingLotCostmap.mat');
%   parkMap = data.parkingLotCostmap;
%  
%   % Define a start and goal pose as [x,y,theta]
%   startPose = [4,  4, 90]; % [meters, meters, degrees]
%   goalPose  = [30, 13, 0];
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
%   % Return 1000 discretized poses along the smooth path 
%   [~, directions, cumLengths, curvatures] = smoothPathSpline(refPoses, refDirections, 1000);
%
%   % Create a velocity profiler by specifying the maximum speed
%   vp = driving.internal.planning.VelocityProfiler('MaxSpeed', 2);
%
%   % Generate velocity profile for the smooth path. The vehicle starts and
%   % end with zero velocity
%   velocities = step(vp, directions, cumLengths, curvatures, 0, 0);
%  
%   % Plot velocity profile along the path
%   plot(cumLengths, velocities)
%
%   Example 2: Automated Parking Valet in Simulink
%   ----------------------------------------------
%   % Steer a vehicle to the final spot in a parking lot.
%   % <a href="matlab:web(fullfile(docroot, 'driving/ug/automated-parking-valet-in-simulink.html'))">View example</a>
%
%   See also smoothPathSpline.

%   References
%   ----------
%   Villagra, Jorge, Vicente Milanes, Joshue Perez, and Jorge Godoy. "Smooth 
%   path and speed planning for an automated public transport vehicle." 
%   Robotics and Autonomous Systems 60, no. 2 (2012): 252-265.
%
%   Copyright 2019-2020 The MathWorks, Inc.

%#codegen

classdef VelocityProfiler < matlab.System

    properties 
        %MaxSpeed Maximum speed (m/s)
        %   Maximum speed of the vehicle on the path.
        %
        %   Default:           10 (meters/second)
        MaxSpeed             = 10
    end
    
    properties (Nontunable)
        %MaxLongitudinalAccel Maximum longitudinal acceleration (m/s^2)
        %
        %   Default:           3 (meters/second^2)
        MaxLongitudinalAccel = 3
        
        %MaxLongitudinalDecel Maximum longitudinal deceleration (m/s^2)
        %
        %   Default:           6 (meters/second^2)
        MaxLongitudinalDecel = 6
        
        %MaxLongitudinalJerk Maximum longitudinal jerk (m/s^3)
        %
        %   Default:           1 (meters/second^3)
        MaxLongitudinalJerk  = 1
        
        %MaxLateralAccel  Maximum lateral acceleration (m/s^2)
        %
        %   Default:           1 (meters/second^2)
        MaxLateralAccel      = 1
        
        %SampleTime Sample time
        %   The sample time for any variable-size signal must be discrete
        %
        %   Default:           -1 (inherit)
        SampleTime           = -1
    end
    
    properties(Access = private)
        %The following two private properties are used to avoid unnecessary
        %   re-computation of the same output. LastVelocities stores
        %   the output from the previous step. If the input does not change, 
        %   reuses LastVelocities as the output.
        
        %LastVelocities Previous velocities output
        %
        LastVelocities
        
        %LastTimes Previous times output
        %
        LastTimes
        
        %LastStartVelocity Previous start velocity input
        %
        LastStartVelocity
        
        %LastEndVelocity Previous end velocity input
        %
        LastEndVelocity
        
        %LastCumLengths Previous cumulative path length input
        %
        LastCumLengths
        
        %LastCurvatures Previous curvatures input
        %
        LastCurvatures
        
        %LastDirections Previous directions input
        %
        LastDirections
    end
    
    properties (Nontunable)
        %ShowOptionalOutport Show times output port
        %
        %   Default:           false
        ShowOptionalOutport (1, 1) logical = false;
    end
    
    %----------------------------------------------------------------------
    % Setter and constructor
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function set.MaxSpeed(obj, v)
            obj.MaxSpeed = validatePositiveProperty(obj, v, 'MaxSpeed');
        end

        %------------------------------------------------------------------
        function set.MaxLongitudinalAccel(obj, accel)
            obj.MaxLongitudinalAccel = validatePositiveProperty( ...
                obj, accel, 'MaxLongitudinalAccel');
        end
        
        %------------------------------------------------------------------
        function set.MaxLongitudinalDecel(obj, decel)
            obj.MaxLongitudinalDecel = validatePositiveProperty( ...
                obj, decel, 'MaxLongitudinalDecel');
        end
        
        %------------------------------------------------------------------
        function set.MaxLongitudinalJerk(obj, jerk)
            obj.MaxLongitudinalJerk = validatePositiveProperty( ...
                obj, jerk, 'MaxLongitudinalJerk');
        end
        
        %------------------------------------------------------------------
        function set.MaxLateralAccel(obj, accel)
            obj.MaxLateralAccel = validatePositiveProperty( ...
                obj, accel, 'MaxLateralAccel');
        end
        
        %------------------------------------------------------------------
        function set.ShowOptionalOutport(obj, flag)
            validateattributes( flag, {'logical'}, {'scalar'}, ...
                class(obj), 'ShowOptionalOutport');
            
            obj.ShowOptionalOutport = flag;
        end
        
        %------------------------------------------------------------------
        function set.SampleTime(obj, st)
            validateattributes( st, {'numeric'}, ...
                {'scalar', 'real', 'nonnan', 'finite', 'nonzero'}, ...
                class(obj), 'SampleTime');
            
            % Only inherited and discrete sample time are supported for
            % variable-size signal inputs
            coder.internal.errorIf((st < 0) && (st ~=-1), ... % Reuse catalog
                'driving:PathSmootherSpline:negativeSampleTime');
            
            obj.SampleTime = st;
        end
        
        %------------------------------------------------------------------
        function obj = VelocityProfiler(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:}, 'MaxSpeed', ...
                'MaxLongitudinalAccel', 'MaxLongitudinalDecel', ...
                'MaxLongitudinalJerk', 'MaxLateralAccel', ...
                'ShowOptionalOutport', 'SampleTime');
        end
    end
    
    %----------------------------------------------------------------------
    % Main function
    %----------------------------------------------------------------------
    methods(Access = protected)
        %------------------------------------------------------------------
        function setupImpl(obj, Directions, ~, ~, ~, ~)            
            %setupImpl Initialize internal properties
            obj.LastVelocities    = zeros(size(Directions), "like", Directions);
            obj.LastTimes         = zeros(size(Directions), "like", Directions);
            obj.LastCumLengths    = zeros(size(Directions), "like", Directions);
            obj.LastDirections    = ones(size(Directions), "like", Directions);
            obj.LastCurvatures    = zeros(size(Directions), "like", Directions);
            obj.LastStartVelocity = cast(0, 'like', Directions);
            obj.LastEndVelocity   = cast(0, 'like', Directions);
        end
        
        %------------------------------------------------------------------
        function varargout = stepImpl(obj, directions, cumLengths, curvatures, startVelocity, endVelocity)
            %stepImpl Implement the main algorithm
            
            % Run-time check of input values
            hasInvalidDirection = any(directions ~=1 & directions ~= -1);
            coder.internal.errorIf(hasInvalidDirection, ...
                'driving:smoothPathSpline:invalidDirections', 'Directions');
            
            validateVelocities(obj, startVelocity, endVelocity, directions, obj.MaxSpeed);
            
            % Check if the path is new. If not, use the previous output.
            if isPathNew(obj, directions, cumLengths, curvatures, startVelocity, endVelocity)
                varargout{1} = obj.LastVelocities; 
                
                if obj.ShowOptionalOutport
                    varargout{2} = obj.LastTimes;
                end              
                return
            end
            
            % Call the internal function
            [velocities, times] = generateVelocityProfile( ...
                directions, cumLengths, curvatures, startVelocity, endVelocity, ...
                'MaxSpeed', obj.MaxSpeed, ...
                'MaxLateralAccel', obj.MaxLateralAccel, ...
                'MaxLongitudinalAccel', obj.MaxLongitudinalAccel, ...
                'MaxLongitudinalDecel', obj.MaxLongitudinalDecel, ...
                'MaxLongitudinalJerk',  obj.MaxLongitudinalJerk);
            
            % Store the inputs and outputs
            obj.LastCumLengths      = cumLengths;            
            obj.LastStartVelocity   = startVelocity;
            obj.LastEndVelocity     = endVelocity;
            obj.LastCurvatures      = curvatures;
            obj.LastDirections      = directions;
            obj.LastVelocities      = velocities;
            
            varargout{1} = velocities;
            
            if obj.ShowOptionalOutport
                varargout{2} = times;
                obj.LastTimes = times;
            end
        end
    end
    
    %----------------------------------------------------------------------
    % Common functions
    %----------------------------------------------------------------------
    methods (Access = protected)
        %------------------------------------------------------------------
        function validateInputsImpl(obj, directions, cumLengths, curvatures, startVelocity, endVelocity)
            % Validate inputs to the step method at initialization
            
            % Validate directions
            checkValue = false;
            driving.internal.validation.checkDirection(directions, ...
                size(directions,1), 'Directions', class(obj), checkValue);
            
            coder.internal.errorIf(isscalar(directions), ...
                'driving:VelocityProfiler:expectedNonScalar', 'Directions');

            % Validate cumLengths
            validateattributes(cumLengths, {'single', 'double'}, ...
                {'real', 'finite', 'nonsparse', 'column', 'nrows', ...
                size(directions, 1)}, class(obj), 'CumLengths');
            
            % Validate curvatures
            validateattributes(curvatures, {'single', 'double'}, ...
                {'real', 'finite', 'nonsparse', 'column', 'nrows', ...
                size(directions, 1)}, class(obj), 'Curvatures');
            
            % Validate startVelocity
            validateattributes(startVelocity, {'single', 'double'}, ...
                {'real', 'finite', 'scalar'}, class(obj), 'StartVelocity');
            
            % Validate endVelocity
            validateattributes(endVelocity, {'single', 'double'}, ...
                {'real', 'finite', 'scalar'}, class(obj), 'EndVelocity');
        end
        
        %------------------------------------------------------------------
        function num = getNumInputsImpl(~)
            num = 5;
        end
        
        %------------------------------------------------------------------
        function num = getNumOutputsImpl(obj)
            if obj.ShowOptionalOutport
                num = 2;
            else
                num = 1;
            end
        end
        
        %------------------------------------------------------------------
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj
            
            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);
            
            % Set private and protected properties
            s.LastVelocities    = obj.LastVelocities;
            s.LastTimes         = obj.LastTimes;
            s.LastStartVelocity = obj.LastStartVelocity;
            s.LastEndVelocity   = obj.LastEndVelocity;
            s.LastCumLengths    = obj.LastCumLengths;
            s.LastCurvatures    = obj.LastCurvatures;
            s.LastDirections    = obj.LastDirections;
        end
        
        %------------------------------------------------------------------
        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s
            
            % Set private and protected properties
            obj.LastVelocities    = s.LastVelocities;
            obj.LastTimes         = s.LastTimes;
            obj.LastStartVelocity = s.LastStartVelocity;
            obj.LastEndVelocity   = s.LastEndVelocity;
            obj.LastCumLengths    = s.LastCumLengths;
            obj.LastCurvatures    = s.LastCurvatures;
            obj.LastDirections    = s.LastDirections;
            
            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end
    end
    
    %----------------------------------------------------------------------
    % Simulink only
    %----------------------------------------------------------------------
    methods (Access = protected)
        %------------------------------------------------------------------
        function flag = isInputSizeMutableImpl(~,~)
            % Return false if input size cannot change
            % between calls to the System object
            flag = true;
        end
        
        %------------------------------------------------------------------
        function varargout = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            varargout{1} = propagatedInputDataType(obj,1); 
            if obj.ShowOptionalOutport
                varargout{2} = propagatedInputDataType(obj,1);
            end
        end
        
        %------------------------------------------------------------------
        function varargout = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            varargout{1} = false;
            if obj.ShowOptionalOutport
                varargout{2} = false;
            end
        end
        
        %------------------------------------------------------------------
        function varargout = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            varargout{1} = true;
            if obj.ShowOptionalOutport
                varargout{2} = true;
            end
        end
        
        %------------------------------------------------------------------
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
            icon = ["Velocity", "Profiler"];
        end
        
        %------------------------------------------------------------------
        function [name1,name2,name3,name4,name5] = getInputNamesImpl(~)
            % Return input port names for System block
            name1 = 'Directions';
            name2 = 'CumLengths';
            name3 = 'Curvatures';
            name4 = 'StartVelocity';
            name5 = 'EndVelocity';
        end
        
        %------------------------------------------------------------------
        function varargout = getOutputNamesImpl(obj)
            % Return output port names for System block
            varargout{1} = 'Velocities';
            if obj.ShowOptionalOutport
                varargout{2} = 'Times';
            end
        end
        
        %------------------------------------------------------------------
        function varargout = getOutputSizeImpl(obj)
            % Return size for each output port
            varargout{1} = propagatedInputSize(obj,3);
            if obj.ShowOptionalOutport
                varargout{2} = propagatedInputSize(obj,3);
            end
        end
    end
    
    %----------------------------------------------------------------------
    % Simulink dialog
    %----------------------------------------------------------------------
    methods(Access = protected, Static)
        %------------------------------------------------------------------
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(...
                'Title','driving:block:VelocityProfilerTitle',...
                'Text','driving:block:VelocityProfilerDesc', ...
                'ShowSourceLink', false);
        end
        
        %------------------------------------------------------------------
        function group = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            vehicleParamNames = {'MaxLongitudinalAccel','MaxLongitudinalDecel'};
            vehicleParamGroup = matlabshared.tracking.internal.getDisplaySection(...
                'driving','VelocityProfiler','VehicleParams',vehicleParamNames);
            
            constraintNames = {'MaxSpeed','MaxLongitudinalJerk', 'MaxLateralAccel'};
            constraintGroup = matlabshared.tracking.internal.getDisplaySection(...
                'driving','VelocityProfiler','ComfortConstraints',constraintNames);   
            
            outputGroup = matlabshared.tracking.internal.getDisplaySection(...
                'driving','VelocityProfiler','Ports',{'ShowOptionalOutport'});
            TsGroup = matlabshared.tracking.internal.getDisplaySection(...
                'driving','VelocityProfiler','',{'SampleTime'});
            
            group = [vehicleParamGroup, constraintGroup, outputGroup, TsGroup];
        end
    end
    
    %----------------------------------------------------------------------
    % Utility functions
    %----------------------------------------------------------------------
    methods(Access = protected)        
        %------------------------------------------------------------------
        function prop = validatePositiveProperty(obj, arg, name)
            validateattributes(arg, {'single', 'double'}, ...
                {'real', 'finite', 'scalar','positive'}, name, class(obj));
            prop = arg;
        end
        
        %------------------------------------------------------------------
        function validateVelocities(~, startVelocity, endVelocity, directions, bound)

            % Check consistency between bound velocities and directions
            driving.internal.validation.checkVelocitySignAndBound( ...
                directions(1), startVelocity, 'StartVelocity', bound);
            driving.internal.validation.checkVelocitySignAndBound( ...
                directions(end), endVelocity, 'EndVelocity', bound);
        end
        
        %------------------------------------------------------------------
        function isNew = isPathNew(obj, directions, cumLengths, curvatures, startVelocity, endVelocity)
            isNew = isequal(cumLengths,    obj.LastCumLengths)    && ...
                    isequal(startVelocity, obj.LastStartVelocity) && ...
                    isequal(endVelocity,   obj.LastEndVelocity)   && ...
                    isequal(directions,    obj.LastDirections)    && ...
                    isequal(curvatures,    obj.LastCurvatures);
        end
    end
end

