%generateVelocityProfile Generate a velocity profile for a reference path
%
%   This function is for internal use only. It may be removed in the future.
%
%   [velocities, times]=driving.internal.planning.generateVelocityProfile(directions, ...
%   cumLengths, curvatures, startVelocity, endVelocity) generates a velocity
%   profile, velocities, given the cumulative path length, cumLengths, the 
%   path curvatures, curvatures, the driving directions at each point of 
%   the path, directions, the start velocity, startVelocity, and the end 
%   velocity, endVelocity.
%   
%   [...] = driving.internal.planning.generateVelocityProfile(..., Name, Value)   
%   specifies additional name-value pair arguments as described below:
%
%   'MaxSpeed'                  - A positive number specifying the maximum
%                                 allowable speed on the reference path. 
%               
%                                 Default: 1 (m/s)
% 
%   'MaxLateralAccel'           - A positive scalar specifying the maximum
%                                 allowable lateral acceleration.
%   
%                                 Default: 1 (m/s^2)
%
%   'MaxLongitudinalAccel'      - A positive scalar specifying the maximum
%                                 allowable longitudinal acceleration.
%
%                                 Default: 3 (m/s^2)
%
%   'MaxLongitudinalDecel'      - A positive scalar specifying the maximum
%                                 allowable longitudinal deceleration.
%
%                                 Default: 6 (m/s^2)
%
%   'MaxLongitudinalJerk'       - A positive scalar specifying the maximum
%                                 allowable longitudinal jerk. The jerk
%                                 of the generated velocity profile is 
%                                 constrained in the range of 
%                                 [-MaxLongitudinalJerk, MaxLongitudinalJerk].
%
%                                 Default: 1 (m/s^3)
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
%   Example: Generate a velocity profile for a smooth path 
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
%   % Generate velocity profile for the smooth path. The vehicle starts and
%   % end with zero velocity
%   velocities = driving.internal.planning.generateVelocityProfiler(directions, cumLengths, curvatures, 0, 0);
%  
%   % Plot velocity profile along the path
%   plot(cumLengths, velocities)

%   Copyright 2019 The MathWorks, Inc.

%#codegen

function [velocities, times] = generateVelocityProfile( ...
    directions, cumLengths, curvatures, startVelocity, endVelocity, varargin)

narginchk(5, 15);

% Parse parameters
% NOTE: input validation is not performed here
[maxSpeed,maxLatAccel,maxLonAccel,maxLonDecel,maxLonJerk] = parseInputs(varargin{:});

% Divide the path to segments based on driving direction
indexOffset = 1;
[segStartIndex, segEndIndex] = driving.internal.planning.getSegmentBoundaryPointIndex(directions, indexOffset);

% Set the boundary speeds of all the segments
numSegments     = size(segStartIndex, 1);
% Use absolute values of boundary velocities in computation
segStartSpeeds  = cast([abs(startVelocity), zeros(1, numSegments-1)], 'like', directions);
segEndSpeeds    = cast([zeros(1, numSegments-1), abs(endVelocity)], 'like', directions);

% Output datatype is the same as the first input argument
velocities      = zeros(size(cumLengths), 'like', directions);
times           = zeros(size(cumLengths), 'like', directions);

% Generate velocity profile for each segment of path
tPreviousSegment = cast(0, 'like', directions);

for j = 1:numSegments
    segCumLengths  = cumLengths (segStartIndex(j):segEndIndex(j));
    
    validateattributes(segCumLengths, {'double', 'single'}, {'increasing'}, ...
        'driving.internal.planning.VelocityProfiler', 'CumulativeLengths along a segment');
    
    segCurvatures = curvatures(segStartIndex(j):segEndIndex(j));
    segDirections = directions(segStartIndex(j):segEndIndex(j));
    
    direction = segDirections(1);
    vStart    = segStartSpeeds(j);
    vEnd      = segEndSpeeds(j);
    
    % Assign to the corresponding segment
    [velocities(segStartIndex(j):segEndIndex(j)), times(segStartIndex(j):segEndIndex(j))] = ...
        generateSegmentVelocityProfile(vStart, vEnd, segCumLengths, segCurvatures, direction, ...
        maxSpeed,maxLatAccel,maxLonAccel,maxLonDecel,maxLonJerk);
    
    % Time is monotonic across segments
    times(segStartIndex(j):segEndIndex(j)) = times(segStartIndex(j):segEndIndex(j)) + tPreviousSegment;
    
    % Update previous segment total time
    tPreviousSegment = times(segEndIndex(j));
end

end

%------------------------------------------------------------------
function [velocities, times] = generateSegmentVelocityProfile(vStart, vEnd, segCumLengths, segCurvatures, direction, ...
    maxSpeed,maxLatAccel,maxLonAccel,maxLonDecel,maxLonJerk)
%generateSegmentVelocityProfile Generate a velcoity profile for
%   a path segment

% Maximum speed needs to satisfy lateral acceleration constraint
vMax = min(maxSpeed, min(sqrt(maxLatAccel./ abs(segCurvatures))));

% Run-time check of velocity inputs
coder.internal.errorIf(vStart>vMax || vEnd>vMax, ...
    'driving:VelocityProfiler:maxLatAccelTooSmall');

% In reverse motion, flip acceleration and deceleration
if direction == 1
    accelMax = maxLonAccel;
    decelMax = maxLonDecel;
else
    accelMax = maxLonDecel;
    decelMax = maxLonAccel;
end

% Calculate the minimum distance that is kinematically feasible
if vStart < vEnd
    minDist = getNonConstSpeedIntervalDistance(vStart, vEnd, accelMax, maxLonJerk, 'accel');
else
    minDist = getNonConstSpeedIntervalDistance(vEnd, vStart, decelMax, maxLonJerk, 'decel');
end
coder.internal.errorIf(segCumLengths(end) < sum(minDist), ...
    'driving:VelocityProfiler:pathTooShort');

% If above the minimum distance, check if vMax can be reached
% within the distance. If not, find a new maximum speed.
sAccel = getNonConstSpeedIntervalDistance(vStart, vMax, accelMax, maxLonJerk, 'accel');
sDecel = getNonConstSpeedIntervalDistance(vEnd,   vMax, decelMax, maxLonJerk, 'decel');

if (sum(sAccel) + sum(sDecel)) > segCumLengths(end)
    vMax = calculateNewMaximumSpeed(vStart, vEnd, accelMax, decelMax, maxLonJerk, segCumLengths(end));
end

% Given the updated vMax, continue with the normal computation

% Distance in intervals 1-3: acceleration intervals
[sAccel, tAccel, vAccel] = getNonConstSpeedIntervalDistance(vStart, vMax, accelMax, maxLonJerk, 'accel');
% Distance in intervals 5-7: deceleration intervals
[sDecel, tDecel, vDecel] = getNonConstSpeedIntervalDistance(vEnd,   vMax, decelMax, maxLonJerk, 'decel');

% Distance in interval 4: constant speed
sConst = segCumLengths(end) - (sum(sAccel) + sum(sDecel));

% Calculate kinematics at interval boundaries
tPoints = cumsum([0 tAccel sConst/vMax tDecel]);
vPoints = [vStart vAccel vDecel vEnd];
% Acceleration [0 accelMax accelMax 0 0 -decelMax -decelMax 0]
sPoints = cumsum([0 sAccel sConst sDecel]);
sPoints(end) = segCumLengths(end); % Avoid numerical issue

[speeds, times] = arrayfun(@(x) querySpeedByDistance( ...
    x, sPoints, accelMax, decelMax, vMax, vPoints, tPoints, ...
    maxLonJerk), segCumLengths);
velocities = speeds * direction;
end

%------------------------------------------------------------------
function [vInterp, tInterp, aInterp] = querySpeedByDistance( ...
    s, sPoints, accelMax, decelMax, vMax, vPoints, tPoints, maxLonJerk)
%querySpeedByDistance Find the speed given the distance from
%   the starting point

% Determine which interval the specified distance is located in
if s == sPoints(8)
    intervalIndex = 7;
else
    intervalIndex = find(sPoints(2:end)>s, 1);
end

% See Algorithm section in help for the definition of each
% intervals
if intervalIndex == 1 % Accelerate from starting velocity to
    % reach maximum longitudinal acceleration
    distToStart = s;
    p = [1/6*maxLonJerk, 0, vPoints(1), -distToStart];
    t = computeTimeFromDistance(p, tPoints(2));
    tInterp = t;
    vInterp = 1/2*maxLonJerk*t^2 + vPoints(1);
    aInterp = maxLonJerk*t;
elseif intervalIndex == 2 % Accelerate at maximum longitudinal acceleration
    distToStart = s - sPoints(2);
    vInterp = real(sqrt(2*distToStart*accelMax + vPoints(2)^2));
    tInterp = (vInterp - vPoints(2))/accelMax + tPoints(2);
    aInterp = accelMax;
elseif intervalIndex == 3 % Decrease acceleration to zero and reach the maximum speed
    distToEnd = sPoints(4) - s;
    p = [1/6*maxLonJerk, 0, -vMax, distToEnd];
    t = computeTimeFromDistance(p, tPoints(4));
    vInterp = vPoints(4)-1/2*maxLonJerk*t^2;
    tInterp = tPoints(4)-t;
    aInterp = maxLonJerk*t;
elseif intervalIndex == 4 % Keep constant velocity
    vInterp = vMax;
    tInterp = (s - sPoints(4))/vMax + tPoints(4);
    aInterp = 0;
elseif intervalIndex == 5 % Decelerate to reach maximum longitudinal deceleration
    distToStart = s - sPoints(5);
    p = [1/6*maxLonJerk, 0, -vMax, distToStart];
    t = computeTimeFromDistance(p, tPoints(6));
    vInterp = vPoints(5)-1/2*maxLonJerk*t^2;
    tInterp = tPoints(5)+t;
    aInterp = -maxLonJerk*t;
elseif intervalIndex == 6 % Decelerate at maximum longitudinal deceleration
    distToStart = s - sPoints(6);
    vInterp = real(sqrt(-2*distToStart*decelMax + vPoints(6)^2));
    tInterp = (vPoints(6)-vInterp)/decelMax + tPoints(6);
    aInterp = -decelMax;
else %7 Decrease the deceleration to zero to reach ending velocity
    distToEnd = sPoints(8) - s;
    p = [1/6*maxLonJerk, 0,  vPoints(8), -distToEnd];
    t = computeTimeFromDistance(p, tPoints(8));
    vInterp = 1/2*maxLonJerk*t^2 + vPoints(8);
    tInterp = tPoints(8)-t;
    aInterp = -maxLonJerk*t;
end
end

%------------------------------------------------------------------
function [S, T, V] = getNonConstSpeedIntervalDistance(vBound, vMax, aMax, maxLonJerk, intervalFlag)
%getNonConstSpeedIntervalDistance Calculate the travelled distance
%   during the acceleration or the deceleration interval, i.e., intervals
%   1-3 or 5-7, given the starting or the ending velocity, vBound,
%   the maximum speed, vMax and the maximum acceleration or
%   deceleration, aMax. Before calculating the distance, it needs
%   to determine if vMax can be reached without reaching aMax,
%   i.e., if interval 2 or 6 exists. intervalFlag is used as a flag to
%   distinguish the acceleration interval from the deceleration one.

% A speed threshold below which aMax is not reached
vMaxBase = vBound + aMax^2/maxLonJerk;

if vMaxBase <= vMax % interval 2 or 6 needed
    deltaT1 = cast(aMax/maxLonJerk, 'like', vMaxBase);
    deltaT2 = (vMax - vBound)/aMax - aMax/maxLonJerk;
    deltaT3 = deltaT1;
    v1 = vBound + 1/2*maxLonJerk*deltaT1^2;
    v2 = v1 + deltaT2*aMax;
    
    deltaS1 = deltaT1*vBound + 1/6*maxLonJerk*deltaT1^3;
    deltaS2 = deltaT2/2*(v1+v2);
    deltaS3 = deltaT3*vMax - 1/6*maxLonJerk*deltaT3^3;
else
    aMaxNew = sqrt((vMax - vBound)*maxLonJerk);
    deltaT1 = aMaxNew/maxLonJerk;
    deltaT2 = cast(0, 'like', deltaT1); % interval 2 or 6 disappears
    deltaT3 = deltaT1;
    v1 = vBound + 1/2*maxLonJerk*deltaT1^2;
    v2 = v1;
    
    deltaS1 = deltaT1*vBound + 1/6*maxLonJerk*deltaT1^3;
    deltaS2 = cast(0, 'like', deltaS1);
    deltaS3 = deltaT3*vMax - 1/6*maxLonJerk*deltaT3^3;
end

if strcmp(intervalFlag, 'accel') % acceleration interval
    % Interval time
    T = [deltaT1 deltaT2 deltaT3];
    % Velocity at the interval boundary
    V = [v1 v2 vMax];
    % Interval distance
    S = [deltaS1, deltaS2, deltaS3];
else % deceleration interval
    % Flip the order
    T = [deltaT3 deltaT2 deltaT1];
    V = [vMax v2 v1];
    S = [deltaS3, deltaS2, deltaS1];
end
end

%------------------------------------------------------------------
function vMax = calculateNewMaximumSpeed(vStart, vEnd, accelMax, decelMax, maxLonJerk, distance)
%calculateNewMaximumSpeed For given distance, calculate the
%   maximum possible speed

% Consider the case where all the seven intervals are valid by
% solving a quadratic equation: A*vMax^2 + B*vMax + C = 0
A = (accelMax+decelMax)/(2*accelMax*decelMax);
B = (accelMax+decelMax)/(2*maxLonJerk);
C = (accelMax*vStart+decelMax*vEnd)/(2*maxLonJerk) - ...
    (vStart^2/(2*accelMax) + vEnd^2/(2*decelMax)) - distance;
sol = (sqrt(B^2-4*A*C) - B)/(2*A); % Positive solution

% Check if the solution is valid
isValidSolution = isreal(sol) && sol >= vStart + accelMax^2/maxLonJerk && ...
    sol >= vEnd + decelMax^2/maxLonJerk;
if isValidSolution
    vMax = sol;
else
    % Search for a new speed starting from max(vEnd, vStart) as
    % analytic solution is intractable
    vMaxCandidate = max(vEnd, vStart);
    
    scale    = 0.1; % percentage
    minStep  = 0.1; % 0.1 m/s
    stepSize = max(min(vMaxCandidate*scale, minStep), minStep);
    solFound = false;
    while ~solFound
        vMaxCandidate = vMaxCandidate+stepSize;
        sAccel = getNonConstSpeedIntervalDistance(vStart, vMaxCandidate, accelMax, maxLonJerk, 'accel');
        sDecel = getNonConstSpeedIntervalDistance(vEnd,   vMaxCandidate, decelMax, maxLonJerk, 'decel');
        solFound =  sum(sAccel) + sum(sDecel) > distance;
    end
    vMax  = vMaxCandidate - stepSize;
end
end

%------------------------------------------------------------------
function t = computeTimeFromDistance(p, tUpper)
%computeTimeFromDistance Solve a cubic equation to get time
%   p(1)*t^3+p(2)*t^2+p(3)*t+p(4) = 0

r   = roots(p);
t   = tUpper;

% Robust numerical comparison to find the smallest positive
% real root to avoid limitation in codegen
for k = 1:numel(r)
    rk = real(r(k));
    if rk >= 0 && rk < t && abs(imag(r(k))) <= sqrt(eps(rk))
        t = rk;
    end
end
end

%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% Parse parameters
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
function [maxSpeed, maxLatAccel, maxLonAccel, maxLonDecel, maxLonJerk] = parseInputs(varargin)
%parseInputs Parse parameters for both simulation and codegen 

% Define default values
defaultParams = struct( ...
    'MaxSpeed', 10, ...
    'MaxLateralAccel', 1, ...
    'MaxLongitudinalAccel', 3, ...
    'MaxLongitudinalDecel', 6, ...
    'MaxLongitudinalJerk', 1);

% Parse inputs
if isempty(coder.target)  % Simulation
    r = parseOptionalInputsSimulation(defaultParams, varargin{:});
else % Code generation
    r = parseOptionalInputsCodegen(defaultParams, varargin{:});
end

% Extract parameters
maxSpeed    = r.MaxSpeed;
maxLatAccel = r.MaxLateralAccel;
maxLonAccel = r.MaxLongitudinalAccel;
maxLonDecel = r.MaxLongitudinalDecel;
maxLonJerk  = r.MaxLongitudinalJerk;
end

%--------------------------------------------------------------------------
function r = parseOptionalInputsSimulation(defaultParams, varargin)
%parseOptionalInputsSimulation Parse parameters for simulation workflow

% Instantiate an input parser
parser = inputParser;

% Specify the optional parameters
parser.addParameter('MaxSpeed',             defaultParams.MaxSpeed);
parser.addParameter('MaxLateralAccel',      defaultParams.MaxLateralAccel);
parser.addParameter('MaxLongitudinalAccel', defaultParams.MaxLongitudinalAccel);
parser.addParameter('MaxLongitudinalDecel', defaultParams.MaxLongitudinalDecel);
parser.addParameter('MaxLongitudinalJerk',  defaultParams.MaxLongitudinalJerk);

% Parse
parser.parse(varargin{:});
r = parser.Results;
end

%--------------------------------------------------------------------------
function r = parseOptionalInputsCodegen(defaultParams, varargin)
%parseOptionalInputsCodegen Parse parameters for codegen workflow

parms = struct( ...
    'MaxSpeed', uint32(10), ...
    'MaxLateralAccel', uint32(1), ...
    'MaxLongitudinalAccel', uint32(3), ...
    'MaxLongitudinalDecel', uint32(6), ...
    'MaxLongitudinalJerk', uint32(1));

popt = struct( ...
        'CaseSensitivity', false, ...
        'StructExpand',    true, ...
        'PartialMatching', false);

optarg = coder.internal.parseParameterInputs(parms, popt, varargin{:});

maxSpeed = coder.internal.getParameterValue(optarg.MaxSpeed,...
    defaultParams.MaxSpeed, varargin{:});

maxLatAccel = coder.internal.getParameterValue(optarg.MaxLateralAccel,...
    defaultParams.MaxLateralAccel, varargin{:});

maxLonAccel = coder.internal.getParameterValue(optarg.MaxLongitudinalAccel,...
    defaultParams.MaxLongitudinalAccel, varargin{:});

maxLonDecel = coder.internal.getParameterValue(optarg.MaxLongitudinalDecel,...
    defaultParams.MaxLongitudinalDecel, varargin{:});

maxLonJerk = coder.internal.getParameterValue(optarg.MaxLongitudinalJerk,...
    defaultParams.MaxLongitudinalJerk, varargin{:});

r = struct( ...
    'MaxSpeed', maxSpeed, ...
    'MaxLateralAccel', maxLatAccel, ...
    'MaxLongitudinalAccel', maxLonAccel, ...
    'MaxLongitudinalDecel', maxLonDecel, ...
    'MaxLongitudinalJerk', maxLonJerk);
end