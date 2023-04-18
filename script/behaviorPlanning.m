function [planNext, nextGoal, speedProfilerConfig, finalReachedGoal] = ...
    behaviorPlanning(reset, routePlanStruct, vehicleInfo)

%behaviorPlanning generate navigation tasks and configurations for the 
%   Motion Planner block and the Trajectory Generator block.

nextGoal   = coder.nullcopy(zeros(2, 3, 'single')); %#ok<*NASGU>
planNext   = coder.nullcopy(logical(0));  %#ok<*LOGL>

persistent goalIndex finalReached
persistent nextGoalPose speedConfig

if isempty(goalIndex) || reset > 0
    % Initialize pose and speed
    finalReached = logical(0);
    goalIndex = 1;
    nextGoalPose = [routePlanStruct.StartPose(goalIndex,:) ;routePlanStruct.EndPose(goalIndex,:)];
    speedConfig.StartSpeed  = vehicleInfo.CurrVelocity;
    speedConfig.EndSpeed    = routePlanStruct.EndSpeed(goalIndex);
    goalIndex = goalIndex + 1;
end

planNext = logical(0);
if ~finalReached
    % Plan a new path when reaching the goal
    if helperGoalChecker(nextGoalPose(end, :), vehicleInfo.CurrPose, vehicleInfo.CurrVelocity, speedConfig.EndSpeed, vehicleInfo.Direction)
        if goalIndex <= routePlanStruct.ValidNum
            nextGoalPose = [routePlanStruct.StartPose(goalIndex,:) ;routePlanStruct.EndPose(goalIndex,:)];
            speedConfig.StartSpeed  = vehicleInfo.CurrVelocity;
            speedConfig.EndSpeed    = routePlanStruct.EndSpeed(goalIndex);
            goalIndex = goalIndex + 1;
            planNext = logical(1);
        else
            % reach the last goal, clear setting pose and velocity.
            nextGoalPose(:, :) = single(0);
            speedConfig.StartSpeed = single(0);
            speedConfig.EndSpeed = single(0);
            finalReached       = logical(1);
        end
    end
end
    
finalReachedGoal    = finalReached;
nextGoal            = nextGoalPose;
speedProfilerConfig = speedConfig;

end
