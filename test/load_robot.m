% Bravo7 workspace calculation
% Marcus Rosette
% 12/08/2022


% Create Robot Representation
bravo = importrobot('bravo7_unchanged.urdf');
bravo.DataFormat = 'column';

% show(bravo,'visuals','off','collision','on')

% Generate Trajectory and Check for Collisions
startConfig = [0, 0, 0, 0, 0, 0, 0, 0, 0]';
goalConfig = [0, 0, pi, 0, 0, 0, 0, 0, 0]';

q = trapveltraj([startConfig goalConfig],100,'EndTime',3);

% Iterate through the traj to see if there are collision using
% checkCollision function
% Turn on exhaustive checking to continue checking for collisions after
% the first is detected.

isConfigInCollision = false(100,1);
configCollisionPairs = cell(100,1);
sepDistForConfig = zeros(bravo.NumBodies+1,bravo.NumBodies+1,100);
for i = 1:length(q)
    [isColliding, sepDist] = checkCollision(bravo,q(:,i),'Exhaustive','on','SkippedSelfCollisions','parent');
    isConfigInCollision(i) = isColliding;
    sepDistForConfig(:,:,i) = sepDist;
end

% Find the indices of the bodies in collision
for i = 1:length(q)
    sepDist = sepDistForConfig(:,:,i);
    [body1Idx,body2Idx] = find(isnan(sepDist));

    collidingPairs = unique(sort([body1Idx,body2Idx],2));
    configCollisionPairs{i} = collidingPairs;
end

any(isConfigInCollision)

firstCollisionIdx = find(isConfigInCollision,1);

% Visualize the first configuration that is in collision.
figure;
show(bravo,q(:,firstCollisionIdx));
% exampleHelperHighlightCollisionBodies(bravo,configCollisionPairs{firstCollisionIdx}+1,gca);


% newStartConfig = wrapToPi(startConfig);
% q = trapveltraj([newStartConfig goalConfig],100,'EndTime',3);
% 
% isConfigInCollision = false(100,1);
% configCollisionPairs = cell(100,1);
% for i = 1:length(q)
%     isColliding = checkCollision(bravo,q(:,i),'Exhaustive','on','SkippedSelfCollisions','parent');
%     isConfigInCollision(i) = isColliding;
% end
% 
% any(isConfigInCollision)