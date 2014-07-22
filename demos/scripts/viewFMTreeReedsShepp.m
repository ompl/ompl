clear all; %close all;

iitree = load ('../../build/Release/fmtree_initial.txt');
itree.size = iitree(1,1);
itree.stepsize = iitree(1,2);
itree.states = iitree(2:end,:);

ttree = load ('../../build/Release/fmtree.txt');
tree.size = ttree(1,1); % Removing the start.
tree.stepsize = ttree(1,2);
tree.start = ttree(2,1:3);
tree.goal = [4, 1, pi/2];
tree.states = ttree(2:end,1:3);
tree.parents = ttree(2:end,4:6);
tree.controls = ttree(2:end,7:end);

clear ttree;
clear iitree;

figure(1);
clf;
hold on;

% TODO: some green states are translated with respect to its corresponding,
% propagated position.

% Initial sampled states and their velocities.
% % % plot(itree.states(:,1),itree.states(:,2), '.r');
% % % quiver(itree.states(:,1),itree.states(:,2),cos(itree.states(:,3)),sin(itree.states(:,3)),'r');

% Final computed states, start and goal.
plot(tree.states(:,1),tree.states(:,2), 'g.');
plot(tree.start(1),tree.start(2), 'mx', 'MarkerSize', 15, 'LineWidth', 3);
plot(tree.goal(1),tree.goal(2), 'mo', 'MarkerSize', 15, 'LineWidth', 3);

% Propagation of steps.
for i = 1 : size(tree.parents,1)
    p = propagateReedsShepp(tree.parents(i,:), ...
                           tree.controls(i,:), tree.stepsize);
    
    plot(p(:,1), p(:,2));
end

% Final computed velocities.
quiver(tree.states(:,1),tree.states(:,2),cos(tree.states(:,3)),sin(tree.states(:,3)),'g');

% Plotting path
tpath = load ('../../build/Release/fmtpath.txt');
path.states = tpath(2:end,1:3);
path.controls = tpath(2:end,4:end);
path.stepsize = tpath(1,3);

pathstates =[];
% Propagation of steps.
for i = 1 : size(path.states,1)-1
    pathstates = [pathstates; propagateReedsShepp(path.states(i,:), ...
                                      path.controls(i,:), path.stepsize)];

end
plot(pathstates(:,1), pathstates(:,2), 'm', 'LineWidth', 2);
 
% Sum of euclidean distances between states of the path..
d = pdist(path.states);
n = size(path.states,1);

cc = 1;
for i = 1:n-2
   cc(i+1) = cc(i) + n-i;
end

sum(d(cc))

% Approximation of the real path length
rdist = 0;
for i = 1:size(pathstates,1)-1
   rdist = rdist + pdist([pathstates(i,:); pathstates(i+1,:)]);
end
rdist

