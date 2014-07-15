clear all; close all;

iitree = load ('../../../build/Release/fmtree_initial.txt');
itree.size = iitree(1,1);
itree.stepsize = iitree(1,2);
itree.state = iitree(2:end,:);

ttree = load ('../../../build/Release/fmtree.txt');
tree.size = ttree(1,1)-1; % Removing the start.
tree.stepsize = ttree(1,2);
tree.start = ttree(2,1:3);
tree.goal = [4, 1, pi/2];
tree.state = ttree(2:end,1:3);
tree.parent = ttree(2:end,4:6);
tree.control = ttree(2:end,7:end);

clear ttree;
clear iitree;

figure;
hold on;

% Initial sampled states and their velocities.
plot(itree.state(:,1),itree.state(:,2), '.r');
quiver(itree.state(:,1),itree.state(:,2),cos(itree.state(:,3)),sin(itree.state(:,3)),'r');

% Final computed states, start and goal.
plot(tree.state(:,1),tree.state(:,2), 'g.');
plot(tree.start(1),tree.start(2), 'mx', 'MarkerSize', 15, 'LineWidth', 3);
plot(tree.goal(1),tree.goal(2), 'mo', 'MarkerSize', 15, 'LineWidth', 3);

% Propagation of steps.
%for i = 1 : tree.size
for i = 1 : size(tree.parent,1)
    p = propagateReedsShepp(tree.parent(i,:), ...
                           tree.control(i,:), tree.stepsize);
    
    plot(p(:,1), p(:,2));
end

% Final computed velocities.
quiver(tree.state(:,1),tree.state(:,2),cos(tree.state(:,3)),sin(tree.state(:,3)),'g');
%axis([-11 11 -11 11]);