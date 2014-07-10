clear all; close all;

ttree = load ('../../../build/Release/fmtree.txt');

tree.size = ttree(1,1)-1; % Removing the start.
tree.stepsize = ttree(1,2);
tree.start = ttree(2,1:4);
tree.goal = [8, 8, 0, 0];
tree.state = ttree(3:end,1:4);
tree.parent = ttree(3:end,5:8);
tree.control = ttree(3:end,9:10);
tree.steps = ttree(3:end,11);

clear ttree;

figure;

plot(tree.start(1),tree.start(2), 'mx');
quiver(tree.start(1),tree.start(2),tree.start(3),tree.start(4),'g');
hold on;
plot(tree.goal(1),tree.goal(2), 'mo');
quiver(tree.goal(1),tree.goal(2),tree.goal(3),tree.goal(4),'g');
plot(tree.state(:,1),tree.state(:,2), 'r.')

for i = 1 : tree.size
    p = propagateDoubleIntegrator(tree.parent(i,:), ...
                                  tree.control(i,:), tree.steps(i), ...
                                  tree.stepsize);
    
    plot(p(:,1), p(:,2));
end
quiver(tree.state(:,1),tree.state(:,2),tree.state(:,3),tree.state(:,4),'g');
axis([-11 11 -11 11]);