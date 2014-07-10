function points = propagateDoubleIntegrator (from, control, steps, stepsize)

state = from;
points = zeros(steps+1,4);
points(1,:) = state;
for i = 1:steps
     state = [ state(1) + state(3)*stepsize+control(1)/2*stepsize^2, ...
               state(2) + state(4)*stepsize+control(2)/2*stepsize^2, ...
               state(3) + control(1)*stepsize, ...
               state(4) + control(2)*stepsize ];
     points(i+1,:) = state;
end