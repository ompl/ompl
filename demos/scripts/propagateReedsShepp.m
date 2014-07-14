function points = propagateReedsShepp (from, control, stepsize)

state = from;
points(1,:) = state;

cc = 2;
for j = 0:4
    idx = 3*j + 1;
    if control(idx) ~= 0
        steps = round(control(idx+2)/stepsize);
        for i = 1:steps
             state = [ state(1) + control(idx)*stepsize*cos(state(3)), ...
                       state(2) + control(idx)*stepsize*sin(state(3)), ...
                       state(3) + control(idx)*control(idx+1)*stepsize ];
             points(cc,:) = state;
             cc = cc +1;
        end
    else
        break;
    end
end