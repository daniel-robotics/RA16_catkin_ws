function set_power(nxt, motorport, power)
    running = true;
    braking = false;
    if power == 0
        running = false;
        braking = true;
    end

    NXT_SetOutputState(motorport, power, running, ~braking, ...
        'IDLE', 0, 'RUNNING', 0, 'dontreply', nxt);
end

