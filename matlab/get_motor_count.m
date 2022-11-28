function count = get_motor_count(nxt, motorport)

    motor_state = NXT_GetOutputState(motorport, nxt); 
    count = motor_state.BlockTachoCount;
end

