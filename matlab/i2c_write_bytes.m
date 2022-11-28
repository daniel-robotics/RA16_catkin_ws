function i2c_write_bytes(nxt, port, i2c_addr, reg_addr, byte_arr)
    i2c_addr = bitshift(uint8(i2c_addr), 1);  % correct address is shifted << 1
    reg_addr = uint8(reg_addr);  
    byte_arr = [i2c_addr;   ...
                reg_addr;   ...
                reshape(byte_arr, [numel(byte_arr),1]) ];
    NXT_LSWrite(port, 0, byte_arr, 'dontreply', nxt);

    % Block until i2c bus is ready for another transaction
    startTime = clock();
    timeOut = 1; % in seconds
    status = -1; % initialize
    while (status ~= 0) && (etime(clock, startTime) < timeOut)
        [dontcare, status] = NXT_LSGetStatus(port, nxt);
    end%while
end
