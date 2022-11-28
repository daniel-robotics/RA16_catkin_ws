function power_mW = poll_power_ina219(nxt, port, i2c_addr)
      
    %% read power register
    reg_addr = hex2dec('03'); % power register
    num_bytes = 2;            % number of bytes in the register
    shift = 0;                % Whole register is used to hold the value
    format = 'int16';         % Register stores this value as a signed int
    scale = 2;                % Power LSB = 2mW
    raw = i2c_read_value(nxt, port, i2c_addr, reg_addr, num_bytes, shift, format);
    power_mW = double(raw) * scale;

end
