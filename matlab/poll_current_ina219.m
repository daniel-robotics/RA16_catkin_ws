function current_mA = poll_current_ina219(nxt, port, i2c_addr)
  
    %% read current register
    reg_addr = hex2dec('04'); % current register
    num_bytes = 2;            % number of bytes in the register
    shift = 0;                % Whole register is used to hold the value
    format = 'int16';         % Register stores this value as a signed int
    scale = 0.1;              % Current LSB = 100uA
    raw = i2c_read_value(nxt, port, i2c_addr, reg_addr, num_bytes, shift, format);
    current_mA = double(raw) * scale;

end
