function voltage_mV = poll_voltage_ina219(nxt, port, i2c_addr)

    %% read bus voltage register
    reg_addr = hex2dec('02'); % Bus voltage register address
    num_bytes = 2;            % Number of bytes in the register
    shift = -3;               % Last 3 bits in this register are unused
    format = 'int16';         % Register stores this value as a signed int
    scale = 4;                % Voltage LSB = 4mV
    raw = i2c_read_value(nxt, port, i2c_addr, reg_addr, num_bytes, shift, format);
    voltage_mV = double(raw) * scale;
    
end
