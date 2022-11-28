function i2cstatus = init_ina219(nxt, port, i2c_addr)
    % i2c_addr can be: 40, 41, 44, 45
    
    % init i2C bus
    NXT_SetInputMode(port, 'LOWSPEED_9V', 'RAWMODE', 'dontreply', nxt);
    pause(0.1);
    NXT_LSGetStatus(port, nxt);
    [~, ~, i2cstatus] = NXT_LSRead(port, nxt);
    
    % init sensor (must set config and calib register on reset)
    % Bits:  Value:  
    % 15     0x01    Reset
    % 13     0x01    Bus Voltage Range: RANGE_32V
    % 12-11  0x03    Gain: DIV_8_320MV
    % 10-07  0x0B    Bus ADC Resolution: ADCRES_12BIT_8S (4.26ms)
    % 06-03  0x0B    Shunt ADC Resolution: ADCRES_12BIT_8S (4.26ms)
    % 02-00  0x07    Mode: SANDBVOLT_CONTINUOUS
    reg_addr = hex2dec('00'); % config register
    config = uint8(bin2dec(['1011 1101';'1101 1111']));
    i2c_write_bytes(nxt, port, i2c_addr, reg_addr, config);
    
    reg_addr = hex2dec('05'); % calib register
    calib = uint16(4096);     % Up to 3.2A max
    i2c_write_value(nxt, port, i2c_addr, reg_addr, calib);
end

