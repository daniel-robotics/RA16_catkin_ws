function byte_arr = i2c_read_bytes(nxt, port, i2c_addr, reg_addr, num_bytes)
    i2c_addr = bitshift(uint8(i2c_addr), 1);  % correct address is shifted << 1
    reg_addr = uint8(reg_addr);  
    byte_arr = COM_ReadI2C(port, num_bytes, i2c_addr, reg_addr, nxt);
end