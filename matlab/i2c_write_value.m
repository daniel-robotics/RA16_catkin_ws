function i2c_write_value(nxt, port, i2c_addr, reg_addr, val)
    byte_arr = typecast(swapbytes(val), 'uint8');
    i2c_write_bytes(nxt, port, i2c_addr, reg_addr, byte_arr);
end

