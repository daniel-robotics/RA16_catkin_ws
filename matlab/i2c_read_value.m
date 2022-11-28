function value = i2c_read_value(nxt, port, i2c_addr, reg_addr, num_bytes, shift, format)
    byte_arr = i2c_read_bytes(nxt, port, i2c_addr, reg_addr, num_bytes);
    switch num_bytes
        case 1
            value = typecast(byte_arr, 'uint8');
        case 2
            value = swapbytes(typecast(byte_arr, 'uint16'));
        case 4
            value = swapbytes(typecast(byte_arr, 'uint32'));
        case 8
            value = swapbytes(typecast(byte_arr, 'uint64'));
    end
    value = bitshift(value, shift);
    value = typecast(value, format);
end