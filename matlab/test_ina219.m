addpath('P:/_Libraries/MATLAB/RWTHMindstormsNXT');

port = SENSOR_2;
motor = MOTOR_A;
i2c_addr = hex2dec('45'); % can be: 40, 41, 44, 45
test_end_times=[5, 10, 15];
test_mtr_power=[0, 50, 100];

%% init NXT
if ~exist('nxt','var')
    warning('off','MATLAB:RWTHMindstormsNXT:noEmbeddedMotorControl');
    nxt = COM_OpenNXT();
end
fprintf('NXT connected\n');

%% init peripherals
init_ina219(nxt, port, i2c_addr);
set_motor_power(nxt,motor,0);
reset_motor_count(nxt, motor);
pause(0.1);

%% Start test
t=[];
v_bus=[];
i_bus=[];
p_bus=[];
v_int=[];
angle=[];

fprintf('Running tests...\n');
t_0 = tic;
t_i = 0;
for test = 1:length(test_end_times)
    end_time = test_end_times(test);
    power = test_mtr_power(test);
    set_motor_power(nxt,motor,power);
    fprintf('Test %d: Motor at pwr=%.0f\n', test, power);
    while t_i <= end_time
        t_i = toc(t_0);
        voltage_mV = poll_voltage_ina219(nxt, port, i2c_addr);
        current_mA = poll_current_ina219(nxt, port, i2c_addr);
        power_mW   = poll_power_ina219(nxt, port, i2c_addr);
        batt = NXT_GetBatteryLevel(nxt);
        count = get_motor_count(nxt, motor);

        t(end+1)  = t_i;
        v_bus(end+1)  = voltage_mV/1000;
        i_bus(end+1) = current_mA;
        p_bus(end+1) = power_mW;
        v_int(end+1) = batt/1000;
        angle(end+1) = count;
    end
end
set_motor_power(nxt,motor,0);

fprintf('Plotting results...\n');
yyaxis left;
plot(t, i_bus);
ylabel('Current (mA)');
ylim([0,3200]);

yyaxis right;
plot(t, v_bus, t, v_int);
ylabel('Bus Voltage (V)');
ylim([0, 10]);

xlim([0,test_end_times(end)]);
xlabel('Time (s)');
grid on;
grid minor;

%% Close NXT
% COM_CloseNXT all