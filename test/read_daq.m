dq = daq("ni");
dq.Rate = 8000;
addinput(dq, "Dev1", "ai0", "Voltage");

% Acquire data for one second at 8000 scans per second.
data = read(dq, seconds(1));

% Plot the data
plot(data.Time, data.Variables);
ylabel("Voltage (V)")