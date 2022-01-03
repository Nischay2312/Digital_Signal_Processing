%{
const_amplitude = 0.04728033115;
a1 = 3;
a2 = -3;
a3 = 1;
b0 = 1 .* const_amplitude;
b1 = 3 .* const_amplitude;
b2 = 3 .* const_amplitude;
b3 = 1 .* const_amplitude;
%}

a1 = 0.359398533213;
b0 = 0.320300733393;
b1 = b0;

for c = 1: 16184
    number(c) = c;
    filtered_data(c) = 0;
end
number = number';
filtered_data = filtered_data';
plot(number, data.Sensor_Data)

xlim([0 16184]);
title("Raw Sensor Data");
filtered_data(1) = 216.168;
%filtered_data(2) = 217.9240;
%filtered_data(3) = 217.9412;
for c = 2: 16184
    %filtered_data(c) = a1.*filtered_data(c - 1) + a2.*filtered_data(c-2) + a3.*filtered_data(c - 3) + b0.*data.Sensor_Data(c) + b1.*data.Sensor_Data(c - 1) + b2.*data.Sensor_Data(c - 2) + b3.*data.Sensor_Data(c - 3); 
    filtered_data(c) = a1.*filtered_data(c-1) + b0.*data.Sensor_Data(c) + b1.*data.Sensor_Data(c-1);
end
figure
plot(number, filtered_data); hold on; plot(number, data.Sensor_Data);
xlim([0 1500]);
ylim([0 250]);
title("Filtered Sensor Data vs Raw Data");
legend('filtered', 'raw');

%Code for FFT
%Task 5F Harmonics
X = number;
Yy = data.Sensor_Data;

tabq = [X Yy];
%plot(X,Y)


Fs = 1/(50e-3);
T = 1/Fs;
wind = 1:round(4/(60*1e-3*(tabq(2,1)-tabq(1,1))));
L = length(tabq(wind,1));
t = (0:L-1)*T;

Y = fft(tabq(wind,2));

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;

figure
plot(f,P1)
title('Single-Sided Amplitude Spectrum of I(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')
xlim([0, 500]);

