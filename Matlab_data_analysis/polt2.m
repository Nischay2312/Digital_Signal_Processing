fs = 1000;
coeff = 2/(1/fs);
cutoff = 2.5*2*pi;
a1 = (coeff - cutoff)/(coeff + cutoff);
b0 = cutoff/(coeff+cutoff);
b1 = cutoff/(coeff+cutoff);


t = 1:1:6390;
t = t';
plot(t, data1.Acc_x);
hold on
plot(t, data1.Acc_y);
xlim([0 6390]);
legend('X', 'Y');
hold off

for c = 1:6390
    filtered_x(c) = 0;
    filtered_y(c) = 0;
end

filtered_x = filtered_x';
filtered_y = filtered_y';

for c = 2:6390
   filtered_x(c) = filtered_x(c-1)*a1 + data1.Acc_x(c)*b0 + data1.Acc_x(c-1)*b1; 
   filtered_y(c) = filtered_y(c-1)*a1 + data1.Acc_y(c)*b0 + data1.Acc_y(c-1)*b1;
end

figure
plot(t, data1.Acc_x);
hold on
plot(t, filtered_x);
xlim([0 6390]);
legend('Raw', 'Filtered'); 
hold off

%Code for FFT
%Task 5F Harmonics
X = t;
Yy = data1.Acc_x;

tabq = [X Yy];
%plot(X,Y)


Fs = 1/(1e-3);
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

