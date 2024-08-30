clear all; close all; clc;

data = readtable('SFO-HighFreqInjSpinNegDir.csv');
data = table2array(data);
data = data(2:end,:);

figure, plot(data(:,1),data(:,2),data(:,1),data(:,3),data(:,1),data(:,4)), grid minor, legend('w_{est}','w_{final,filt}','w_{fb}'), xlabel('[sec]'), ylabel('[Ra/sec]')
figure, plot(data(:,1),data(:,5),data(:,1),data(:,6)), grid minor, legend('\theta_{r,est}','\theta_{r,fb}'), xlabel('time [sec]'), ylabel('[Ra]')
figure, plot(data(:,1),data(:,7),data(:,1),data(:,8)), grid minor, legend('\theta_{s,est}','\theta_{s,fb}'), xlabel('time [sec]'), ylabel('[Ra]')
figure, plot(data(:,1),data(:,9),data(:,1),data(:,10)), grid minor, legend('\delta_{est}','\delta_{fb}'), xlabel('time [sec]'), ylabel('[Ra]')
figure, plot(data(:,1),data(:,11),data(:,1),data(:,12)), grid minor, legend('\lambda_{d,s,est}','\lambda_{d,s,fb}'), xlabel('time [sec]'), ylabel('[Ra]')
figure, plot(data(:,1),data(:,15),data(:,1),data(:,17),data(:,1),data(:,19)), grid minor, legend('i_{q,r,tot}','i_{q,r,hf}','i_{q,r}'), xlabel('[sec]'), ylabel('[A]')
figure, plot(data(:,1),data(:,16),data(:,1),data(:,18),data(:,1),data(:,20)), grid minor, legend('i_{d,r,tot}','i_{d,r,hf}','i_{d,r}'), xlabel('[sec]'), ylabel('[A]')
