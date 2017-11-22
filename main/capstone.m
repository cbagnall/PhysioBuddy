y = [0,-10,-20,-30,-40,-50,-60,-70,-80,-90]; % ideal angles (degree)
x = [0,-17,-29,-45,-59,-70,-79,-84,-88,-90]; % actual angles (degree)

z = polyfit(x,y,2); % raw data function
zv = polyval(z,x); % 0 to -90 degree data (interval: 10 degrees)
y1 = polyfit(x,x,2); % ideal curve
c = y1 - z % offset curve
cv = polyval(c,x) % 0 to -90 degree of offset data (interval: 10 degrees)

figure(1);
plot(x,zv,'r--') %% curve from sensor
hold on
plot(x,x) %% ideal curve
plot(x,cv,'b--') %% offset curve

d = z + c; % add offset to the sensor data, should achieve ideal 
dv = polyval(d,x); % 10 data points

figure(2);
plot(dv,x);



