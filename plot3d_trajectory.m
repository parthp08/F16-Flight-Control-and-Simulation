% plot the aircraft trajectory from simulation
x = out.pos.Data(:,1);
y = out.pos.Data(:,2);
h = out.pos.Data(:,3);
figure(1);
plot3(x,y,h);
xlabel('Xe (in ft)');
ylabel('Ye (in ft)');
zlabel('h (in ft)');
hold on;

