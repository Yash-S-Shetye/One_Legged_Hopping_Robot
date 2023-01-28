function draw(y,dim,h)%dim=[z0,bodyLength]
bl=dim(2);
z1=y(1);z2=y(3);th=0;
plot([-2,2],[z2,z2],'b','LineWidth',2);
hold on
plot([0,0],[z1,z2],'r','LineWidth',3);
plot([-2,2],[z1,z1],'k','LineWidth',4);
%plot([-10,10],[h+dim(1),h+dim(1)],'k');


axis([-10,10,0,50]);

hold off
drawnow
end
