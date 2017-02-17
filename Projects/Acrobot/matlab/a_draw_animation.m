function a_draw_animation(theta_1)
global p1 p2

x1 = sin(theta_1);
y1 = cos(theta_1);

Ax = [0,x1];
Ay = [0,y1];
Az = [0,0];

set(p1,'XData',Ax,'YData',Ay,'ZData',Az);


Ax = [x1,x1];
Ay = [y1,y1];
Az = [0,0];

set(p2,'XData',Ax,'YData',Ay,'ZData',Az);

drawnow

 %%Time-delay%%%%%%%
 for i=0:1:10     %%
 end              %%
 %%%%%%%%%%%%%%%%%%%

