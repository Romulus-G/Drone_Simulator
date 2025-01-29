close all;

p = load("p.mat").ans;
q = load("q.mat").ans;

t = p(1,:);
p = p(2:end,:)';
q = q(2:end,:)';

p1 = p(:,1);
p2 = p(:,2);    
p3 = p(:,3);

f1 = figure;
% ax = [subplot(2,2,1), subplot(2,2,2), subplot(2,2,3), subplot(2,2,4)];

h = tiledlayout(2,2, Padding="compact", TileSpacing="compact");
ax = [nexttile, nexttile, nexttile, nexttile];

grid(ax, "on");
axis(ax, "equal");
set(ax, 'ZDir','reverse');
set(ax, 'YDir','reverse');

d = 0.6;
xlim(ax, [min(p1) - d, max(p1) + d]); 
ylim(ax, [min(p2) - d, max(p2) + d]);
zlim(ax, [min(p3) - d, max(p3) + d]);

xlabel(ax, 'X[m]'); ylabel(ax, 'Y[m]'); zlabel(ax, 'Z[m]');
hold(ax, 'on');

view(ax(1), 0, 0);
view(ax(2), 90, 0);
view(ax(3), 0, 90);
view(ax(4), 68, 53);

for i = 1:4
    drone = make_drone(ax(i));
    ax_M_drone(i) = hgtransform(Parent=ax(i));
    drone_M_mesh(i) = hgtransform(Parent=ax_M_drone(i), Matrix=makehgtform('xrotate', pi));
    set(drone, Parent=drone_M_mesh(i));
    objective = plot3(ax(i), 0, 0, 0, 'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'b');
    trajectory(i) = plot3(ax(i), NaN, NaN, NaN, 'b-', LineWidth=1);
end
   
dt = diff(t);
for i = 1:length(dt)
    set(trajectory, XData=p1(1:i), YData=p2(1:i), ZData=p3(1:i));
    set(ax_M_drone, Matrix = trvec2tform(p(i,:)) * quat2tform(q(i,:)));
    drawnow
    pause(dt(i));
    title(ax, t(i));
end


function drone = make_drone(ax)
    % Written by Jitendra Singh
    %% Define design parameters
    D2R = pi/180;
    R2D = 180/pi;
    b   = 0.6;     % the length of total square cover by whole body of quadcopter in meter
    a   = b/3;     % the legth of small square base of quadcopter(b/4)
    H   = 0.06;    % height of drone in Z direction (4cm)
    H_m = H + H/2; % height of motor in z direction (5 cm)
    r_p = b/4;     % radius of propeller
    
    %% Conversions
    ro = 45*D2R;                   % angle by which rotate the base of quadcopter
    Ri = [cos(ro) -sin(ro) 0;
          sin(ro) cos(ro)  0;
           0       0       1];     % rotation matrix to rotate the coordinates of base 
    base_co = [-a/2  a/2 a/2 -a/2; % Coordinates of Base 
               -a/2 -a/2 a/2 a/2;
                 0    0   0   0];
    base = Ri*base_co;             % rotate base Coordinates by 45 degree 
    to = linspace(0, 2*pi);
    xp = r_p*cos(to);
    yp = r_p*sin(to);
    zp = zeros(1,length(to));

    %% Design Different parts
    % design the base square
     drone(1) = patch(ax, [base(1,:)],[base(2,:)],[base(3,:)],'r');
     drone(2) = patch(ax, [base(1,:)],[base(2,:)],[base(3,:)+H],'r');
     alpha(drone(1:2),0.7);
    % design 2 perpendicular legs of quadcopter 
     [xcylinder, ycylinder, zcylinder] = cylinder([H/2 H/2]);
     drone(3) =  surface(ax, b*zcylinder-b/2,ycylinder,xcylinder+H/2,'facecolor','b');
     drone(4) =  surface(ax, ycylinder,b*zcylinder-b/2,xcylinder+H/2,'facecolor','b') ; 
     alpha(drone(3:4),0.6);
    % design 4 cylindrical motors 
     drone(5) = surface(ax, xcylinder+b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
     drone(6) = surface(ax, xcylinder-b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
     drone(7) = surface(ax, xcylinder,ycylinder+b/2,H_m*zcylinder+H/2,'facecolor','r');
     drone(8) = surface(ax, xcylinder,ycylinder-b/2,H_m*zcylinder+H/2,'facecolor','r');
     alpha(drone(5:8),0.7);
    % design 4 propellers
     drone(9)  = patch(ax, xp+b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
     drone(10) = patch(ax, xp-b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
     drone(11) = patch(ax, xp,yp+b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
     drone(12) = patch(ax, xp,yp-b/2,zp+(H_m+H/2),'p','LineWidth',0.5);

     drone(13) = quiver3(ax, 0,0,0, 0.7,0,0, LineWidth=2, MaxHeadSize=2, Color='r');

     alpha(drone(9:12),0.3);
end