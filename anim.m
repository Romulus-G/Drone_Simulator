p = load("p.mat").ans;
q = load("q.mat").ans;

p = p(2:end,:)';
q = q(2:end,:)';

disp(size(p))

drone_animation(p,q);

function drone_animation(p,q)

    %% Define Figure plot
    fig = figure('pos', [0 50 800 600]);
    hg = gca;
    view(68,53);
    grid on;
    axis equal;
    xlim([-3 3]); ylim([-3 3]); zlim([-3 3]);
    title('Drone Animation');
    xlabel('X[m]'); ylabel('Y[m]'); zlabel('Z[m]');
    hold(gca, 'on');
    
    drone = make_drone();
    combinedobject = hgtransform('parent', hg);
    set(drone,'parent',combinedobject)

    plot3(0, 0, 0, 'ro', 'MarkerSize', 3, 'MarkerFaceColor', 'b');

    for i = 1:length(q)
        translation = makehgtform('translate', p(i,:));
        qs = q(i,1);
        qv = q(i,2:4);
        rotation = makehgtform('axisrotate', qv, 2 * atan2(norm(qv), qs));
        set(combinedobject,'matrix', translation * rotation);
        drawnow
        pause(0.01);
    end
end

function drone = make_drone()
    % Written by Jitendra Singh
    %% Define design parameters
    D2R = pi/180;
    R2D = 180/pi;
    b   = 0.6;     % the length of total square cover by whole body of quadcopter in meter
    a   = b/3;     % the legth of small square base of quadcopter(b/4)
    H   = 0.06;    % hight of drone in Z direction (4cm)
    H_m = H + H/2; % hight of motor in z direction (5 cm)
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
     drone(1) = patch([base(1,:)],[base(2,:)],[base(3,:)],'r');
     drone(2) = patch([base(1,:)],[base(2,:)],[base(3,:)+H],'r');
     alpha(drone(1:2),0.7);
    % design 2 perpendicular legs of quadcopter 
     [xcylinder, ycylinder, zcylinder] = cylinder([H/2 H/2]);
     drone(3) =  surface(b*zcylinder-b/2,ycylinder,xcylinder+H/2,'facecolor','b');
     drone(4) =  surface(ycylinder,b*zcylinder-b/2,xcylinder+H/2,'facecolor','b') ; 
     alpha(drone(3:4),0.6);
    % design 4 cylindrical motors 
     drone(5) = surface(xcylinder+b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
     drone(6) = surface(xcylinder-b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
     drone(7) = surface(xcylinder,ycylinder+b/2,H_m*zcylinder+H/2,'facecolor','r');
     drone(8) = surface(xcylinder,ycylinder-b/2,H_m*zcylinder+H/2,'facecolor','r');
     alpha(drone(5:8),0.7);
    % design 4 propellers
     drone(9)  = patch(xp+b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
     drone(10) = patch(xp-b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
     drone(11) = patch(xp,yp+b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
     drone(12) = patch(xp,yp-b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
     alpha(drone(9:12),0.3);
end