function [szOut]=anim

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% anim.m: Fonction d'animation du robot.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% load inputs
load ref;
ref=ans';
save ref.data ref -ascii;
load cart;
cart= ans';
save cart.data cart -ascii;


% Tracé du premier
xyt=[ref(1,2), ref(1,3), ref(1,4),cart(1,2),cart(1,3),cart(1,4)];
draw(xyt,1);
refresh;

% On fait l'animation
cl0=clock;
  for index = 2:1:size(ref,1)
    while  etime(clock,cl0) < ref(index,1),
     etime(clock,cl0);
    end;
    xyt=[ref(index,2), ref(index,3),ref(index,4),cart(index,2),cart(index,3),cart(index,4)];
    draw(xyt,index);
  end;
void=input('Press any key to close the window \n');
close;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [szOut] = draw(x,index)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% draw.m: Fonction qui fait le tracé du robot et du repère de référence
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global hc;

% Definition de la géométrie

L=1;
dref= [x(1) x(2)]';
thetar	= x(3); 
drob= [x(4) x(5)]';
theta	= x(6);
XY	= [ 0 L nan 0  0 nan   -0.5*L  0.5*L nan -0.5*L -0.5*L nan...
          -0.5*L  0.5*L nan 0.5*L L nan L 0.5*L nan -0.3*L 0.3*L nan...
          -0.3*L 0.3*L; 
          0 0 nan -0.7*L 0.7*L nan 0.5*L 0.5*L nan -0.5*L 0.5*L nan...
          -0.5*L -0.5*L nan -0.5*L 0 nan 0 0.5*L nan -0.7*L -0.7*L nan...
          0.7*L 0.7*L];
RMat	= [cos(theta) -sin(theta); sin(theta) cos(theta)];
XYr =[ 0 L nan 0 0 nan L 0.8*L nan L 0.8*L nan  0 0.2*L nan 0 -0.2*L;
       0 0 nan 0 L nan 0 0.2*L nan 0 -0.2*L nan L 0.8*L nan L 0.8*L];
RMatr	= [cos(thetar) -sin(thetar); sin(thetar) cos(thetar)];
for i=1:26
  XY(:,i) = RMat * XY(:,i) + drob;
end;
for i=1:17
  XYr(:,i) = RMatr * XYr(:,i) + dref;
end;

% Tracé de la figure

findobj('Name','Animation Window');
if index==1

  % Initialisation

  close all;
  figure(1);
  clf;
  set(gcf, 'Name', 'Animation Window');
  set(gcf,'Position',[ 10 200 700 700]);
  set(1,'BackingStore','on');
  hold on;

set(gca, 'UserData', hc,'NextPlot', 'add','Visible', 'on','DataAspectRatio', [1 1 1], ...
	  'Color', 'k', 'SortMethod', 'childorder','Xlim',[-10 10],'Ylim',[-10 10]);
 
  hc(1)=plot(XY(1,:),XY(2,:),'g-','Linewidth',2);
  hc(2)=plot(XYr(1,:),XYr(2,:),'r-','Linewidth',3);
 
  drawnow;

else
  set(hc(1),'XData',XY(1,:),'YData',XY(2,:));
  set(hc(2),'XData',XYr(1,:),'YData',XYr(2,:));
  drawnow;

end
