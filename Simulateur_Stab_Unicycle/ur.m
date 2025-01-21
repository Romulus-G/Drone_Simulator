%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fonction that defines the velocities of the reference trajectory from the relation dgr=X(gr)vr 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z]= ur(tps)

 if tps< 3
  vr=0;
  wr=0;
 elseif tps<10
  vr= 4*sigmo(tps,3,10);
  wr= 0;
 elseif tps<15
  vr= -5*sigmo(tps,10,15);
  wr= 0;
 elseif tps<20
  vr= 5*sigmo(tps,15,20);
  wr= pi/10; 
 elseif tps<25
  vr= -5*sigmo(tps,20,25);
  wr= -pi/10;   
 elseif tps<30
  vr= 4*sigmo(tps,25,30);
  wr= 3*sin((tps-25)*pi);  
 else
  vr=0;
  wr=0; 
 end;
z=[vr;wr];

function [func]= sigmo(t,ti,ts)
  func= sin(pi*(t-ti)/(ts-ti));
