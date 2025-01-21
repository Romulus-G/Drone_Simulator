%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controlleur
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [output]=control(inp)

% Variables
  p1=inp(1); p2=inp(2); theta=inp(3); p=[p1;p2];        % etat du vehicule
  pr1=inp(4); pr2=inp(5); thetar=inp(6); pr=[pr1;pr2]; % etat de la reference
  vr= inp(7); wr= inp(8); % vitesses de la référence
 
  %method = 1; % commenter cette ligne pour l'asservissement en position/orientation
  method = 2; % commenter cette ligne pour l'asservissement en position
   
 if method == 1  %%%%%%  Stabilisation de la position
  dpr= [vr*cos(thetar);vr*sin(thetar)];
  v= rot(-theta)*(dpr-1*([p(1)+cos(theta);p(2)+sin(theta)]-pr)); 
  v= [v;p-pr];
else  %%%%%%  Stabilisation de la position + orientation
  k1=2;
  k2=2;
  k3=4;
  pt= rot(-thetar)*(p-pr);
  thetat= theta-thetar;
  
  v1= vr-k1*abs(vr)*pt(1);
  v2= wr-k2*vr*pt(2)-k3*abs(vr)*thetat;
  v= [v1;v2;pt];  
end;

output= v;

function [out]= rot(theta) % Rotation matrix in the plane
  out= [cos(theta) -sin(theta); sin(theta) cos(theta)]; 

