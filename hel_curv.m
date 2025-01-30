close all;

p = load("p_hel_curv.mat").ans;
q = load("q_hel_curv.mat").ans;
p_ref = load("p_ref_hel_curv.mat").ans;

t = p(1,:);
p = p(2:end,:)';
q = q(2:end,:)';
p_ref = p_ref(2:end,:);
p1 = p(:,1);
p2 = p(:,2);    
p3 = p(:,3);

f1 = figure;
subplot(2,3,5);
plot(t, vecnorm(p' - p_ref));
grid on;
title('Error [m]');
ax = [subplot(1,6,1), subplot(1,6,2), subplot(2,6,3), subplot(1,3,3)];

% h = tiledlayout(1,4, Padding="compact", TileSpacing="compact");
% ax = [nexttile, nexttile, nexttile, nexttile];


animate(p, q, p_ref, p1, p2, p3, t, ax)