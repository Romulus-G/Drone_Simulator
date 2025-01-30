close all;

p = load("p_ell.mat").ans;
q = load("q_ell.mat").ans;
p_ref = load("p_ref_ell.mat").ans;

t = p(1,:);
p = p(2:end,:)';
q = q(2:end,:)';
p_ref = p_ref(2:end,:);
p1 = p(:,1);
p2 = p(:,2);    
p3 = p(:,3);

f1 = figure;
subplot(2,4,6);
plot(t, vecnorm(p' - p_ref));
grid on;
title('Error [m]');

ax = [subplot(2,4,1), subplot(2,4,2), subplot(2,4,5), subplot(1,2,2)];

% h = tiledlayout(2,2, Padding="compact", TileSpacing="compact");
% ax = [nexttile, nexttile, nexttile, nexttile];
% f2 = figure;
% ax(4) = gca;

animate(p, q, p_ref, p1, p2, p3, t, ax)