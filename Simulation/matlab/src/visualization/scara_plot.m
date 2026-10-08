function f = scara_plot(out,p,visibility)
if nargin<3,visibility='on';end
f=figure('Name','SCARA - tracking and contact','Color','w','Visible',visibility, ...
    'Position',[100 100 1100 720]);
if isprop(f,'Theme'),f.Theme='light';end
tiledlayout(f,2,2,'TileSpacing','compact');
target=out.target;target(target(:,3)>=0,:)=nan;
nexttile;plot(target(:,1)*1000,target(:,2)*1000,'--','LineWidth',1.2);hold on;
ink=out.xyz(:,1:2);ink(~out.draw,:)=nan;
plot(ink(:,1)*1000,ink(:,2)*1000,'-','LineWidth',1.5);
axis equal;grid on;xlabel('X (mm)');ylabel('Y (mm)');legend('Desired','Actual contact');
title(sprintf('XY RMS %.4f mm | maximum %.4f mm',out.rmsErrorMm,out.maxErrorMm));
nexttile;plot(out.t,rad2deg(out.q(:,1:2)),'LineWidth',1);grid on;
xlabel('Time (s)');ylabel('Joint angle (deg)');legend('Shoulder S','Relative elbow E');
nexttile;plot(out.t,out.force,'LineWidth',1.3);grid on;
xlabel('Time (s)');ylabel('Normal force (N)');title('Brush spring-damper estimate');
nexttile;yyaxis left;plot(out.t,out.actuator(:,1:2));ylabel('Joint torque (Nm)');
yyaxis right;plot(out.t,out.actuator(:,3));ylabel('Z force (N)');
grid on;xlabel('Time (s)');legend('Shoulder','Elbow','Z');
sgtitle(sprintf('X-SCARA 2R + Z | L1 = %.0f mm, L2 = %.0f mm | estimated dynamics',p.L1*1000,p.L2*1000));
end
