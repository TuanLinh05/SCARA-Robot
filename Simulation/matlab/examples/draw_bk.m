function [out,ref,f] = draw_bk(visibility)
% Simulate and export the supplied BK sketch. Run from Simulation/matlab:
% startup_scara; draw_bk
if nargin<1,visibility='on';end
p=scara_params();ref=scara_reference('bk',p);out=scara_simulate(ref,p);
data=fullfile(p.outputRoot,'data','matlab','bk');scara_export(out,ref,p,data);
f=figure('Name','BK - reference and SCARA drawing','Color','w', ...
    'Visible',visibility,'Position',[100 100 1080 500]);
if isprop(f,'Theme'),f.Theme='light';end
tiledlayout(f,1,2,'TileSpacing','compact');
desired=ref.xyz;desired(~ref.pen,:)=nan;
source=(desired(:,1:2)-ref.geometry.offset)/ref.geometry.scale;
nexttile;plot(source(:,1),source(:,2),'LineWidth',2,'Color',[.12 .25 .42]);
axis equal;grid on;xlim([-.04 .55]);ylim([-.04 .55]);
xticks([0 .25 .5]);yticks([0 .25 .5]);xlabel('Source X');ylabel('Source Y');
title('BK from the supplied sketch');
nexttile;plot(desired(:,1)*1000,desired(:,2)*1000,'--','LineWidth',1.4);hold on;
ink=out.xyz;ink(~out.draw,:)=nan;
plot(ink(:,1)*1000,ink(:,2)*1000,'LineWidth',1.5,'Color',[.15 .17 .2]);
rectangle('Position',[-p.paperHalfSize'*1000 2*p.paperHalfSize'*1000], ...
    'EdgeColor',[.6 .65 .7],'LineStyle',':');
axis equal;grid on;xlim([-55 55]);ylim([-55 55]);xlabel('X (mm)');ylabel('Y (mm)');
legend('Reference','Actual ink','Location','southoutside');
title(sprintf('SCARA | %.0f x %.0f mm | XY RMS %.4f mm', ...
    ref.geometry.sizeMm,ref.geometry.sizeMm,out.rmsErrorMm));
sgtitle('Three ink strokes; pen lifted for transfers');
figures=fullfile(p.outputRoot,'figures');if ~isfolder(figures),mkdir(figures);end
exportgraphics(f,fullfile(figures,'bk_drawing.png'),'Resolution',150);
tracking=scara_plot(out,p,visibility);
exportgraphics(tracking,fullfile(figures,'bk_tracking.png'),'Resolution',140);
if strcmp(visibility,'off'),close(tracking);end
fprintf('BK: %.1f x %.1f mm, %.2f s, RMS %.6f mm, MAX %.6f mm.\n', ...
    ref.geometry.sizeMm,ref.geometry.sizeMm,ref.t(end),out.rmsErrorMm,out.maxErrorMm);
fprintf('Saved BK CSV/MAT in %s\n',data);
end
