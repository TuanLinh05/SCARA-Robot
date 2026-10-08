function f = scara_app()
% Interactive STL view, manual joints, trajectory playback and PID simulation.
p=scara_params(); q=scara_ik([0;0;p.penLift],p);
f=figure('Name','X-SCARA - brush drawing simulator','NumberTitle','off', ...
    'Color',[0.96 0.97 0.98],'Position',[60 70 1350 800]);
if isprop(f,'Theme'),f.Theme='light';end
ax=axes(f,'Position',[0.05 0.16 0.58 0.78]);scene=scara_scene(ax,p);
title(ax,'STL arm + brush concept | 2R + Z');
xyax=axes(f,'Position',[0.70 0.62 0.26 0.30]);hold(xyax,'on');grid(xyax,'on');axis(xyax,'equal');
xlim(xyax,[-50 50]);ylim(xyax,[-50 50]);xlabel(xyax,'X (mm)');ylabel(xyax,'Y (mm)');
target=plot(xyax,nan,nan,'--','Color',[0.15 0.55 0.8]);
ink=plot(xyax,nan,nan,'Color',[0.15 0.17 0.2],'LineWidth',1.5);
title(xyax,'Paper: desired / actual contact');
uicontrol(f,'Style','text','String','Trajectory','Units','normalized','Position',[0.68 .52 .13 .03],'HorizontalAlignment','left');
pathchoice=uicontrol(f,'Style','popupmenu','String',{'flower','circle','square','BK','Custom CSV'}, ...
    'Units','normalized','Position',[0.81 .515 .15 .04]);
mode=uicontrol(f,'Style','popupmenu','String',{'Kinematics (IK)','PID + dynamics'}, ...
    'Value',2,'Units','normalized','Position',[0.68 .47 .28 .045]);
ff=uicontrol(f,'Style','checkbox','String','Model feedforward','Value',p.feedforward, ...
    'Units','normalized','Position',[0.68 .435 .28 .03]);
runbtn=uicontrol(f,'Style','pushbutton','String','Run + animate','Units','normalized', ...
    'Position',[0.68 .395 .17 .045],'Callback',@runexample);
stopbtn=uicontrol(f,'Style','togglebutton','String','Stop','Units','normalized', ...
    'Position',[0.87 .395 .09 .045]);
uicontrol(f,'Style','pushbutton','String','Open Simulink model','Units','normalized', ...
    'Position',[0.68 .33 .28 .045],'Callback',@opensimulink);
uicontrol(f,'Style','pushbutton','String','Plot / export last run','Units','normalized', ...
    'Position',[0.68 .27 .28 .045],'Callback',@exportlast);
status=uicontrol(f,'Style','text','String','Ready. Rotate the 3D view with the mouse.', ...
    'Units','normalized','Position',[0.66 .17 .32 .07],'HorizontalAlignment','left','FontSize',10);
uicontrol(f,'Style','text','String', ...
    'Estimated masses, friction and brush stiffness. Frame and Z stack approximate.', ...
    'Units','normalized','Position',[.05 .015 .9 .03],'HorizontalAlignment','left','FontSize',10);
names={'Shoulder S (deg)','Relative elbow E (deg)','Free tip Z (mm)'};
mins=[rad2deg(p.jointLimits(:,1));p.zLimits(1)*1000];
maxs=[rad2deg(p.jointLimits(:,2));p.zLimits(2)*1000];
vals=[rad2deg(q(1:2));q(3)*1000]; sliders=gobjects(3,1);labels=gobjects(3,1);
for j=1:3
    xx=.05+(j-1)*.30;
    labels(j)=uicontrol(f,'Style','text','String',names{j},'Units','normalized', ...
        'Position',[xx .112 .28 .027],'HorizontalAlignment','left');
    sliders(j)=uicontrol(f,'Style','slider','Min',mins(j),'Max',maxs(j),'Value',vals(j), ...
        'Units','normalized','Position',[xx .072 .26 .035],'Callback',@manual);
end
rotate3d(f,'on');
last=[];lastref=[];
manual();
    function manual(~,~)
        q=[deg2rad(sliders(1).Value);deg2rad(sliders(2).Value);sliders(3).Value/1000];
        scara_update(scene,q); tip=scara_fk(q,p);
        for z=1:3,labels(z).String=sprintf('%s: %.2f',names{z},sliders(z).Value);end
        [axesCoord,~]=scara_motor_coordinates(q,p);
        status.String=sprintf('Tip X %.2f / Y %.2f mm\nAxis A %.2f / B %.2f deg', ...
            tip(1)*1000,tip(2)*1000,rad2deg(axesCoord(1)),rad2deg(axesCoord(2)));
    end
    function kind=chosenpath()
        kind=pathchoice.String{pathchoice.Value};
        if strcmp(kind,'Custom CSV')
            [file,folder]=uigetfile('*.csv','CSV: x_mm,y_mm,pen');
            if isequal(file,0),kind=[];return;end
            kind=fullfile(folder,file);
        end
    end
    function runexample(~,~)
        kind=chosenpath();if isempty(kind),return;end
        runbtn.Enable='off';set(sliders,'Enable','off');
        stopbtn.Value=0;status.String='Computing trajectory and simulation...';drawnow;
        try
            p.feedforward=logical(ff.Value);
            ref=scara_reference(kind,p);
            if mode.Value==2
                out=scara_simulate(ref,p);
            else
                out=scara_results(ref.t,ref.q,ref.dq,zeros(size(ref.q)),ref,p);
            end
            last=out;lastref=ref;
            desired=ref.xyz;desired(~ref.pen,:)=nan;
            set(target,'XData',desired(:,1)*1000,'YData',desired(:,2)*1000);
            set(scene.target,'XData',desired(:,1)*1000,'YData',desired(:,2)*1000, ...
                'ZData',max(0,desired(:,3))*1000);
            contact=out.xyz;contact(~out.draw,:)=nan;contact(:,3)=0.0002;
            stride=max(1,round(0.04/p.dt));
            for k=[1:stride:numel(out.t) numel(out.t)]
                if ~isgraphics(f) || stopbtn.Value,break;end
                scara_update(scene,out.q(k,:)');
                live=[rad2deg(out.q(k,1:2)) out.q(k,3)*1000];
                for j=1:3
                    sliders(j).Value=live(j);
                    labels(j).String=sprintf('%s: %.2f',names{j},live(j));
                end
                set(ink,'XData',contact(1:k,1)*1000,'YData',contact(1:k,2)*1000);
                set(scene.ink,'XData',contact(1:k,1)*1000,'YData',contact(1:k,2)*1000, ...
                    'ZData',contact(1:k,3)*1000);
                status.String=sprintf('t %.2f s | force %.3f N\nXY RMS %.4f mm | max %.4f mm', ...
                    out.t(k),out.force(k),out.rmsErrorMm,out.maxErrorMm);
                drawnow;pause(0.02);
            end
        catch exception
            if isgraphics(f),errordlg(exception.message,'SCARA simulation');end
        end
        if isgraphics(f),runbtn.Enable='on';set(sliders,'Enable','on');end
    end
    function opensimulink(~,~)
        try
            kind=chosenpath();if isempty(kind),return;end
            path=fullfile(p.modelsRoot,'scara_brush.slx');
            if ~isfile(path),build_scara_simulink(false);end
            scara_setup(kind,p);open_system(path);
        catch exception,errordlg(exception.message,'Simulink');end
    end
    function exportlast(~,~)
        if isempty(last),status.String='Run a trajectory first.';return;end
        scara_export(last,lastref,p);scara_plot(last,p);
        status.String='Saved output/data/matlab/simulation.csv and simulation.mat';
    end
end
