function path = build_scara_simulink(showModel)
% Build an editable continuous feedback model; requires MATLAB + Simulink.
if nargin<1,showModel=true;end
assert(license('test','Simulink'),'scara:license','Simulink license required.');
p=scara_params(); mdl='scara_brush'; path=fullfile(p.modelsRoot,[mdl '.slx']);
if bdIsLoaded(mdl)
    assert(strcmp(get_param(mdl,'Dirty'),'off'),'scara:dirty', ...
        'Save edits in scara_brush before rebuilding.');
    close_system(mdl,0);
end
scara_setup('flower',p);new_system(mdl);
set_param(mdl,'Solver','ode4','SolverType','Fixed-step','FixedStep','scara_p.dt', ...
    'StopTime','scara_ref.t(end)','ReturnWorkspaceOutputs','on', ...
    'SaveTime','on','TimeSaveName','tout','SignalLogging','off', ...
    'InitFcn',['addpath(fileparts(fileparts(fileparts(get_param(bdroot,''FileName''))))); ' ...
    'startup_scara(); scara_setup();']);
add_block('simulink/Sources/From Workspace',[mdl '/Reference q dq ddq'], ...
    'VariableName','scara_ref_signal','Interpolate','on','Position',[35 80 175 120]);
add_block('simulink/Signal Routing/Mux',[mdl '/Controller input'], ...
    'Inputs','4','Position',[235 80 240 220]);
matlabblock('PID and feedforward','scara_controller',18,6,[300 105 485 160]);
add_block('simulink/Signal Routing/Demux',[mdl '/Torque and integral rate'], ...
    'Outputs','[3 3]','Position',[535 105 540 175]);
add_block('simulink/Continuous/Integrator',[mdl '/Integral error'], ...
    'InitialCondition','[0;0;0]','Position',[370 265 425 295]);
add_block('simulink/Signal Routing/Mux',[mdl '/Plant input q dq actuator'], ...
    'Inputs','3','Position',[610 50 615 155]);
matlabblock('Coupled 2R Z and brush contact','scara_accel',9,3,[665 70 870 125]);
add_block('simulink/Continuous/Integrator',[mdl '/Velocity dq'], ...
    'InitialCondition','[0;0;0]','Position',[925 80 980 115]);
add_block('simulink/Continuous/Integrator',[mdl '/Position q'], ...
    'InitialCondition','scara_q0','Position',[1030 80 1085 115]);
add_block('simulink/Signal Routing/Mux',[mdl '/Display q actuator'], ...
    'Inputs','2','Position',[1140 80 1145 160]);
add_block('simulink/Sinks/Scope',[mdl '/Joint positions and forces'], ...
    'Position',[1195 80 1245 130]);
logblock('q_log',[1130 220 1250 250]);
logblock('dq_log',[920 280 1040 310]);
logblock('ei_log',[520 280 640 310]);
connect('Reference q dq ddq/1','Controller input/1');
connect('Position q/1','Controller input/2');connect('Velocity dq/1','Controller input/3');
connect('Integral error/1','Controller input/4');
connect('Controller input/1','PID and feedforward/1');
connect('PID and feedforward/1','Torque and integral rate/1');
connect('Torque and integral rate/2','Integral error/1');
connect('Position q/1','Plant input q dq actuator/1');
connect('Velocity dq/1','Plant input q dq actuator/2');
connect('Torque and integral rate/1','Plant input q dq actuator/3');
connect('Plant input q dq actuator/1','Coupled 2R Z and brush contact/1');
connect('Coupled 2R Z and brush contact/1','Velocity dq/1');
connect('Velocity dq/1','Position q/1');
connect('Position q/1','Display q actuator/1');
connect('Torque and integral rate/1','Display q actuator/2');
connect('Display q actuator/1','Joint positions and forces/1');
connect('Position q/1','q_log/1');connect('Velocity dq/1','dq_log/1');
connect('Integral error/1','ei_log/1');
Simulink.BlockDiagram.arrangeSystem(mdl);
note=Simulink.Annotation(mdl, ...
    sprintf(['X-SCARA 2R + Z / brush drawing\nUnits: rad, m, N, Nm\n' ...
    'Dynamics, friction and brush constants are editable estimates.\n' ...
    'Change scara_controller.m or replace its block to test your algorithm.\n' ...
    'scara_setup(''circle'') selects a new reference; run scara_simulink to plot/export.']));
blocks=find_system(mdl,'SearchDepth',1,'Type','Block');bottom=0;
for k=1:numel(blocks),pos=get_param(blocks{k},'Position');bottom=max(bottom,pos(4));end
note.Position=[40 bottom+55];
save_system(mdl,path);
if showModel,open_system(mdl);end
    function connect(src,dst),add_line(mdl,src,dst,'autorouting','on');end
    function logblock(name,pos)
        add_block('simulink/Sinks/To Workspace',[mdl '/' name], ...
            'VariableName',name,'SaveFormat','Array','MaxDataPoints','inf','Position',pos);
    end
    function matlabblock(name,fn,insize,outsize,pos)
        block=[mdl '/' name];
        add_block('simulink/User-Defined Functions/MATLAB Function',block,'Position',pos);
        root=sfroot;chart=root.find('-isa','Stateflow.EMChart','Path',block);
        chart.Script=sprintf('function y = fcn(u,scara_numeric_p)\n%%#codegen\ny = %s(u,scara_numeric_p);\nend\n',fn);
        parameter=chart.find('-isa','Stateflow.Data','Name','scara_numeric_p');
        parameter.Scope='Parameter';parameter.Tunable=false;
        input=chart.find('-isa','Stateflow.Data','Name','u');
        input.Props.Array.Size=num2str(insize);
        output=chart.find('-isa','Stateflow.Data','Name','y');
        output.Props.Array.Size=num2str(outsize);
    end
end
