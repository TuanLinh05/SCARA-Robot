function path = build_scara_simulink(showModel)
% Editable pulse-driven plant using only standard Simulink library blocks.
if nargin<1,showModel=true;end
assert(license('test','Simulink'),'scara:license','Simulink required.');
p=scara_params();mdl='scara_stepper';path=fullfile(p.modelsRoot,[mdl '.slx']);
if bdIsLoaded(mdl)
    assert(strcmp(get_param(mdl,'Dirty'),'off'),'scara:dirty','Save manual model edits before rebuilding.');
    close_system(mdl,0);
end
scara_setup('flower',p);new_system(mdl);load_system('simulink');
set_param(mdl,'Solver','FixedStepDiscrete','SolverType','Fixed-step','FixedStep','scara_p.dt', ...
    'StopTime','scara_ref.t(end)','ReturnWorkspaceOutputs','on','SaveTime','on', ...
    'TimeSaveName','tout','SignalLogging','off','SaveOutput','off', ...
    'InitFcn',['addpath(fileparts(fileparts(fileparts(get_param(bdroot,''FileName''))))); ' ...
    'startup_scara(); if ~exist(''scara_steps_signal'',''var'') || ~exist(''scara_ref'',''var'') ' ...
    '|| ~isfield(scara_ref,''steps''), scara_setup(''flower''); end']);
source(mdl,'Pulse counts Z A B','scara_steps_signal');
source(mdl,'Cartesian target','scara_xyz_signal');
source(mdl,'Pen state','scara_pen_signal');
source(mdl,'Heartbeat and validity','scara_link_signal');
source(mdl,'NC switches HIGH means open','scara_nc_signal');
decode([mdl '/Command coordinates']);
wire(mdl,'Pulse counts Z A B/1','Command coordinates/1');
sub([mdl '/Safety']);
s=[mdl '/Safety'];port(s,'Inport','Command q',1);port(s,'Inport','Link',2);port(s,'Inport','NC',3);
blk(s,'Signal Routing/Demux','Link split','Outputs','2');
blk(s,'Logic and Bit Operations/Compare To Constant','Valid','relop','>','const','.5');
blk(s,'Logic and Bit Operations/Compare To Constant','Age below 350 ms','relop','<=','const','scara_p.watchdogSeconds');
wire(s,'Link/1','Link split/1');wire(s,'Link split/1','Valid/1');wire(s,'Link split/2','Age below 350 ms/1');
blk(s,'Logic and Bit Operations/Logical Operator','All safe','Operator','AND','Inputs','11');
wire(s,'Valid/1','All safe/1');wire(s,'Age below 350 ms/1','All safe/2');
blk(s,'Signal Routing/Demux','Joint split','Outputs','3');
wire(s,'Command q/1','Joint split/1');
bounds={'scara_p.jointLimits(1,1)','scara_p.jointLimits(1,2)'; ...
    'scara_p.jointLimits(2,1)','scara_p.jointLimits(2,2)';'scara_p.zLimits(1)','scara_p.zLimits(2)'};
for i=1:3
    lo=sprintf('Lower %d',i);hi=sprintf('Upper %d',i);
    blk(s,'Logic and Bit Operations/Compare To Constant',lo,'relop','>=','const',bounds{i,1});
    blk(s,'Logic and Bit Operations/Compare To Constant',hi,'relop','<=','const',bounds{i,2});
    wire(s,sprintf('Joint split/%d',i),[lo '/1']);wire(s,sprintf('Joint split/%d',i),[hi '/1']);
    wire(s,[lo '/1'],sprintf('All safe/%d',2*i+1));wire(s,[hi '/1'],sprintf('All safe/%d',2*i+2));
end
% Directional NC guards: allow retreat from one open end; reject both open.
blk(s,'Discrete/Unit Delay','Previous q','InitialCondition','scara_q0','SampleTime','scara_p.dt');
blk(s,'Math Operations/Sum','Change q','Inputs','+-');
wire(s,'Command q/1','Previous q/1');wire(s,'Command q/1','Change q/1');wire(s,'Previous q/1','Change q/2');
blk(s,'Signal Routing/Demux','Direction split','Outputs','3');wire(s,'Change q/1','Direction split/1');
blk(s,'Signal Routing/Demux','NC split','Outputs','6');wire(s,'NC/1','NC split/1');
order=[3 1 2]; % NC order Z, J1, J2; q order J1,J2,Z.
for i=1:3
    names={sprintf('Moving plus %d',i),sprintf('Moving minus %d',i), ...
        sprintf('Into plus NC %d',i),sprintf('Into minus NC %d',i), ...
        sprintf('Both NC open %d',i),sprintf('NC blocked %d',i),sprintf('NC safe %d',i)};
    pol=sprintf('Dir polarity %d',i);
    blk(s,'Math Operations/Gain',pol,'Gain',sprintf('2*double(scara_p.positiveHigh(%d))-1',i));
    blk(s,'Logic and Bit Operations/Compare To Constant',names{1},'relop','>','const','1e-12');
    blk(s,'Logic and Bit Operations/Compare To Constant',names{2},'relop','<','const','-1e-12');
    wire(s,sprintf('Direction split/%d',order(i)),[pol '/1']);
    wire(s,[pol '/1'],[names{1} '/1']);
    wire(s,[pol '/1'],[names{2} '/1']);
    for j=3:5,blk(s,'Logic and Bit Operations/Logical Operator',names{j},'Operator','AND','Inputs','2');end
    wire(s,[names{1} '/1'],[names{3} '/1']);wire(s,[names{2} '/1'],[names{4} '/1']);
    wire(s,sprintf('NC split/%d',i*2-1),[names{3} '/2']);
    wire(s,sprintf('NC split/%d',i*2),[names{4} '/2']);
    wire(s,sprintf('NC split/%d',i*2-1),[names{5} '/1']);
    wire(s,sprintf('NC split/%d',i*2),[names{5} '/2']);
    blk(s,'Logic and Bit Operations/Logical Operator',names{6},'Operator','OR','Inputs','3');
    for j=1:3,wire(s,[names{j+2} '/1'],sprintf('%s/%d',names{6},j));end
    blk(s,'Logic and Bit Operations/Logical Operator',names{7},'Operator','NOT');
    wire(s,[names{6} '/1'],[names{7} '/1']);wire(s,[names{7} '/1'],sprintf('All safe/%d',i+8));
end
% Stop remains latched for this replay; a new simulation resets the fault.
blk(s,'Logic and Bit Operations/Logical Operator','Unsafe','Operator','NOT');
blk(s,'Logic and Bit Operations/Logical Operator','Latch fault','Operator','OR','Inputs','2');
blk(s,'Discrete/Unit Delay','Fault memory','InitialCondition','0','SampleTime','scara_p.dt');
blk(s,'Logic and Bit Operations/Logical Operator','Permit','Operator','NOT');
wire(s,'All safe/1','Unsafe/1');wire(s,'Unsafe/1','Latch fault/1');wire(s,'Fault memory/1','Latch fault/2');
wire(s,'Latch fault/1','Fault memory/1');wire(s,'Latch fault/1','Permit/1');
port(s,'Outport','Allowed',1);wire(s,'Permit/1','Allowed/1');arrange(s);
wire(mdl,'Command coordinates/1','Safety/1');wire(mdl,'Heartbeat and validity/1','Safety/2');
wire(mdl,'NC switches HIGH means open/1','Safety/3');
sub([mdl '/Hold counts on stop']);s=[mdl '/Hold counts on stop'];
port(s,'Inport','Counts',1);blk(s,'Ports & Subsystems/Enable','Enable');
port(s,'Outport','Held counts',1);set_param([s '/Held counts'],'OutputWhenDisabled','held', ...
    'InitialOutput','scara_ref.steps(1,:)');
wire(s,'Counts/1','Held counts/1');arrange(s);
wire(mdl,'Pulse counts Z A B/1','Hold counts on stop/1');
wire(mdl,'Safety/1','Hold counts on stop/Enable');
decode([mdl '/Measured belt transform']);
wire(mdl,'Hold counts on stop/1','Measured belt transform/1');
sub([mdl '/Mechanical sensitivity']);s=[mdl '/Mechanical sensitivity'];
port(s,'Inport','Command q',1);blk(s,'Signal Routing/Demux','Split','Outputs','3');
wire(s,'Command q/1','Split/1');blk(s,'Signal Routing/Mux','Join','Inputs','3');
blk(s,'Math Operations/Gain','Motor B coupling','Gain','scara_p.coupling');
blk(s,'Math Operations/Sum','Motor B relative','Inputs','++');
blk(s,'Math Operations/Gain','Belt decouple','Gain','scara_p.coupling');
blk(s,'Math Operations/Sum','Elbow decouple','Inputs','+-');
for i=1:3
    origin=sprintf('Origin %d',i);minus=sprintf('Relative %d',i);back=sprintf('Backlash %d',i);
    lag=sprintf('Lag %d',i);flag=sprintf('Use lag %d',i);sw=sprintf('Optional lag %d',i);plus=sprintf('Absolute %d',i);
    blk(s,'Sources/Constant',origin,'Value',sprintf('scara_q0(%d)',i));
    blk(s,'Math Operations/Sum',minus,'Inputs','+-');
    wire(s,sprintf('Split/%d',i),[minus '/1']);wire(s,[origin '/1'],[minus '/2']);
    blk(s,'Discontinuities/Backlash',back,'BacklashWidth',sprintf('scara_p.backlash(%d)',i), ...
        'InitialOutput','0','SampleTime','scara_p.dt');
    if i==2
        wire(s,'Relative 1/1','Motor B coupling/1');
        wire(s,[minus '/1'],'Motor B relative/1');
        wire(s,'Motor B coupling/1','Motor B relative/2');
        wire(s,'Motor B relative/1',[back '/1']);
    else
        wire(s,[minus '/1'],[back '/1']);
    end
    pole=sprintf('exp(-scara_p.dt/max(scara_p.lagSeconds(%d),1e-9))',i);
    blk(s,'Discrete/Discrete Transfer Fcn',lag,'Numerator',['[1-' pole ' 0]'], ...
        'Denominator',['[1 -' pole ']'],'InitialStates','0','SampleTime','scara_p.dt');
    wire(s,[back '/1'],[lag '/1']);
    blk(s,'Sources/Constant',flag,'Value',sprintf('scara_p.lagSeconds(%d)>0',i));
    blk(s,'Signal Routing/Switch',sw,'Criteria','u2 ~= 0');
    wire(s,[lag '/1'],[sw '/1']);wire(s,[flag '/1'],[sw '/2']);wire(s,[back '/1'],[sw '/3']);
    blk(s,'Math Operations/Sum',plus,'Inputs','++');
    if i==2
        wire(s,'Optional lag 1/1','Belt decouple/1');
        wire(s,[sw '/1'],'Elbow decouple/1');
        wire(s,'Belt decouple/1','Elbow decouple/2');
        wire(s,'Elbow decouple/1',[plus '/1']);
    else
        wire(s,[sw '/1'],[plus '/1']);
    end
    wire(s,[origin '/1'],[plus '/2']);
    wire(s,[plus '/1'],sprintf('Join/%d',i));
end
port(s,'Outport','Actual q',1);wire(s,'Join/1','Actual q/1');arrange(s);
wire(mdl,'Measured belt transform/1','Mechanical sensitivity/1');
fk([mdl '/Forward kinematics']);wire(mdl,'Mechanical sensitivity/1','Forward kinematics/1');
blk(mdl,'Math Operations/Sum','XYZ model error','Inputs','+-');
wire(mdl,'Forward kinematics/1','XYZ model error/1');wire(mdl,'Cartesian target/1','XYZ model error/2');
logs={'q_log','Mechanical sensitivity/1';'xyz_log','Forward kinematics/1'; ...
    'steps_log','Hold counts on stop/1';'allowed_log','Safety/1';'error_log','XYZ model error/1';'pen_log','Pen state/1'};
for i=1:size(logs,1)
    blk(mdl,'Sinks/To Workspace',logs{i,1},'VariableName',logs{i,1},'SaveFormat','Array','MaxDataPoints','inf');
    wire(mdl,logs{i,2},[logs{i,1} '/1']);
end
blk(mdl,'Sinks/Scope','XYZ target model and error','NumInputPorts','3');
wire(mdl,'Cartesian target/1','XYZ target model and error/1');
wire(mdl,'Forward kinematics/1','XYZ target model and error/2');wire(mdl,'XYZ model error/1','XYZ target model and error/3');
arrange(mdl);
a=Simulink.Annotation(mdl,['OPEN LOOP STEPPER | R9 + real-log calibration | counts Z/A/B' newline ...
    'A=J1; B=J2 + measured k*J1 | q in rad, Z in metres' newline ...
    'Geometry/TCP/backlash/lag scenarios are assumptions, not physical measurements.' newline ...
    'GUI planner generates offline pulses; this model does not flash or drive hardware.' newline ...
    'Home calibration data are loaded, not re-measured by an encoder model.']);
list=find_system(mdl,'SearchDepth',1,'Type','Block');bottom=0;
for i=1:numel(list),pos=get_param(list{i},'Position');bottom=max(bottom,pos(4));end
a.Position=[30 bottom+65];
if ~isfolder(p.modelsRoot),mkdir(p.modelsRoot);end
save_system(mdl,path);if showModel,open_system(mdl);end
end
function source(s,name,var)
blk(s,'Sources/From Workspace',name,'VariableName',var,'Interpolate','off', ...
    'SampleTime','scara_p.dt','OutputAfterFinalValue','Holding final value');
end
function sub(s)
add_block('simulink/Ports & Subsystems/Subsystem',s);Simulink.SubSystem.deleteContents(s);
end
function port(s,type,name,n)
if strcmp(type,'Inport'),lib='Sources/In1';else,lib='Sinks/Out1';end
blk(s,lib,name,'Port',num2str(n));
end
function blk(s,lib,name,varargin)
add_block(['simulink/' lib],[s '/' name],'Position',[30 30 120 65],varargin{:});
end
function wire(s,a,b)
add_line(s,a,b,'autorouting','on');
end
function arrange(s)
Simulink.BlockDiagram.arrangeSystem(s);
end
function decode(s)
sub(s);port(s,'Inport','Counts Z A B',1);
blk(s,'Signal Routing/Demux','Split','Outputs','3');wire(s,'Counts Z A B/1','Split/1');
blk(s,'Math Operations/Gain','Z metres','Gain','1/scara_p.stepsPerMeterZ');
blk(s,'Math Operations/Gain','A radians','Gain','pi/(180*scara_p.stepsPerDegree(1))');
blk(s,'Math Operations/Gain','B radians','Gain','pi/(180*scara_p.stepsPerDegree(2))');
wire(s,'Split/1','Z metres/1');wire(s,'Split/2','A radians/1');wire(s,'Split/3','B radians/1');
blk(s,'Math Operations/Gain','Belt coupling','Gain','scara_p.coupling');
wire(s,'A radians/1','Belt coupling/1');
blk(s,'Math Operations/Sum','Elbow relative','Inputs','+-');
wire(s,'B radians/1','Elbow relative/1');wire(s,'Belt coupling/1','Elbow relative/2');
blk(s,'Signal Routing/Mux','q1 q2 Z','Inputs','3');
wire(s,'A radians/1','q1 q2 Z/1');wire(s,'Elbow relative/1','q1 q2 Z/2');wire(s,'Z metres/1','q1 q2 Z/3');
port(s,'Outport','q',1);wire(s,'q1 q2 Z/1','q/1');arrange(s);
end
function fk(s)
sub(s);port(s,'Inport','q1 q2 Z',1);
blk(s,'Signal Routing/Demux','Split','Outputs','3');wire(s,'q1 q2 Z/1','Split/1');
blk(s,'Math Operations/Sum','Outer angle','Inputs','++');wire(s,'Split/1','Outer angle/1');wire(s,'Split/2','Outer angle/2');
angles={'J1 sin','J1 cos','Outer sin','Outer cos'};ops={'sin','cos','sin','cos'};
for i=1:4
    blk(s,'Math Operations/Trigonometric Function',angles{i},'Operator',ops{i});
    if i<=2,wire(s,'Split/1',[angles{i} '/1']);else,wire(s,'Outer angle/1',[angles{i} '/1']);end
end
terms={'X inner','X outer','X TCP','Y inner','Y outer','Y TCP'};
gains={'-scara_p.actualL1','-(scara_p.actualL2+scara_p.actualToolOffset(2))','scara_p.actualToolOffset(1)', ...
    'scara_p.actualL1','scara_p.actualL2+scara_p.actualToolOffset(2)','scara_p.actualToolOffset(1)'};
src={'J1 sin/1','Outer sin/1','Outer cos/1','J1 cos/1','Outer cos/1','Outer sin/1'};
for i=1:6
    blk(s,'Math Operations/Gain',terms{i},'Gain',gains{i});wire(s,src{i},[terms{i} '/1']);
end
blk(s,'Sources/Constant','Base X','Value','scara_p.base(1)');blk(s,'Sources/Constant','Base Y','Value','scara_p.base(2)');
blk(s,'Math Operations/Sum','X','Inputs','++++');blk(s,'Math Operations/Sum','Y','Inputs','++++');
for i=1:3,wire(s,[terms{i} '/1'],sprintf('X/%d',i));wire(s,[terms{i+3} '/1'],sprintf('Y/%d',i));end
wire(s,'Base X/1','X/4');wire(s,'Base Y/1','Y/4');
blk(s,'Signal Routing/Mux','XYZ','Inputs','3');wire(s,'X/1','XYZ/1');wire(s,'Y/1','XYZ/2');wire(s,'Split/3','XYZ/3');
port(s,'Outport','XYZ metres',1);wire(s,'XYZ/1','XYZ metres/1');arrange(s);
end
