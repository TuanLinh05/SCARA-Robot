function [xyz, elbow, J] = scara_fk(q,p)
% q = [shoulder angle; relative elbow angle; free brush tip height].
% Zero angle points along +Y; positive joint angle rotates toward -X.
q=q(:); a=q(1); b=a+q(2);
R=[cos(b) -sin(b); sin(b) cos(b)];
elbow=p.base+p.L1*[-sin(a);cos(a)];
outer=R*(p.toolOffset+[0;p.L2]);
xyz=[elbow+outer;q(3)];
J=[-p.L1*cos(a)-outer(2), -outer(2), 0; ...
   -p.L1*sin(a)+outer(1), outer(1), 0; 0 0 1];
end
