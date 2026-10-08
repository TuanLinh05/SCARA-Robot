function q = scara_ik(xyz,p,branch,checkLimits)
% Analytic IK, including local brush XY offset. Rejects unreachable targets.
% branch +1 is the original working elbow branch; -1 is the other solution.
if nargin<3, branch=1; end
if nargin<4, checkLimits=true; end
xyz=xyz(:);
assert(numel(xyz)==3 && all(isfinite(xyz)), 'scara:target', ...
    'Target must be three finite values in metres.');
assert(abs(branch)==1,'scara:branch','Branch must be +1 or -1.');
v=p.toolOffset+[0;p.L2]; L=norm(v); delta=atan2(-v(1),v(2));
xy=xyz(1:2)-p.base; r=norm(xy);
c=(r*r-p.L1^2-L^2)/(2*p.L1*L);
if abs(c)>1+1e-10 || r<1e-10
    error('scara:unreachable','Target outside reachable workspace or at undefined folded origin.');
end
e=branch*acos(max(-1,min(1,c)));
if abs(sin(e))<p.singularityMargin
    error('scara:singular','Target too close to a singular configuration.');
end
a=atan2(-xy(1),xy(2))-atan2(L*sin(e),p.L1+L*cos(e));
a=atan2(sin(a),cos(a));
q=[a;e-delta;xyz(3)];
if checkLimits
    if any(q(1:2)<p.jointLimits(:,1)) || any(q(1:2)>p.jointLimits(:,2)) ...
            || q(3)<p.zLimits(1) || q(3)>p.zLimits(2)
        error('scara:limits','Target violates the configured (assumed) joint or Z limits.');
    end
end
end
