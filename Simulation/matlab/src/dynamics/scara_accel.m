function ddq = scara_accel(u,p)
% Simulink plant input u = [q(3); dq(3); actuator(3)].
q=u(1:3); dq=u(4:6); actuator=u(7:9);
[M,C,Fcontact]=scara_dynamics(q,dq,p);
friction=p.viscous.*dq+p.coulomb.*tanh(dq/0.005);
ddq=M\(actuator-C-friction-[0;0;p.zMass*p.gravity]+Fcontact);
end
