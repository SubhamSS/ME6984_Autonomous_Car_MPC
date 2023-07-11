clear
%% Dynamics

m = 4.78;
lr = 0.18;
lf = 0.18;

syms x y psi vx vy r delta Fx steer

qq = [x;y;psi;vx;vy;r;delta];
uu = [Fx;steer];
Comb = [vx*cos(psi)-vy*sin(psi);...
    vx*sin(psi)+vy*cos(psi);...
    r;...
    Fx/m;...
    (steer*vx + delta*Fx/m)*(lr/(lr+lf));...
    (steer*vx + delta*Fx/m)*(1/(lr+lf));...
    steer];

AA = jacobian(Comb,qq);
BB = jacobian(Comb,uu);

AAVal = matlabFunction(AA,'vars',{qq,uu},"File","AAVal");
BBVal = matlabFunction(BB,'vars',{qq,uu},"File","BBVal");
CombVal = matlabFunction(Comb,'vars',{qq,uu},"File","CombVal");

% write to mex file 
codegen AAVal -args {zeros(7,1),zeros(2,1)}  -o AAVal_sym -report
codegen BBVal -args {zeros(7,1),zeros(2,1)}  -o BBVal_sym -report
codegen CombVal -args {zeros(7,1),zeros(2,1)}  -o CombVal_sym -report

