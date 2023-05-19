function alphaBeta = xyToAngles(xy)
%Converts end coordinates column vector xy into angle coordinates column 
%vector alphaBeta using "inverse kinematics" lol

x=xy(1);
y=xy(2);

A=.015;     %Linkage lengths in m
B=.050;
C=.035;

if sqrt(x^2+y^2) > A+B+C
    error("Impossible movement request.")
end

syms a b

eqn1=A*cos(a)+B*cos(b)+C*cos(a)==x;
eqn2=A*sin(a)+B*sin(b)+C*sin(a)==y;
eqns=[eqn1,eqn2];

soln=solve(eqns,[a b],'Real',true);

alphaSolns=soln.a;
betaSolns=soln.b;

%Each x,y has two solutions. Choose the one where alpha is greater, like
%should always be the case on the robot.
if alphaSolns(1) > betaSolns(1)
    alphaAns=double(alphaSolns(1));
    betaAns=double(betaSolns(1));
elseif alphaSolns(2) > betaSolns(2)
    alphaAns=double(alphaSolns(2));
    betaAns=double(betaSolns(2));
else
    error('No solution found.')
end

%This solver tends to give answers where both alpha and beta are in the
%negative direction. Add 2*pi rads to get positive output angles.
alphaAns=alphaAns+2*pi;
betaAns=betaAns+2*pi;

alphaBeta=[alphaAns;betaAns];

end


