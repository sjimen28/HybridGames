%--------------------------------------------------------------------------
% Matlab M-file Project: 2-Player Zero-Sum HyGames @  Hybrid Systems Laboratory (HSL), 
% Filename: SaddlePointBouncingBall.m
%--------------------------------------------------------------------------
% Project: Example - Jumps Actuated Bouncing Ball under Attack as a 
%           Two-Player Zero-Sum Hybrid Game - Saddle Point Behavior
% Author: Santiago Jimenez Leudo
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%   Make sure to install HyEQ Toolbox (Beta) at
%   https://www.mathworks.com/matlabcentral/fileexchange/102239-hybrid-equations-toolbox-beta 
%   Copyright @ Hybrid Systems Laboratory (HSL),
%   Revision: 0.0.0.2 Date: 01/23/2022 9:45:00

%   Run ExJumpsActBouncingBallUnderAttack.m
%   This M-file is called therein
% --------------------------------------------------------------
%%% System Evolution + Cost Computation
% --------------------------------------------------------------
%
% --------------------------------------------------------------     
%%% Variation in inputs to diplay saddle behavior in the cost of continuous
%%% solutions

k=22;       %Amount of variations on input
ku=9;       %Position of optimal cost in u with respect to [1,k]
kw=11;      %Position of optimal cost in w with respect to [1,k]
for indexu=1:k
    epsilonu=1+(0.4*(-ku+indexu));      %Factor of perturbation in u w.r.t optimal
    for indexw=1:k-4
        epsilonw=1+(0.5*(-kw+indexw));  %Factor of perturbation in w w.r.t optimal
               
        % Initial State
        xt(:,1)=x0;     %Perturbed Trajectory
        
        % Perturbed Input
        ud1nopt(1)=epsilonu*ud1f(xt(:,1));
        ud2nopt(1)=epsilonw*ud2f(xt(:,1));
        
        %Initial Cost
        Jt(1)=0;
        
        jt=0;   %Discrete time for perturbed solution
              
        for i=1:size(t,2)-1          
            if jt <=JSPAN(2)
                if xt(1,i)<=0.01 && xt(2,i)<=0  % Check if x is in the Jump Set with some tolerance
                    Jt(i)=Jt(i)+Ld(xt(:,i),[ud1nopt(i),ud2nopt(i)]);        % Add Discrete Cost
                    xt(:,i)=g(xt(:,i),[ud1nopt(i);ud2nopt(i)],gammad);      % Evolve via jump
                    jt=jt+1;                                                % Report a jump
                    
                    xt(:,i+1)=xt(:,i)+(t(i+1)-t(i))*f(xt(:,i),uC,gammac);   % Evolve via flow
                    Jt(i+1)=Jt(i)+(t(i+1)-t(i))*Lc(xt(:,i),uC);             % Add Continuous Cost

                else
                    xt(:,i+1)=xt(:,i)+(t(i+1)-t(i))*f(xt(:,i),uC,gammac); 	% Evolve via flow
                    Jt(i+1)=Jt(i)+(t(i+1)-t(i))*Lc(xt(:,i),uC);             % Add Continuous Cost
                end
                ud1nopt(i+1)=epsilonu*ud1f(xt(:,i+1));  %Update Input P1 for next time step   
                ud2nopt(i+1)=epsilonw*ud2f(xt(:,i+1));  %Update Input P2 for next time step
            end  
        end
        
        % Store Perturbed Slution Data
        phit=xt;
        Jp=Jt;
        wn=ud2nopt;
        un=ud1nopt;
        Jtotal(indexu,indexw)=Jt(end);
        epsilonwv(indexw)=epsilonw;
        clear xt Jt ud1nopt ud2nopt
    end
    epsilonuv(indexu)=epsilonu;
end

%
% --------------------------------------------------------------
%%% Plot
% --------------------------------------------------------------
%
[Xk,Yk] = meshgrid(epsilonuv,epsilonwv);
figure(2)
surf(Xk',Yk',Jtotal)
hold on
plot3(epsilonuv(ku),epsilonwv(kw),Jtotal(ku,kw),'r.','MarkerSize',30)
set(gca,'TickLabelInterpreter','latex')
legend('$\mathcal{J}(\xi,u)$','$\mathcal{J}(\xi,u_\kappa)$','Interpreter','Latex', 'Location', 'best')
zlabel('$\mathcal{J}(\xi,(\epsilon_u u_{\kappa_1},\epsilon_w u_{\kappa_2}))$','Interpreter','Latex')
ylabel('$\epsilon_w$','Interpreter','Latex')
xlabel('$\epsilon_u$','Interpreter','Latex')
set(gca,'TickLabelInterpreter','latex')
