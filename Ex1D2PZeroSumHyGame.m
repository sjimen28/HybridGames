%--------------------------------------------------------------------------
% Matlab M-file Project: 2-Player Zero-Sum HyGames @  Hybrid Systems Laboratory (HSL), 
% Filename: Ex1D2PZeroSumHyGame.m
%--------------------------------------------------------------------------
% Project: Example - 1D Linear Quadratic Hybrid Game with Nonunique Solutions
% Author: Santiago Jimenez Leudo
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%   Make sure to install HyEQ Toolbox (Beta) at
%   https://www.mathworks.com/matlabcentral/fileexchange/102239-hybrid-equations-toolbox-beta 
%   Copyright @ Hybrid Systems Laboratory (HSL),
%   Revision: 0.0.0.2 Date: 01/23/2022 9:45:00

clear all
clc 
% --------------------------------------------------------------
%%% Initialization
% --------------------------------------------------------------
%

% Simulation Horizon
TSPAN=[0 3];
JSPAN = [0 20];
t=0:0.001:TSPAN(2);

%%% Continuous Dynamics
%f(x,uC)=a*x+b*uC     Flow Map
%b=[b1;b2];
%C=[0,delta]              Flow Set
%uC=(uc1,uc2)         Continuos Input
a=-1;
b1=1;
b2=1;
delta=2;

%%% Stage Cost during Flows
Q=1;  
R1=1.304;
R2=-4;
Lc=@(x,u1,u2) x^2*Q+u1^2*R1+u2^2*R2;


%%% Discrete Dynamics
%g(x,uD)=sigma
%D={mu}
sigma=0.5;
mu=1;

%%% Stage Cost during Jumps
P=0.4481;
% Caluculated as the positive answer of the robust CARE:
%   syms Pv
%   P=simplify(solve(Q+2*Pv*a-Pv^2/R1-Pv^2/R2==0,Pv))
Ld=@(x) P*(x^2-0.5^2);

%%% Discrete Input
ud=0;


%%% Lyapunov Function
V=@(x) P*x^2;


%%% Initial State 
xi = 2;


% --------------------------------------------------------------
%%% System Evolution + Cost Computation
% --------------------------------------------------------------
%
for jump=[0 1] %Run the continuous solution when j=0 and 
               %    the hybrid solution when j=1.
               %At x=1, the solution can either flow or jump
    if jump>0
        clear x uc1 uc2 J
    end
       
    x(1)=xi;                % Initial State
    uc1(1)=-b1*P*x(1)/R1;   % Intial Input Player P1
    uc2(1)=-b2*P*x(1)/R2;   % Intial Input Player P2
    J(1)=0;                 % Initial Cost
    
    for i=1:length(t)-1
        if abs(x(i)-mu)>=0.01 && 0<=x(i)<=2    % Check if x is in the Flow Set
            x(i+1)=x(i)+(t(i+1)-t(i))*(a*x(i)+b1*uc1(i)+b2*uc2(i)); % Evolve via flow
            J(i+1)=J(i)+(t(i+1)-t(i))*(Lc(x(i),uc1(i),uc2(i)));     % Add Continuous Cost
        else                    % Check if x is in the Jump Set
            if jump==1          % Hybrid Solution
                J(i)=J(i)+Ld(x(i));     % Add Discrete Cost
                x(i)=sigma;             % Evolve via jump
                uc1(i)=-b1*P*x(i)/R1;   % Update Input P1 for flow
                uc2(i)=-b2*P*x(i)/R2;   % Update Input P2 for flow
            end
            x(i+1)=x(i)+(t(i+1)-t(i))*(a*x(i)+b1*uc1(i)+b2*uc2(i)); % Evolve via flow after jump
            J(i+1)=J(i)+(t(i+1)-t(i))*Lc(x(i),uc1(i),uc2(i));
            jj=i;                       % Save index of jump
        end
        
        uc1(i+1)=-b1*P*x(i+1)/R1;       % Update Input P1 for next time step
        uc2(i+1)=-b2*P*x(i+1)/R2;       % Update Input P2 for next time step     
    end
    
    if jump==0 % Store Continuous Solution Data 
        phik=x;
        uk=uc1;
        wk=uc2;
        Jk=J;
    else       % Store Hybrid Solution Data
        phih=x;
        uh=uc1;
        wh=uc2;
        Jh=J;
    end
end

% Create Hybrid Solution by using HyEQ Toolbox

jv=[linspace(0,0,jj-1), linspace(1,1,size(t,2)-jj+1)];  % Discrete Time 

solphih=HybridSolution(t',jv',phih');                   % Hybrid Response
soluh=HybridSolution(t',jv',uh');                       % P1 Action for Hybrid Solution
solwh=HybridSolution(t',jv',wh');                       % P2 Action for Hybrid Solution
solcost=HybridSolution(t',jv',Jh');                     % Cost of Hybrid Solution

%
% --------------------------------------------------------------
%%% Plot
% --------------------------------------------------------------
%
clf
figure (1)
set(0,'defaultfigurecolor',[1 1 1])                 
set(0,'defaulttextinterpreter','latex')
set(gcf,'color','w');

plot_builder_bb = HybridPlotBuilder();

subplot(3,1,1)
plot_builder_bb.flowColor('#017daa') ...            $ Set Plot Properties             
    .legend('$\phi_h$')...
    .slice(1)...
    .labels('$x_1$') ...
    .autoSubplots('off')...
    .configurePlots(@apply_plot_settings)...
    .jumpColor('#e43d43')...
    .plotFlows(solphih);                            % Plot Hybrid Response
hold on
pk=plot(t,phik, 'color', '#3f9f38')                 % Plot Continuos Response
plot_builder_bb.addLegendEntry(pk,'$\phi_\kappa$'); 
ylabel('$x$','Interpreter','Latex')


subplot(3,1,2)
plot_builder_bb.flowColor('#017daa') ...            $ Set Plot Properties
    .legend('$u_h$')...
    .slice([1])...
    .configurePlots(@apply_plot_settings)...
    .autoSubplots('off')...
    .jumpColor('#e43d43')...
    .plotFlows(soluh);                              % Plot P1 Action for HySolution
hold on
plot_builder_bb.flowColor('#017daa') ...            $ Set Plot Properties
    .legend('')...
    .slice([1])...
    .configurePlots(@apply_plot_settings)...
    .autoSubplots('off')...
    .jumpColor('#e43d43')...
    .plotFlows(solwh);                              % Plot P2 Action for HySolution
hold on
puk=plot(t,uk,'color','#3f9f38')                    % Plot P1 Action for ContSolution
hold on
pwk=plot(t, wk,'color','#3f9f38')                   % Plot P2 Action for ContSolution
hold on
plot_builder_bb.addLegendEntry(puk,'$u_\kappa$');
ylabel('$u_C$','Interpreter','Latex')

%

subplot(3,1,3)
plot_builder_bb.flowColor('#017daa') ...            % Set Plot Properties
    .legend('$J(\xi,u_h)$')...
    .labels('$J$') ...
    .configurePlots(@apply_plot_settings)...
    .autoSubplots('off')...
    .jumpColor('#e43d43')...
    .legend('$J(2,u_h)$','Location', 'southeast')...
    .plotFlows(solcost)                             % Plot Cost of Hybrid Solution
hold on
ck=plot(t,Jk, 'color', '#3f9f38')                   % Plot Cost of Continuous Solution
hold on
V0=V(xi);
X=V0*ones(1,length(t));
vf=plot(t,X, 'color', 'black');                     % Estimated Optimal Cost 
set(gca,'TickLabelInterpreter','latex')
yticks([0 .5 1 1.5 V0])
yticklabels({'0' '0.5' '1' '1.5' '$V(2)$'})
plot_builder_bb.addLegendEntry(ck,'$J(2,u_\kappa)$');
plot_builder_bb.addLegendEntry(vf,'$J^*(2)$');


function apply_plot_settings(component)
xlabel('$t$ [s]')
ax = gca;
ax.TickLabelInterpreter='latex';
ax.LineWidth=0.25;
end