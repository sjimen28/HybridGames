%--------------------------------------------------------------------------
% Matlab M-file Project: 2-Player Zero-Sum HyGames @  Hybrid Systems Laboratory (HSL), 
% Filename: ExJumpsActBouncingBallUnderAttack.m
%--------------------------------------------------------------------------
% Project: Example - Jumps Actuated Bouncing Ball under Attack as a 
%           Two-Player Zero-Sum Hybrid Game
% Author: Santiago Jimenez Leudo
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
%   Make sure to install HyEQ Toolbox (Beta) v3.0.0.22 from
%   https://www.mathworks.com/matlabcentral/fileexchange/102239-hybrid-equations-toolbox-beta 
%   (View Version History) 
%   Copyright @ Hybrid Systems Laboratory (HSL),
%   Revision: 00.0.0.4 Date: 02/07/2022 17:20:00

clear all
clc

% --------------------------------------------------------------
%%% Initialization
% --------------------------------------------------------------
%   Paremeters: Ac, Bc1, Bc2, gammac, lambda, RD1, RD2, QD, x0
%   Modify any parameter in this section to simulate the system/case of interest

% Simulation Horizon
TSPAN=[0 30];   %Second entry is the maximum amount of seconds allowed
JSPAN = [0 12]; %Second entry is the maximum amount of jumps allowed
Ts=0.01;        %Steptime
t=0:Ts:TSPAN(2);


%%% Continuous Dynamics
%f(x,u_C)=Ac*x+Bc*u_C       Flow Map
%C={x:x1>=0}                Flow Set        
%uC=(uc1,uc2)         Continuos Input
Ac=[0 1; 0 0];
Bc1=0;
Bc2=0;
Bc=[Bc1 Bc2];
gammac=[0;-1];
f=@(x,u_C,gammac) Ac*x+Bc*u_C+gammac;

%%% Stage Cost during Flows
Qc=zeros(2);  
Rc1=0;
Rc2=0;
Lc=@(x,uC) x'*Qc*x+uC(1)^2*Rc1+uC(2)^2*Rc2;

%%% Continuous Input
uC=[0;0];


%%% Discrete Dynamics
%g(x,u_D)=0.5
%D={1}
lambda=0.8;             %Coefficient of Restitution
Ad=[0 0; 0 -lambda];
Bd=[0 0;1 1];
gammad=0;
g=@(x,u_D, gammad) Ad*x+Bd*u_D+gammad;

%%% Stage Cost during Jumps
Rd1=10;
Rd2=-20;
Qd=(- 2*Rd1*Rd2*lambda^2 + Rd1 + Rd2 + 2*Rd1*Rd2)/(2*Rd1 + 2*Rd2 + 4*Rd1*Rd2);
Ld=@(x,uD) x(2)^2*Qd+uD(1)^2*Rd1+uD(2)^2*Rd2;

%%% Discrete Input
ud1f=@(x) 2*lambda*Rd2*x(2)/((1+2*Rd1)*(1+2*Rd2)-1);
ud2f=@(x) 2*lambda*Rd1*x(2)/((1+2*Rd1)*(1+2*Rd2)-1);


%%% Lyapunov Function
V=@(x) 0.5*x(2)^2+x(1);


%%% Initial State 
x0 = [1,1];


% --------------------------------------------------------------
%%% System Evolution 
% --------------------------------------------------------------
%
%%%  Create Hybrid Simulation by using HyEQ Toolbox
 system_bb = BouncingBallZSGame();                  % Create a BouncingBallZSGame HySytem
 config = HybridSolverConfig('MaxStep', 0.1);       % Make plots smoother
 sol_bb = system_bb.solve(x0, TSPAN, JSPAN, config) % Run Simulation
                                                    % sol_bb has the data of the simulation
 ud1opt=2*lambda*Rd2*sol_bb.x(:,2)/((1+2*Rd1)*(1+2*Rd2)-1); %Calculate Optimal Control Action
 ud2opt=2*lambda*Rd1*sol_bb.x(:,2)/((1+2*Rd1)*(1+2*Rd2)-1); %Calculate Optimal Attack
 
% --------------------------------------------------------------
%%% Cost Computation 
% --------------------------------------------------------------
%
 
 J(1)=0;                                                    %Initial Cost
for i=1:size(sol_bb.t,1)-1       
    if sol_bb.j(i+1,1)>sol_bb.j(i,1)                        % Check if a jump is reported
        J(i)=J(i)+Ld(sol_bb.x(i,:),[ud1opt(i),ud2opt(i)]);  % Augment cost via jump
        J(i+1)=J(i)+(sol_bb.t(i+1)-sol_bb.t(i))*Lc(sol_bb.x(i,:)',uC); 
                                                            % Augment cost via flow
    else  
        J(i+1)=J(i)+(sol_bb.t(i+1)-sol_bb.t(i))*Lc(sol_bb.x(i,:)',uC);
                                                            % Augment cost via flow
    end
end

% Create Hybrid Trajectory for Cost by using HyEQ Toolbox
solcost=HybridArc(sol_bb.t,sol_bb.j,J');

%
% --------------------------------------------------------------
%%% Plot
% --------------------------------------------------------------
%
figure(1)
set(0,'defaultfigurecolor',[1 1 1])                 % White Bakground
set(0,'defaulttextinterpreter','latex')

plot_builder_bb = HybridPlotBuilder();

subplot(5,1,1)
plot_builder_bb.flowColor('#017daa') ...            $ Set Plot Properties
    .slice([1])...
    .labels('$x_1$') ...
    .autoSubplots('off')...
    .configurePlots(@apply_plot_settings)...
    .jumpColor('#e43d43')...
    .plotFlows(sol_bb);                             % Plot Ball's Position 

subplot(5,1,2)
plot_builder_bb.flowColor('#017daa') ...            $ Set Plot Properties
    .slice([2])...
    .labels('$x_2$') ...
    .configurePlots(@apply_plot_settings)...
    .autoSubplots('off')...
    .jumpColor('#e43d43')...
    .plotFlows(sol_bb);                             % Plot Ball's Valocity

subplot(5,1,3)
plot_builder_bb.flowColor('#017daa') ...            $ Set Plot Properties
    .slice(2)...
    .labels('$u_{\kappa 1}$') ...
    .configurePlots(@apply_plot_settings)...
    .autoSubplots('off')...
    .jumpColor('#e43d43')...
    .plotFlows(sol_bb, @(x2) 2*lambda*Rd2*x2/((1+2*Rd1)*(1+2*Rd2)-1));
                                                    % Control Action

subplot(5,1,4)
plot_builder_bb.flowColor('#017daa') ...            $ Set Plot Properties
    .labels('$u_{\kappa 2}$') ...
    .configurePlots(@apply_plot_settings)...
    .autoSubplots('off')...
    .jumpColor('#e43d43')...
    .plotFlows(sol_bb, @(x2) 2*lambda*Rd1*x2/((1+2*Rd1)*(1+2*Rd2)-1))...
                                                    % Attacker's Action
subplot(5,1,5)
plot_builder_bb.flowColor('#017daa') ...            $ Set Plot Properties
    .labels('$\mathcal{J}$') ...
    .configurePlots(@apply_plot_settings)...
    .autoSubplots('off')...
    .jumpColor('#e43d43')...
    .slice(1)...
    .plotFlows(solcost)                             % Cost of Solution
hold on
V0=V(x0);
X=V0*ones(1,length(sol_bb.t));
vf=plot(sol_bb.t,X, 'color', 'black');              % Estimated Optimal Cost 
set(gca,'TickLabelInterpreter','latex')
xlabel('$t$ [s]')
if x0 == [1,1]
    yticks([0 0.5 1 V0])
    yticklabels({'0' '0.5' '1' '$V([1;1])$'})
    axis([0 14 0 2])
end


run('SaddlePointBouncingBall.m')

function apply_plot_settings(ax, component)
    xlabel('')
    ax.TickLabelInterpreter='latex';
    ax.LineWidth=0.25;
end