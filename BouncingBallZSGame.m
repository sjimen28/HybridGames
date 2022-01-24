classdef BouncingBallZSGame < HybridSystem

    properties          %Make them match with the global declaration
        gravity = 1;
        lambda = 0.8;
        Rd1=10;
        Rd2=-20;
    end

    methods 
        % The data of the system is defined based on an implementation of 
        % the abstract functions from HybridSystem.m

        function xdot = flowMap(this, x, t, j)
            xdot = [x(2); -this.gravity];
        end

        function xplus = jumpMap(this, x, t, j)
            xplus = [0;...
                -this.lambda*x(2)+2*this.lambda*(this.Rd2+this.Rd1)*x(2)/((1+2*this.Rd1)*(1+2*this.Rd2)-1)];
        end
        
        function C = flowSetIndicator(this, x, t, j) 
            C = x(1) >= 0;
        end

        function D = jumpSetIndicator(this, x, t, j) 
            D = x(1) <= 0 && x(2) <= 0;
        end
    end

end