
function mpc_inp = MPC_Car(Ad,Bd,P,Q,R,hor,X_in,Dd,path2, uMax, uMin, xx_lb, xx_ub)     
% equality constraints
    %ny1 = size(AA,2); 
    nu =size(Bd,2); ny = size(Ad,1);
    bigA = zeros(ny*hor,ny*hor);
    for ii = 1:ny*hor
        bigA(ii,ii)=1;
    end
    for ii = 1:hor-1
        for jj = 1:ny
            for kk = 1:ny
                bigA(ny*ii+jj,ny*(ii-1)+kk)=-Ad(jj,kk); 
            end
        end
    end
%     bigA = [AA, zeros(ny,ny1*(hor-1))];
%     for ii = 1:hor-1
%         bigA = [bigA ; zeros(ny,ny1*(ii-1)),-AA,AA,zeros(ny,ny1*(hor-ii-1))];
%     end
%     
    bigB = zeros(ny*hor,nu*hor);
    for ii = 0:hor-1
        for jj = 1:ny
            for kk = 1:nu
                bigB(ny*ii+jj,nu*ii+kk)=-Bd(jj,kk); 
            end
        end
    end
    Eq_RHS = [Ad*X_in;zeros(ny*(hor-1),1)] + repmat(Dd,hor,1);
    bigEq = [bigA,bigB];
    
    
    % Cost function Matrix 
    Qbar = zeros(ny*hor,ny*hor);
    for ii = 0:hor-1
        for jj = 1:ny
            for kk = 1:ny
                if ii<hor-1
                    Qbar(ny*ii+jj,ny*ii+kk)=Q(jj,kk); 
                else
                    Qbar(ny*ii+jj,ny*ii+kk)=P(jj,kk); 
                end
            end
        end
    end
    
    Rbar = zeros(nu*hor,nu*hor);
    for ii = 0:hor-1
        for jj = 1:nu
            for kk = 1:nu
                    Rbar(nu*ii+jj,nu*ii+kk)=R(jj,kk); 
            end
        end
    end
    QR_bar = [Qbar,zeros(ny*hor,nu*hor);zeros(nu*hor,ny*hor),Rbar];
    
    % Inequality constraints
    
    u_lb = repmat(uMin,hor,1);
    u_ub = repmat(uMax,hor,1);

    %xx_lb = [-1000;-1000;-1000;-1000]; %taken randomly for now
    x_lb = repmat(xx_lb,hor,1);
    %xx_ub = [10;10;1000;1000];
    x_ub = repmat(xx_ub,hor,1);
   
    Aineq = [eye(hor*(ny+nu));-eye(hor*(ny+nu))];
    Bineq = [x_ub;u_ub;-x_lb;-u_lb];

    %qp solve
    ff = [];
    for ii = 1:1:hor-1
        ff = [ff,-1*path2(:,ii)'*Q];
    end
    ff = [ff,-1*path2(:,hor)'*P];
    ff = [ff, zeros(1,nu*hor)];

    ff1 = ff';
    %x1_u1 = quadprog(QR_bar,ff1,Aineq,Bineq,bigEq,Eq_RHS);
    [x1_u1,~,~]=qpSWIFT(sparse(QR_bar),ff1,sparse(bigEq),Eq_RHS,sparse(Aineq),Bineq);

    % plots

    mpc_inp = x1_u1(7*hor+1:7*hor+2);
       
end

