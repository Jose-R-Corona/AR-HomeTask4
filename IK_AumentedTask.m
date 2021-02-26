function q = IK_AumentedTask(Td, q0, L0)
    
    eps_p = 1e-06; % min error in position
    eps_o = 1e-05; % min error in orientation
     
    K = 5000; % Max number of iteration 
    k = 0;
    % initialization of the variables
    q = q0';
    %just to avoid errors in q initial with 0 position
    if (norm(q0)==0)
        q = q0';
        q(1)=q(1)+0.01;
    end
    
    pd = Td(1:3, 4);            %position we want go
    
    T=FK(q,L0);%double(subs(FowardK, [q_symbs], [q'] ));
    p = T(1:3, 4);
    e_p = pd - p;   %current position error
    
    
    
    % Complete the initialization and cycle responsible for implementing the iterative method
    while(((norm(e_p) > eps_p)) && (k < K))
        T = FK(q,L0);%double(subs(FowardK, [q_symbs], [q'] ));
        J = Jacobian(q,L0);%double(subs(Jacobian, [q_symbs], [q'] ));

        p = T(1:3, 4);
        e_p = pd - p;   %is the error in position, destination position - position of this iteration
        e_o= [0;0;0];
        e = [e_p; e_o]; %error in position and error in orientation 1x6
        
        
        
        q = q + delta_q;
        k = k + 1; % the iteration number is uodated
    end
    i=1;
    while (i<7) % angles  #
        q(i)=wrapToPi(q(i));
        i=i+1;
    end
end