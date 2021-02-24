function q = IK_pseudoInverseJacobbian(Td, q0, L0, only_position)
    
    e_o =1;
    eps_p = 1e-06; % error in position
    eps_o = 1e-05; % error in orientation
    % Max number of iteration  
    K = 500; 
    k = 0;
    % initialization of the variables
    q = q0';
    %just to avoid errors in q initial with 0 position
    if (norm(q0)==0)
        q = q0';
        q(1)=q(1)+0.01;
    end
    
    pd = Td(1:3, 4);            %position we want go
    Rd = Td(1:3, 1:3);          %orientation we want go
%     sd = rot2cuat(Rd); 
    
    T=FK(q,L0);%double(subs(FowardK, [q_symbs], [q'] ));
    p = T(1:3, 4);
    R = T(1:3, 1:3);
%     s = rot2cuat(R);
    
    e_p = pd - p;   %current position error
%     s_e = multcuat(sd, invcuat(s));
%     e_o = s_e(2:end); %current orientation error
    e = 1;
    
    % Complete the initialization and cycle responsible for implementing the iterative method
    %while((norm(e) > eps) && (k < K))
    while(((norm(e_p) > eps_p) || (norm(e_o) > eps_o)) && (k < K))
        T = FK(q,L0);%double(subs(FowardK, [q_symbs], [q'] ));
        J = Jacobian(q,L0);%double(subs(Jacobian, [q_symbs], [q'] ));
        
        p = T(1:3, 4);
        e_p = pd - p;   %is the error in position, destination position - position of this iteration
        
        if(only_position == 1)
            %J = J(1:3,:);   %first 3 rows, Jacobian secction for joints velocity 
            Ji = J'/(J*J'+0.1*eye(6)); %add small increment
            %Ji = pinv(J);
            e_o= [0;0;0];
        elseif(only_position == 2)  
            W=[2 0 0 0 0 0 0;   
               0 2 0 0 0 0 0;
               0 0 1 0 0 0 0;
               0 0 0 1 0 0 0;
               0 0 0 0 0.5 0 0;
               0 0 0 0 0 0.5 0;
               0 0 0 0 0 0 0.5;];           
            Ji=inv(W)*J'/(J*inv(W)*J'+0.1*eye(6))   ;         
            %Ji = pinv(J);
            e_o= [0;0;0];
        else
%             R = T(1:3, 1:3);
%             s = rot2cuat(R);
%             s_e = multcuat(sd, invcuat(s));
%             e_o = s_e(2:end);
%            
%             %Ji = J'/(J*J'+0.1*eye(6)); %pseudo inverse
%             Ji = pinv(J); %pseudo inverse
        end
        e = [e_p; e_o]; %error in position and error in orientation 1x6
        q = q + Ji*e; % The solution is updated
        k = k + 1; % the iteration number is uodated
    end
    i=1;
    while (i<7) % angles  #
        q(i)=wrapToPi(q(i));
        i=i+1;
    end
end