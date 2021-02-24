function Jq1 = Jacobian(q,L)

    %Jacobian Numerical method

    H = Rz(q(1))*Tz(L(1))*Rx(q(2))*Tz(L(2))*Rz(q(3))*Tz(L(3))*Rx(-q(4))*Tz(L(4))*Rz(q(5))*Tz(L(5))*Rx(q(6))*Tz(L(6))*Rz(q(7))*Tz(L(7));% forward kinematics
    %H=simplify(H);
    R = H(1:3,1:3);
    %R = simplify(H(1:3,1:3));  % extract rotation matrix
    % diff by q1
    Td=Rzd(q(1))*Tz(L(1))*Rx(q(2))*Tz(L(2))*Rz(q(3))*Tz(L(3))*Rx(-q(4))*Tz(L(4))*Rz(q(5))*Tz(L(5))*Rx(q(6))*Tz(L(6))*Rz(q(7))*Tz(L(7))*...
        [R^-1 zeros(3,1);0 0 0 1];
    J_1 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ; % extract 6 components from 4x4 Td matrix to Jacobian 1st column
    % diff by q2
    Td=Rz(q(1))*Tz(L(1))*Rxd(q(2))*Tz(L(2))*Rz(q(3))*Tz(L(3))*Rx(-q(4))*Tz(L(4))*Rz(q(5))*Tz(L(5))*Rx(q(6))*Tz(L(6))*Rz(q(7))*Tz(L(7))*...
        [R^-1 zeros(3,1);0 0 0 1];
    J_2 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ; % extract 6 components from 4x4 Td matrix to Jacobian 1st column
    % diff by q3
    Td=Rz(q(1))*Tz(L(1))*Rx(q(2))*Tz(L(2))*Rzd(q(3))*Tz(L(3))*Rx(-q(4))*Tz(L(4))*Rz(q(5))*Tz(L(5))*Rx(q(6))*Tz(L(6))*Rz(q(7))*Tz(L(7))*...
        [R^-1 zeros(3,1);0 0 0 1];
    J_3 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ; % extract 6 components from 4x4 Td matrix to Jacobian 1st column

    Td=Rz(q(1))*Tz(L(1))*Rx(q(2))*Tz(L(2))*Rz(q(3))*Tz(L(3))*Rxd(-q(4))*Tz(L(4))*Rz(q(5))*Tz(L(5))*Rx(q(6))*Tz(L(6))*Rz(q(7))*Tz(L(7))*...
        [R^-1 zeros(3,1);0 0 0 1];
    J_4 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ; % extract 6 components from 4x4 Td matrix to Jacobian 1st column

    Td=Rz(q(1))*Tz(L(1))*Rx(q(2))*Tz(L(2))*Rz(q(3))*Tz(L(3))*Rx(-q(4))*Tz(L(4))*Rzd(q(5))*Tz(L(5))*Rx(q(6))*Tz(L(6))*Rz(q(7))*Tz(L(7))*...
        [R^-1 zeros(3,1);0 0 0 1];
    J_5 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ; % extract 6 components from 4x4 Td matrix to Jacobian 1st column

    Td=Rz(q(1))*Tz(L(1))*Rx(q(2))*Tz(L(2))*Rz(q(3))*Tz(L(3))*Rx(-q(4))*Tz(L(4))*Rz(q(5))*Tz(L(5))*Rxd(q(6))*Tz(L(6))*Rz(q(7))*Tz(L(7))*...
        [R^-1 zeros(3,1);0 0 0 1];
    J_6 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ; % extract 6 components from 4x4 Td matrix to Jacobian 1st column

    Td=Rz(q(1))*Tz(L(1))*Rx(q(2))*Tz(L(2))*Rz(q(3))*Tz(L(3))*Rx(-q(4))*Tz(L(4))*Rz(q(5))*Tz(L(5))*Rx(q(6))*Tz(L(6))*Rzd(q(7))*Tz(L(7))*...
        [R^-1 zeros(3,1);0 0 0 1];
    J_7 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ; % extract 6 components from 4x4 Td matrix to Jacobian 1st column

    
        
    % Full Jacobian 6x7
    Jq1 = [J_1, J_2, J_3, J_4, J_5, J_6, J_7];
end
