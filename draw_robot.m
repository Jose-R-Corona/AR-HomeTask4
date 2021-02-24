function draw_robot(q,L,color)
%draw a 7 degree of fredom robot, with FK archive!!
%figure;
%hold on
%view(200,25)
%grid on
%axis equal

T1=FK([q(1) 0 0 0 0 0 0],[L(1) 0 0 0 0 0 0]);
plot3(0,0,0,'ro','MarkerSize',10,'LineWidth', 5);
plot3([0 T1(1,4)],[0 T1(2,4)],[0 T1(3,4)],'-b','LineWidth', 5);

T_last=T1;
T_new=FK([q(1) q(2) 0 0 0 0 0],[L(1) L(2) 0 0 0 0 0]);
plot3(T_last(1,4),T_last(2,4),T_last(3,4),'ro','MarkerSize',10,'LineWidth', 5);
plot3([T_last(1,4) T_new(1,4)],[T_last(2,4) T_new(2,4)],[T_last(3,4) T_new(3,4)],color,'LineWidth', 5);

T_last=T_new;
T_new=FK([q(1) q(2) q(3) 0 0 0 0],[L(1) L(2) L(3) 0 0 0 0]);
plot3(T_last(1,4),T_last(2,4),T_last(3,4),'ro','MarkerSize',10,'LineWidth', 5);
plot3([T_last(1,4) T_new(1,4)],[T_last(2,4) T_new(2,4)],[T_last(3,4) T_new(3,4)],color,'LineWidth', 5);

T_last=T_new;
T_new=FK([q(1) q(2) q(3) q(4) 0 0 0],[L(1) L(2) L(3) L(4) 0 0 0]);
plot3(T_last(1,4),T_last(2,4),T_last(3,4),'ro','MarkerSize',10,'LineWidth', 5);
plot3([T_last(1,4) T_new(1,4)],[T_last(2,4) T_new(2,4)],[T_last(3,4) T_new(3,4)],color,'LineWidth', 5);

T_last=T_new;
T_new=FK([q(1) q(2) q(3) q(4) q(5) 0 0],[L(1) L(2) L(3) L(4) L(5) 0 0]);
plot3(T_last(1,4),T_last(2,4),T_last(3,4),'ro','MarkerSize',10,'LineWidth', 5);
plot3([T_last(1,4) T_new(1,4)],[T_last(2,4) T_new(2,4)],[T_last(3,4) T_new(3,4)],color,'LineWidth', 5);

T_last=T_new;
T_new=FK([q(1) q(2) q(3) q(4) q(5) q(6) 0],[L(1) L(2) L(3) L(4) L(5) L(6) 0]);
plot3(T_last(1,4),T_last(2,4),T_last(3,4),'ro','MarkerSize',10,'LineWidth', 5);
plot3([T_last(1,4) T_new(1,4)],[T_last(2,4) T_new(2,4)],[T_last(3,4) T_new(3,4)],color,'LineWidth', 5);

T_last=T_new;
T_new=FK([q(1) q(2) q(3) q(4) q(5) q(6) q(7)],[L(1) L(2) L(3) L(4) L(5) L(6) L(7)]);
plot3(T_last(1,4),T_last(2,4),T_last(3,4),'ro','MarkerSize',10,'LineWidth', 5);
plot3([T_last(1,4) T_new(1,4)],[T_last(2,4) T_new(2,4)],[T_last(3,4) T_new(3,4)],'-r','LineWidth', 5);

plot3(T_new(1,4),T_new(2,4),T_new(3,4),'ro','MarkerSize',10,'LineWidth', 5);


