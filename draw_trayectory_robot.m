function draw_trayectory_robot(q1,q2,q3,L)

figure;
hold on
view(200,25)
grid on
axis equal


draw_robot(q1,L,'-b')
draw_robot(q2,L,'-g')
draw_robot(q3,L,'-r')

T_last=FK(q1,L);
T_new=FK(q2,L);
plot3([T_last(1,4) T_new(1,4)],[T_last(2,4) T_new(2,4)],[T_last(3,4) T_new(3,4)],'--g','LineWidth', 3);

T_last=T_new;
T_new=FK(q3,L);
plot3([T_last(1,4) T_new(1,4)],[T_last(2,4) T_new(2,4)],[T_last(3,4) T_new(3,4)],'--g','LineWidth', 3);

