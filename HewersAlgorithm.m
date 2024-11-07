A = [0.99 0.03 -0.02 -0.32;
     0.01 0.47 4.7 0.0; 
     0.02 -0.06 0.4 0.0;
     0.01 -0.04 0.72 0.99];

B = [0.01 0.88;
    -3.22 1.66;
    -0.83 0.44;
    -0.47 0.25];

x0 = [1; 1; 1; 1]; %assumed initial condition 


%LQR Optimal Control 
Q = eye(4); 
R = eye(2);  

%Test for controllability and observability
CO = ctrb(A, B); 
rankCO = rank(CO); %If rank is equal to n (system order) implies controlability. 

OM = obsv(A, Q);
rOM = rank(OM); %If rank equal to n implies observability

%Here is a direct solution of the disrete-time optimal LQ problem 
[Fopt, P] = dlqr(A, B, Q, R); 

Jopt = (1/2) * x0' * P * x0; 
    
uopt = -Fopt * x0; 

%Reinforcement Learning solution (Hewer's Algorithm) 


F0 = zeros(2, 4); 


for i = 1:1:10
    Pi = dlyap ((A-B*F0)', Q+F0'*R*F0); 
    Jopti = 0.5* x0' * Pi * x0; 
    Jopt_values(i) = Jopti; 
    F0 = inv(R+B'*Pi*B)*B'*Pi*A;
    F0_values(:,:,i) = F0;
    P(:,:,i) = Pi;
end

%Controls at different gains at iterations 1, 2, 3, 4, and 5
for i = 1:1:6
    u(:,:,i) = -F0_values(:,:,i)*x0;
end



display(Jopt_values);

display(u);

display(F0_values);

display(P);



%Approximate performance criterion
figure(1)
i = 1:1:10;
plot(i, Jopt_values(i), '*b');
grid; 
xlabel('Iteration number');
ylabel('Approximate oprtimal performance')


%State response with optimal gain 
k=1:1:30; %Array of [1,...,100]
for j=1:1:30
    x(:,j) = ((A-B*Fopt)^k(j))*x0;
    x_values=((A-B*Fopt)^k(j))*x0;
    x_system_state_response(:,:,j) = x_values;
end

display(x_system_state_response);

        
figure (2)
plot(k,x(1,:),'r')
grid; 
xlabel('Time [s]');
ylabel('State variable x1')

figure (3)
plot(k,x(2,:),'b')
grid; 
xlabel('Time [s]');
ylabel('State variable x2')

figure (4)
plot(k,x(3,:),'k')
grid; 
xlabel('Time [s]');
ylabel('State variable x3')

figure (5)
plot(k,x(4,:),'g')
grid; 
xlabel('Time [s]');
ylabel('State variable x4')


%Controls at different gains 
k = 1:1:6;
figure(6)
plot(k, u(1,:));
grid; 
xlabel('Time [s]');
ylabel('Control variable u1')

figure(7)
plot(k, u(2,:));
grid; 
xlabel('Time [s]');
ylabel('Control variable u2')


%%%%%

k = 1:1:30;
figure(8)
plot(k,x(1,:),'r')
grid;
xlabel('Time [s]');
ylabel('State Response')



hold on
plot(k,x(2,:),'b') 
plot(k,x(3,:),'k')
plot(k,x(4,:),'g')

legend('x1','x2','x3','x4')
hold off

%%

%Controls together at different gains 
k = 1:1:6;
figure(9)
plot(k, u(1,:))
grid; 
xlabel('Time [s]')
ylabel('Controls')


hold on
plot(k, u(2,:))

legend('u1', 'u2')
hold off

%% 


%Simulation
figure(10)
out = sim("LQR.slx");

timetable = out.x;

plot(timetable.Time, timetable.Data);
grid;
xlabel('Time');
ylabel('State Response')
legend('x1','x2','x3','x4');

% 

%F1
k=1:1:30; %Array of [1,...,100]
for j=1:1:30
    xF1(:,j) = ((A-B*F0_values(:,:,1))^k(j))*x0;
    x_Foptimal_1 = ((A-B*F0_values(:,:,1))^k(j))*x0;
    x_Foptimal_1_state_trajectory(:,:,j) = x_Foptimal_1;
end


figure(11)
plot(k,xF1(1,:),'r')
grid;
xlabel('Time [s]');
ylabel('State response with gain at iteration 1')



hold on
plot(k,xF1(2,:),'b') 
plot(k,xF1(3,:),'k')
plot(k,xF1(4,:),'g')

legend('x1','x2','x3','x4')
hold off



%F2
k=1:1:30; %Array of [1,...,100]
for j=1:1:30
    xF2(:,j) = ((A-B*F0_values(:,:,2))^k(j))*x0;
    x_Foptimal_2 = ((A-B*F0_values(:,:,2))^k(j))*x0;
    x_Foptimal_2_state_trajectory(:,:,j) = x_Foptimal_2;
end

figure(12)
plot(k,xF2(1,:),'r')
grid;
xlabel('Time [s]');
ylabel('State response with gain at iteration 2')



hold on
plot(k,xF2(2,:),'b') 
plot(k,xF2(3,:),'k')
plot(k,xF2(4,:),'g')

legend('x1','x2','x3','x4')
hold off





%F3
k=1:1:30; %Array of [1,...,100]
for j=1:1:30
    xF3(:,j) = ((A-B*F0_values(:,:,3))^k(j))*x0;
    x_Foptimal_3 = ((A-B*F0_values(:,:,3))^k(j))*x0;
    x_Foptimal_3_state_trajectory(:,:,j) = x_Foptimal_2;
end

figure(13)
plot(k,xF3(1,:),'r')
grid;
xlabel('Time [s]');
ylabel('State response with gain at iteration 3')



hold on
plot(k,xF3(2,:),'b') 
plot(k,xF3(3,:),'k')
plot(k,xF3(4,:),'g')

legend('x1','x2','x3','x4')
hold off





%F4
k=1:1:30; %Array of [1,...,100]
for j=1:1:30
    xF4(:,j) = ((A-B*F0_values(:,:,4))^k(j))*x0;
    x_Foptimal_4 = ((A-B*F0_values(:,:,4))^k(j))*x0;
    x_Foptimal_4_state_trajectory(:,:,j) = x_Foptimal_2;
end

figure(14)
plot(k,xF4(1,:),'r')
grid;
xlabel('Time [s]');
ylabel('State response with gain at iteration 4')



hold on
plot(k,xF4(2,:),'b') 
plot(k,xF4(3,:),'k')
plot(k,xF4(4,:),'g')

legend('x1','x2','x3','x4')
hold off




%F5
k=1:1:30; %Array of [1,...,100]
for j=1:1:30
    xF5(:,j) = ((A-B*F0_values(:,:,5))^k(j))*x0;
    x_Foptimal_5 = ((A-B*F0_values(:,:,5))^k(j))*x0;
    x_Foptimal_5_state_trajectory(:,:,j) = x_Foptimal_2;
end

figure(15)
plot(k,xF5(1,:),'r')
grid;
xlabel('Time [s]');
ylabel('State response with gain at iteration 5')



hold on
plot(k,xF5(2,:),'b') 
plot(k,xF5(3,:),'k')
plot(k,xF5(4,:),'g')

legend('x1','x2','x3','x4')
hold off

%Comparing x1 at different gains 

figure(16)
plot(k,xF1(1,:),'r')
grid;
xlabel('Time [s]');
ylabel('x1 at different iterations')



hold on
plot(k,xF2(1,:),'g') 
plot(k,xF3(1,:),'b')
plot(k,xF4(1,:),'o-c')
plot(k,xF5(1,:),'k')

legend('At Iteration 1','At Iteration 2','At Iteration 3','At Iteration 4','At Iteration 5')
hold off

%Comparing x2 at different gains 

figure(17)
plot(k,xF1(2,:),'r')
grid;
xlabel('Time [s]');
ylabel('x2 at different iterations')



hold on
plot(k,xF2(2,:),'g') 
plot(k,xF3(2,:),'b')
plot(k,xF4(2,:),'c')
plot(k,xF5(2,:),'o-k')

legend('At Iteration 1','At Iteration 2','At Iteration 3','At Iteration 4','At Iteration 5')
hold off

%Comparing x3 at different gains 

figure(18)
plot(k,xF1(3,:),'r')
grid;
xlabel('Time [s]');
ylabel('x3 at different iterations')



hold on
plot(k,xF2(3,:),'g') 
plot(k,xF3(3,:),'b')
plot(k,xF4(3,:),'o-c')
plot(k,xF5(3,:),'k')

legend('At Iteration 1','At Iteration 2','At Iteration 3','At Iteration 4','At Iteration 5')
hold off

%Comparing x4 at different gains 

figure(19)
plot(k,xF1(3,:),'r')
grid;
xlabel('Time [s]');
ylabel('x4 at different iterations')



hold on
plot(k,xF2(4,:),'g') 
plot(k,xF3(4,:),'o-b')
plot(k,xF4(4,:),'c')
plot(k,xF5(4,:),'k')

legend('At Iteration 1','At Iteration 2','At Iteration 3','At Iteration 4','At Iteration 5')
hold off











