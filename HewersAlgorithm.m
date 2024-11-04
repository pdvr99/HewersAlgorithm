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
    u(:,:,i) = -F0*x0;
    P(:,:,i) = Pi;
end


display(Jopt_values);

display(u);

display(F0_values);

display(P);



%Approximate performance criterion
figure(1)
i = 1:1:10;
plot(i, Jopt_values(i), '*-b');
grid; 
xlabel('Iteration number');
ylabel('Approximate oprtimal performance')

k=1:1:10; %Array of [1,...,100]
for j=1:1:10
    x(:,j) = ((A-B*Fopt)^k(j))*x0;
    x_values=((A-B*Fopt)^k(j))*x0;
    x_system_state_response(:,:,j) = x_values;
end

display(x_system_state_response);


    
figure (2)
plot(k,x(1,:),'*-r')
    grid; 
xlabel('Time [s]');
ylabel('State variable x1')

figure (3)
plot(k,x(2,:),'*-b')
grid; 
xlabel('Time [s]');
ylabel('State variable x2')

figure (4)
plot(k,x(3,:),'*-k')
grid; 
xlabel('Time [s]');
ylabel('State variable x3')

figure (5)
plot(k,x(4,:),'*-g')
grid; 
xlabel('Time [s]');
ylabel('State variable x4')

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

k = 1:1:10;

figure(8)
plot(k,x(1,:),'*-r')
grid;
xlabel('Time [s]');
ylabel('State variable')



hold on
plot(k,x(2,:),'o-b') 
plot(k,x(3,:),'*-k')
plot(k,x(4,:),'o-g')

legend('x1','x2','x3','x4')
hold off

%%

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

figure(10)
out = sim("LQR.slx");

timetable = out.x;

plot(timetable.Time, timetable.Data);
grid;
xlabel('Time');
ylabel('State Response')
legend('x1','x2','x3','x4');







