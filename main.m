%%
%Input Vektor for initial position
x0 = zeros(8,1);
x0(1) = 0;          %x
x0(2) = 0;          %y
x0(3) = 0;          %theta0
x0(4) = 0;          %theta1
x0(5) = 0;          %phi

x0(6) = 1;          %xi1
x0(7) = 0;          %xi2
x0(8) = 0;          %xi3

x1 = zeros(8,1);
x1(1) = 20;          %x
x1(2) = 20;          %y
x1(3) = 0;          %theta0
x1(4) = 0;          %theta1
x1(5) = 0;          %phi

x1(6) = 0;          %xi1
x1(7) = 0;          %xi2
x1(8) = 0;          %xi3

x_start = x0(1);
x_end = x1(1);

global d0;
global d1;
global T;
d0 = 1;
d1 = 1;
T = 10;

coef = PathPlanner(x0,x1,d0,d1);


initialState = [x0(1)+d1*cos(x0(4));x0(2)+d1*sin(x0(4));x0(3:8)];
[t,state] = ode45(@myfun,[0,T],initialState,[],coef,x_start,x_end);

i = 1;
for t = 0:0.1:T
    [y_ref(i),x(i)] = loop(t,x_start,x_end,coef);
    i = i+1;
end

plot(state(:,1)-d1*cos(state(:,4)),state(:,2)-d1*sin(state(:,4)))
hold on 
plot(x,y_ref)
  
function [y_ref,x ]= loop(t,x_start,x_end,coef)
    global T
    tau = t/T;
    s = 3*tau^2-2*tau^3;
    
    x        = x_start+(x_end-x_start)*s;
    y_ref = polyval(coef,x);
end
    
function dx_dt=myfun(t,state,coef,x_start,x_end)

    global d1;
    global d0;
    global T;
    
    tau = t/T;
    s = 3*tau^2-2*tau^3;
    
    x        = x_start+(x_end-x_start)*s;
    x_dot    = 1/T*(x_end-x_start)*(6*tau-6*tau^2);




    y_ref = polyval(coef,x);
    yx = polyval(polyder(coef),x);
    yxx = polyval(polyder(polyder(coef)),x);
    yxxx = polyval(polyder(polyder(polyder(coef))),x);
    yxxxx = polyval(polyder(polyder(polyder(polyder(coef)))),x);
    
    LD = Truck_1T_LieDeriv(state(1:5), state(6:8), d0, d1);
    
    ysigma = LD.Lf_h2;
    ysigma2 = LD.Lf2_h2;
    ysigma3 = LD.Lf3_h2;
    
    xsigma  = LD.Lf_h1;
    xsigma2 = LD.Lf2_h1;
    xsigma3 = LD.Lf3_h1;

    [x_ref_prime_4,y_ref_prime_4] =  Fourth_Derivatives_of_References(yx,yxx,yxxx,yxxxx);
    
    eta = sqrt(1+(yx)^2);
        
    x_ref_prime = 1/eta;
    y_ref_prime = yx/eta;
    
    x_ref_prime_2 = -yx*yxx/eta^4;
    y_ref_prime_2 = yxx/eta^2 - yx^2*yxx/eta^4;
    
    x_ref_prime_3 = -(yxx^2+yx*yxxx)/eta^5+4*yx^2*yxx^2/eta^7;
    y_ref_prime_3 = yxxx/eta^3-4*yxx^2*yx/eta^5-yx^2*yxxx/eta^5+4*yx^3*yxx^2/eta^7;
    
    e_x = x - (state(1) - d1 * cos(state(4)));
    e_y = y_ref - (state(2) - d1 * sin(state(4)));

    coef_k = poly(-1/d0*ones(1,4));
    k3 = coef_k(2);
    k2 = coef_k(3);
    k1 = coef_k(4);
    k0 = coef_k(5);
    
 
    nu_1 = x_ref_prime_4  +  k3 * (x_ref_prime_3-xsigma3) + k2 * (x_ref_prime_2-xsigma2) + k1 * (x_ref_prime-xsigma) + k0 *e_x; 
    nu_2 = y_ref_prime_4  +  k3 * (y_ref_prime_3-ysigma3) + k2 * (y_ref_prime_2-ysigma2) + k1 * (y_ref_prime-ysigma) + k0 *e_y;
    
    w = [LD.L_g1_Lf3_h1 LD.L_g2_Lf3_h1; LD.L_g1_Lf3_h2, LD.L_g2_Lf3_h2]^-1*([nu_1;nu_2]-[LD.Lf4_h1;LD.Lf4_h2]);
    w_1 = w(1);
    w_2 = w(2);

    dx_dt = zeros(8,1);
    v0 = state(6)/(cos(state(3)-state(4))) * eta * x_dot;
    dx_dt(1) = v0*cos(state(3));
    dx_dt(2) = v0*sin(state(3));
    dx_dt(3) = v0/d0*tan(state(5));
    dx_dt(4) = v0/d1*sin(state(3)-state(4));
    dx_dt(5) = w_2 * eta *x_dot;
    dx_dt(6) = state(7) * eta * x_dot;
    dx_dt(7) = state(8) * eta * x_dot;
    dx_dt(8) = w_1 * eta * x_dot;
end

