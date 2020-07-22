function DrawCar(centerLocation,theta,phi)

%%
%Setup
x = centerLocation(1);
y = centerLocation(2);

x_shift = x-x;
y_shift = y-y;

global L;
L =2;
radius = L./tan(phi);

phi_VL = atan(L/(radius-0.5));
phi_VR = atan(L/(radius+0.5));


R = [cos(theta), -sin(theta);
     sin(theta),  cos(theta)];


%%
%Draw Car
x_coord = [x_shift-0.5, x_shift+2.5, x_shift+2.5, x_shift-0.5, x_shift-0.5];
y_coord = [y_shift-0.6, y_shift-0.6, y_shift+0.6, y_shift+0.6, y_shift-0.6];

x_rcoord = zeros(length(x_coord));
y_rcoord = zeros(length(y_coord));
for n=1:length(x_coord)
    x_rcoord(n) = R(1,1)*x_coord(n)+R(1,2)*y_coord(n);
    y_rcoord(n) = R(2,1)*x_coord(n)+R(2,2)*y_coord(n);
end
line(x_rcoord+x,y_rcoord+y)


%%
%Draw weheel 1
x_coord = [x_shift-0.2,x_shift+0.2,x_shift+0.2,x_shift-0.2,x_shift-0.2];
y_coord = [y_shift-0.45, y_shift-0.45, y_shift-0.55, y_shift-0.55, y_shift-0.45];
x_rcoord = zeros(length(x_coord));
y_rcoord = zeros(length(y_coord));
for n=1:length(x_coord)
    x_rcoord(n) = R(1,1)*x_coord(n)+R(1,2)*y_coord(n);
    y_rcoord(n) = R(2,1)*x_coord(n)+R(2,2)*y_coord(n);
end
line(x_rcoord+x,y_rcoord+y)

%%
%Draw weheel 2
x_coord = [x_shift-0.2,x_shift+0.2,x_shift+0.2,x_shift-0.2,x_shift-0.2];
y_coord = [y_shift+0.45, y_shift+0.45, y_shift+0.55, y_shift+0.55, y_shift+0.45];
x_rcoord = zeros(length(x_coord));
y_rcoord = zeros(length(y_coord));
for n=1:length(x_coord)
    x_rcoord(n) = R(1,1)*x_coord(n)+R(1,2)*y_coord(n);
    y_rcoord(n) = R(2,1)*x_coord(n)+R(2,2)*y_coord(n);
end

line(x_rcoord+x,y_rcoord+y)

%%
%Draw wheel 3
R_phi = [cos(phi_VL), -sin(phi_VL);
        sin(phi_VL),  cos(phi_VL)];

x_shift = x-x-L;
y_shift = y-y-0.5;
x_coord = [x_shift+1.8,x_shift+2.2,x_shift+2.2,x_shift+1.8,x_shift+1.8];
y_coord = [y_shift+0.45, y_shift+0.45, y_shift+0.55, y_shift+0.55, y_shift+0.45];
x_rcoord = zeros(length(x_coord));
y_rcoord = zeros(length(y_coord));
for n=1:length(x_coord)
    x_rcoord(n) = R_phi(1,1)*x_coord(n)+R_phi(1,2)*y_coord(n);
    y_rcoord(n) = R_phi(2,1)*x_coord(n)+R_phi(2,2)*y_coord(n);
end

x_rcoord = x_rcoord+L;
y_rcoord = y_rcoord+0.5;

x_rrcoord = zeros(length(x_coord));
y_rrcoord = zeros(length(y_coord));

for n=1:length(x_coord)
    x_rrcoord(n) = R(1,1)*x_rcoord(n)+R(1,2)*y_rcoord(n);
    y_rrcoord(n) = R(2,1)*x_rcoord(n)+R(2,2)*y_rcoord(n);
end

line(x_rrcoord+x,y_rrcoord+y)

%%
%Draw wheel 4
R_phi = [cos(phi_VR), -sin(phi_VR);
        sin(phi_VR),  cos(phi_VR)];
    
x_shift = x-x-L;
y_shift = y-y+0.5;
x_coord = [x_shift+1.8,x_shift+2.2,x_shift+2.2,x_shift+1.8,x_shift+1.8];
y_coord = [y_shift-0.45, y_shift-0.45, y_shift-0.55, y_shift-0.55, y_shift-0.45];
x_rcoord = zeros(length(x_coord));
y_rcoord = zeros(length(y_coord));
for n=1:length(x_coord)
    x_rcoord(n) = R_phi(1,1)*x_coord(n)+R_phi(1,2)*y_coord(n);
    y_rcoord(n) = R_phi(2,1)*x_coord(n)+R_phi(2,2)*y_coord(n);
end

x_rcoord = x_rcoord+L;
y_rcoord = y_rcoord-0.5;

x_rrcoord = zeros(length(x_coord));
y_rrcoord = zeros(length(y_coord));

for n=1:length(x_coord)
    x_rrcoord(n) = R(1,1)*x_rcoord(n)+R(1,2)*y_rcoord(n);
    y_rrcoord(n) = R(2,1)*x_rcoord(n)+R(2,2)*y_rcoord(n);
end
line(x_rrcoord+x,y_rrcoord+y)

end


