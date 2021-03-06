clear all
load('ScanTP3.v6.mat')
plot((Scan{1}(2,:)).*cosd(Scan{1}(1,:)), (Scan{1}(2,:)).*sind(Scan{1}(1,:)), 'x')
hold on
p = ones(3,size(Scan{1},2));
p(size(p,1), size(p,2)) = 1;
h = [1,0,0; 0,1,0;0,0,1];
for i= 1:size(U,2)
    c = cosd(H(i));
    s = sind(H(i));
    h_new = [c,-s,c*U(i); s,c,s*U(i);0,0,1];
    p(1:2,:)=[Scan{i+1}(2,:).*cosd(Scan{i+1}(1,:)); (Scan{i+1}(2,:)).*sind(Scan{i+1}(1,:))];
    h = h*h_new;
    plot(h(1,3), h(2,3),'o-')
    p_tr = h*p;
    plot(p_tr(1,:), p_tr(2,:), 'x');
    pause(1)
end