% Plots an isosceles triangle with orientations R
%
% plot_triangle(R,dt), with dt>0, plots a movie of the triangle
% with varying orientations R(1:3,1:3,k) and delay dt between frames
%
% plot_triangle(R) is the same as plot_triangle(R,1/30)
%
% plot_triangle(R,0) plots a sequence of triangles one over the other,
% with varying orientations R(1:3,1:3,k)
%
% If the figure is closed or changed while plotting, it displays an error
% message and returns without causing an error
function plot_triangle(R,dt)

if nargin<2
    dt=1/30;
end
movie=dt>0;

p1 = [-2;1;0];
p2 = [2;0;0];
p3 = [-2;-1;0];
p4 = p1;

tr = [p1 p2 p3 p4];
ax=[-2,2,-2,2,-2,2];

h=gcf;
if ~movie
    hold on
end
for k=1:size(R,3)
    R_tr=R(1:3,1:3,k)*tr;
    plot3(R_tr(1,:),R_tr(2,:),R_tr(3,:)); axis(ax); grid on;
    if movie
        pause(dt);
        if gcf~=h
            disp('!! Plot stopped before the end !!');
            break
        end
    end
end