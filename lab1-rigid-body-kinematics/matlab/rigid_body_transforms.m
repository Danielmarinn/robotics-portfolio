                 %% Aula robotica 26/09
                 % EXERCISE 1
                 
clear; clc; close all;

obj=[-1, -1, -1, -1, +1, +1, +1, +1
     +2, -2, -2, +2, +2, -2, -2, +2
     -4, -4, +4, +4, -4, -4, +4, +4];

obj_homog=[obj; ones(1,8)];

obji_World=[0 1 0 -8; -1 0 0 3; 0 0 1 10; 0 0 0 1];

% Define faces using vertex indices
faces = {[1, 2, 3, 4],    ... back face
         [5, 6, 7, 8],    ... front face
         [1, 2, 6, 5],    ... bottom face
         [4, 3, 7, 8],    ... top face
         [1, 4, 8, 5],    ... right face
         [2, 3, 7, 6]};   ... left face

%% 1: Original Object in the world

figure(1);clf;hold on; grid on; axis equal
axis([-25, +25, -25, +25, -5, +25]); view(45,30);


obji_rot_world = obji_World * obj_homog;
[box.hFaces, box.hPts]=plot_object(obji_rot_world(1:3, :), faces, 'Original Object', 'r');

h=trplot(obji_World, ...
    'frame', 'Fobj', ...     % Frame name
    'color', 'y', ...        % Axis color
    'length', 10, ...         % Size of the arrows
    'thickness', 5, ...      % Thickness of the lines
    'arrow', ...             % Use arrows instead of lines
    'text_opts', {'FontSize', 10});     % Text options
axis([-25, +25, -25, +25, -5, +25]);
view(45, 30);
grid on

%% 2: Rotation of 45° about the OY axis of the World coordinate system

delta1=45;
for i = 1:delta1
    Ry = rotation_y(deg2rad(i));
    obj1_obji=inv(obji_World)*Ry*obji_World;
    obj1_world=Ry*obji_World;
    obj1_rot_world=obj1_world*obj_homog;
    
    % atualizar as 6 faces
    for f = 1:numel(faces)
        idx = faces{f};
        set(box.hFaces(f), 'XData', obj1_rot_world(1,idx), 'YData', obj1_rot_world(2,idx), 'ZData', obj1_rot_world(3,idx));
    end

    % Atualizar os vértices
    set(box.hPts, 'XData', obj1_rot_world(1,:), 'YData', obj1_rot_world(2,:), 'ZData', obj1_rot_world(3,:));

    drawnow limitrate
    pause(0.05)
end

%% 3: Rotation of 60° about the [1,1,-1] axis of the initial Object coordinate system

delta2=60;
axis1 = [1; 1; -1];

for i = 1:delta2
    R = rotation_axis(axis1, deg2rad(i));
    obj2_obj1=inv(obj1_obji)*R*obj1_obji;
    obj2_world=obj1_world*obj2_obj1;
    obj2_rot_world=obj2_world*obj_homog;

    % atualizar as 6 faces
    for f = 1:numel(faces)
        idx = faces{f};
        set(box.hFaces(f), 'XData', obj2_rot_world(1,idx), 'YData', obj2_rot_world(2,idx), 'ZData', obj2_rot_world(3,idx));
    end

    % Atualizar os vértices
    set(box.hPts, 'XData', obj2_rot_world(1,:), 'YData', obj2_rot_world(2,:), 'ZData', obj2_rot_world(3,:));

    drawnow limitrate
    pause(0.05)
end


%% 4: Displacement of +8 units along the OX axis of the current Object coordinate system

for i = 1:8
    Tx_obj = [ 1 0 0 i;
           0 1 0 0;
           0 0 1 0;
           0 0 0 1 ];

    obj3_world=obj2_world*Tx_obj;
    obj3_rot_world=obj3_world*obj_homog;

    % atualizar as 6 faces
    for f = 1:numel(faces)
        idx = faces{f};
        set(box.hFaces(f), 'XData', obj3_rot_world(1,idx), 'YData', obj3_rot_world(2,idx), 'ZData', obj3_rot_world(3,idx));
    end

    % Atualizar os vértices
    set(box.hPts, 'XData', obj3_rot_world(1,:), 'YData', obj3_rot_world(2,:), 'ZData', obj3_rot_world(3,:));

    drawnow limitrate
    pause(0.05)
end




%% 5: Rotation of the World coordinate system by -60° about its own OZ axis

delta3=60;

for i = 1:delta3
    Rz = rotation_z(deg2rad(i));
    obj3_worldn=Rz*obj3_world;
    obj3_rot_worldn=obj3_worldn*obj_homog;


    % atualizar as 6 faces
    for f = 1:numel(faces)
        idx = faces{f};
        set(box.hFaces(f), 'XData', obj3_rot_worldn(1,idx), 'YData', obj3_rot_worldn(2,idx), 'ZData', obj3_rot_worldn(3,idx));
    end

    % Atualizar os vértices
    set(box.hPts, 'XData', obj3_rot_worldn(1,:), 'YData', obj3_rot_worldn(2,:), 'ZData', obj3_rot_worldn(3,:));

    drawnow limitrate
    pause(0.05)
end
                            %% Used functions
                       
function [hFaces, hPts]=plot_object(vertices, faces, title_str, color)
    hFaces = gobjects(1,numel(faces));
    for i = 1:numel(faces)
        idx = faces{i};
        X = vertices(1, idx); Y = vertices(2, idx); Z = vertices(3, idx);
        hFaces(i) = fill3(X, Y, Z, color, 'FaceAlpha',0.6, 'EdgeColor','k', 'LineWidth',1);
    end

    hPts = scatter3(vertices(1,:), vertices(2,:), vertices(3,:), 50, 'k', 'filled');
    title(title_str); xlabel('X'); ylabel('Y'); zlabel('Z');
end

function Rz = rotation_z(delta)
Rz=[cos(delta) -sin(delta) 0 0
    sin(delta) cos(delta) 0 0
    0 0 1 0
    0 0 0 1];
end

function Ry = rotation_y(delta)
Ry=[cos(delta) 0 sin(delta) 0
    0 1 0 0
    -sin(delta) 0 cos(delta) 0
    0 0 0 1]; 
end


function R = rotation_axis(axis, angle)
    % Rodriguez rotation formula
    axis = axis / norm(axis);
    ux = axis(1); uy = axis(2); uz = axis(3);
    
    c = cos(angle);
    s = sin(angle);
    t = 1 - c;
    
    R = [t*ux^2 + c,    t*ux*uy - s*uz, t*ux*uz + s*uy, 0;
         t*ux*uy + s*uz, t*uy^2 + c,    t*uy*uz - s*ux, 0;
         t*ux*uz - s*uy, t*uy*uz + s*ux, t*uz^2 + c, 0
         0, 0, 0, 1];
end