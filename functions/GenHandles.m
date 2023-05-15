function h = GenHandles(STL_str)
% Generate initial patches of manipulator CAD from STLs

for ii = 1:1:numel(STL_str)
    % read stl into faces and vertices
    fv = stlread(STL_str{ii});
    h(ii) =  patch(fv,'FaceColor', [0.9020, 0.9020, 0.9020], ...
                   'EdgeColor','k','FaceLighting','gouraud', ...
                   'AmbientStrength', 0.15);
end
    
end

