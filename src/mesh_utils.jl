function transform_mesh(mesh, tf::Transform)
    c, f = GeometryBasics.coordinates(mesh), GeometryBasics.faces(mesh)
    c2 = map(p -> GeometryBasics.Point3f(tf * SVector{3, Float32}(p)), c)
    return GeometryBasics.normal_mesh(c2, f)
end

transform_mesh(mesh, tf::Nothing) = mesh

function rescale_mesh(mesh, scale)
    c, f = GeometryBasics.coordinates(mesh), GeometryBasics.faces(mesh)
    if prod(scale)< 0
        # If scaling will turn the mesh inside out, then reverse the chirality of the faces by
        # swapping two elements of each triangle, so that the mesh is still valid.
        TRI = eltype(f)::(Type{GeometryBasics.NgonFace{3, I}} where I)
        f = [TRI(tri[2], tri[1], tri[3]) for tri in f]
    end 
    c2 = map(p -> GeometryBasics.Point3f(scale .* SVector{3, Float32}(p)), c)
    return GeometryBasics.normal_mesh(c2, f)
end

function hacky_normal_mesh(primitive)
    GeometryBasics.normal_mesh(primitive)
end