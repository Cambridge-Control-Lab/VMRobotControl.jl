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
    pointtype=GeometryBasics.Point3f
    facetype=GeometryBasics.GLTriangleFace
    normaltype=GeometryBasics.Vec3f
    # This exists because GeometryBasics.normal_mesh does not result in correct normal vectors
    # for visualization in GLMakie (see [this](https://discourse.julialang.org/t/mesh-plotting-in-glmakie-with-geometrybasics-are-normals-broken/118227))

    # This function is a hacky workaround to fix the normal vectors of a mesh
    # it works by duplicating vertices so that they we can associate different normal vectors
    # to the same point, resulting in inefficient meshes.
    
    positions, faces = GeometryBasics.decompose_triangulate_fallback(primitive; pointtype=pointtype, facetype=facetype)

    # We want to preserve any existing attributes!
    attrs = GeometryBasics.attributes(primitive)
    # Make sure this doesn't contain position, we'll add position explicitely via meta!
    delete!(attrs, :position)


    normals, normal_faces = let
        normals = Vector{normaltype}(undef, length(faces))
        normal_faces = Vector{facetype}(undef, length(faces))
        for (i, face) in enumerate(faces)
            @assert length(face) == 3
            v1, v2, v3 = GeometryBasics.metafree.(getindex.((positions,), (face[1], face[2], face[3])))
            # we can get away with two edges since faces are planar.
            n = normalize(GeometryBasics.orthogonal_vector(v1, v2, v3))
            normals[i] = n
            normal_face = facetype(length(normals), length(normals), length(normals))
            normal_faces[i] = normal_face
        end
        normals .= normalize.(normals)
        normals, normal_faces
    end    
    DigitalAssetExchangeFormatIO.to_geometrybasics_mesh(positions, normals, faces, normal_faces)
end