module_dir = joinpath(splitpath(Base.source_dir())[1:end-1])
if pwd() != module_dir
    @info "Changing directory to folder $(module_dir)"
    cd(module_dir)
end
using Pkg
Pkg.activate("./docs") # Activate docs environment

using LiveServer

serve(dir="./docs/build", host="0.0.0.0", port=8001, launch_browser=true)