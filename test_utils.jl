using ProgressMeter
using Test

function test_on_mechanisms(test, mechanisms::Vector)
    @showprogress desc=string(test) for m in mechanisms
        @testset "Robot: '$(name(m))'" begin
            test(m)
        end
    end
end

