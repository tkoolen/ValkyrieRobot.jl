using ValkyrieRobot
using ValkyrieRobot.BipedControlUtil
using MechanismGeometries
using Compat.Test

@testset "side" begin
    @test -left == right
    @test -right == left

    sides = [rand(Side) for i = 1 : 10000];
    @test isapprox(count(side -> side == left, sides) / length(sides), 0.5; atol = 0.05)

    @test flipsign_if_right(2., left) == 2.
    @test flipsign_if_right(2., right) == -2.
end

@testset "load geometries" begin
    val = Valkyrie()
    visuals = URDFVisuals(ValkyrieRobot.urdfpath(); package_path = [ValkyrieRobot.packagepath()])
    meshgeometry = visual_elements(val.mechanism, visuals)
    @test length(meshgeometry) == 81
end
