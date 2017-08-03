using ValkyrieRobot
using RigidBodyTreeInspector
using Base.Test

val = Valkyrie()
geometries = parse_urdf(ValkyrieRobot.urdfpath(), val.mechanism; package_path = [ValkyrieRobot.packagepath()]);
@test length(geometries) == 81
