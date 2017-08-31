module ValkyrieRobot

using RigidBodyDynamics
using RigidBodyDynamics.Contact
using StaticArrays

export Valkyrie,
    packagepath,
    urdfpath

include("bipedcontrolutil.jl")
using .BipedControlUtil

packagepath() = Pkg.dir("ValkyrieRobot", "deps")
urdfpath() = joinpath(packagepath(), "Valkyrie", "valkyrie.urdf")

function default_contact_model()
    SoftContactModel(hunt_crossley_hertz(k = 500e3), ViscoelasticCoulombModel(0.8, 20e3, 100.))
end

type Valkyrie{T}
    mechanism::Mechanism{T}
    feet::Dict{Side, RigidBody{T}}
    palms::Dict{Side, RigidBody{T}}
    pelvis::RigidBody{T}
    head::RigidBody{T}
    hippitches::Dict{Side, Joint{T}}
    knees::Dict{Side, Joint{T}}
    anklepitches::Dict{Side, Joint{T}}
    floatingjoint::Joint{T}

    function (::Type{Valkyrie{T}}){T}(; contactmodel = default_contact_model())
        mechanism = RigidBodyDynamics.parse_urdf(T, urdfpath())

        # pelvis
        pelvis = findbody(mechanism, "pelvis")

        # head
        head = findbody(mechanism, "head")

        # floating joint
        fixedjoint = joint_to_parent(pelvis, mechanism)
        floatingjoint = Joint(fixedjoint.name, frame_before(fixedjoint), frame_after(fixedjoint), QuaternionFloating{T}())
        replace_joint!(mechanism, fixedjoint, floatingjoint)

        # extremities
        feet = Dict(side => findbody(mechanism, "$(side)Foot") for side in instances(Side))
        hands = Dict(side => findbody(mechanism, "$(side)Palm") for side in instances(Side))
        # TODO: amputate fingers

        # relevant joints
        hippitches = Dict(side => findjoint(mechanism, "$(side)HipPitch") for side in instances(Side))
        knees = Dict(side => findjoint(mechanism, "$(side)KneePitch") for side in instances(Side))
        anklepitches = Dict(side => findjoint(mechanism, "$(side)AnklePitch") for side in instances(Side))

        # add sole frames
        soleframes = Dict(side => CartesianFrame3D("$(side)Sole") for side in instances(Side))
        for side in instances(Side)
            foot = feet[side]
            soleframe = soleframes[side]
            add_frame!(foot, Transform3D(soleframe, default_frame(foot), SVector(0.067, 0., -0.09)))
        end

        # add foot contact points
        for side in instances(Side)
            foot = feet[side]
            frame = default_frame(foot)
            z = -0.09
            add_contact_point!(foot, ContactPoint(Point3D(frame, -0.038, flipsign_if_right(0.55, side), z), contactmodel))
            add_contact_point!(foot, ContactPoint(Point3D(frame, -0.038, flipsign_if_right(-0.55, side), z), contactmodel))
            add_contact_point!(foot, ContactPoint(Point3D(frame, 0.172, flipsign_if_right(0.55, side), z), contactmodel))
            add_contact_point!(foot, ContactPoint(Point3D(frame, 0.172, flipsign_if_right(-0.55, side), z), contactmodel))
        end

        new{T}(mechanism, feet, hands, pelvis, head, hippitches, knees, anklepitches, floatingjoint)
    end
end

Valkyrie{T}(::Type{T} = Float64; kwargs...) = Valkyrie{T}(; kwargs...)

end # module
