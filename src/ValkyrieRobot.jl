module ValkyrieRobot

using RigidBodyDynamics
using RigidBodyDynamics.Contact
using StaticArrays

export Valkyrie,
    packagepath,
    urdfpath

include("bipedcontrolutil.jl")
using .BipedControlUtil

const module_tempdir = joinpath(Base.tempdir(), string(module_name(current_module())))
const datadir = joinpath(module_tempdir, "Valkyrie")

function cached_download(url::String, localFileName::String, cacheDir::String)
    if !ispath(cacheDir)
        mkpath(cacheDir)
    end
    fullCachePath = joinpath(cacheDir, localFileName)
    if !isfile(fullCachePath)
        download(url, fullCachePath)
    end
    fullCachePath
end

packagepath() = module_tempdir
urdfpath() = joinpath(datadir, "valkyrie.urdf")

const meshpaths = ["urdf/model/meshes/arms/aj1_left.obj";
    "urdf/model/meshes/arms/aj1_right.obj";
    "urdf/model/meshes/arms/aj2_left.obj";
    "urdf/model/meshes/arms/aj2_right.obj";
    "urdf/model/meshes/arms/aj3_left.obj";
    "urdf/model/meshes/arms/aj3_right.obj";
    "urdf/model/meshes/arms/aj4_left.obj";
    "urdf/model/meshes/arms/aj4_right.obj";
    "urdf/model/meshes/arms/aj5_left.obj";
    "urdf/model/meshes/arms/aj5_right.obj";
    "urdf/model/meshes/arms/aj6_left.obj";
    "urdf/model/meshes/arms/aj6_right.obj";
    "urdf/model/meshes/arms/palm_left.obj";
    "urdf/model/meshes/arms/palm_right.obj";
    "urdf/model/meshes/fingers/indexj1_left.obj";
    "urdf/model/meshes/fingers/indexj1_right.obj";
    "urdf/model/meshes/fingers/indexj2_left.obj";
    "urdf/model/meshes/fingers/indexj2_right.obj";
    "urdf/model/meshes/fingers/indexj3_left.obj";
    "urdf/model/meshes/fingers/indexj3_right.obj";
    "urdf/model/meshes/fingers/middlej1_left.obj";
    "urdf/model/meshes/fingers/middlej1_right.obj";
    "urdf/model/meshes/fingers/middlej2_left.obj";
    "urdf/model/meshes/fingers/middlej2_right.obj";
    "urdf/model/meshes/fingers/middlej3_left.obj";
    "urdf/model/meshes/fingers/middlej3_right.obj";
    "urdf/model/meshes/fingers/pinkyj1_left.obj";
    "urdf/model/meshes/fingers/pinkyj1_right.obj";
    "urdf/model/meshes/fingers/pinkyj2_left.obj";
    "urdf/model/meshes/fingers/pinkyj2_right.obj";
    "urdf/model/meshes/fingers/pinkyj3_left.obj";
    "urdf/model/meshes/fingers/pinkyj3_right.obj";
    "urdf/model/meshes/fingers/thumbj1_left.obj";
    "urdf/model/meshes/fingers/thumbj1_right.obj";
    "urdf/model/meshes/fingers/thumbj2_left.obj";
    "urdf/model/meshes/fingers/thumbj2_right.obj";
    "urdf/model/meshes/fingers/thumbj3_left.obj";
    "urdf/model/meshes/fingers/thumbj3_right.obj";
    "urdf/model/meshes/fingers/thumbj4_left.obj";
    "urdf/model/meshes/fingers/thumbj4_right.obj";
    "urdf/model/meshes/head/head_multisense.obj";
    "urdf/model/meshes/head/head_multisense_no_visor.obj";
    "urdf/model/meshes/head/multisense_hokuyo.obj";
    "urdf/model/meshes/head/neckj1.obj";
    "urdf/model/meshes/head/neckj2.obj";
    "urdf/model/meshes/legs/foot.obj";
    "urdf/model/meshes/legs/lj1_left.obj";
    "urdf/model/meshes/legs/lj1_right.obj";
    "urdf/model/meshes/legs/lj2_left.obj";
    "urdf/model/meshes/legs/lj2_right.obj";
    "urdf/model/meshes/legs/lj3_left.obj";
    "urdf/model/meshes/legs/lj3_right.obj";
    "urdf/model/meshes/legs/lj4_left.obj";
    "urdf/model/meshes/legs/lj4_right.obj";
    "urdf/model/meshes/legs/lj5.obj";
    "urdf/model/meshes/multisense/head_camera.obj";
    "urdf/model/meshes/pelvis/pelvis.obj";
    "urdf/model/meshes/torso/torso.obj";
    "urdf/model/meshes/torso/torsopitch.obj";
    "urdf/model/meshes/torso/torsoyaw.obj"]

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
        valkyrie_examples_url = "https://raw.githubusercontent.com/RobotLocomotion/drake/6e3ca768cbaabf15d0f2bed0fb5bd703fa022aa5/drake/examples/Valkyrie"
        urdf_url = valkyrie_examples_url * "/urdf/urdf/valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf"
        urdfdir, urdffile = splitdir(urdfpath())
        urdf = cached_download(urdf_url, urdffile, urdfdir)
        for meshpath in meshpaths
            meshdir, meshfilename = splitdir(meshpath)
            cached_download(joinpath(valkyrie_examples_url, meshpath), meshfilename, joinpath(datadir, meshdir))
        end
        mechanism = RigidBodyDynamics.parse_urdf(T, urdf)

        # pelvis
        pelvis = findbody(mechanism, "pelvis")

        # head
        head = findbody(mechanism, "head")

        # floating joint
        pelvis_to_world = joint_to_parent(pelvis, mechanism)
        pelvis_to_world.jointType = QuaternionFloating{Float64}()

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

        new{T}(mechanism, feet, hands, pelvis, head, hippitches, knees, anklepitches, pelvis_to_world)
    end
end

Valkyrie{T}(::Type{T} = Float64; kwargs...) = Valkyrie{T}(; kwargs...)

end # module
