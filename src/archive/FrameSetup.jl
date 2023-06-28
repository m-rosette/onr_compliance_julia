# Function sourced from Hannah Kolano Pitch Prediction study (02/15/2023)

# Modified to fit planar Bravo arm

function setup_frames!(mech, frame_names_cob, frame_names_com, cob_vecs, com_vecs, cob_frames, com_frames)
    num_joints = length(joints(mech))
    vis_element = 2
    for i in 1:num_joints
        bod = bodies(mech)[i+1]
        frame_cob = CartesianFrame3D(frame_names_cob[i])
        frame_com = CartesianFrame3D(frame_names_com[i])
        cob_vec = cob_vecs[i]
        com_vec = com_vecs[i]
        cob_transform = Transform3D(frame_cob, default_frame(bod), cob_vec)
        com_transform = Transform3D(frame_com, default_frame(bod), com_vec)

        if !(RigidBodyDynamics.is_fixed_to_body(bod, frame_cob))
            add_frame!(bod, cob_transform)
            push!(cob_frames, frame_cob)
        end
        if !(RigidBodyDynamics.is_fixed_to_body(bod, frame_com))
            add_frame!(bod, com_transform)
            push!(com_frames, frame_com)
        end
        # if desired, visualize the new frames on the body
        if i == vis_element
            setelement!(mvis, default_frame(bod))
            setelement!(mvis, frame_cob, 0.3)
            setelement!(mvis, frame_com, 0.2)
        end 
    end

    # bravobase_com_wrt_linkframe = SVector{3, Float64}([-0.018, -0.004, -0.001]) # Sourced from link 0 in Bravo K&D manual
    bravobase_com_wrt_linkframe = com_vecs[2] # TODO verify this matches the line above

    # Arm base is rigidly attached to vehicle, so it has a transform in the vehicle's frame. It's the 19th body in the URDF attached to the vehicle. WITH RESPECT TO FLOATING BASE JOINT
    # TODO: It's between 44th and 45th body in the URDF attached to the vehicle. WITH RESPECT TO FIXED BASE JOINT (How do I fix this?)

    link = 19
    linkframe_wrt_vehframe = translation(RigidBodyDynamics.frame_definitions(vehicle_body)[link])
    # IF THE ARM IS ROTATED THIS HAS TO CHANGE!!!!
    bravobase_com_wrt_vehframe = bravobase_com_wrt_linkframe + linkframe_wrt_vehframe
    bravobase_com_frame = CartesianFrame3D("armbase_com_cob")
    com_transform = Transform3D(bravobase_com_frame, default_frame(vehicle_body), bravobase_com_wrt_vehframe)

    if !(RigidBodyDynamics.is_fixed_to_body(vehicle_body, bravobase_com_frame))
        add_frame!(vehicle_body, com_transform)
        push!(cob_frames, bravobase_com_frame)
        push!(com_frames, bravobase_com_frame)
        # setelement!(mvis, bravobase_com_frame)
    end
    print("THIS SHOULD SAY after_bravo_joint: ")
    println(RigidBodyDynamics.frame_definitions(vehicle_body)[link].from)
end