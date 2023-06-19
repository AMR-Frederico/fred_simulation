function sysCall_init() -- The child script initialization
    
    -- get objects handles 
    A_RightmotorHandle = sim.getObjectHandle('roda4')
    B_RightmotorHandle = sim.getObjectHandle('roda2')
    
    A_LeftmotorHandle =  sim.getObjectHandle('roda3')
    B_LeftmotorHandle =  sim.getObjectHandle('roda1')

    Ultrasonic_LeftHandle = sim.getObjectHandle('ultrasonic_front_left')
    Ultrasonic_RightHandle = sim.getObjectHandle('ultrasonic_front_right')
    Ultrasonic_BackHandle = sim.getObjectHandle('ultrasonic_back')
    
    RobotHandle =  sim.getObjectHandle('base_link_visual')  
    OdomHandle =  sim.getObjectHandle('odom')
    
    -- set wheels velocities for 0
    w = 0
    vx = 0

    if simROS then 
        -- advertise ros topics 
        -- odometry 
        odomPub = simROS.advertise('/odom', 'nav_msgs/Odometry')

        -- joy connection (for safe_twist)
        joyConnection_Pub = simROS.advertise('/joy/controler/connected', 'std_msgs/Bool')
        simROS.publish(joyConnection_Pub, {data = true})

        -- modo auto (for machine states)
        modoAuto_Pub = simROS.advertise('/machine_state/control_mode/switch', 'std_msgs/Bool')
        simROS.publish(modoAuto_Pub, {data = true})
        Odometry_data={}
    else
        sim.addLog(sim.verbosity_scripterrors,"ROS interface was not found. Cannot run.")
    end
end 

function sysCall_actuation() -- put your actuation code here
    
    velSub=simROS.subscribe('/cmd_vel/safe', 'geometry_msgs/Twist', 'cmd_vel_callback')

end

function sysCall_sensing() -- put your sensing code here
    
    quaternion = sim.getObjectQuaternion(OdomHandle,-1)
    position=sim.getObjectPosition(OdomHandle,-1)
    linear_vel,angular_vel=sim.getObjectVelocity(OdomHandle)
    
    Odometry_data.header={seq=0,stamp=simROS.getTime(),frame_id="odom"}
    Odometry_data.pose={
                pose={
                    position={x=position[1],y=position[2],z=position[3]},
                    orientation={x=quaternion[1],y=quaternion[2],z=quaternion[3],w=quaternion[4]}
                    }}
    Odometry_data.twist={
                twist={
                    linear={x=linear_vel[1],y=linear_vel[2],z=linear_vel[3]},
                    angular={x=angular_vel[1],y=angular_vel[2],z=angular_vel[3]}
                    }}
    
    simROS.publish(odomPub, Odometry_data)

    transformations = {}
    transformations[1] = (getTransformStamped(RobotHandle,'base_link',-1,'odom'))
    transformations[2] = (getTransformStamped(Ultrasonic_BackHandle, 'back_ultrasonic_link', RobotHandle, 'base_link'))
    transformations[3] = (getTransformStamped(Ultrasonic_LeftHandle, 'left_ultrasonic_link', RobotHandle, 'base_link'))
    transformations[4] = (getTransformStamped(Ultrasonic_RightHandle, 'right_ultrasonic_link', RobotHandle, 'base_link'))
    
    simROS.sendTransforms(transformations)

end



function __getObjectPosition__(a,b)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Param(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.getObjectPosition(a,b)
end

function __getObjectQuaternion__(a,b)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Param(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.getObjectQuaternion(a,b)
end

function cmd_vel_callback(msg)
    vx =  -msg.angular.z;
    w =  -msg.linear.x;
   

    -- Base_controller --
    r = 1.0000e-01 -- (m) wheel radius
    L = 0.25    
           
    Vright = (vx - w*(L/2))/r
    Vleft =  (vx + w*(L/2))/r
    
    sim.setJointTargetVelocity(A_LeftmotorHandle,Vleft)
    sim.setJointTargetVelocity(B_LeftmotorHandle,Vleft)
    sim.setJointTargetVelocity(A_RightmotorHandle,Vright)
    sim.setJointTargetVelocity(B_RightmotorHandle,Vright)

end

function getTransformStamped(objHandle,name,relTo,relToName)
    -- This function retrieves the stamped transform for a specific object
    t=sim.getSystemTime()
    p=sim.getObjectPosition(objHandle,relTo)
    o=sim.getObjectQuaternion(objHandle,relTo)
    return {
        header={
            stamp=t,
            frame_id=relToName
        },
        child_frame_id=name,
        transform={
            translation={x=p[1],y=p[2],z=p[3]},
            rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
        }
    }
end


    
