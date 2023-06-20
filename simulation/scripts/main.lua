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


function sysCall_init()
    -- [[Objects handles]]
    robot_handle=sim.getObjectHandle(sim.handle_self)
    base_point_handle=sim.getObjectHandle('base_point')
    
       -- get objects handles 
       A_RightmotorHandle = sim.getObjectHandle('roda4')
       B_RightmotorHandle = sim.getObjectHandle('roda2')
       
       A_LeftmotorHandle =  sim.getObjectHandle('roda3')
       B_LeftmotorHandle =  sim.getObjectHandle('roda1')
   
       Ultrasonic_LeftHandle = sim.getObjectHandle('ultrasonic_front_left')
       Ultrasonic_RightHandle = sim.getObjectHandle('ultrasonic_front_right')
       Ultrasonic_BackHandle = sim.getObjectHandle('ultrasonic_back')

       w = 0
       vx = 0

    
    if simROS then
        --[[ ROS nodes and topics initialization ]]--
        ROScommunication()
    else
        print("<font color='#F00'>ROS interface was not found. Cannot run.</font>@html")
    end
end

function sysCall_actuation()
    if simROS then
        --[[ Get object transformations and send to ROS ]]
        getTransform()
    end
       
end

function sysCall_sensing()
    if simROS then

        --[[ Get odometry data and send to ROS ]]
        odometry()

    end
end

function ROScommunication()
    -- write treking on the topics that we are publising
    print("<font color='#0F0'>ROS interface was found.</font>@html")

    odometryTopicName = 'odom'
    velocityTopicName = 'cmd_vel/safe'
    joyConnection_TopicName = 'joy/controler/connected'
    breakMode_TopicName = 'joy/controler/ps4/break'
    modoAuto_TopicName = 'joy/controler/ps4/triangle'
    
    --[[ Prepare publishers: ]]--
    odometryPub=simROS.advertise('/'..odometryTopicName,'nav_msgs/Odometry')
    
    -- joy connection (for safe_twist)
    joyConnection_Pub = simROS.advertise('/'..joyConnection_TopicName, 'std_msgs/Bool')
    simROS.publish(joyConnection_Pub, {data = true})
    
    -- stop break mode
    breakMode_Pub = simROS.advertise('/'..breakMode_TopicName, 'std_msgs/Int16')
    simROS.publish(breakMode_Pub, {data = 1})
    
    -- modo auto (for machine states)
    modoAuto_Pub = simROS.advertise('/'..modoAuto_TopicName, 'std_msgs/Int16')
    simROS.publish(modoAuto_Pub, {data = 1})

    --[[ Prepare subscribers: ]]--
    velocitySub = simROS.subscribe('/'..velocityTopicName, 'geometry_msgs/Twist', 'cmd_vel_callback')
  

end

function cmd_vel_callback(msg) -- o robo rotaciona o eixo z, e anda linearmente no eixo x, não permite movimentos dos demais casos
    
    
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
    t=simROS.getTime()
    p=__getObjectPosition__(objHandle,relTo)
    o=__getObjectQuaternion__(objHandle,relTo)
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

function getTransform()
    transformations = {}
    transformations[1] = (getTransformStamped(base_point_handle,'base_link',-1,'odom'))
    transformations[2] = (getTransformStamped(Ultrasonic_BackHandle, 'back_ultrasonic_link', base_point_handle, 'base_link'))
    transformations[3] = (getTransformStamped(Ultrasonic_LeftHandle, 'left_ultrasonic_link', base_point_handle, 'base_link'))
    transformations[4] = (getTransformStamped(Ultrasonic_RightHandle, 'right_ultrasonic_link', base_point_handle, 'base_link'))

    simROS.sendTransforms(transformations)
end

function odometry()
    local position = sim.getObjectPosition(base_point_handle,-1)
    local rotation = sim.getObjectQuaternion(base_point_handle,-1)
    local velocity_linear, velocity_angular = sim.getObjectVelocity(base_point_handle)
    
    local odom_data = {}
    odom_data['header'] = {seq = 0, stamp = simROS.getTime(), frame_id = "odom"}
    odom_data['child_frame_id'] = "map"
    odom_data['pose'] = {}
    odom_data['pose']['pose'] = {}
    odom_data['pose']['pose']['position'] = {x = position[1], y = position[2], z = position[3]}
    odom_data['pose']['pose']['orientation'] = {x = rotation[1], y = rotation[2], z = rotation[3], w = rotation[4]}
    odom_data['pose']['covariance'] = {0}
    odom_data['twist'] = {}
    odom_data['twist']['twist'] = {}
    odom_data['twist']['twist']['linear'] = {x = velocity_linear[1], y = velocity_linear[2], z = velocity_linear[3]}
    odom_data['twist']['twist']['angular'] = {x = velocity_angular[1], y = velocity_angular[2], z = velocity_angular[3]}
    odom_data['twist']['covariance'] = {0}
    
    simROS.publish(odometryPub, odom_data)
end

