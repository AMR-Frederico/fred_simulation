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
if (sim_call_type==sim.syscb_init) then

	
    A_RightmotorHandle = sim.getObjectHandle('roda4')
    B_RightmotorHandle = sim.getObjectHandle('roda2')
    
    A_LeftmotorHandle =  sim.getObjectHandle('roda3')
    B_LeftmotorHandle =  sim.getObjectHandle('roda1')
    
    w = 0
    vx = 0

    RobotHandle =  sim.getObjectHandle('base_link_visual')  
    OdomHandle =  sim.getObjectHandle('odom')  

    odomPub=simROS.advertise('/odom', 'nav_msgs/Odometry')
end    



if (sim_call_type==sim.syscb_actuation) then
    velSub=simROS.subscribe('/cmd_vel/safe', 'geometry_msgs/Twist', 'cmd_vel_callback')

end



function getTransformStamped(objHandle,name,relTo,relToName)
    t=sim.getSystemTime()
    p=__getObjectPosition__(objHandle,relTo)
    o=__getObjectQuaternion__(objHandle,relTo)
    linearVelocity, angularVelocity = sim.getObjectVelocity(objHandle)
    return {
        header={
            stamp=t,
            frame_id=relToName
        },
        child_frame_id=name,
        pose = {
            pose = {
                position = {
                x = p[1],
                y = p[2],
                z = p[3]
                },
                orientation = {
                    x = o[1],
                    y = o[2],
                    z = o[3],
                    w = o[4],
                },
            }
        },
        twist = {
            twist = {
                linear = {
                x = linearVelocity[1],
                y = linearVelocity[2],
                z = linearVelocity[3]
                },
                angular = {
                    x = angularVelocity[1],
                    y = angularVelocity[2],
                    z = angularVelocity[3]
                },
            }
        },
    }
end



if (sim_call_type==sim.syscb_sensing) then

	--Odometry Covariance matrix
    odomcovariance={
         0.001,0,0,0,0,0,
         0,0.001,0,0,0,0,
         0,0,0.001,0,0,0,
         0,0,0,0.001,0,0,
         0,0,0,0,0.001,0,
         0,0,0,0,0,0.001,
         0.001,0,0,0,0,0,
         0,0.001,0,0,0,0,
         0,0,0.001,0,0,0,
         0,0,0,0.001,0,0 }

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
if (sim_call_type==sim.syscb_init) then

	
     A_RightmotorHandle = sim.getObjectHandle('roda4')
     B_RightmotorHandle = sim.getObjectHandle('roda2')
    
     A_LeftmotorHandle =  sim.getObjectHandle('roda3')
     B_LeftmotorHandle =  sim.getObjectHandle('roda1')

    w = 0
    vx = 0

    RobotHandle =  sim.getObjectHandle('base_link')  
    OdomHandle =  sim.getObjectHandle('odom')  

    odomPub=simROS.advertise('/odom', 'nav_msgs/Odometry')
end    



if (sim_call_type==sim.syscb_actuation) then
    velSub=simROS.subscribe('/cmd_vel/safe', 'geometry_msgs/Twist', 'cmd_vel_callback')

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
    t=sim.getSystemTime()
    p=__getObjectPosition__(objHandle,relTo)
    o=__getObjectQuaternion__(objHandle,relTo)
    linearVelocity, angularVelocity = sim.getObjectVelocity(objHandle)
    return {
        header={
            stamp=t,
            frame_id=relToName
        },
        child_frame_id=name,
        pose = {
            pose = {
                position = {
                x = p[1],
                y = p[2],
                z = p[3]
                },
                orientation = {
                    x = o[1],
                    y = o[2],
                    z = o[3],
                    w = o[4],
                },
            }
        },
        twist = {
            twist = {
                linear = {
                x = linearVelocity[1],
                y = linearVelocity[2],
                z = linearVelocity[3]
                },
                angular = {
                    x = angularVelocity[1],
                    y = angularVelocity[2],
                    z = angularVelocity[3]
                },
            }
        },
    }
end



if (sim_call_type==sim.syscb_sensing) then

	--Odometry Covariance matrix
    odomcovariance={
         0.001,0,0,0,0,0,
         0,0.001,0,0,0,0,
         0,0,0.001,0,0,0,
         0,0,0,0.001,0,0,
         0,0,0,0,0.001,0,
         0,0,0,0,0,0.001,
         0.001,0,0,0,0,0,
         0,0.001,0,0,0,0,
         0,0,0.001,0,0,0,
         0,0,0,0.001,0,0,
         0,0,0,0,0.001,0,
         0,0,0,0,0,0.001}
    OdomCovar=sim.packFloatTable(odomcovariance)
    sim.setStringSignal('OdomCovariance',OdomCovar)
        
    -- Odom publisher
    simROS.publish(odomPub, getTransformStamped(RobotHandle, 'RobotHandle', OdomHandle, 'OdomHandle'))
   
end

    OdomCovar=sim.packFloatTable(odomcovariance)
    sim.setStringSignal('OdomCovariance',OdomCovar)
        
    -- Odom publisher
    simROS.publish(odomPub, getTransformStamped(RobotHandle, 'RobotHandle', OdomHandle, 'OdomHandle'))
   
end
