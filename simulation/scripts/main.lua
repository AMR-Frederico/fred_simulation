function sysCall_init()
    -- do some initialization here
    motorHandles={-1,-1,-1,-1}

    motorHandles[1]=sim.getObjectHandle('joint1')
    motorHandles[2]=sim.getObjectHandle('joint2')
    motorHandles[3]=sim.getObjectHandle('joint3')
    motorHandles[4]=sim.getObjectHandle('joint4')

end

function sysCall_actuation()
    -- put your actuation code here
    sim.setJointTargetVelocity(motorHandles[1],0.5)
    sim.setJointTargetVelocity(motorHandles[2],-0.5)
    sim.setJointTargetVelocity(motorHandles[3],-0.5)
    sim.setJointTargetVelocity(motorHandles[4],0.5)
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
