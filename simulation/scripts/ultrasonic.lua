if (sim_call_type==sim.syscb_init) then

    -- get sensor handle
    Ultrasonic_LeftHandle = sim.getObjectHandle('ultrasonic_front_left')
    Ultrasonic_RightHandle = sim.getObjectHandle('ultrasonic_front_right')
    Ultrasonic_BackHandle = sim.getObjectHandle('ultrasonic_back')
    
    -- create a topic 
    ultrasonicLeft_Pub = simROS.advertise('/sensor/range/ultrasonic/left', 'sensor_msgs/Range')
    ultrasonicRight_Pub = simROS.advertise('/sensor/range/ultrasonic/right', 'sensor_msgs/Range')
    ultrasonicBack_Pub = simROS.advertise('/sensor/range/ultrasonic/back', 'sensor_msgs/Range')

end


if (sim_call_type==sim.syscb_actuation) then

    simROS.publish(ultrasonicBack_Pub,getSensorData(Ultrasonic_BackHandle, back_ultrasonic_link))
    simROS.publish(ultrasonicLeft_Pub,getSensorData(Ultrasonic_LeftHandle, left_ultrasonic_link))
    simROS.publish(ultrasonicRight_Pub,getSensorData(Ultrasonic_RightHandle, right_ultrasonic_link))
end

function getSensorData(sensor_handle, frame_name) 
    
    res,dist,pt=sim.handleProximitySensor(sensor_handle)
    if dist == nil then
        rangeValue = -1  -- Nenhum objeto detectado
    elseif dist == 0 then
        rangeValue = 4
    else
        rangeValue = dist
    end

    local msg={}

    t = sim.getSystemTime()

    return {
        header={
            stamp = t,
            frame_id = frame_name
        },
        radiation_type = 0,
        field_of_view = 0.26,
        min_range = 0.05,
        max_range = 4,
        range = rangeValue,
    }

end
