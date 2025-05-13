

# 参考部分代码

等待物块到达

```python
function waituntil_boxcome()
    while 1 do
        ProximitySensor=sim.getInt32Signal('ProximitySensor')
        if ProximitySensor==1 then
            
            break
        end
    end
end
```

若物块为有瑕疵物块则直接用UR5的RG2从正上方夹取，并放到左边车辆。代码如下

```python
function up_xi()
    dummy = sim.getObjectHandle('up_target#1')
    local pose =sim.getObjectPose(dummy,-1)
    local pickpose=sim.getObjectPose(dummy,-1)
    
    local posx = sim.getFloatSignal('posx')
    local posy = sim.getFloatSignal('posy')
    
    if  posx~=nil then
        local world_posx = pose[1]-0.2*posx
        local world_posy = pose[2]-0.2*posy
        pose[1] = world_posx
        pose[2] = world_posy
        pose[3]=pose[3]+0.1
        
        pickpose[1] = world_posx
        pickpose[2] = world_posy
        pickpose[3] = pickpose[3]+0.01
        
    end

    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pickpose,data)
    
    setGripperData(false)
    sim.wait(0.3)
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,pose,data)
end
```

在夹取无瑕疵物块后先正运动学到初始状态，防止打到传送带和小车，再通过逆运动学稳定准确地放到car2上的dummy点，代码如下：

```python
//放置纯色方块
function put_to_pure_target(i)
    layTarget = sim.getObjectHandle('target_c'..i..'#1')
    layPose = sim.getObjectPose(layTarget,-1)
    loosePose = sim.getObjectPose(layTarget,-1)
    layPose[3]=layPose[3]+0.18
    loosePose[3]=loosePose[3]
    --
    moveToConfig_viaFK(maxVel,maxAccel,maxJerk, startConfig,data)
    --
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,layPose,data)
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,loosePose,data)
    setGripperData(true)
    sim.wait(0.1)
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,layPose,data)
    take_phote()
    
end 
```

```lua
--放置混色方块
function put_to_mix_target(i)
    layTarget = sim.getObjectHandle('target'..i..'#1')
    --layTarget = sim.getObjectHandle('target'..'8'..'#1')
    layPose = sim.getObjectPose(layTarget,-1)
    loosePose = sim.getObjectPose(layTarget,-1)
    layPose[3]=layPose[3]+0.22
    
   
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,layPose,data)
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,loosePose,data)
    setGripperData(true)
    sim.wait(0.1)
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,layPose,data)

    take_phote()
   
end 
```



```lua
function get_changepos()
    side_flag=sim.getInt32Signal('green')
    pure_block=sim.getInt32Signal('pure_block')
    mix_block=sim.getInt32Signal('mix_block')

    if(mix_block==1)
    then
        up_xi()
    else--pure 
        --front_xi()
        if(side_flag==0)then
            front_xi()
            
        end
        if(side_flag==1)then
            side_xi()
        end
    end
end
```



## 视觉部分

在函数readVisionSensor返回的数据包中，auxiliaryValuePacket2中包含色块的数量、大小、位置、方向和颜色信息





# 大作业代码

## 传送带

### 初始化部分

```lua
--初始化部分
function sysCall_init() 
    
    sensor=sim.getObjectHandle('conveyorBelt_sensor')
    pathHandle=sim.getObjectHandle("Path")
    forwarder=sim.getObjectHandle('conveyorForwarder')
    sim.setPathTargetNominalVelocity(pathHandle,0) -- for backward compatibility
    tick=0
    counter=1
    cubicSize={0.05,0.05,0.05}
    local CircleSizes = {0.025, 0.025, 0.05} -- 尺寸
    cubicWeight=0.2
    shapeType_Cubic=0
    shapeTyppe_Circle=2
    
    math.randomseed(tostring(os.time()):reverse():sub(1, 7))
    
    --定义物品个数
    redCubeCounter=16
    redCircleCounter=16
    blueCubeCounter=16
    blueCircleCounter=16
    whiteCubeter=16
    whiteCircleCounter=16
    totalCubeCounter=48
    totalCircleCounter=48
    totalCounter=96
    
    trgCount=0
    previousTrgCount=0
    poseMatrix=sim.getObjectMatrix(Box_a_handle,-1)
    poseMatrix[12]=0.135
    sim.setInt32Signal("ProximitySensor",0)
    beltVelocity=sim.getScriptSimulationParameter(sim.handle_self,"conveyorBeltVelocity")
end
--定义清除指令
function sysCall_cleanup() 
 
end 
```

```lua
function sysCall_init() 
    -- ?????????
    sensor=sim.getObjectHandle('conveyorBelt_sensor')
    -- ????????
    pathHandle=sim.getObjectHandle("Path")
    -- ?????????
    forwarder=sim.getObjectHandle('conveyorForwarder')
    -- ????????????????
    sim.setPathTargetNominalVelocity(pathHandle,0)
    -- ??????????
    tick=0
    counter=1

    cubicSize={0.05,0.05,0.05}
    cubicSize_s={0.05,0.04,0.05}
    cubicWeight=0.2
    -- ??????
    shapeType_Cubic=0
    -- ????
    red={1.00,0.29,0.21}
    green={0.28,1,0.21}
    -- ???????
    math.randomseed(tostring(os.time()):reverse():sub(1, 7))
    -- ??????
    redCounter=32
    greenCounter=32
    whiteCounter=0
    totalCounter=redCounter+greenCounter+whiteCounter
    -- ???????
    poseMatrix=sim.getObjectMatrix(forwarder,-1)
    -- ??????
    poseMatrix[4]=poseMatrix[4]+0.4
    poseMatrix[12]=poseMatrix[12]+0.15
end

function sysCall_cleanup() 
end 

function sysCall_actuation() 
    -- ???????
    beltVelocity=sim.getScriptSimulationParameter(sim.handle_self,"conveyorBeltVelocity")
    -- ???????
    ss=sim.readProximitySensor(sensor)
    -- ???????????????????????
    if sim.readProximitySensor(sensor)>0 or sim.getInt32Signal('getdata')==1 then
        beltVelocity=0
    end
    -- ?????????????
    local dt=sim.getSimulationTimeStep()
    local pos=sim.getPathPosition(pathHandle)
    -- ??????
    pos=pos+beltVelocity*dt
    sim.setPathPosition(pathHandle,pos) -- update the path's intrinsic position
    
    
    local relativeLinearVelocity={-beltVelocity,0,0}
    sim.resetDynamicObject(forwarder)
    local m=sim.getObjectMatrix(forwarder,-1)
    m[4]=0 -- Make sure the translation component is discarded
    m[8]=0 -- Make sure the translation component is discarded
    m[12]=0 -- Make sure the translation component is discarded
    local absoluteLinearVelocity=sim.multiplyVector(m,relativeLinearVelocity)
    -- ???????????
    sim.setObjectFloatParam(forwarder,sim.shapefloatparam_init_velocity_x,absoluteLinearVelocity[1])
    sim.setObjectFloatParam(forwarder,sim.shapefloatparam_init_velocity_y,absoluteLinearVelocity[2])
    sim.setObjectFloatParam(forwarder,sim.shapefloatparam_init_velocity_z,absoluteLinearVelocity[3])
    -- ?????????
    tick=tick+1
    cubicHandles={}
    -- ????????????????????
    if (tick%4==0)and(totalCounter>0)and(beltVelocity>0) then
        -- ?????????????
        cubicHandles[counter]=sim.createPureShape(shapeType_Cubic,14,cubicSize,cubicWeight,nil)
        random_num=math.random(redCounter+greenCounter+whiteCounter)
        noiseR=math.random()/10
        noiseG=math.random()/10
        noiseB=math.random()/10
        sim.addStatusbarMessage(string.format("r=%f,g=%f,b=%f",noiseR,noiseG,noiseB))
         if(random_num<=redCounter) then
            red={1.00-noiseR,0.29+noiseG,0.21+noiseB}
            sim.setShapeColor(cubicHandles[counter],nil,sim.colorcomponent_ambient_diffuse,red)
            redCounter=redCounter-1
        else if(random_num>redCounter)and(random_num<=(redCounter+greenCounter)) then
                green={0.28+noiseR,1-noiseG,0.21-noiseB}
                sim.setShapeColor(cubicHandles[counter],nil,sim.colorcomponent_ambient_diffuse,green)
                greenCounter=greenCounter-1
             else
                white={1-noiseR,1-noiseG,1-noiseB}
                sim.setShapeColor(cubicHandles[counter],nil,sim.colorcomponent_ambient_diffuse,white)
                whiteCounter=whiteCounter-1
             end
        end
        -- ?????????????????
        sim.setObjectSpecialProperty(cubicHandles[counter],(sim.objectspecialproperty_detectable_all+sim.objectspecialproperty_renderable))
        totalCounter=totalCounter-1
        -- ???????????
        local CubicMatrix=poseMatrix
        CubicMatrix[8]=poseMatrix[8]+0.06*math.random()-0.03
        if(CubicMatrix[8]>-2.0000e-01) then
            CubicMatrix[8] = -2.0000e-01
        end
        if(CubicMatrix[8]<-3.0000e-01) then
            CubicMatrix[8] = -3.0000e-01
        end
        sim.setObjectMatrix(cubicHandles[counter],-1,poseMatrix)
        counter=counter+1
    end
end 

```



### 主函数(逻辑判断还有问题)

```lua
function sysCall_actuation() 
    
    beltVelocity=sim.getScriptSimulationParameter(sim.handle_self,"conveyorBeltVelocity")
    ss=sim.readProximitySensor(sensor)
    sim.addStatusbarMessage(string.format("ss=%f",ss))
    if sim.readProximitySensor(sensor)>0 then
        --beltVelocity=0
    end
    local dt=sim.getSimulationTimeStep()
    local pos=sim.getPathPosition(pathHandle)
    pos=pos+beltVelocity*dt
    sim.setPathPosition(pathHandle,pos) -- update the path's intrinsic position

    
    local relativeLinearVelocity={-beltVelocity,0,0}

    sim.resetDynamicObject(forwarder)
    -- Compute the absolute velocity vector:
    local m=sim.getObjectMatrix(forwarder,-1)
    m[4]=0 -- Make sure the translation component is discarded
    m[8]=0 -- Make sure the translation component is discarded
    m[12]=0 -- Make sure the translation component is discarded
    local absoluteLinearVelocity=sim.multiplyVector(m,relativeLinearVelocity)
    -- Now set the initial velocity of the dynamic rectangle:
    sim.setObjectFloatParam(forwarder,sim.shapefloatparam_init_velocity_x,absoluteLinearVelocity[1])
    sim.setObjectFloatParam(forwarder,sim.shapefloatparam_init_velocity_y,absoluteLinearVelocity[2])
    sim.setObjectFloatParam(forwarder,sim.shapefloatparam_init_velocity_z,absoluteLinearVelocity[3])
    tick=tick+1
    cubicHandles={}
    if (tick%50==0)and(totalCounter>0) then
        --每50个单位时间生成一次固体
        
        cubicHandles[counter]=sim.createPureShape(shapeType_Cubic,14,cubicSize,cubicWeight,nil)
        circleHandles[counter]=sim.createPureShape(shapeType_Circle,14,CircleSizes,cubicWeight,nil)
        
        --计算随机数
        random_num=math.random(redCounter+greenCounter+whiteCounter)
        --生成随机噪音
        noiseR=math.random()/10
        noiseG=math.random()/10
        noiseB=math.random()/10
        --向状态栏发送信息
        sim.addStatusbarMessage(string.format("r=%f,g=%f,b=%f",noiseR,noiseG,noiseB))
        --如果随机数小于红色的数量，设置为红色的方块
        if(random_num<=redCounter) then
            red={1.00-noiseR,0.29+noiseG,0.21+noiseB}
            if(random_num<=redCubeCounter)then
            	sim.setShapeColor(cubicHandles[counter],nil,sim.colorcomponent_ambient_diffuse,red)
                redCubeCounter=redCubeCounter-1
            end
            else
            	sim.setShapeColor(circleHandles[counter],nil,sim.colorcomponent_ambient_diffuse,red)
            	redCircleCounter=redCircleCounter-1
            end
            redCounter=redCounter-1
        --如果随机数大于红色的数量但小于红色数量加绿色的数量，设置为绿色的方块
        else if(random_num>redCounter)and(random_num<=(redCounter+greenCounter)) then
                green={0.28+noiseR,1-noiseG,0.21-noiseB}
            if(random_num<=greenCubeCounter+redCounter)then
                sim.setShapeColor(cubicHandles[counter],nil,sim.colorcomponent_ambient_diffuse,green)
                greenCubeCounter=greenCubeCounter-1
            end
            else
            	sim.setShapeColor(circleHandles[counter],nil,sim.colorcomponent_ambient_diffuse,green)
            	greenCircleCounter=greenCircleCounter-1
            end
                greenCounter=greenCounter-1
        --如果随机数大于红色加绿色数量和，设置为白色的方块
        else if(random_num>(redCounter+greenCounter))
                white={1-noiseR,1-noiseG,1-noiseB}
            if(random_num<=whiteCubeCounter+redCounter+greenCounter)
                sim.setShapeColor(cubicHandles[counter],nil,sim.colorcomponent_ambient_diffuse,white)
                whiteCounter=whiteCounter-1
             end
            else
           	sim.setShapeColor(circleHandles[counter],nil,sim.colorcomponent_ambient_diffuse,white)
            end
        end
        
        
        
        
        --sim.setObjectSpecialProperty(cubicHandles[counter],sim.objectspecialproperty_renderable )
        sim.setObjectSpecialProperty(cubicHandles[counter],(sim.objectspecialproperty_detectable_all+sim.objectspecialproperty_renderable))
        totalCounter=totalCounter-1
        sim.addStatusbarMessage(string.format("x=%f,y=%f,z=%f",poseMatrix[4],poseMatrix[8],poseMatrix[12]))
        local CubicMatrix=poseMatrix
        CubicMatrix[8]=poseMatrix[8]+0.06*math.random()-0.03
        sim.setObjectMatrix(cubicHandles[counter],-1,poseMatrix)
        conter=counter+1
    end
end 

```



## 码垛机械臂UR5

### 初始化函数

```lua
function sysCall_init()
    corout=coroutine.create(coroutineMain)
end


function sysCall_actuation()
    -- 检查协程状态是否不是 'dead'
    if coroutine.status(corout)~='dead' then
         -- 尝试恢复协程的执行
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
end
```



### 主循环函数

```lua
function coroutineMain()

    jointHandles={-1,-1,-1,-1,-1,-1}
    for i=1,6,1 do
        jointHandles[i]=sim.getObjectHandle('UR5_joint'..i)
    end
    
    ikGroupHandle=sim.getIkGroupHandle('UR5')
    ikTip=sim.getObjectHandle('UR5_ik')
    ikTarget=sim.getObjectHandle('UR5_target')
    waitHandle = sim.getObjectHandle('wait')
    --从视觉传感器中传递指针cam
    cam = sim.getObjectHandle('Vision_sensor0')
    --从传送大传感器中传递指针sensor
    sensor=sim.getObjectHandle('Beltsensor')

    -- Set-up some of the RML vectors:
    vel=180
    accel=40
    jerk=80
    currentVel={0,0,0,0,0,0,0}
    currentAccel={0,0,0,0,0,0,0}
    maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
    maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
    maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}
    targetVel={0,0,0,0,0,0}

    ikMaxVel0={0.6,0.6,0.6,1.8}
    ikMaxAccel0={0.8,0.8,0.8,0.9}
    ikMaxVel={0.9,0.9,0.9,2.7}
    ikMaxAccel={1.2,1.2,1.2,1.35}
    ikMaxVel2={18,18,18,48}
    ikMaxAccel2={24,24,24,27}
    ikMaxJerk={0.6,0.6,0.6,0.8}
    
    --九个小车上面的方块位置
    position = {{-0.35,1.6},{-0.35,1.675},{-0.35,1.75},
                {-0.425,1.6},{-0.425,1.675},{-0.425,1.75},
                {-0.5,1.6},{-0.5,1.675},{-0.5,1.75}}
    
    tempposition = {-0.7,1.6}
    occupy={}
    for i=1,9,1 do
        occupy[i]=0
    end
    tempoccupy=0
    
    -- 获取对象的世界坐标位置
    local waitPos = sim.getObjectPosition(waitHandle,-1)
    -- 获取对象的世界坐标四元数
    waitQuat = sim.getObjectQuaternion(waitHandle,-1)
    -- 启用逆运动学
    enableIk(true)
   
 --waitPos 这是一个三维向量，指定目标对象要移动到的最终位置
 --waitQuat这是一个四元数，指定目标对象要旋转到的最终姿态。
   sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,waitPos,waitQuat,nil)
    while true do
        if true then
            --等待信号arrive5:小车到达分类的位置
            sim.waitForSignal('arrive5')
            local data = {}
            --从cam中读取数据
            data = sim.unpackTable(sim.readCustomDataBlock(cam, 'data'))
            --如果不为空
            if(#data~=0) then
                for i=1,#data,1 do
                    local size=0
                    sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{data[i][1],data[i][2],0.33},waitQuat,nil)
                    sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel0,ikMaxAccel0,ikMaxJerk,{data[i][1],data[i][2],0.30},waitQuat,nil)
                    sim.wait(0.1)
                    --这个函数用于读取接近传感器的状态。
                    if sim.readProximitySensor(sensor)>0 then
                        size=1
                    end
                    if(size==1) then
                        sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{data[i][1],data[i][2],0.29},waitQuat,nil)
                    else
                        sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{data[i][1],data[i][2],0.28},waitQuat,nil)
                    end
                    
                    --开启吸盘
                    suctionCupCmd('on')
                    sim.wait(0.08)
                    sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{data[i][1],data[i][2],0.38},waitQuat,nil)
                    if(i==#data) then
                        --将finish5信号设为1
                        --将arrive5信号清零
                        sim.setInt32Signal('finish5',1)
                        sim.clearInt32Signal('arrive5')
                    end
                    
                    --设置仿真中物体的朝向
                    sim.setObjectOrientation(waitHandle,-1, {0,0,-data[i][3]})
                    local temp = sim.getObjectQuaternion(waitHandle,-1)
                    local targetQuat = {temp[1], temp[2], temp[3], temp[4]}
                    sim.setObjectOrientation(waitHandle,-1, {0,0,0})
                    if(size==0) then
                        local n=1
                        for j=1,9,1 do
                            if(occupy[j]~=2) then
                                n=n+1
                            else
                                break
                            end
                        end
                        if(n==10) then
                            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{tempposition[1],tempposition[2],0.35},targetQuat,nil)
                            if(tempoccupy==0) then
                                height=0.04
                            else
                                height=0.04*tempoccupy+0.04
                            end
                            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{tempposition[1],tempposition[2],height+0.1},targetQuat,nil)
                            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{tempposition[1],tempposition[2],height+0.01},targetQuat,nil)
                            sim.wait(0.08)
                            --吸盘关闭
                            suctionCupCmd('off')
                            sim.wait(0.05)
                            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{tempposition[1],tempposition[2],0.33},targetQuat,nil)
                            tempoccupy=tempoccupy+1
                        else
                            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{position[n][1],position[n][2],0.35},targetQuat,nil)
                            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{position[n][1],position[n][2],0.22},targetQuat,nil)
                            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{position[n][1],position[n][2],0.14+0.005},targetQuat,nil)
                            sim.wait(0.05)
                            --吸盘关闭
                            suctionCupCmd('off')
                            sim.wait(0.05)
                            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{position[n][1],position[n][2],0.33},targetQuat,nil)
                            occupy[n]=3
                        end
                    else
                        local m=1
                        for j=1,9,1 do
                            if(occupy[j]>=2) then
                                m=m+1
                            else
                                break
                            end
                        end
                        sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{position[m][1],position[m][2],0.35},targetQuat,nil)
                        sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{position[m][1],position[m][2],0.22},targetQuat,nil)
                        if(occupy[m]==0) then
                            height=0.05
                        else
                            height=0.05*occupy[m]+0.05
                        end
                        sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{position[m][1],position[m][2],height+0.005},targetQuat,nil)
                        sim.wait(0.05)
                        suctionCupCmd('off')
                        sim.wait(0.05)
                        sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{position[m][1],position[m][2],0.33},targetQuat,nil)
                        occupy[m]=occupy[m]+1
                    end
                end
            else
                --发送信号finish5为1
                --清除信号arrive5
                sim.setInt32Signal('finish5',1)
                sim.clearInt32Signal('arrive5')
            end
            
            --
            if(tempoccupy~=0) then
                local p=tempoccupy
                for i=1,p,1 do
                    local n=1
                    for j=1,9,1 do
                        if(occupy[j]~=2) then
                            n=n+1
                        else
                            break
                        end
                    end
                    if(n==10) then
                       
 --移动到等待位置
                        sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,waitPos,waitQuat,nil)
                        break
                    else
                        high=0.04*tempoccupy
                        if(high>0.3) then
                            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{tempposition[1],tempposition[2],high+0.1},waitQuat,nil)
                        else
                            --sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{tempposition[1],tempposition[2],0.3},waitQuat,nil)
                            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{tempposition[1],tempposition[2],high+0.1},waitQuat,nil)
                            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{tempposition[1],tempposition[2],high+0.005},waitQuat,nil)
                            sim.wait(0.05)
                            --打开吸盘
                            suctionCupCmd('on')
                            sim.wait(0.05)
                            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{tempposition[1],tempposition[2],0.3},waitQuat,nil)
                            --sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{position[n][1],position[n][2],0.3},waitQuat,nil)
                            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{position[n][1],position[n][2],0.22},waitQuat,nil)
                            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{position[n][1],position[n][2],0.14+0.005},waitQuat,nil)
                            sim.wait(0.05)
                            suctionCupCmd('off')
                            sim.wait(0.05)
                            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{position[n][1],position[n][2],0.33},waitQuat,nil)
                            tempoccupy=tempoccupy-1
                            occupy[n]=3
                        end
                    end
                end
                sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,waitPos,waitQuat,nil)
            else
                sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,waitPos,waitQuat,nil)
            end
        end
    end
    enableIk(false)
end
```

### 码垛部分

```lua
function put_to_RedCubeTarget(i,layer_para)
    --获取对象句柄和初始姿态：
    layTarget = sim.getObjectHandle('RedCube_Dummy'..i)
    layPose = sim.getObjectPose(layTarget,-1)
    loosePose = sim.getObjectPose(layTarget,-1)
    --调整姿态的 Z 坐标
    layPose[3]=layPose[3]+0.08*layer_para+0.2
    loosePose[3]=loosePose[3]+0.06*(layer_para-1)+0.04
   --通过逆运动学进行移动
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,layPose,data)
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,loosePose,data)
    --解除真空吸盘
    sim.setInt32Signal(vacuumCupName..'_active',0)
 
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,layPose,data)
end 


```



### 从移动机器人上面卸货

```lua
function ban_from_car1(i)
    
    --获取对象句柄和初始姿态
    layTarget = sim.getObjectHandle('target_c'..i..'#1')
    layPose = sim.getObjectPose(layTarget,-1)
    loosePose = sim.getObjectPose(layTarget,-1)
    
    sim.setInt32Signal(vacuumCupName..'_active',1)
    layPose[3]=layPose[3]+0.2
    loosePose[3]=loosePose[3]+0.035
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,layPose,data)
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,loosePose,data)
    
 
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,layPose,data)
end

function ban_from_car2(i)
    --获取对象句柄和初始姿态：
    --目标点的名字和车上的名字相同
    layTarget = sim.getObjectHandle('target'..i..'#1')
    layPose = sim.getObjectPose(layTarget,-1)
    loosePose = sim.getObjectPose(layTarget,-1)
    --解除真空吸盘
    sim.setInt32Signal(vacuumCupName..'_active',1)
    
    layPose[3]=layPose[3]+0.25
    loosePose[3]=loosePose[3]+0.035
    
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,layPose,data)
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,loosePose,data)
    moveToPose_viaIK(ikMaxVel,ikMaxAccel,ikMaxJerk,layPose,data)
end


```

## 传动带机械臂UR5

```lua
function sysCall_init()
    corout=coroutine.create(coroutineMain)
end

function sysCall_actuation()
    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
end

-- ON OFF 吸盘的开关控制
function suctionCupCmd(command)
    modelBase=sim.getObjectHandle(sim.handle_self)
    modelName=sim.getObjectName(modelBase)
    if(command=='on') then
        sim.setInt32Signal(modelName .."_suctionCup",1)
    else if(command=='off') then
        sim.setInt32Signal(modelName .."_suctionCup",0)
        end
    end
end

-- TURE FALSE 逆运动学的开关
enableIk=function(enable)
    if enable then
        sim.setObjectMatrix(ikTarget,-1,sim.getObjectMatrix(ikTip,-1))
        
        for i=1,#jointHandles,1 do
            sim.setJointMode(jointHandles[i],sim.jointmode_ik,1)
        end

        sim.setExplicitHandling(ikGroupHandle,0)
    else
        sim.setExplicitHandling(ikGroupHandle,1)
        
        for i=1,#jointHandles,1 do
            sim.setJointMode(jointHandles[i],sim.jointmode_force,0)
        end
        
    end
end


--主控制程序的循环
function coroutineMain()
    jointHandles={-1,-1,-1,-1,-1,-1}
    for i=1,6,1 do
        jointHandles[i]=sim.getObjectHandle('UR5_joint'..i..'#0')
    end
    
    --指针初始化
    ikGroupHandle=sim.getIkGroupHandle('UR50')
    ikTip=sim.getObjectHandle('UR5_ik0')
    ikTarget=sim.getObjectHandle('UR5_target0')
    cam=sim.getObjectHandle('Vision_sensor0#0')
    sensor=sim.getObjectHandle('Beltsensor#0')
    waitHandle = sim.getObjectHandle('wait#0')


    -- Set-up some of the RML vectors:
    vel=180
    accel=40
    jerk=80
    currentVel={0,0,0,0,0,0,0}
    currentAccel={0,0,0,0,0,0,0}
    maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
    maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
    maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}
    targetVel={0,0,0,0,0,0}

    ikMaxVel={1.2,1.2,1.2,3.6}
    ikMaxAccel={1.6,1.6,1.6,1.8}
    ikMaxVel2={24,24,24,72}
    ikMaxAccel2={32,32,32,36}
    ikMaxJerk={0.6,0.6,0.6,0.8}
    
    -- -1表示相对于世界坐标系
    startPos = sim.getObjectPosition(waitHandle,-1)
    workQuat = sim.getObjectQuaternion(waitHandle,-1)
    stertQuat = sim.getObjectQuaternion(ikTip,-1)
    
    enableIk(true)
    
    while true do
        if true then
            --等待信号getdata:
            --等待信号arrive1:有小车到达
            sim.waitForSignal('getdata')
            sim.waitForSignal('arrive1')
            if(sim.getInt32Signal('arrive1')==1) then
                --获取小车的指针
                Pioneer=sim.getObjectHandle('P1')
            end
            if(sim.getInt32Signal('arrive1')==2) then
                --获取小车的指针
                Pioneer=sim.getObjectHandle('P2')
            end
            if(sim.getInt32Signal('arrive1')==3) then
                --获取小车的指针
                Pioneer=sim.getObjectHandle('P3')
            end
            if(sim.getInt32Signal('arrive1')==4) then
                --获取小车的指针
                Pioneer=sim.getObjectHandle('P4')
            end
            --将arrive1的信号清除
            sim.clearInt32Signal('arrive1')
            local data={}
            data = sim.unpackTable(sim.readCustomDataBlock(cam, 'data3'))
            PioneerPosition= sim.getObjectPosition(Pioneer,-1)
            --在移动机器人上面创建3*3的位置方块，其中中间一个不放，0-7；
            PioneerPositionMatrix={{PioneerPosition[1]+0.075,PioneerPosition[2]+0.075},{PioneerPosition[1],PioneerPosition[2]+0.075},{PioneerPosition[1]-0.075,PioneerPosition[2]+0.075},
                                    {PioneerPosition[1]+0.075,PioneerPosition[2]},{PioneerPosition[1]-0.075,PioneerPosition[2]},{PioneerPosition[1]+0.075,PioneerPosition[2]-0.075},
                                    {PioneerPosition[1],PioneerPosition[2]-0.075},{PioneerPosition[1]-0.075,PioneerPosition[2]-0.075},{PioneerPosition[1],PioneerPosition[2]}}
            --将移动机器人上放置点的状态置零，表示该点上没有方块
            local PioneerPositionOccupy={}
            for i=1,9,1 do
                PioneerPositionOccupy[i]=0
            end
            for i=5,#data,1 do
                local size=0
                sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{data[i][1],data[i][2],0.27},workQuat,nil)
                sim.wait(0.1)
                if sim.readProximitySensor(sensor)>0 then
                    size=1
                end

                if(size==1) then
                    sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{data[i][1],data[i][2],0.155},workQuat,nil)
                else
                    sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{data[i][1],data[i][2],0.145},workQuat,nil)
                end
				
                suctionCupCmd('on')
                sim.wait(0.05) --S
                sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{data[i][1],data[i][2],0.3},workQuat,nil)
                --轮流将四个传送带上的方块放到移动车上
                for k=1,4,1 do
                    if(PioneerPositionOccupy[k]==0) then
                        sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{PioneerPositionMatrix[k][1],PioneerPositionMatrix[k][2],0.3},workQuat,nil)
                        sim.wait(0.05)
                        if(size==1) then
                            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{PioneerPositionMatrix[k][1],PioneerPositionMatrix[k][2],0.29},workQuat,nil)
                            --sim.wait(0.1)
                        else
                            sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{PioneerPositionMatrix[k][1],PioneerPositionMatrix[k][2],0.28},workQuat,nil)
                            --sim.wait(0.1)
                        end
                        --关闭吸盘
                        suctionCupCmd('off')
                        sim.wait(0.05)
                        sim.rmlMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel2,ikMaxAccel2,ikMaxJerk,{PioneerPositionMatrix[k][1],PioneerPositionMatrix[k][2],0.35},workQuat,nil)
                        sim.wait(0.05)
                        PioneerPositionOccupy[k]=1
                        break
                    end
                end
            end
            --发送信号finish1：码垛结束
            sim.setInt32Signal('finish1',1)
            while true do
                if(sim.getInt32Signal('done')==1) then
                    --清除信号getdata:
                    --清除信号done:
                    --发送信号finishDone：
                    sim.clearInt32Signal('getdata')
                    sim.clearInt32Signal('done')
                    sim.setInt32Signal('finishDone',1)
                    break
                end
            end
            --清楚信号getdata:
            sim.clearInt32Signal('getdata')
        end
    end
    enableIk(false)
end

```



## 摄像头

### 主函数

```lua
function sysCall_actuation()
    --get signal
    take_photo=sim.getInt32Signal("Take_side_photo")
    if sim.readProximitySensor(sensor)==0 then
        tick=0
    end
    --if sim.readProximitySensor(sensor)>0 and(tick==0)and(take_photo~=0)then
    if (take_photo~=0)then
        tick=1
        sim.handleVisionSensor(cam) -- the image processing camera is handled explicitely, since we do not need to execute that command at each simulation pass
        result,t0,t1=sim.readVisionSensor(cam)
        blobCount=t1[1]  --total blobs detected
        dataSizePerBlob=t1[2] --data size per blob
        blob_num = 0
        sim.setInt32Signal("Take_side_photo",0)
        for i=1,blobCount,1 do
            if t1[2+(i-1)*dataSizePerBlob+1]>0.001 then
                blob_num = blob_num+1
            end
        end
        print("blobnum="..blob_num)
        if (t1) then
            -- Now we go through all blobs:            
            if blob_num==1 then
                blobRelativeSize=t1[3]
                blobOrientation=t1[4]
                blobRelativePos={t1[5],t1[6]}
                color = sim.getVisionSensorImage(cam,res[1]*blobRelativePos[1],res[2]*blobRelativePos[2],1,1,0)
                if color[2]>color[1] then 
                    sim.setInt32Signal("green",1)
                    print("side Colour is Green")
                else
                    sim.setInt32Signal("green",0)
                    print("side Colour is Red")
                end
             elseif blob_num>1 then
                print("blob_num>1")
                blobRelativeSize=t1[3]
                if blobRelativeSize>=0.04 then
                   blobRelativePos={t1[5],t1[6]}
                    color = sim.getVisionSensorImage(cam,res[1]*blobRelativePos[1],res[2]*blobRelativePos[2],1,1,0)
                    if color[2]>color[1] then 
                        sim.setInt32Signal("green",1)
                        print("side Colour is Green B1")
                    else
                        sim.setInt32Signal("green",0)
                        print("side Colour is Red B1")
                    end
                else
                    blobRelativePos={t1[12],t1[13]}
                    color = sim.getVisionSensorImage(cam,res[1]*blobRelativePos[1],res[2]*blobRelativePos[2],1,1,0)
                    if color[2]>color[1] then 
                        sim.setInt32Signal("green",1)
                        print("side Colour is Green B2")
                    else
                        sim.setInt32Signal("green",0)
                        print("side Colour is Red B2")
                    end
                end
             end
             
        end
    end
end
```

