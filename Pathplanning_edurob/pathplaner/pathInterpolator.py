import numpy as np 
from enum import Enum
import csv


def func(x,point1, point2):
    m = (point1[1]-point2[1])/(point1[0]-point2[0])
    b = point1[1]-m*point1[0]
    return x*m+b

def lengthTwoPoints(point1, point2):
    return np.sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2 )

class Movement(Enum):
    Acceleration = 1
    Deceleration = 2
    Constant = 3

def translationMovement(v_trans, world_xv, world_yv, pose, movement, direction, runtime, interval, acceleration):
    timestamp = 0
    if movement == Movement.Acceleration or movement == Movement.Deceleration:
        intendet_pose = [pose[-1][0] + (0.5 * np.cos(direction)*acceleration * runtime**2), pose[-1][1] + (0.5 * np.sin(direction)*acceleration * runtime**2)]
    else:
        intendet_pose = [pose[-1][0] + (np.cos(direction)*v_trans * runtime), pose[-1][1] + (np.sin(direction)*v_trans * runtime)]
        pose[-1] + (v_trans * runtime)

    if movement == Movement.Acceleration:
        v_temp = 0
    elif movement == Movement.Deceleration or movement == Movement.Constant:
        v_temp = v_trans 
        
    while round(timestamp + interval, 4) <= runtime:
        timestamp = timestamp + interval
        if movement == Movement.Acceleration:
            ## Calculating the new trans Velocity
            v_temp = v_temp + acceleration * interval #Velocity increases because of Acceleration 
        elif movement == Movement.Deceleration:
            ## Calculating the new trans Velocity
            v_temp = v_temp - acceleration * interval #Velocity decreases because of Deceleration 

        world_xv.append(np.cos(direction)*v_temp) #Velocity in x Direction 
        world_yv.append(np.sin(direction)*v_temp) #Velocity in y Direction

        pose.append([pose[-1][0] + world_xv[-1]*interval, pose[-1][1] + world_yv[-1]*interval]) #new x and y Pose based on last pose and Velocity
 
    if(movement != Movement.Deceleration): 
        pose.append(intendet_pose)
        world_xv.append(np.cos(direction)*v_trans) 
        world_yv.append(np.sin(direction)*v_trans)
    else:
        pose.append(intendet_pose)
        world_xv.append(0) 
        world_yv.append(0)

def rotationMovement(v_rot, world_thetav, theta, movement, runtime, interval, acceleration):
    timestamp = 0; 
    if(movement == Movement.Acceleration or movement == Movement.Deceleration):
        intendet_theta = theta[-1] + (0.5 * acceleration * runtime ** 2) #The intendet Angle that should have been reached after ac/deceleration 
    else:
        intendet_theta = theta[-1] + (v_rot * runtime) #The intendet Angle that should have been reached after constant Movement 

    if movement == Movement.Acceleration:
        v_temp = 0
    elif movement == Movement.Deceleration or movement == Movement.Constant:
        v_temp = v_rot     
        
    while round(timestamp + interval, 4) < runtime:
        timestamp = timestamp + interval
        if movement == Movement.Acceleration:
            ## Calculating the new rot Velocity
            v_temp = v_temp + acceleration * interval #Velocity increases because of Acceleration 
        elif movement == Movement.Deceleration:
            ## Calculating the new rot Velocity
            v_temp = v_temp - acceleration * interval #Velocity decreases because of Deceleration 
        
        world_thetav.append(v_temp) #new Rotational Vel based in Rot Acceleration

        theta.append(theta[-1] + world_thetav[-1]*interval) #new theta based on last theta and Rotational velocity Mod 2PI to start at 0 after a full rotation

    

    if(movement != Movement.Deceleration): 
        theta.append(intendet_theta)
        world_thetav.append(v_rot)
    else:
        theta.append(intendet_theta)
        world_thetav.append(0)


def pathInterpolator(points, angles, v_trans_init,interval=50, a_trans = 0.05, v_rot_init= 0.2, a_rot=0.1):
    ##Check if the parameters are valid 
    # -1 = number of points and angles not equal
    # -2 = velocity or acceleration is 0 or negativ
    if(len(points,) != len(angles)):
        return -1
    elif(v_trans_init <= 0 or a_trans <= 0 or v_rot_init <= 0 or a_rot <= 0):
        return -2
    csv_file = []

    for x in range(len(angles)): #Convert angle from Degree to Rad 
        angles[x] = np.deg2rad(angles[x])

    interval = interval/1000 #interval in ms but vel in m/s -> convert ms in s
    v_trans = v_trans_init
    v_rot = v_rot_init

    pose = [points[0]]
    world_xv = [0]
    world_yv = [0]

    theta = [angles[0]]
    world_thetav = [0]

    for x in range(len(points)-1): #For loop to traverse through all points and Plan path between
        pointA = points[x]
        pointB = points[x+1]
        s_AB = lengthTwoPoints(pointA, pointB) #distance to be covert by translation
        theta_AB = angles[x+1] - angles[x] # angle to be covert by rotation
        movementDirection = np.arctan2(points[x+1][1]-points[x][1],points[x+1][0]-points[x][0]) ## atan2(y_dot, x_dot) determins movementDirection
        v_trans = v_trans_init
        v_rot = v_rot_init

        t_trans = s_AB / v_trans +  v_trans / a_trans #Time for translational Movement with Ac/Decelerationramp

        t_rot = theta_AB / v_rot + v_rot / a_rot #Time for rotational Movement with Ac/Decelerationramp

        if t_trans > t_rot: #translational Movement leading
            ########### Ac/Decelaration Ramp for translational ##########
            t_trans_ramp = abs(v_trans / a_trans)
            s_ramp = 0.5 * a_trans * (t_trans_ramp)**2 # s = 0.5 * a * t^2 --> distance covered during ac/deceleration time

            #Case distance to pass smaller than distance covert by ramp
            if abs(s_AB) < abs(2*s_ramp): 
                t_trans_ramp = np.sqrt(s_AB/2 * 2 * a_trans) #s_AB/2 because half the distance is covert in Acceleration and the other half in Deceleration
                t_trans_constant = 0 #No need to drive when distance is covert by ramp
            else: 
                t_trans_constant = t_trans - 2*t_trans_ramp #time for the section AB without de/acceleration
        	
            #Acceleration Ramp:
            translationMovement(v_trans, world_xv, world_yv, pose, Movement.Acceleration, movementDirection, t_trans_ramp, interval, a_trans)
            
            #Constant Movement:
            if t_trans_constant != 0:
                translationMovement(v_trans, world_xv, world_yv, pose, Movement.Constant, movementDirection, t_trans_constant, interval, a_trans)
            
            #Deceleration Ramp:
            translationMovement(v_trans, world_xv, world_yv, pose, Movement.Deceleration, movementDirection, t_trans_ramp, interval, a_trans)


            # ##########Ac/Decelaration Ramp for rotational##########
            v_rot = (a_rot * t_trans)/2 - np.sqrt(((a_rot * t_trans)/2)**2 - (theta_AB * a_rot)) # calc the max rot Vel to Sync to the trans Vel
            t_rot_ramp = abs(v_rot / a_rot)
            theta_ramp = 0.5 * a_rot * (t_rot_ramp)**2 # theta = 0.5 * a * t^2 --> angle covered during Ac/Deceleration
            a_rot_temp = a_rot if theta_AB > 0 else -a_rot

            #Case distance to pass smaller than distance covert by ramp
            if abs(theta_AB) < abs(2*theta_ramp): 
                t_rot_ramp = np.sqrt(theta_AB/2 * 2 * a_rot) #theta_AB/2 because half the distance is covert in Acceleration and the other half in Deceleration
                t_rot_constant = 0 #No need to drive when distance is covert by ramp
            else: 
                t_rot_constant = t_trans - 2*t_rot_ramp #time for the section AB without de/acceleration (adapted to the translational Movement)


            #Acceleration Ramp
            rotationMovement(v_rot, world_thetav, theta, Movement.Acceleration, t_rot_ramp, interval, a_rot_temp)
            
            #Constant Rot Vel
            if t_rot_constant != 0:
                rotationMovement(v_rot, world_thetav, theta, Movement.Constant, t_rot_constant, interval, a_rot_temp)


            #Deceleration Ramp
            rotationMovement(v_rot, world_thetav, theta, Movement.Deceleration, t_rot_ramp, interval, a_rot_temp)
            

            #Delete points from the middle if rotational and translational movement not even because of rounding error
            difference = len(theta) - len(pose)
            if difference > 0: ##Theta has more points than pose
                for x in range(abs(difference)):
                    index = (int) (len(theta)/2)
                    theta.pop(index)
                    world_thetav.pop(index)
            if difference < 0: ##Pose has more points than theta
                for x in range(abs(difference)):
                    index = (int) (len(pose)/2)
                    pose.pop(index)
                    world_xv.pop(index)
                    world_yv.pop(index)


        else: #rotational Movement leading
            ########### Ac/Decelaration Ramp for rotational##########
            t_rot_ramp = abs(v_rot / a_rot)
            theta_ramp = 0.5 * a_rot * (t_rot_ramp)**2 # theta = 0.5 * a * t^2 --> angle covered during Ac/Deceleration
            a_rot_temp = a_rot if theta_AB > 0 else -a_rot ##Acceleration is Negative if The Angle ist Negative

            #Case distance to pass smaller than distance covert by ramp
            if abs(theta_AB) < abs(2*theta_ramp): 
                t_rot_ramp = np.sqrt(theta_AB/2 * 2 * a_rot) #theta_AB/2 because half the distance is covert in Acceleration and the other half in Deceleration
                t_rot_constant = 0 #No need to drive when distance is covert by ramp
            else: 
                t_rot_constant = t_rot - 2*t_rot_ramp #time for the section AB without de/acceleration

            rotationMovement(v_rot, world_thetav, theta, Movement.Acceleration, t_rot_ramp, interval, a_rot_temp)
            
            #Constant Rot Vel
            if t_rot_constant != 0:
                rotationMovement(v_rot, world_thetav, theta, Movement.Constant, t_rot_constant, interval, a_rot_temp)


            #Deceleration Ramp
            rotationMovement(v_rot, world_thetav, theta, Movement.Deceleration, t_rot_ramp, interval, a_rot_temp)

            ########## Ac/Decelaration Ramp for translational ##########
            v_trans = (a_trans * t_rot)/2 - np.sqrt(((a_trans * t_rot)/2)**2 - (s_AB * a_trans)) # calc the trans Vel to Sync to the rot Vel

            t_trans_ramp = abs(v_trans / a_trans)
            s_ramp = 0.5 * a_trans * (t_trans_ramp)**2 # s = 0.5 * a * t^2 --> distance covered during ac/deceleration time

            #Case distance to pass smaller than distance covert by ramp
            if abs(s_AB) < abs(2*s_ramp): 
                t_trans_ramp = np.sqrt(s_AB/2 * 2 * a_trans) #s_AB/2 because half the distance is covert in Acceleration and the other half in Deceleration
                t_trans_constant = 0 #No need to drive when distance is covert by ramp
            else: 
                t_trans_constant = t_rot - 2*t_trans_ramp #time for the section AB without de/acceleration
        	
            #Acceleration Ramp:
            translationMovement(v_trans, world_xv, world_yv, pose, Movement.Acceleration, movementDirection, t_trans_ramp, interval, a_trans)
            
            #Constant Movement:
            if t_trans_constant != 0:
                translationMovement(v_trans, world_xv, world_yv, pose, Movement.Constant, movementDirection, t_trans_constant, interval, a_trans)
            
            #Deceleration Ramp:
            translationMovement(v_trans, world_xv, world_yv, pose, Movement.Deceleration, movementDirection, t_trans_ramp, interval, a_trans)
        
            #Delete points from the middle if rotational and translational movement not even because of rounding error
            difference = len(theta) - len(pose)
            if difference > 0: ##Theta has more points than pose
                for x in range(abs(difference)):
                    index = (int) (len(theta)/2)
                    theta.pop(index)
                    world_thetav.pop(index)
            if difference < 0: ##Pose has more points than theta
                for x in range(abs(difference)):
                    index = (int) (len(pose)/2)
                    pose.pop(index)
                    world_xv.pop(index)
                    world_yv.pop(index)
        
        pose.append(pose[-1])
        theta.append(theta[-1])
        world_thetav.append(0)
        world_xv.append(0)
        world_yv.append(0)

    for x in range(len(pose)):
        csv_file.append([round(pose[x][0], 5), round(pose[x][1], 5), round(theta[x], 5), round(world_xv[x], 5), round(world_yv[x], 5), round(world_thetav[x],5)])


    return csv_file

if __name__=='__main__':
    main()