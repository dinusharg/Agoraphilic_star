import sys
import os

import rclpy 
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist     

import numpy as np
# import open3d as o3d
from matplotlib import pyplot as plt 

import ros2_numpy as rnp
import math
import time


plt.ion()

# Point cloud data scatter plot
fig_pointcloud,pc_scatter=plt.subplots()
pc_scatter.set_title("Point cloud data")

# Histrogram plots
fig_histraogram, plot_histrogram = plt.subplots(figsize=(6, 5))
plot_histrogram.set_title("Histraogram")
plot_histrogram.set_xlim(-190, 190)
# plot_histrogram.set_ylim(0, 1.1)
plot_histrogram.legend(loc="upper right")

# XY plots
fig_xy_plot, plot_xy = plt.subplots(figsize=(6, 5))
plot_xy.set_title("XY Plot")
plot_xy.set_xlim(-10, 10)
plot_xy.set_ylim(0, 25)

class PCDListener(Node):

    def __init__(self):
        super().__init__('pcd_subsriber_node')

        # x,z,y data from camara topics 
        self.xzy1=[]    # cam 1
        self.XYZ=[] # X,Y,Z data from all camaras

        self.histrogramData=[]  
                                # [ 0: sector angle, 
                                #   1: sector number, 
                                #   2: histrogram value, 
                                #   3: normalized histrogram value, 
                                #   4: Normalized squar histraogramvalue
                                #   5: shaping coeficent
                                #   6: initial driving force ]

        self.cam1_data=False

        # Set up a subscription to the 'pcd' topic with a callback to the 
        # function `listener_callback`
        self.pcd_subscriber = self.create_subscription(
            sensor_msgs.PointCloud2,    # Msg type
            '/depth_camera/points', # topic
            self.cam1_callback,      # Function to call
            10                          # QoS
        )


        self.scan_subscriber = self.create_subscription(
            TFMessage,
            '/tf',
            self.odom_callback,
            1
        )

        self.publisher_ = self.create_publisher(
            Twist, 
          '/skid/cmd_vel',
            10
        )


        self.msg_v = Twist()
        self.msg_v.linear.x = 0.0
        self.msg_v.linear.y = 0.0
        self.msg_v.linear.z = 0.0
        self.msg_v.angular.x = 0.0
        self.msg_v.angular.y = 0.0
        self.msg_v.angular.z = 0.0 

        self.robot_rotation_speed=0.1
        self.robot_forward_speed=0.1

        self.rotate_direction=1 # anticlockwise
        self.target_roate_angle=0

        self.yaw_degree=0
        self.pitch_degree=0
        self.roll_degree=0
        
        self.odom_x=0
        self.odom_y=0
        self.odom_z=0

        self.mainLoop=False
        self.robotRotate=False
        self.waitForPositionData=False
        self.waitForPositionDataItaration=0
        self.direction1=True
        self.plotTarget=True
        self.coe_Total=1
        self.angle_ratio=0.5
        self.distance_ratio=1-self.angle_ratio
        self.last_anguler_velocity=0
        self.goal_sector=0
        self.sec_strart_angle=0
        self.sec_end_angle=359  
        self.safty_distance=2
        self.moving_direction_angle=0
        self.moving_direction_sector=0
        self.total_force=0
        self.sec_angle=0
        self.robot_width=0
        self.robot_length=0
        self.robot_climbing_angle=0
        self.sensor_max=0
        self.sensor_min=0
        self.cell_width=0
        self.number_of_filter_cells=0
        self.target_x=0
        self.target_y=0

        self.initParameters()

        self.sec_descreeate_angels=np.arange(self.sec_strart_angle,self.sec_end_angle,self.sec_angle)  # descreate sector angles
        self.cell_distance=np.arange(self.sensor_min,self.sensor_max,self.cell_width) # decreate cell distance

    def initParameters(self):
        print('Init parameters')
            
        self.sec_angle=5        # Sector angle

        #-------- Robot parameters
        self.robot_width=0.75            # Robot width
        self.robot_length=0.9           # Robot length
        self.robot_climbing_angle=35    # Robot maximum climbing angle

        #-------- Sensor parameters
        self.sensor_max=2 # Maximum sensor range
        self.sensor_min=0.3 # Minimum sensor range
            
        #-------- Cell parameters
        self.cell_width=0.1 # cell width
        self.number_of_filter_cells=3 # number of cell acount to filter the TFSI

        #---------- Target X Y -----------------------
        self.target_x=3
        self.target_y=11

    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians



    def odom_callback(self, msg):
        for tf in msg.transforms:
            transform=tf.transform
            translation=transform.translation
            rotation=transform.rotation
            
            self.odom_x=translation.x
            self.odom_y=translation.y
            self.odom_z=translation.z
            self.odom_angle=rotation.z

            if self.waitForPositionData==True:
                if self.waitForPositionDataItaration > 5:
                    self.waitForPositionData=False
                    self.waitForPositionDataItaration =0
                    self.mainLoop=False
                self.waitForPositionDataItaration=self.waitForPositionDataItaration +1 

            (self.roll, self.pitch, self.yaw) = self.euler_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)

            self.yaw_degree=math.degrees(self.yaw)
            if self.yaw_degree<=0:
                self.yaw_degree=-1*self.yaw_degree
            else:
                self.yaw_degree=360-self.yaw_degree

            self.pitch_degree=round(math.degrees(-1*self.pitch),0)
            self.roll_degree=round(math.degrees(self.roll),0)

            if self.robotRotate==True:
                if (self.target_roate_angle-2)<self.yaw_degree < (self.target_roate_angle+2):
                    self.stopRotation()
                    self.robotRotate=False
                    self.go_forward(self.total_force)

            self.yaw_degree=round(self.yaw_degree)
            if self.yaw_degree==360:
                self.yaw_degree=0
            

    def rotateRobot(self):
        print('Robot rotate')
        self.msg_v.angular.z= self.rotate_direction * self.robot_rotation_speed
        self.publisher_.publish(self.msg_v)
        self.robotRotate=True

    def stopRotation(self):
        self.msg_v.angular.z= 0.0
        self.publisher_.publish(self.msg_v)
        print('Robot rotate : Stop')

    def go_forward(self,force):
        if force==0:
            force=1
        force_ratio=force/self.coe_Total
        self.msg_v.linear.x=float(force_ratio*15)

        self.last_anguler_velocity=self.msg_v.linear.x
        print('Start moving the robot')
        self.publisher_.publish(self.msg_v)
        time.sleep(1)
        self.msg_v.linear.x = 0.0
        self.publisher_.publish(self.msg_v)
        print('Robot moving complete')
        self.waitForPositionData=True

    # calculate the target angle
    def calculate_target_angle(self):
        tana=abs((self.target_y-self.odom_y))/abs((self.target_x-self.odom_x))
        atan=math.atan(tana)
        atan_degree=math.degrees(atan)
        if (self.target_x-self.odom_x) > 0 and (self.target_y-self.odom_y) < 0:
            atan_degree=atan_degree
        elif (self.target_x-self.odom_x) < 0 and (self.target_y-self.odom_y) < 0:
            atan_degree=180-atan_degree
        elif (self.target_x-self.odom_x) < 0 and (self.target_y-self.odom_y) > 0:
            atan_degree=180+atan_degree
        else:
            atan_degree=360-atan_degree

        return atan_degree
    
    # Target sector calculation
    def calculate_target_sector(self,target_angle):
        if (target_angle-self.yaw_degree)<0:
            target_sector_a=360+(target_angle-self.yaw_degree)
        else:
            target_sector_a=target_angle-self.yaw_degree

        target_sector=int(target_sector_a/self.sec_angle)

        print('Target sector: ',target_sector)
        return target_sector
    

    
    def main_Loop(self):
        if self.mainLoop==False:

            if self.plotTarget==True:
                self.plotTarget=False
                self.plotTargetXY()

            print('-----------main loop---------------')
            self.mainLoop=True
            ta=self.calculate_target_angle()
            self.goal_sector=self.calculate_target_sector(ta)
            self.pointCloudPreProcess()
            self.histrogramPreperation()
            self.shapingCoeficent()
            self.drivingForceCalculation()
            self.rotationParameterCalculation()
            self.XYpositionPlot()

            if self.moving_direction_sector==0:
                self.go_forward(self.total_force)
            else:
                self.rotateRobot()

            

    # -------- Process the 3D point cloud data from the depth camera -------
    def pointCloudPreProcess(self):
        # X direction - up ; Y direction right; clockwise angle reprecentation

        _XZY=self.xzy1  # read the point cloud data from depth camera topic

        # round the xyz data ; decimals=1 x.y (10 cm resalution) ; decimals=2 x.yy (1 cm resalution)
        _XZY[:,0]=np.round(_XZY[:,0],decimals=1)
        _XZY[:,1]=-1*np.round(_XZY[:,1],decimals=1) # rotate Z data 180 degree
        _XZY[:,2]=np.round(_XZY[:,2],decimals=1)

        self.XYZ=_XZY
        self.XYZ[:,[1,2]]=self.XYZ[:,[2,1]] # interchage the array data to XZY to XYZ
        self.XYZ=np.unique(self.XYZ,axis=0) # filter the unique data points to reduce the number of data in the array


        # Point cloud data XY scatter plot 
        pc_scatter.clear()
        scatter_1=pc_scatter.scatter(self.XYZ[:,0],self.XYZ [:,1],s=5,c=self.XYZ [:,2],cmap='jet')
        pc_scatter.set_xlim(-5,5)
        pc_scatter.set_ylim(0,5)
        pc_scatter.set_title("Point cloud data")
        fig_pointcloud.canvas.draw()
        fig_pointcloud.canvas.flush_events()


    # -------- Histrogram preperation -------
    def histrogramPreperation(self):

        self.histrogramData=[] # clear the histrogram data 

        sector_number=0

        # sectorwise data analizis
        for ang in self.sec_descreeate_angels:

            one_sector_data=[] # one sctor data  [sector angle, cell, cell angle, TFSI, Filtered TFSI]  
            # clockwisw transform
            ang_c=-1*ang # clockwise transform angle

            #Tarnsform matrix
            rot_matrix=np.array([[np.cos(math.radians(ang_c)), -np.sin(math.radians(ang_c)),0],
                                 [np.sin(math.radians(ang_c)), np.cos(math.radians(ang_c)),0],
                                 [0,0,1],])
            
            rotData = np.dot(self.XYZ,rot_matrix) # tansformed data by ang_c in clockwise

            # conditions to filter the data in a sector by robot width
            condition1=rotData[rotData[:,0]> -1*(self.robot_width)/2]
            condition2=condition1[condition1[:,0]< (self.robot_width)/2]

            # conditions to filter the each sector data by maximum and minumum sensor range
            condition3=condition2[condition2[:,1]>self.sensor_min]
            sec_xyz_rot=condition3[condition3[:,1]<self.sensor_max]
            
            if ang>180:
                ang_map=ang-360
            else:
                ang_map=ang
            ang_map=ang_map
            

            # cell devide
            for c in self.cell_distance:
                c_max=c+self.cell_width       # cell upper boundry
                c_min=c_max-self.robot_length # cell lower boundry

                # conditions to filter the cell data
                condition4=sec_xyz_rot[sec_xyz_rot[:,1]>c_min]
                cell_data=condition4[condition4[:,1]<c_max]

             
                # Calculate the cell angle by fitting the plane
                if len(cell_data) >0:

                    centroid         = cell_data.mean(axis = 0)
                    xyzR             = cell_data - centroid   
                    u, sigma, v       = np.linalg.svd(xyzR)
                    normal            = v[2,:]                          
                    normal            = normal / np.linalg.norm(normal)

                    normal_sum=normal[2]

                    ang_r=math.acos(normal_sum)  # plane angle in radians
                    ang_d=math.degrees(ang_r)    # plane angle in degree
                    if ang_d >90:
                        ang_d=180-ang_d

                    if self.pitch_degree > self.robot_climbing_angle:
                        t_cost=1
                    else:
                        t_cost=ang_d/self.robot_climbing_angle  # calculate the TFSI 

                    if self.pitch_degree > self.robot_climbing_angle:
                        t_cost1=1
                    elif ang_d > self.pitch_degree:
                        t_cost1=(ang_d-self.pitch_degree)/self.robot_climbing_angle  # calculate the TFSI 
                    else:
                        t_cost1=ang_d/self.robot_climbing_angle  # calculate the TFSI 
                    
                    new_row=np.array([ang,c,ang_d,t_cost,t_cost,t_cost1,t_cost1])
                else:
                    new_row=np.array([ang,c,np.nan,np.nan,np.nan,np.nan,np.nan])
                    
                if len(one_sector_data)==0:
                    one_sector_data=new_row
                else:
                    one_sector_data=np.vstack((one_sector_data,new_row))


            # Filter the TFSI 
            filter=False
            i=0
            max_cell=0
            for t in one_sector_data[:,3]:
                if filter==False:
                    if t>=1:
                        sub_array=one_sector_data[i+1:i+self.number_of_filter_cells,3]
                        window_average=np.average(sub_array)
                        if(window_average<1):
                            one_sector_data[i,4]=window_average
                            max_cell=one_sector_data[i,1]
                        else:
                            one_sector_data[i,4]=np.nan
                            filter=True
                    else:
                        one_sector_data[i,4]=t
                        if np.isnan(t)==False:
                            max_cell=one_sector_data[i,1]
                else:
                    one_sector_data[i,4]=np.nan
                i=i+1

            filter=False
            i=0
            for t1 in one_sector_data[:,5]:
                if filter==False:
                    if t1>=1:
                        sub_array=one_sector_data[i+1:i+self.number_of_filter_cells,5]
                        window_average=np.average(sub_array)
                        if(window_average<1):
                            one_sector_data[i,6]=window_average
                        else:
                            one_sector_data[i,6]=np.nan
                            filter=True
                    else:
                        one_sector_data[i,6]=t1
                else:
                    one_sector_data[i,6]=np.nan
                i=i+1


            # remove nan values and calculate the histrogram values in each cell
            h=one_sector_data[:,4]
            nan_mask = np.isnan(h)
            h1 = h[~nan_mask]
            h2=1-h1
            hValue=np.sum(h2)  # calculate the histrogram value
            hNorm=hValue/len(self.cell_distance) # calculate the normalized histrogram value
            hNormSquar=hNorm**2

            h2=one_sector_data[:,6]
            nan_mask2 = np.isnan(h2)
            h3 = h2[~nan_mask2]
            h4=1-h3
            hValue1=np.sum(h4)  # calculate the histrogram value
            hNorm2=hValue1/len(self.cell_distance) # calculate the normalized histrogram value
            hNormSquar2=hNorm2**2

            #update the histrogram data
            if ang > 180 :
                ang_a=ang-360
            else:
                ang_a=ang

            if max_cell <self.safty_distance:
                max_cell=0
            max_cell_norm=max_cell/self.sensor_max
            _row=np.array([ang,sector_number,hValue,hNorm,hNormSquar,0,0,0,0,ang_a,max_cell,max_cell_norm,hNormSquar2])  
            if len(self.histrogramData)==0:
                self.histrogramData=_row
            else:
                self.histrogramData=np.vstack((self.histrogramData,_row))

        sector_number=sector_number+1

        # plot the histrogram data
        plot_histrogram.clear()
        plot_histrogram.bar(self.histrogramData[:,9],self.histrogramData [:,4],label='TFSH') # normalized squar histrogram
        
        if self.goal_sector*self.sec_angle > 180:
            sec_angle_a=self.goal_sector*self.sec_angle - 360
        else:
            sec_angle_a=self.goal_sector*self.sec_angle
        plot_histrogram.scatter(sec_angle_a,1,s=5,c="red", marker="^",label='Target')  # target direction
        plot_histrogram.set_title("Histraogram")
        plot_histrogram.legend()


     # ----------- Shaping coeficent calculation. -------------------
    def shapingCoeficent(self):

        numberOfSectores=len(self.histrogramData[:,0])
        m=0
        for hv in self.histrogramData[:,2]:

            if m<= self.goal_sector:
                coe=abs((2/numberOfSectores)*(m-self.goal_sector)+1)
            else:
                coe=abs((-2/numberOfSectores)*(m-self.goal_sector)+1)

            self.histrogramData[m,5]=coe
            m=m+1

        plot_histrogram.bar(self.histrogramData[:,9]+3,self.histrogramData [:,5],label='SC',color ='red')
        self.coe_Total=np.sum(self.histrogramData[:,5])
        plot_histrogram.legend()


    # ----------- Driving force calculation-------------------
    def drivingForceCalculation(self):
        self.histrogramData[:,6]=self.histrogramData[:,4]*self.histrogramData[:,5]  # each sector force 

        self.histrogramData[:,7]=self.histrogramData[:,6]*np.cos(np.radians(self.histrogramData[:,0])) # each sector X direction force
        self.histrogramData[:,8]=self.histrogramData[:,6]*np.sin(np.radians(self.histrogramData[:,0])) # each sector Y direction force

        plot_histrogram.bar(self.histrogramData[:,9]+4,self.histrogramData [:,6],label='Driving Force',color ='green',width=2)
  
        F_Cos_Total=np.sum(self.histrogramData[:,7]) # Total X direction force
        F_Sin_Total=np.sum(self.histrogramData[:,8]) # Total Y direction force
        direction_angle=math.degrees(math.atan((abs(F_Sin_Total)/abs(F_Cos_Total)))) # direction angle

        self.total_force=math.sqrt(F_Cos_Total**2+F_Sin_Total**2)   # Total moving force


        if  np.isnan(direction_angle):
            # rotate to goal direction when no driving force
            self.moving_direction_angle= self.goal_sector*self.sec_angle
            F_Cos_Total=0
            F_Sin_Total=0
        else:
            if self.total_force <1:
                self.total_force =1
                self.moving_direction_angle=self.goal_sector*self.sec_angle
            else:
                # calculat the direction angle in 360 plane
                if F_Cos_Total >=0:
                    if F_Sin_Total >= 0:
                        self.moving_direction_angle=direction_angle
                    else:
                        self.moving_direction_angle=360-direction_angle
                else:
                    if F_Sin_Total >= 0:
                        self.moving_direction_angle=180-direction_angle
                    else:
                        self.moving_direction_angle=180+direction_angle

      

        self.moving_direction_sector=round(self.moving_direction_angle/self.sec_angle)  # moving direction sector
        if self.moving_direction_sector*self.sec_angle > 180:
            sec_angle_a=self.moving_direction_sector*self.sec_angle -360
        else:
            sec_angle_a=self.moving_direction_sector*self.sec_angle 

        plot_histrogram.scatter(sec_angle_a,1,s=5,c="green", marker="o",label='Driving Direction') # moving direction
        plot_histrogram.legend()



    # ----------- Rotation parameter calculation -------------------
    def rotationParameterCalculation(self):

        target_angle=self.moving_direction_angle+self.yaw_degree

        if target_angle >360:
            target_angle=target_angle-360

        self.target_roate_angle=target_angle  # target robot rotation angle

        rd=0
        if self.yaw_degree>target_angle:
            self.direction1=False
            if ( (self.yaw_degree-target_angle) < (360-self.yaw_degree+target_angle)):
                self.rotate_direction=1
                rd=abs(self.yaw_degree-target_angle)
            else:
                self.rotate_direction=-1
                rd=abs(360+self.yaw_degree-target_angle)
        else:
            self.direction1=True
            if ( (target_angle-self.yaw_degree) < (360-target_angle+self.yaw_degree)):
                self.rotate_direction=-1
                rd=abs(self.yaw_degree-target_angle)
            else:
                self.rotate_direction=1
                rd=abs(360+self.yaw_degree-target_angle)


        if rd <15:
            self.robot_rotation_speed=0.1
        else:
            self.robot_rotation_speed=0.3


    #------------------ Plot the XY position ---------------------------
    def XYpositionPlot(self):
        plot_xy.scatter(-1*self.odom_y,self.odom_x,s=5,c="red", marker="o")
        plot_xy.set_title("XY Plot")


     #------------------ Plot the start end xy ---------------------------
    def plotTargetXY(self):  
        plot_xy.scatter(-1*self.odom_y,self.odom_x,s=10,c="yellow", marker="o",label='Start')     
        plot_xy.scatter(-1*self.target_y,self.target_x,s=10,c="green", marker="o",label='Goal')
        plot_xy.legend()

           

    def cam1_callback(self, msg):
        data = rnp.numpify(msg)
        pcd_xyz=data['xyz']

        filtered_pcd_xyz1 = pcd_xyz[~np.isinf(pcd_xyz[:,0])]
        filtered_pcd_xyz2 = filtered_pcd_xyz1[~np.isinf(filtered_pcd_xyz1[:,1])]
        _xyz1 = filtered_pcd_xyz2[~np.isinf(filtered_pcd_xyz2[:,2])]
        self.xzy1 =_xyz1
        self.cam1_data=True
        self.main_Loop()




def main(args=None):
    rclpy.init(args=args)
    pcd_listener = PCDListener()
    rclpy.spin(pcd_listener)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pcd_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()