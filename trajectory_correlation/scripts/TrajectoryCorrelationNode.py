#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import numpy

class TrajectoryCorrelation:

    def __init__(self):

        ### read params ###
        self.GpsTopic = rospy.get_param("GroundTruthTopic")
        self.VoTopic = rospy.get_param("EstimatedTrajectoryTopic")
        self.Offset = rospy.get_param("Offset")
        self.Scale = rospy.get_param("Scale")
        self.MaxDifference = rospy.get_param("MaxDifference")
        self.Save = rospy.get_param("Save")
        self.SaveAssociations = rospy.get_param("SaveAssociations")
        self.Plot = rospy.get_param("Plot")
        self.Verbose = rospy.get_param("Verbose")
        self.WindowSize = rospy.get_param("WindowSize")

        self.Gps_Subscriber = rospy.Subscriber(self.GpsTopic,Odometry,self.gps_callback)
        self.vo_callback = rospy.Subscriber(self.VoTopic,Odometry,self.vo_callback)

        self.gps_buffer_global = []
        self.vo_buffer_global = []

    def gps_callback(self,msg):
        self.gps_buffer_global.append(msg)

    def vo_callback(self,msg):
        self.vo_buffer_global.append(msg)

    def msg_to_dict(self,buffer):
        """
        This function converts the incoming msg into dictionart of timestamps and list of odometry
        
        Input
        buffer -- vo or gps buffer

        Output
        dict -- dictionary of timestamp and list of odometry data

        """
        temp = []
        for msg in buffer:
            stamp = msg.header.stamp.to_sec()
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            
            temp.append((stamp,[x,y,z,qx,qy,qz,qw]))

        return dict(temp)

    def associate(self,gps_dict,vo_dict,offset,max_difference):
        """
        Associate two dictionaries of (stamp,data). as the time stamps never match exactly, we aim
        to find the closest match for every input tuple.

        Input:
        vo_dict -- dictionary of (stamp,data) tuples of visual odometry
        gps_dict -- dictionary od (stamp,data) tuples of gps

        output:
        matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))
        """

        gps_keys = list(gps_dict.keys())
        vo_keys = list(vo_dict.keys())

        potential_matches = [(abs(a-(b+offset)),a,b)
                             for a in gps_keys
                             for b in vo_keys
                             if abs(a - (b+offset)) < max_difference]

        potential_matches.sort()
        matches = []
        for diff , a , b in potential_matches:
            if a in gps_keys and b in vo_keys:
                gps_keys.remove(a)
                vo_keys.remove(b)
                matches.append((a,b))
        matches.sort()

        return matches

    def align(self,model,data):
        numpy.set_printoptions(precision=3,suppress=True)

        model_zerocentered = model-model.mean(1)
        data_zerocentered = data - data.mean(1)

        w = numpy.zeros((3,3))

        for column in range(model.shape[1]):
            w += numpy.outer(model_zerocentered[:,column],data_zerocentered[:,column])

        U , d , Vh = numpy.linalg.svd(w.transpose())

        S = numpy.matrix(numpy.identity(3))

        if (numpy.linalg.det(U) * numpy.linalg.det(Vh) < 0):
            S[2,2] = -1

        rot = U * S * Vh

        trans = data.mean(1) - rot * model.mean(1)

        model_aligned = rot * model + trans

        alignment_error = model_aligned - data

        trans_error = numpy.sqrt(numpy.sum(numpy.multiply(alignment_error,alignment_error),0)).A[0]

        return rot , trans , trans_error

    def plot_traj(self,ax,stamps,traj,style,color,label):
        stamps.sort()
        interval = numpy.median([s-t for s,t in zip(stamps[1:],stamps[:-1])])
        x = []
        y = []
        last = stamps[0]
        for i in range(len(stamps)):
            if stamps[i]-last < 2*interval:
                x.append(traj[i][0])
                y.append(traj[i][1])
            elif len(x)>0:
                ax.plot(x,y,style,color=color,label=label)
                label=""
                x = []
                y = []
            last = stamps[i]
        if len(x) > 0:
            ax.plot(x,y,style,color=color,label=label)

    def print_params(self):

        print("+---------------- PARAMS ----------------+")
        print(f"GpsTopic : {self.GpsTopic}")
        print(f"VoTopic : {self.VoTopic}")
        print(f"Offset : {self.Offset}")
        print(f"Scale : {self.Scale}")
        print(f"MaxDifference : {self.MaxDifference}")
        print(f"Save : {self.Save}")
        print(f"SaveAssociations : {self.SaveAssociations}")
        print(f"Plot : {self.Plot}")
        print(f"Verbose : {self.Verbose}")
        print(f"WIndowSize : {self.WindowSize}")


if __name__ == "__main__":

    rospy.init_node("trajectory_correlation")

    TC_Node = TrajectoryCorrelation()

    TC_Node.print_params()

    ### buffer to hold a data of vo and gps of required size
    vo_buffer_local = list()
    gps_buffer_local = list()
    buffer_pointer = 0

    Rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        if len(TC_Node.vo_buffer_global) > TC_Node.WindowSize and len(TC_Node.gps_buffer_global) > TC_Node.WindowSize:
            vo_buffer_local = TC_Node.vo_buffer_global[buffer_pointer:TC_Node.WindowSize]
            gps_buffer_local = TC_Node.gps_buffer_global[buffer_pointer:TC_Node.WindowSize]

            buffer_pointer += 1
            TC_Node.WindowSize += 1

            # print(f"len of vo_buffer_local : {len(vo_buffer_local)}")
            # print(f"len of gps_buffer_local : {len(gps_buffer_local)}")

            print("========== vo buffer ===========")
            for i,j in TC_Node.msg_to_dict(vo_buffer_local).items():
                print(f"{i} --- {j}")

            print("========== gps buffer ===========")
            for i,j in TC_Node.msg_to_dict(gps_buffer_local).items():
                print(f"{i} --- {j}")

            # dictionary of vo
            vo_dict = TC_Node.msg_to_dict(vo_buffer_local)
            # dictionary of gps
            gps_dict = TC_Node.msg_to_dict(gps_buffer_local)

            # find the matches between vo and gps timestamps
            matches = TC_Node.associate(gps_dict,vo_dict,TC_Node.Offset,TC_Node.MaxDifference)

            for i in matches:
                print(i)
            print(len(matches))


            # check if the matches has atleast two matched timestamps
            if len(matches) < 2:
                rospy.logerr("couldn't find matching timestamp pairs between GPS and vo !")
                exit()

            gps_xyz = numpy.matrix([[float(value) for value in gps_dict[a][0:3]] for a,b in matches]).transpose()
            vo_xyz = numpy.matrix([[float(value)*TC_Node.Scale for value in vo_dict[b][0:3]] for a,b in matches]).transpose()

            print("-------- gps_xyz -------")
            print(gps_xyz , gps_xyz.shape)
            print("-------- vo_xyz -------")
            print(vo_xyz,vo_xyz.shape)

            rot , trans , trans_error = TC_Node.align(vo_xyz,gps_xyz)

            print("----- rot ----")
            print(rot , rot.shape)
            print("---- trans -----")
            print(trans , trans.shape)
            print("---- trans_error -----")
            print(trans_error , trans_error.shape)

            vo_xyz_aligned = rot * vo_xyz + trans

            print("-----vo_xyz_aligned-----")
            print(vo_xyz_aligned , vo_xyz_aligned.shape)

            gps_stamps = list(gps_dict.keys())
            gps_stamps.sort()

            gps_xyz_full = numpy.matrix([[float(value) for value in gps_dict[b][0:3]] for b in gps_stamps]).transpose()

            vo_stamps = list(vo_dict.keys())
            vo_stamps.sort()

            vo_xyz_full = numpy.matrix([[float(value)*TC_Node.Scale for value in vo_dict[b][0:3]] for b in vo_stamps]).transpose()
            vo_xyz_full_aligned = rot * vo_xyz_full + trans

            if TC_Node.Verbose:
                print(f"Compared pose paairs {len(trans_error)}")
                print(f"absolute_translational_error.rmse {numpy.sqrt(numpy.dot(trans_error,trans_error)/len(trans_error))}")
                print(f"absolute_translational_error.mean {numpy.mean(trans_error)}")
                print(f"absolute_translational_error.median {numpy.median(trans_error)}")
                print(f"absolute_translational_error.std {numpy.std(trans_error)}")
                print(f"absolute_translational_error.min {numpy.min(trans_error)}")
                print(f"absolute_translational_error.max {numpy.max(trans_error)}")

            else:
                print(f"{numpy.sqrt(numpy.dot(trans_error,trans_error)/len(trans_error))}")

            if TC_Node.Plot:
                import matplotlib
                matplotlib.use("Agg")
                import matplotlib.pyplot as plt
                import matplotlib.pylab as pylab
                from matplotlib.patches import Ellipse

                fig = plt.figure()
                ax = fig.add_subplot(111)

                TC_Node.plot_traj(ax,gps_stamps,gps_xyz_full.transpose().A,"-","black","GPS")
                TC_Node.plot_traj(ax,vo_stamps,vo_xyz_full_aligned.transpose().A,"-","blue","VO")

                label = "difference"

                for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(matches,gps_xyz.transpose().A,vo_xyz_aligned.transpose().A):
                    ax.plot([x1,x2],[y1,y2],"-",color="red",label=label)
                    label = ""
                ax.legend()
                ax.set_xlabel('x[m]')
                ax.set_ylabel('y[m]')

                plt.savefig(f"/home/tharun/vins_ws/src/trajectory_correlation/scripts/{rospy.Time.now()}",dpi=90)
                # plt.show()

            Rate.sleep()

    rospy.spin()