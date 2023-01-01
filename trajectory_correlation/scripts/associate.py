#! /usr/bin/env python3

def msg_to_dict(buffer):
    """
    This function converts the incoming msg into dictionart of timestamps and list of odometry
    
    Input
    buffer -- vo or gps buffer

    Output
    dict -- dictionary of timestamp and list of odometry data

    """
    temp = []
    for msg in buffer:
        stamp = msg.header.stamp
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        qx = x = msg.pose.pose.orientation.x
        qy = x = msg.pose.pose.orientation.y
        qz = x = msg.pose.pose.orientation.z

        temp.append((stamp,[x,y,z,qx,qy,qz]))

    return dict(temp)
