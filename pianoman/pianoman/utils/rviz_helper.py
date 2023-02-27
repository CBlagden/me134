from visualization_msgs.msg import Marker

def create_pt_marker(msg, time):
    markermsg = Marker()
    markermsg.header.frame_id = "/base_link"
    markermsg.header.stamp = time
    markermsg.type = 2
    markermsg.id = 0
    markermsg.scale.x = 0.05
    markermsg.scale.y = 0.05
    markermsg.scale.z = 0.05
    markermsg.color.r = 0.1
    markermsg.color.g = 1.0
    markermsg.color.b = 0.0
    markermsg.color.a = 1.0
    markermsg.pose.position.x = msg.x
    markermsg.pose.position.y = msg.y
    markermsg.pose.position.z = msg.z
    return markermsg

def create_pt_marker(x, y, z, time):
    markermsg = Marker()
    markermsg.header.frame_id = "/base_link"
    markermsg.type = 2
    markermsg.id = 0
    markermsg.scale.x = 0.02
    markermsg.scale.y = 0.02
    markermsg.scale.z = 0.02
    markermsg.color.r = 0.1
    markermsg.color.g = 1.0
    markermsg.color.b = 0.0
    markermsg.color.a = 1.0
    markermsg.pose.position.x = x
    markermsg.pose.position.y = y
    markermsg.pose.position.z = z
    return markermsg
