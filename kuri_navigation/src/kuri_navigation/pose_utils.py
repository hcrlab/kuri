import geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def se2_to_pose(se2):
    '''
    se2 (tup of 3 floats): (x, y z)
    '''
    p = geometry_msgs.msg.Pose()
    p.position.x = se2[0]
    p.position.y = se2[1]
    q = quaternion_from_euler(0, 0, se2[2])
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    return p


def pose_to_se2(pose):
    '''
    Converts a geometry_msgs.Pose to (x, y, theta) tuple.
    '''
    q = pose.orientation
    return (pose.position.x, pose.position.y,
            euler_from_quaternion([q.x, q.y, q.z, q.w])[2])


def _cov_list(x, y, t):
    return [x * x, 0, 0, 0, 0, 0,
            0, y * y, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, t*t]


def _pose_to_posecov(pose, cov_params):
    '''
    pose is [x, y, t]
    pose_cov is [x_cov, y_cov, theta_cov]
    '''
    ps = geometry_msgs.msg.PoseWithCovariance()
    ps.pose = pose
    ps.covariance = _cov_list(*cov_params)
    return ps