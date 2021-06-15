from geometry_msgs.msg import Pose
from RobotControl.ur5eIKFast import ur5e_ik_fast


if __name__ == '__main__':
    pose1 = Pose()
    pose1.position.x = -0.271569271951
    pose1.position.y = -0.241139416964
    pose1.position.z = 0.709063871582
    pose1.orientation.x = 0.0017886399091
    pose1.orientation.y = -0.000163417056506
    pose1.orientation.z = -0.999993960405
    pose1.orientation.w = 0.00297543552536
    print(ur5e_ik_fast(pose1))

