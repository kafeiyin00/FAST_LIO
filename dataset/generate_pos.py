import os
import numpy as np
from scipy.spatial.transform import Rotation   


frame_path = "/home/ljp/DATA/frames/"
pose_file_path = "/home/ljp/DATA/frame_poses.txt"

pose_file = open(pose_file_path,"w")

all_files = os.listdir(frame_path)
stems = []

for file in all_files:
    if(file.endswith(".jpg")):
        # print(file)
        base_name=os.path.splitext(file)[0]
        stems.append(base_name)

stems.sort()

for stem in stems:
    odom_path = frame_path+stem+".odom"
    # os.system("cat "+ odom_path)
    pose = np.loadtxt(odom_path)
    rot = pose[0:3,0:3]
    r =  Rotation.from_matrix(rot)
    angles = r.as_euler("zxy",degrees=True)
    pose_file.write('{} {} {} {}\n'.format(stem+".jpg", pose[0,3],pose[1,3],pose[2,3]))

pose_file.close()    