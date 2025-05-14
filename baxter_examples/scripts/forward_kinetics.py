from baxter_pykdl import baxter_kinematics

baxter_kinematics = baxter_kinematics("left")

joint_angles_left = {'left_s0':  1,
                            'left_s1': 0,
                            'left_e0': 0,
                            'left_e1': 0,
                            'left_w0': 0,
                            'left_w1': 0,
                            'left_w2': 0}

pose = baxter_kinematics.forward_position_kinematics(joint_angles_left)

print(pose)
    