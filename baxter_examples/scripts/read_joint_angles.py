joint_names= "head_nod, head_pan, left_e0, left_e1, left_s0, left_s1, left_w0, left_w1, left_w2, right_e0, right_e1, right_s0, right_s1, right_w0, right_w1, right_w2, torso_t0"
lst_joint_names = joint_names.split(", ")
joint_angles = [0.0, -0.04065049087896346, -0.0015339807878854137, 0.03489806292439316, 0.998237997716433, -0.0019174759848567672, 0.0015339807878854137, -0.0030679615757708274, -0.0023009711818281204, 0.5042961840173298, 1.9819031779479546, 0.22281070944035636, -1.0722525707319042, -0.14304370847031483, -0.8329515678217797, -0.13460681413694506, -12.565987119160338]
order = [4, 5, 2, 3, 6, 7, 8, 11, 12, 9, 10, 13, 14, 15]
desired_joints = {'left_s0':  -0.6,
                            'left_s1': -1.0,
                            'left_e0': 0,
                            'left_e1': 2,
                            'left_w0': 0.2,
                            'left_w1': -0.9,
                            'left_w2': -0.1,
                            'right_s0':  0.22123,
                            'right_s1': -1.0741,
                            'right_e0': 0.50658,
                            'right_e1': 1.984,
                            'right_w0': -0.14492,
                            'right_w1': -0.83544,
                            'right_w2': -0.13709}

for i in range(len(order)):
    print(lst_joint_names[order[i]] +": " +str(joint_angles[order[i]]))
    print('error in angle ' +lst_joint_names[order[i]]+":" + str(joint_angles[order[i]] - desired_joints[lst_joint_names[order[i]]]))




