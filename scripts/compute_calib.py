import numpy as np
from scipy.spatial.transform import Rotation   
import json

r1_points = {
    0: {
        "x": 0,
        "y": 0,
        "robot_x": 0.22000214574170682,
        "robot_y": -0.1406398611051523
    },
    1: {
        "x": 23,
        "y": 0,
        "robot_x": 0.4034203129302426,
        "robot_y":  -0.1379746634861015
    },
    2: {
        "x": 23,
        "y": 46,
        "robot_x": 0.3981234024498227,
        "robot_y": 0.22915046723627688
    },
    3: {
        "x": 0,
        "y": 46,
        "robot_x": 0.21474305391747062,
        "robot_y": 0.22614566233917838
    },
    4: {
        "x": 4,
        "y": 0,
        "robot_x": 0.251521685222123,
        "robot_y": -0.14048952133044798
    },
    5: {
        "x": 8,
        "y": 0,
        "robot_x": 0.2832043200281195,
        "robot_y": -0.1398952738685307
    },
    6: {
        "x": 8,
        "y": 6,
        "robot_x": 0.2825273463703032,
        "robot_y": -0.09182964910127303
    },
    7: {
        "x": 4,
        "y": 6,
        "robot_x": 0.25083751772438523,
        "robot_y": -0.09224600593392226
    },
    8: {
        "x": 0,
        "y": 6,
        "robot_x": 0.21882883461428462,
        "robot_y": -0.09271176639952028
    },
    
    
}

r2_points = {
    0: {
        "x": 0,
        "y": 46,
        "robot_x": 0.277327907857116,
        "robot_y": 0.12382007236392487
    },
    1: {
        "x": 0,
        "y": 40,
        "robot_x": 0.27809751981125813,
        "robot_y": 0.07587794534364982
    },
    2: {
        "x": 4,
        "y": 40 ,
        "robot_x": 0.3095814357459285,
        "robot_y": 0.07659417073222209
    },
    3: {
        "x": 8,
        "y": 40,
        "robot_x": 0.34164477046094194,
        "robot_y": 0.07722385241538297
    },
    4: {
        "x": 12,
        "y": 40,
        "robot_x": 0.373733254163431,
        "robot_y": 0.07784994484733229
    },
    5: {
        "x": 23,
        "y": 46,
        "robot_x": 0.46052034158879557,
        "robot_y": 0.12755110516096899
    },
    6: {
        "x": 23,
        "y": 0,
        "robot_x": 0.46770379825787683,
        "robot_y": -0.2395774144629258
    },
    7: {
        "x": 0,
        "y": 0,
        "robot_x": 0.28423025591956386,
        "robot_y": -0.24312326749943888
    }

}

def solve_calibration(points):
    """
    Solve for transformation matrix
    Returns 2x3 transformation matrix
    """
    num_points = len(points)
    A = np.zeros((2*num_points, 4))
    b = np.zeros(2*num_points)

    topleft_offset = np.identity(4)
    brick_offset = np.identity(4)
    brick_center_offset = np.identity(4)
    P_len = 0.008
    brick_len_offset = 0
    brick_height_m = 0.0096
    plate_height = 48
    plate_width = 48
    brick_height = 0
    brick_width = 2
 

    brick_center_offset[0, 3] = 0#(brick_height * P_len - brick_len_offset) / 2.0
    brick_center_offset[1, 3] = (brick_width * P_len - brick_len_offset) / 2.0
    brick_center_offset[2, 3] = 0

    topleft_offset[0, 3] = -(plate_height * P_len - brick_len_offset) / 2.0
    topleft_offset[1, 3] = -(plate_width * P_len - brick_len_offset) / 2.0
    topleft_offset[2, 3] = 0

    lego_poses = []
    world_poses = []
    for i, point in points.items():
        brick_offset[0, 3] = point["x"] * P_len - brick_len_offset
        brick_offset[1, 3] = point["y"] * P_len - brick_len_offset
        brick_offset[2, 3] = 1 * brick_height_m
        
        right_mtx = topleft_offset @ brick_offset @ brick_center_offset
        x = right_mtx[0, 3]
        y = right_mtx[1, 3]
        lego_poses.append([x, y])
        world_poses.append([point["robot_x"], point["robot_y"]])
    
    lego_poses = np.array(lego_poses)
    world_poses = np.array(world_poses)

    lego_centroid = np.mean(lego_poses, axis=0)
    world_centroid = np.mean(world_poses, axis=0)

    lego_centered = lego_poses - lego_centroid
    world_centered = world_poses - world_centroid

    # compute rotation with svd
    H = np.zeros((2, 2))
    for i in range(num_points):
        H += np.outer(world_centered[i], lego_centered[i])
    
    U, S, Vt = np.linalg.svd(H)
    R = U @ Vt
    # Ensure proper rotation matrix (determinant = 1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = U @ Vt

    # Get rotation angle
    theta = np.arctan2(R[1,0], R[0,0])
    #print("R", R, np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]))

    # Compute translation
    t = world_centroid - R @ lego_centroid

    # Compute error
    predicted_world = (R @ lego_poses.T).T + t
    error = np.linalg.norm(predicted_world - world_poses, axis=1)
    #print("predicted", predicted_world)
    #print("actual", world_poses)
    print("error", error)

    return t[0], t[1], theta, np.rad2deg(theta)

    # for i, point in points.items():

    #     brick_offset[0, 3] = point["x"] * P_len - brick_len_offset
    #     brick_offset[1, 3] = point["y"] * P_len - brick_len_offset
    #     brick_offset[2, 3] = 1 * brick_height_m
        
    #     right_mtx = topleft_offset @ brick_offset @ brick_center_offset
    #     x = right_mtx[0, 3]
    #     y = right_mtx[1, 3]

    #     A[2*i] = [x, -y, 1, 0]
    #     A[2*i + 1] = [y, x, 0, 1]
    #     b[2*i] = point["robot_x"]
    #     b[2*i + 1] = point["robot_y"]

    # x, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
    # print("condition number", np.linalg.cond(A), "residual", residuals, 'rank', rank)
    
    # # Reshape solution into transformation matrix
    # plate_x = x[2]
    # plate_y = x[3]
    # # plate_rot = np.array([
    # #     [x[0], x[1]],
    # #     [x[3], x[4]]
    # # ])
    # # compute yaw from 2d rotation matrix
    # cos = x[0] / np.sqrt(x[0]**2 + x[1]**2)
    # sin = x[1] / np.sqrt(x[0]**2 + x[1]**2)
    # yaw = np.arctan2(x[1], x[0])
    # print(np.array([[cos, sin, x[2]], [-sin, cos, x[3]]]))
    # print("yaw", yaw)
    # print(A @ np.array([cos, sin, x[2], x[3]]))

    # yaw_deg = yaw / np.pi * 180

    # return plate_x, plate_y, yaw, yaw_deg

def compute_r1r2(r1_x, r1_y, r1_z, r1_yaw, r2_x, r2_y, r2_z, r2_yaw):
    """ transform the frame from r1 to r2, i.e. r2 in r1 frame"""

    # r1_yaw = np.deg2rad(r1_yaw)
    # r2_yaw = np.deg2rad(r2_yaw)

    R1 = np.eye(4)
    R2 = np.eye(4)

    R1[0, 3] = r1_x
    R1[1, 3] = r1_y
    R1[2, 3] = r1_z
    R1[0, 0] = np.cos(r1_yaw)
    R1[0, 1] = -np.sin(r1_yaw)
    R1[1, 0] = np.sin(r1_yaw)
    R1[1, 1] = np.cos(r1_yaw)

    R2[0, 3] = r2_x
    R2[1, 3] = r2_y
    R2[2, 3] = r2_z
    R2[0, 0] = np.cos(r2_yaw)
    R2[0, 1] = -np.sin(r2_yaw)
    R2[1, 0] = np.sin(r2_yaw)
    R2[1, 1] = np.cos(r2_yaw)

    z180 = np.eye(4)
    z180[0, 0] = -1
    z180[1, 1] = -1
    
    world_R2 = np.eye(4)
    world_R2[2, 3] = 0.33
    #print(r1_yaw, r2_yaw)
    R = R1 @ z180 @ np.linalg.inv(R2) @ world_R2
    return R

if __name__ == "__main__":
    r1_transform = solve_calibration(r1_points)
    # r1_transform = (0.409, r1_transform[1], r1_transform[2], r1_transform[3])
    #print("r1 x", r1_transform[0], "r1 y", r1_transform[1], "r1 yaw", r1_transform[2], "deg", r1_transform[3])
    r2_transform = solve_calibration(r2_points)
    #print("r2 x", r2_transform[0], "r2 y", r2_transform[1], "r2 yaw", r2_transform[2], "deg", r2_transform[3])
    
    r1_z = 0.3105 - 0.111 - 0.0096
    r2_z = 0.3105 - 0.111 - 0.0096

    print("plate calibration")
    print(json.dumps({"x": r1_transform[0], 
           "y": r1_transform[1], 
           "z": r1_z,
           "roll": 0,
           "pitch": 0,
           "yaw": r1_transform[2], "deg": r1_transform[3],
           "width": 48,
           "height": 48
    }, indent=4))


    print(r1_transform[3], r2_transform[3])

    #R = compute_r1r2(r1_x, r1_y, r1_z, r1_yaw, r2_x, r2_y, r2_z, r2_yaw)
    R = compute_r1r2(r1_transform[0], r1_transform[1], r1_z, r1_transform[2], r2_transform[0], r2_transform[1], r2_z, r2_transform[2])
    #R = compute_r1r2(r2_transform[0], r2_transform[1], r2_z, r2_transform[2], r1_transform[0], r1_transform[1], r1_z, r1_transform[2])
    # print without scientific notation
    np.set_printoptions(suppress=True)
    print("r2_base")
    # Print the matrix in the desired format
    for row in R:
        print(' '.join(f'{elem: .8f}' for elem in row))
    r =  Rotation.from_matrix(R[:3, :3])
    yaw = r.as_euler("zyx",degrees=False)[0]

    # Extract xyz values
    x = R[0, 3]
    y = R[1, 3]
    z = 0.9246 + 0.33 - R[2, 3]

    # Print the XML line
    xml_line = (
        f'    <xacro:Yaskawa_gp4 arm_id="$(arg arm_id_2)" '
        f'connected_to="base" xyz="{x:.8f} {y:.8f} {z:.8f}" '
        f'rpy="0 0 {yaw:.8f}" />'
    )
    print("xacro line:")
    print(xml_line)