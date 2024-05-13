import numpy as np
import matplotlib.pyplot as plt

def plot_transformed_frame(T, ax, scale=1):
    axes = np.identity(3) * scale
    
    transformed_axes = T[:3, :3] @ axes
    
    origin = T[:3, 3]
    
    colors = ['r', 'g', 'b']
    labels = ['X-axis', 'Y-axis', 'Z-axis']
    for i in range(3):
        start_point = origin
        end_point = origin + transformed_axes[:, i]
        ax.quiver(start_point[0], start_point[1], start_point[2],
                  end_point[0] - start_point[0], end_point[1] - start_point[1], end_point[2] - start_point[2],
                  color=colors[i], label=labels[i], arrow_length_ratio=0.1)
    
    ax.legend()
    ax.set_xlim([-scale, scale])    
    ax.set_ylim([-scale, scale])
    ax.set_zlim([-scale, scale])
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_zlabel('Z Coordinate')

baseToL = np.array([[-0.51308207,  0.80024277, -0.31041633, -0.06022148],
 [ 0.68763173 , 0.16677023, -0.70664722,  0.00596226],
 [-0.51372113 ,-0.57602014, -0.63583913, -0.07802738],
 [ 0. ,         0.   ,       0.     ,     1.        ]]
)
baseToR = np.array([[ 0.16146282, -0.98401623 ,-0.07511196  ,0.00952503],
 [-0.68763173, -0.16677023 , 0.70664722 ,-0.00596226],
 [-0.70787878, -0.06244789, -0.70356798 ,-0.09810286],
 [ 0.     ,     0.    ,      0.     ,     1.        ]]
)
wristToL = np.array([[ 9.60983761e-01,  2.76604793e-01 , 1.88454451e-17 , 1.96149816e-34],
 [-6.02130048e-17 ,-7.09586270e-17 ,-1.00000000e+00 ,-1.04083409e-17],
 [-2.76604793e-01,  9.60983761e-01,  2.49967047e-17 , 2.60174223e-34],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00 , 1.00000000e+00]]

)
wristToR = np.array([[-9.60983761e-01 , 2.76604793e-01 ,-2.67282331e-18 ,-2.78196560e-35],
 [ 3.50300881e-17 , 7.14527411e-17 , 1.00000000e+00,  1.04083409e-17],
 [ 2.76604793e-01,  9.60983761e-01 , 3.11904162e-17 , 3.24640484e-34],
 [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]]
)
ati = np.linalg.inv(np.array([[ 0.88252565 , 0.46996386 , 0.025736039, 0],
 [-0.470243 ,   0.88302846 , 0.00291199, 0],
 [-0.02156212 ,-0.01477434 , 0.99984409, 0], 
 [0, 0, 0, 1]]
))
test = np.array([[0, 0, 1, 0], 
                 [-1, 0, 0, 0], 
                 [0, -1, 0, 0], 
                 [0, 0, 0, 1]])
test1 = np.linalg.inv(test)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plot_transformed_frame(np.eye(4), ax)
# plot_transformed_frame(baseToL, ax)
# plot_transformed_frame(baseToR, ax)
# plot_transformed_frame(np.linalg.inv(wristToL), ax)
# plot_transformed_frame(np.linalg.inv(wristToR), ax)
plot_transformed_frame(np.linalg.inv(ati), ax)
plt.show()
