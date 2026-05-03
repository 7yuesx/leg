import scipy.io
import numpy as np
from scipy.signal import cont2discrete
# 加载文件
data = scipy.io.loadmat('sys_matrices.mat')

# 提取矩阵 (注意：loadmat 返回的是字典)


n = 100000

z = np.zeros((n, 10, 1))
u = np.zeros((n, 6, 1))
u_best = np.zeros((n, 6, 1))

P = np.zeros((n, 10, 10))
F = np.zeros((n, 6, 10))

q_diag = np.array([
    800,    # x: 100
    800,     # w: 10
    10000,   # theta: 5000 (身体俯仰是最重要的！)
    50,    # theta_l: 500
    50,    # theta_r: 500
    15,     # x_dot
    1,      # w_dot
    15,    # theta_dot
    1,     # theta_dot_l
    1      # theta_dot_r
])

S = np.diag(q_diag)
# np.array(
#             [[1,0,0,0,0,0,0,0,0,0],
#              [0,1,0,0,0,0,0,0,0,0],
#              [0,0,1,0,0,0,0,0,0,0],
#              [0,0,0,1,0,0,0,0,0,0],
#              [0,0,0,0,1,0,0,0,0,0],
#              [0,0,0,0,0,1,0,0,0,0],
#              [0,0,0,0,0,0,1,0,0,0],
#              [0,0,0,0,0,0,0,1,0,0],
#              [0,0,0,0,0,0,0,0,1,0],
#              [0,0,0,0,0,0,0,0,0,1]])



Q = S
# np.array(
#             [[1,0,0,0,0,0,0,0,0,0],
#              [0,1,0,0,0,0,0,0,0,0],
#              [0,0,1,0,0,0,0,0,0,0],
#              [0,0,0,1,0,0,0,0,0,0],
#              [0,0,0,0,1,0,0,0,0,0],
#              [0,0,0,0,0,1,0,0,0,0],
#              [0,0,0,0,0,0,1,0,0,0],
#              [0,0,0,0,0,0,0,1,0,0],
#              [0,0,0,0,0,0,0,0,1,0],
#              [0,0,0,0,0,0,0,0,0,1]])
R = 50*np.array(
            [[1,0,0,0,0,0],
             [0,1,0,0,0,0],
             [0,0,1,0,0,0],
             [0,0,0,1,0,0],
             [0,0,0,0,0.5,0],
             [0,0,0,0,0,0.5]])

J = np.zeros(n)
J_best = np.zeros(n)

A = data['A']
B = data['B']
C = data['C']
D = data['D']
system_discrete = cont2discrete((A, B, C, D), 0.002, method='zoh')
Ad, Bd, Cd, Dd, dt = system_discrete
print("A matrix shape:", Ad.shape)
print(Ad)

# P[n-1] = S
# J[n-1] = 1/2 * z[n-1].T @ P[n-1] @ z[n-1] 
# J_best[n-1]=1/2*(z[n-1].T @ P[n-1] @ z[n-1])

# J[n-2] = 1/2 * (A @ z[n-2] + B @ u[n-2]).T @ P[n-1] @ (A @ z[n-2] + B @ u[n-2]) + 1/2 * z[n-2].T @ Q @ z[n-2] + 1/2 * u[n-2].T @ R @ u[n-2]
# F[n-1] = np.linalg.inv(B.T @ P[n-1] @ B + R) @ B.T @ P[n-1] @ A
# u_best[n-1] = -F[n-1] @ z[n-1]
# P[n-2]=(A-B@F[n-1]).T @ P[n-1] @ (A-B@F[n-1])+F[n-1].T @ R @ F[n-1]+Q
# J_best[n-2]=1/2*(z[n-2].T @ P[n-2] @ z[n-2])


# def solve_riccati(A, B, Q, R, S, n=1000, tol=1e-6):
#     P[n-1] = S
#     F[n-1] = np.linalg.inv(B.T @ P[n-1] @ B + R) @ B.T @ P[n-1] @ A
#     for i in range(n-2, -1, -1):
#         P[i]=(A-B@F[i+1]).T @ P[i+1] @ (A-B@F[i+1])+F[i+1].T @ R @ F[i+1]+Q
#         F[i] = np.linalg.inv(B.T @ P[i] @ B + R) @ B.T @ P[i] @ A

#         # 计算差值矩阵
    
#         # 计算范数

#         diff_norm = np.linalg.norm(P[i] - P[i+1], 'fro')
#         ref_norm = np.linalg.norm(P[i+1], 'fro')
            
#         if ref_norm < 1e-15:
#             error = diff_norm  # 绝对误差
#         else:
#             error = diff_norm / ref_norm  # 相对误差
            
#             # 4. 检查收敛
#         if error < tol:
#             print(f"收敛于迭代 {n-i-1}, 误差 = {error:.2e}")
#             print(f"稳态P =\n{P[i]}")
#             print(f"稳态F =\n{F[i]}")
#             return P[i], F[i], True, i
        
#         if i==0:
#             print("错误: 未收敛")
#             return None, None, False, i
from scipy.linalg import solve_discrete_are




#[P_convergence,F_convergence,number]=solve_riccati(A, B, Q, R, S) 



P_steady = solve_discrete_are(Ad, Bd, Q, R)
#F_steady
F_convergence= np.linalg.inv(Bd.T @ P_steady @ Bd + R) @ (Bd.T @ P_steady @ Ad)
print(F_convergence)


# 库函数计算结果
# 在 Python 中执行
Ad_cl = Ad - Bd @ F_convergence
e = np.linalg.eigvals(Ad_cl)
print(f"闭环最大特征值: {np.max(np.abs(e))}")

