import numpy as np

n = 1000

z = np.zeros((n, 6, 1))
u = np.zeros((n, 2, 1))
u_best = np.zeros((n, 2, 1))

P = np.zeros((n, 6, 6))
F = np.zeros((n, 2, 6))
S = np.eye(6)
Q = np.eye(6)
R = np.eye(2)
A = np.zeros((6, 6))
B = np.zeros((6, 2))
J = np.zeros(n)
J_best = np.zeros(n)


# P[n-1] = S
# J[n-1] = 1/2 * z[n-1].T @ P[n-1] @ z[n-1] 
# J_best[n-1]=1/2*(z[n-1].T @ P[n-1] @ z[n-1])

# J[n-2] = 1/2 * (A @ z[n-2] + B @ u[n-2]).T @ P[n-1] @ (A @ z[n-2] + B @ u[n-2]) + 1/2 * z[n-2].T @ Q @ z[n-2] + 1/2 * u[n-2].T @ R @ u[n-2]
# F[n-1] = np.linalg.inv(B.T @ P[n-1] @ B + R) @ B.T @ P[n-1] @ A
# u_best[n-1] = -F[n-1] @ z[n-1]
# P[n-2]=(A-B@F[n-1]).T @ P[n-1] @ (A-B@F[n-1])+F[n-1].T @ R @ F[n-1]+Q
# J_best[n-2]=1/2*(z[n-2].T @ P[n-2] @ z[n-2])

def solve_riccati(A, B, Q, R, S, n=1000, tol=1e-6):
    P[n-1] = S
    F[n-1] = np.linalg.inv(B.T @ P[n-1] @ B + R) @ B.T @ P[n-1] @ A
    for i in range(n-2, -1, -1):
        P[i]=(A-B@F[i+1]).T @ P[i+1] @ (A-B@F[i+1])+F[i+1].T @ R @ F[i+1]+Q
        F[i] = np.linalg.inv(B.T @ P[i] @ B + R) @ B.T @ P[i] @ A

        # 计算差值矩阵
        diff = P[i] - P[i+1]
        
        # 计算范数

        diff_norm = np.linalg.norm(P[i] - P[i+1], 'fro')
        ref_norm = np.linalg.norm(P[i+1], 'fro')
            
        if ref_norm < 1e-15:
            error = diff_norm  # 绝对误差
        else:
            error = diff_norm / ref_norm  # 相对误差
            
            # 4. 检查收敛
        if error < tol:
            print(f"收敛于迭代 {n-i-1}, 误差 = {error:.2e}")
            print(f"稳态P =\n{P[i]}")
            print(f"稳态F =\n{F[i]}")
            return P[i], F[i], True, i
        
        if i==0:
            print("错误: 未收敛")
            return None, None, False, i
    
solve_riccati(A, B, Q, R, S) 

