import numpy as np
from fuzzyoutput import fuzzyoutput  # 导入 fuzzyoutput 函数

def fuzzy_membership_fcn(x):
    """
    参数:
        x: 输入向量 (4x1)
    返回:
        beta: 激活权重向量
        wdot: 系统输出向量
    """
    # 初始化规则中心矩阵 (5x4)
    xm = np.zeros((5, 4))
    
    # 第一列 (x1)
    for j in range(1, 6):  # j = 1:5
        xm[j-1, 0] = -10.0 + 5.0*(j-1) + 0.1
        
    # 第二列 (x2)
    for j in range(6, 11):  # j = 6:10
        xm[j-6, 1] = -50.0 + 25.0*(j-5-1) + 0.1
    
    # 第三列 (u1_prev)
    for j in range(11, 16):  # j = 11:15
        xm[j-11, 2] = -40.0 + 20*(j-10-1) + 0.1
    
    # 第四列 (u2_prev)
    for j in range(16, 21):  # j = 16:20
        xm[j-16, 3] = -40.0 + 20*(j-15-1) + 0.1
    
    r = xm.shape[0]  # 规则数量
    n = xm.shape[1]  # 前件变量维度
    
    # 初始化隶属度函数矩阵
    ux = np.zeros((r, n))
    
    # 计算隶属度值 (与MATLAB代码完全一致的逻辑)
    for k in range(n):  # k = 0:n-1
        # 处理左边界情况
        if x[k] <= xm[0, k]:
            ux[0, k] = 1.0
        # 处理右边界情况
        if x[k] >= xm[r-1, k]:
            ux[r-1, k] = 1.0
        
        # 计算下降沿隶属度 (从左到右)
        for i in range(r-1):  # i = 1:r-1
            if x[k] >= xm[i, k] and x[k] <= xm[i+1, k]:
                diff = abs(xm[i, k] - xm[i+1, k])
                u_val = (x[k] - xm[i, k]) / diff
                ux[i, k] = 0.5 * np.sin(np.pi * u_val + 0.5 * np.pi) + 0.5
        
        # 计算上升沿隶属度 (从左到右)
        for i in range(1, r):  # i = 2:r
            if x[k] >= xm[i-1, k] and x[k] <= xm[i, k]:
                diff = abs(xm[i-1, k] - xm[i, k])
                u_val = (x[k] - xm[i, k]) / diff
                ux[i, k] = 0.5 * np.sin(np.pi * u_val + 0.5 * np.pi) + 0.5
    
    # 设置参数并调用模糊输出函数
    kx = 1  # 关注第一个输入
    jz = 1  # 关注第一个规则维度
    beta, wdot = fuzzyoutput(ux, x, xm, kx, jz)
    
    return beta, wdot