import numpy as np

def fuzzyoutput(ux, x, xm, kx, jz):
    """
    模糊系统输出计算函数
    参数:
        ux: 隶属度矩阵 (规则数 × 输入维度)
        x:  输入向量 (输入维度 × 1)
        xm: 规则中心矩阵 (规则数 × 输入维度)
        kx: 指定输入位置 (1-based 索引)
        jz: 指定规则位置 (1-based 索引)
    返回:
        beta: 激活权重向量
        wdot: 系统输出向量
    """
    im = 1  # 索引起始值
    jm = 0  # 初始化
    bias = 1
    Basic = 0
    
    r = ux.shape[0]  # 单维度规则数量
    n = ux.shape[1]  # 前件变量维度
    dem = r ** n  # 隶属度函数的重数数量:总规则数量

    # 初始化输出向量
    beta = np.zeros((dem))
    wdot = np.zeros((dem))
    
    # 调用递归函数
    ij = 1  # 初始化ij
    [Basic, beta, wdot, jm, ij] = myfun(im, jm, ux, bias, Basic, beta, wdot, 
                                      x, xm, kx, jz, r, n, ij)
    
    # 归一化处理
    if Basic != 0:
        beta = beta / Basic
        wdot = wdot / Basic
    
    return beta, wdot

def myfun(im, jm, ux, bias, Basic, beta, wdot, x, xm, kx, jz, r, n, ij):
    """
    递归辅助函数用于计算模糊输出
    
    参数:
        im: 当前输入维度索引 (1-based)
        jm: 当前输出位置索引 (0-based)
        ux: 隶属度矩阵
        bias: 当前累积隶属度
        Basic: 当前基础值
        beta: 激活权重向量
        wdot: 系统输出向量
        x: 输入向量
        xm: 规则中心矩阵
        kx: 指定输入位置 (1-based)
        jz: 指定规则位置 (1-based)
        r: 规则数量
        n: 输入维度
        ij: 当前规则索引
    """
    # 索引转换：MATLAB是1-based，Python是0-based
    # 但此处保持MATLAB索引计数方式，仅在数组访问时转换
    
    # 当所有输入维度处理完毕时
    if im > ux.shape[1]:  # length(ux(1,:))为前件输入的维度
        Basic = Basic + bias
        jm = jm + 1
        beta[jm - 1] = bias  # beta(jm,1) = bias
        # 注意索引转换：MATLAB索引转为Python索引
        wdot[jm - 1] = beta[jm - 1] * x[kx - 1] * xm[ij - 1, 0]
        return Basic, beta, wdot, jm, ij
    else:
        for km in range(1, r + 1):  # 规则数量，1-based索引
            if im == jz:  # 对应a（km,im），jz是对应z_ji
                ij = km
            
            # 保存当前状态用于回溯
            bias_last = bias
            
            # MATLAB索引转为Python索引: ux(km,im) -> ux[km-1, im-1]
            bias = bias * ux[km - 1, im - 1]
            im = im + 1
            
            # 递归调用
            [Basic, beta, wdot, jm, ij] = myfun(im, jm, ux, bias, Basic, beta, wdot, 
                                             x, xm, kx, jz, r, n, ij)
            
            # 回溯：恢复状态
            im = im - 1
            bias = bias_last
        
        return Basic, beta, wdot, jm, ij