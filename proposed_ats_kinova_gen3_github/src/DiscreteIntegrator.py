import numpy as np

class DiscreteIntegrator:
    """
    离散时间向量积分器（固定时间步长）
    采用后向欧拉法: y[n] = y[n-1] + K * u[n] * dt
    """
    def __init__(self, dim=1, dt=None, initial_condition=0.0, gain=1.0):
        """
        :param dim: 向量维度
        :param dt: 固定时间步长（秒）
        :param initial_condition: 初始积分值
        :param gain: 积分增益
        """
        self.dim = dim
        self.Hz = 1000 if dt is None else 1/dt  #频率1000Hz
        self.dt = 1/self.Hz if dt is None else dt
        self.gain = gain
        
        # 初始化状态变量
        if isinstance(initial_condition, (int, float)):
            # 标量扩展为向量
            self.output = np.full(dim, initial_condition, dtype=float)
            self.last_output = self.output.copy()
        else:
            # 确保维度匹配
            if len(initial_condition) != dim:
                raise ValueError(f"初始条件维度({len(initial_condition)})与指定维度({dim})不匹配")
            self.output = np.array(initial_condition, dtype=float)
            self.last_output = self.output.copy()
    
    def step(self, input_vector):
        """
        执行单步积分计算
        :param input_vector: 当前输入向量或标量
        """
        # 处理标量输入
        if np.isscalar(input_vector):
            if self.dim != 1:
                raise ValueError(f"标量输入({input_vector})与积分器维度({self.dim})不匹配")
            input_vector = np.array([input_vector])
        
        # 确保输入向量与积分器维度匹配
        if len(input_vector) != self.dim:
            raise ValueError(f"输入向量维度({len(input_vector)})与积分器维度({self.dim})不匹配")
        
        # 使用后向欧拉法计算积分
        self.output = self.last_output + self.gain * np.array(input_vector) * self.dt
        self.last_output = self.output.copy()
        return self.output.copy()
    
    def reset(self, initial_condition=0.0):
        """重置积分器状态"""
        if np.isscalar(initial_condition):
            self.output.fill(initial_condition)
            self.last_output.fill(initial_condition)
        else:
            if len(initial_condition) != self.dim:
                raise ValueError(f"初始条件维度({len(initial_condition)})与积分器维度({self.dim})不匹配")
            self.output = np.array(initial_condition)
            self.last_output = self.output.copy()