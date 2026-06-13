# AT-S_fuzzy_control_for_HRI_system

https://github.com/user-attachments/assets/180bafd0-7812-4f9b-a3f8-51e0bd93270f

<img width="505" height="449" alt="屏幕截图 2026-06-13 210141" src="https://github.com/user-attachments/assets/e60e4109-c2d1-4201-902c-fe908b597bd1" />

https://github.com/user-attachments/assets/b6b6c6be-d588-4486-b501-26d7a278a1f0

<img width="1865" height="262" alt="experimental_process" src="https://github.com/user-attachments/assets/f20ea02a-b7e4-4443-8dbc-ff42cc24ff5f" />

<img width="1931" height="817" alt="experimental_process4" src="https://github.com/user-attachments/assets/ea5c31ae-9162-4036-bd74-6df1aae729a0" />



# Kinova Gen3 上的 AT-S 阻抗控制方法

本文件夹提供了一个轻量化、适合上传 GitHub 的运行入口，用于在 Kinova Gen3 7 自由度机械臂上运行所提出的自适应 Takagi-Sugeno（AT-S）补偿阻抗控制器。

控制器主程序位于：

```text
../Response1/comparison_proposed_method_ats.py
```

该控制器主要包括：

* 等效末端阻抗控制；
* AT-S 模糊自适应补偿；
* 基于加权外部关节力矩的力感知衰减机制；
* 通过 Kinova Kortex Python API 实现底层力矩控制。

## 1. 硬件与软件要求

目标平台：

* Kinova Gen3 7 自由度机械臂；
* 连接到机器人网络的 Ubuntu/Linux 主机；
* Python 3.8 或更高版本；
* 已安装并可导入 `kortex_api` 的 Kinova Kortex Python API；
* Python 路径中可访问 Kinova API 示例辅助文件 `utilities.py`。

安装依赖：

```bash
pip install -r requirements.txt
```

Kinova Kortex Python API 通常需要从 Kinova 官方 Kortex API 包中安装，而不是从 PyPI 安装。安装后可用以下命令检查：

```bash
python3 -c "import kortex_api; print('kortex_api OK')"
```

## 2. 项目目录结构

该启动器默认项目结构如下：

```text
ATS_impedance_control/
+-- Response1/
|   +-- comparison_proposed_method_ats.py
+-- ats_imcontrol_sin_cos_jo.py
+-- Kinematic_fcn.py
+-- DiscreteIntegrator.py
+-- ts_fuzzy_output.py
+-- proposed_ats_kinova_gen3_github/
    +-- README.md
    +-- requirements.txt
    +-- setup_env.sh
    +-- run_kinova_gen3.sh
    +-- scripts/
        +-- check_environment.py
        +-- run_proposed_ats.py
```

如果 `utilities.py` 不在项目根目录，请从 Kinova Kortex Python 示例文件夹中复制，或将其所在路径加入 `PYTHONPATH`。

## 3. 配置机器人连接

复制环境变量示例文件：

```bash
cp .env.example .env
```

然后在 `.env` 中填写自己的机器人网络配置：

```bash
KINOVA_IP=<YOUR_KINOVA_ROBOT_IP>
KINOVA_USERNAME=<YOUR_KINOVA_USERNAME>
KINOVA_PASSWORD=<YOUR_KINOVA_PASSWORD>
```

不要将真实机器人 IP、用户名或密码发布到公开仓库中。

## 4. 环境检查

在当前文件夹中运行：

```bash
python3 scripts/check_environment.py
```

该脚本会检查：

* Python 版本；
* 所需 Python 依赖；
* `kortex_api` 是否可导入；
* 本地控制器文件是否存在；
* Kinova `utilities.py` 是否可访问。

## 5. 在 Kinova Gen3 上运行

创建 `.env` 后，运行：

```bash
./run_kinova_gen3.sh
```

等效的直接运行命令为：

```bash
python3 scripts/run_proposed_ats.py --ip <YOUR_KINOVA_ROBOT_IP> -u <YOUR_KINOVA_USERNAME> -p <YOUR_KINOVA_PASSWORD>
```

如果机械臂已经处于安全初始位姿，可以跳过初始关节运动：

```bash
python3 scripts/run_proposed_ats.py --ip <YOUR_KINOVA_ROBOT_IP> -u <YOUR_KINOVA_USERNAME> -p <YOUR_KINOVA_PASSWORD> --skip-initial-move
```

程序默认会先移动到初始位姿，除非使用 `--skip-initial-move`。之后程序会等待键盘输入：

```text
s = start torque control
q = quit
```

在输入 `s` 之前，不会发送任何力矩控制命令。

## 6. 数据输出

控制器会将工作目录固定在 `comparison_proposed_method_ats.py` 附近。通常会生成以下文件夹：

```text
Response1/impedance_improved_data/
Response1/proposed_method_plots/
```

记录数据包括关节跟踪误差、外部力矩、控制力矩、AT-S 补偿量以及相关统计信息。

## 7. 安全注意事项

每次真实机器人实验前，请务必确认：

* 急停按钮在可触及范围内；
* 工作空间已清空；
* 先进行低速、无接触测试；
* 按下 `s` 前确认机器人处于安全位姿；
* 不要在没有现场人员可立即急停的情况下，通过远程 SSH 运行底层力矩控制；
* 确认 Kinova Web App 或其他程序没有占用不兼容的伺服模式。

更详细的安全检查请见 `SAFETY.md`。

## 8. 常见问题

### `ModuleNotFoundError: No module named 'kortex_api'`

请安装 Kinova Kortex Python API，并确认已激活正确的 Python 环境。

### `ModuleNotFoundError: No module named 'utilities'`

请确保 Kinova 示例中的 `utilities.py` 位于项目根目录，或已加入 `PYTHONPATH`。

### `WRONG_SERVOING_MODE`

可能有其他程序修改了机械臂伺服模式。请停止其他控制脚本，在 Kinova Web App 中恢复机器人状态，然后重新运行程序。

### 程序启动后机器人不动

这是正常现象。程序会等待输入 `s` 后才开始力矩控制。

## 9. 引用占位符

如果在学术工作中使用本代码，请在论文发表后引用对应论文。

```bibtex
@article{yanwen_ats_impedance_kinova,
  title   = {Adaptive Takagi-Sugeno Impedance Control for Human-Robot Interaction},
  author  = {Yanwen et al.},
  journal = {To be updated},
  year    = {2026}
}
```
