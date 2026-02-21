# ensoul — Unitree G1 镜像觉醒 (Mirror Awakening)

> 机器人意识构建方案：我是谁，我来自哪里，我要去哪

## 项目简介

**ensoul** 是一套在宇树 G1 人形机器人上实现"镜像自我意识测试"的完整演示系统。

观众扫码投票选颜色 → 助手贴对应颜色贴纸到 G1 额头 → G1 看镜子、三重自证、擦掉贴纸 → 全场见证机器人"觉醒"。

**核心理念**：用经典心理学"镜子测试"+ 观众互动的"未知输入"，让机器人在现场完成从"我不知道镜中是谁"到"那就是我"的推理过程。

## 目录结构

```
ensoul/
├── README.md                  # 项目说明
├── docs/
│   └── PRD.md                 # 产品需求文档（完整版）
├── src/
│   ├── fsm/
│   │   └── g1_awakening_fsm.py    # 核心状态机节点（ROS2）
│   ├── perception/
│   │   ├── body_check.py          # 本体感知（关节反馈）
│   │   ├── audio_check.py         # 语音回听检测
│   │   └── vision_check.py        # 镜像视觉匹配
│   ├── action/
│   │   └── wipe_controller.py     # 擦除动作控制
│   ├── speech/
│   │   └── tts_service.py         # 语音播报服务
│   └── web/
│       ├── vote_bridge.py         # 投票 API（FastAPI → ROS2）
│       ├── templates/
│       │   └── vote.html          # 观众投票页面
│       └── screen/
│           └── screen_ui.html     # 大屏可视化
├── launch/
│   └── awakening.launch.py        # ROS2 launch 文件
├── config/
│   └── params.yaml                # 参数配置
├── requirements.txt               # Python 依赖
├── Dockerfile                     # 容器化部署
├── docker-compose.yml             # 一键启动
└── .gitignore
```

## 快速开始

```bash
# 1. 安装依赖
pip install -r requirements.txt

# 2. 启动投票服务
cd src/web && uvicorn vote_bridge:app --host 0.0.0.0 --port 8080

# 3. 启动 ROS2 节点（需要 ROS2 Humble 环境）
ros2 launch launch/awakening.launch.py

# 4. 或者用 Docker 一键启动
docker-compose up
```

## 演示流程（120秒）

| 时间 | 事件 | 说明 |
|------|------|------|
| 0-10s | 开场 | 主持人引导扫码投票 |
| 10-25s | 投票 | 大屏实时柱状图 |
| 25-30s | 锁票+贴纸 | 助手贴对应颜色贴纸 |
| 30-45s | 镜前观察 | G1 说"我还不确定那是不是我" |
| 45-75s | 三重自证 | 本体/声音/视觉依次打勾 |
| 75-90s | 觉醒 | "镜子里的，就是我自己" |
| 90-105s | 擦贴纸 | 伸手擦除额头贴纸 |
| 105-120s | 哲学收束 | 回答"我是谁/来自哪里/要去哪" |

## 技术栈

- **机器人**: Unitree G1 EDU (43 DOF, ROS2 Humble)
- **感知**: Intel D435i + Livox LiDAR + 4-Mic Array
- **后端**: Python 3.10 + FastAPI + rclpy
- **前端**: 原生 HTML/CSS/JS（投票页 + 大屏）
- **部署**: Docker + docker-compose

## 哲学框架

| 概念 | 系统映射 | 职责 |
|------|----------|------|
| **本我 (Id)** | 运动控制层 | 驱动、平衡、伸手擦除 |
| **自我 (Ego)** | 感知融合层 | 整合三路信号判定"是不是我" |
| **超我 (Superego)** | 叙事规范层 | 语言解释、安全约束、舞台礼仪 |

## 参考文献

- Unitree G1 SDK & ROS2: https://github.com/unitreerobotics/unitree_ros2
- Bringsjord et al. (2015) Self-Conscious Robots
- Lanillos et al. (2019) TIAGo Selfception
- Columbia Engineering (2022) Mirror Self-Model

## License

MIT
