# VLM Robot Navigation

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

Vision-Language Model powered mobile robot navigation using ROS 2. Enable robots to understand their environment through vision and natural language, then make intelligent navigation decisions.

## 🎯 Project Vision

This project combines **Visual Question Answering (VQA)** with **mobile robot navigation** to create an intelligent system that can:
- 🔍 Understand its environment through vision
- 💬 Interact with humans using natural language
- 🚀 Make navigation decisions based on visual understanding
- 🤖 Autonomously navigate to described locations or objects

## ✨ Current Features

### Visual Question Answering System
- **BLIP-2 Integration**: Lightweight vision-language model for scene understanding
- **ROS 2 Native**: Seamless integration with ROS 2 ecosystem
- **Real-time Processing**: Answer questions about live camera feeds
- **Natural Language Interface**: Ask questions in plain English
- **Multi-modal Understanding**: Combines vision and language for better comprehension

## 📦 Packages

### [`robot_vqa`](robot_vqa/)
Core VQA system with BLIP-2 model integration
- VQA ROS 2 node for processing camera images
- CLI client for interactive questioning
- Launch files and configuration

### [`robot_vqa_interfaces`](robot_vqa_interfaces/)
Custom ROS 2 service definitions for VQA
- `AskQuestion.srv` - Service interface for visual question answering

## 🚀 Quick Start

### Prerequisites
- ROS 2 Humble or later
- Python 3.8+
- CUDA-capable GPU (optional, but recommended)

### Installation

```bash
# Clone the repository
git clone https://github.com/ParagChourasia/vlm-robot-navigation.git
cd vlm-robot-navigation

# Install Python dependencies
pip3 install -r robot_vqa/requirements.txt

# Build the workspace
colcon build
source install/setup.bash
```

### Run VQA System

```bash
# Terminal 1: Launch VQA node
ros2 launch robot_vqa vqa_system.launch.py

# Terminal 2: Ask questions
ros2 run robot_vqa vqa_client "What do you see?"
ros2 run robot_vqa vqa_client "How many objects are there?"
ros2 run robot_vqa vqa_client "What is on the left side?"
```

See the [QUICKSTART Guide](robot_vqa/QUICKSTART.md) for detailed instructions.

## 🗺️ Roadmap

### ✅ Phase 1: Visual Question Answering (Complete)
- [x] BLIP-2 model integration
- [x] ROS 2 service interface
- [x] Camera topic subscription
- [x] CLI client for interaction
- [x] Static image testing

### 🔄 Phase 2: Mobile Robot Integration (In Progress)
- [ ] Husky robot simulation integration
- [ ] Real-time VQA during navigation
- [ ] Object detection and localization
- [ ] Spatial reasoning capabilities

### 📋 Phase 3: Vision-Guided Navigation (Planned)
- [ ] "Navigate to the red chair" - Object-based navigation
- [ ] "Go to the room with the plant" - Description-based navigation
- [ ] Obstacle avoidance with visual understanding
- [ ] Dynamic replanning based on visual feedback

### 🎯 Phase 4: Advanced Features (Future)
- [ ] Multi-robot coordination using shared visual understanding
- [ ] Long-term memory of environment
- [ ] Human-robot collaboration through natural language
- [ ] Integration with larger VLMs (GPT-4V, LLaVA, etc.)

## 🏗️ System Architecture

```
┌─────────────────┐
│  Camera Topic   │
│ /camera/image   │
└────────┬────────┘
         │
         ▼
┌─────────────────────────────┐
│     VQA Node                │
│  ┌──────────────────────┐   │
│  │  BLIP-2 Model        │   │
│  │  - Image Encoder     │   │
│  │  - Language Model    │   │
│  └──────────────────────┘   │
└────────┬────────────────────┘
         │
         ▼
┌─────────────────────────────┐
│  /ask_question Service      │
│  Request: question          │
│  Response: answer           │
└─────────────────────────────┘
         │
         ▼
┌─────────────────────────────┐
│  Navigation Controller      │
│  (Future Integration)       │
└─────────────────────────────┘
```

## 📚 Documentation

- [README](robot_vqa/README.md) - Detailed VQA package documentation
- [QUICKSTART](robot_vqa/QUICKSTART.md) - Step-by-step testing guide
- [API Documentation](robot_vqa_interfaces/) - Service definitions

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 👤 Author

**Parag Chourasia**
- Email: parag067@gmail.com
- GitHub: [@ParagChourasia](https://github.com/ParagChourasia)

## 🙏 Acknowledgments

- [BLIP-2](https://github.com/salesforce/LAVIS) by Salesforce Research
- [Hugging Face Transformers](https://huggingface.co/docs/transformers/)
- ROS 2 Community

## 📊 Project Status

**Current Status**: Early Development - VQA System Functional

The Visual Question Answering system is working and tested with static images. Mobile robot navigation integration is the next milestone.

---

⭐ **Star this repo** if you find it useful!
