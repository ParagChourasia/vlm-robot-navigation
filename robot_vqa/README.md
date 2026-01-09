# Robot VQA - Visual Question Answering for ROS 2

A beginner-friendly ROS 2 package that enables robots to answer natural language questions about their visual environment using the BLIP-2 Vision-Language Model.

## Features

- 🤖 **ROS 2 Integration**: Seamless integration with ROS 2 camera topics
- 🧠 **BLIP-2 Model**: Lightweight and powerful vision-language understanding
- 💬 **Natural Language Q&A**: Ask questions in plain English
- 🖼️ **Image Captioning**: Automatic scene description
- ⚡ **CPU/GPU Support**: Runs on both CPU and GPU
- 🔧 **Easy to Use**: Simple service interface and CLI client

## Installation

### Prerequisites

- ROS 2 (Humble or later)
- Python 3.8+
- PyTorch

### Build Instructions

1. Clone this repository into your ROS 2 workspace:
```bash
cd ~/vision
```

2. Install Python dependencies:
```bash
cd robot_vqa
pip3 install -r requirements.txt
```

3. Build the workspace:
```bash
cd ~/vision
colcon build
source install/setup.bash
```

## Usage

### Quick Start

1. **Launch the VQA node**:
```bash
ros2 launch robot_vqa vqa_system.launch.py
```

The first time you run this, it will download the BLIP-2 model (~5GB). This may take a few minutes.

2. **Ask questions using the CLI client**:
```bash
# In another terminal
source ~/vision/install/setup.bash
ros2 run robot_vqa vqa_client "What do you see?"
ros2 run robot_vqa vqa_client "How many objects are there?"
ros2 run robot_vqa vqa_client "What color is the wall?"
```

### Testing with Static Images

You can test the system by publishing images from a file or using a simulated camera.

### Integration with Husky Robot

To use with your Husky robot in Gazebo:

```bash
# Terminal 1: Launch Husky simulation with camera
ros2 launch husky_gazebo husky_world.launch.py

# Terminal 2: Launch VQA node with Husky camera topic
ros2 launch robot_vqa vqa_system.launch.py camera_topic:=/camera/image_raw

# Terminal 3: Ask questions
ros2 run robot_vqa vqa_client "Describe the scene"
```

## Configuration

Edit `config/vqa_params.yaml` to customize:
- Camera topic
- BLIP-2 model variant
- Device (CPU/GPU)
- Image buffer size

## Available Models

- `Salesforce/blip2-opt-2.7b` (default) - Lightweight, fast
- `Salesforce/blip2-opt-6.7b` - Better performance, slower
- `Salesforce/blip2-flan-t5-xl` - Alternative architecture

## Service Interface

The VQA node provides a service `/ask_question` with the following interface:

**Request:**
- `string question` - Natural language question
- `bool use_latest_image` - Use most recent image
- `builtin_interfaces/Time image_timestamp` - Specific timestamp (optional)

**Response:**
- `string answer` - Generated answer
- `float32 confidence` - Confidence score (0-1)
- `bool success` - Success flag
- `string error_message` - Error details if failed

## Examples

```bash
# Scene description
ros2 run robot_vqa vqa_client "What is in the image?"

# Object counting
ros2 run robot_vqa vqa_client "How many chairs are there?"

# Color recognition
ros2 run robot_vqa vqa_client "What color is the table?"

# Spatial reasoning
ros2 run robot_vqa vqa_client "What is on the left side?"

# Caption generation (empty question)
ros2 run robot_vqa vqa_client ""
```

## Troubleshooting

### Model loading is slow
- First run downloads ~5GB model
- Subsequent runs load from cache (~30 seconds on CPU)

### Out of memory errors
- Use CPU mode: `device:=cpu`
- Or use smaller model variant

### No camera images received
- Check camera topic: `ros2 topic list | grep image`
- Check topic name in config matches your camera

## Project Structure

```
robot_vqa/
├── robot_vqa/
│   ├── blip2_model.py      # BLIP-2 wrapper
│   ├── vqa_node.py          # Main ROS 2 node
│   └── vqa_client.py        # CLI client
├── launch/
│   └── vqa_system.launch.py
├── config/
│   └── vqa_params.yaml
└── requirements.txt
```

## License

MIT

## Author

Parag (parag067@gmail.com)
