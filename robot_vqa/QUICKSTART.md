# Quick Start Guide - VQA System

This guide will help you test the Visual Question Answering system with static images.

## Prerequisites

Make sure all Python dependencies are installed:
```bash
pip3 install transformers torch torchvision Pillow opencv-python --user
```

## Build the workspace

```bash
cd ~/vision
colcon build
source install/setup.bash
```

## Test with Static Images

### Terminal 1: Start the VQA Node

```bash
cd ~/vision
source install/setup.bash
ros2 run robot_vqa vqa_node
```

**Note**: The first time you run this, it will download the BLIP-2 model (~5GB). This will take a few minutes. Subsequent runs will load the cached model.

### Terminal 2: Publish Test Images

```bash
cd ~/vision
source install/setup.bash
python3 robot_vqa/test_image_publisher.py robot_vqa/test_images/kitchen.png
```

### Terminal 3: Ask Questions

```bash
cd ~/vision
source install/setup.bash

# Test different questions
ros2 run robot_vqa vqa_client "What do you see on the table?"
ros2 run robot_vqa vqa_client "How many objects are on the table?"
ros2 run robot_vqa vqa_client "What color is the mug?"
ros2 run robot_vqa vqa_client "Describe this scene"
ros2 run robot_vqa vqa_client ""  # Empty question = caption mode
```

## Available Test Images

1. **kitchen.png** - Kitchen scene with apple, blue mug, and banana on table
2. **office.png** - Office desk with laptop, plant, and books
3. **living_room.png** - Living room with couch, coffee table, and lamp

Change the image by restarting Terminal 2 with a different image file.

## Expected Timeline

1. First run: ~5 minutes (model download)
2. Subsequent runs: ~30 seconds (model loading on CPU)
3. Each question: ~5-10 seconds on CPU, ~1-2 seconds on GPU

## Troubleshooting

### "No images received yet"
- Make sure Terminal 2 (image publisher) is running
- Check topic: `ros2 topic list | grep image`
- Echo to verify: `ros2 topic echo /camera/image_raw --once`

### Model loading errors
- Check disk space (need ~10GB free)
- Verify internet connection for first download
- Try: `pip3 install --upgrade transformers torch`

### Out of memory
- Close other applications
- Use smaller model in vqa_params.yaml
- Ensure you have at least 8GB RAM

## Next Steps

Once testing with static images works:
1. Test with webcam or USB camera
2. Integrate with Husky simulation in Gazebo
3. Try different BLIP-2 model variants
4. Add GPU acceleration if available
