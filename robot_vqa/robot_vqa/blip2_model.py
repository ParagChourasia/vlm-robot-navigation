"""
BLIP-2 Model Wrapper for Visual Question Answering

This module provides a wrapper class for the BLIP-2 model from Hugging Face,
optimized for robotics applications with ROS 2.
"""

import torch
from PIL import Image
from transformers import Blip2Processor, Blip2ForConditionalGeneration
import logging


class BLIP2Model:
    """Wrapper class for BLIP-2 vision-language model."""
    
    def __init__(self, model_name="Salesforce/blip2-opt-2.7b", device=None):
        """
        Initialize the BLIP-2 model.
        
        Args:
            model_name: HuggingFace model identifier
            device: 'cuda', 'cpu', or None (auto-detect)
        """
        self.logger = logging.getLogger('BLIP2Model')
        self.model_name = model_name
        
        # Auto-detect device if not specified
        if device is None:
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
        else:
            self.device = device
            
        self.logger.info(f"Initializing BLIP-2 model: {model_name}")
        self.logger.info(f"Using device: {self.device}")
        
        try:
            # Load processor and model
            self.processor = Blip2Processor.from_pretrained(model_name)
            self.model = Blip2ForConditionalGeneration.from_pretrained(
                model_name,
                torch_dtype=torch.float16 if self.device == "cuda" else torch.float32
            )
            self.model.to(self.device)
            self.model.eval()  # Set to evaluation mode
            
            self.logger.info("BLIP-2 model loaded successfully")
            
        except Exception as e:
            self.logger.error(f"Failed to load BLIP-2 model: {e}")
            raise
    
    def answer_question(self, image, question):
        """
        Answer a question about an image.
        
        Args:
            image: PIL Image or numpy array
            question: Natural language question string
            
        Returns:
            tuple: (answer_text, confidence_score)
        """
        try:
            # Convert numpy array to PIL Image if needed
            if not isinstance(image, Image.Image):
                image = Image.fromarray(image)
            
            # Prepare inputs
            inputs = self.processor(
                images=image,
                text=question,
                return_tensors="pt"
            ).to(self.device)
            
            # Generate answer
            with torch.no_grad():
                outputs = self.model.generate(
                    **inputs,
                    max_length=50,
                    min_length=1,
                    num_beams=5,
                    early_stopping=True
                )
            
            # Decode the answer
            answer = self.processor.decode(outputs[0], skip_special_tokens=True)
            
            # For now, set a default confidence (BLIP-2 doesn't provide scores directly)
            # In production, you could extract logits or use beam scores
            confidence = 0.8
            
            self.logger.debug(f"Q: {question} | A: {answer}")
            
            return answer.strip(), confidence
            
        except Exception as e:
            self.logger.error(f"Error during inference: {e}")
            return "", 0.0
    
    def generate_caption(self, image):
        """
        Generate a caption for an image (no question).
        
        Args:
            image: PIL Image or numpy array
            
        Returns:
            tuple: (caption_text, confidence_score)
        """
        try:
            # Convert numpy array to PIL Image if needed
            if not isinstance(image, Image.Image):
                image = Image.fromarray(image)
            
            # Prepare inputs without text prompt
            inputs = self.processor(images=image, return_tensors="pt").to(self.device)
            
            # Generate caption
            with torch.no_grad():
                outputs = self.model.generate(
                    **inputs,
                    max_length=50,
                    min_length=10,
                    num_beams=5
                )
            
            # Decode the caption
            caption = self.processor.decode(outputs[0], skip_special_tokens=True)
            
            confidence = 0.8
            
            self.logger.debug(f"Caption: {caption}")
            
            return caption.strip(), confidence
            
        except Exception as e:
            self.logger.error(f"Error during caption generation: {e}")
            return "", 0.0
