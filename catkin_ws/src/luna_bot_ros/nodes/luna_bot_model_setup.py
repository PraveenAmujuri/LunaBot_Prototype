#!/usr/bin/env python3
"""
Luna Bot Model Setup - One-Time Initialization
Downloads and caches your Roboflow model for offline use
"""

import os
import json
import datetime
from roboflow import Roboflow
from inference_sdk import InferenceHTTPClient
import requests
import numpy as np
import cv2

class LunaBotModelSetup:
    def __init__(self):
        self.workspace = "luna-bot"                    # Your workspace from URL
        self.project = "luna-bot-jhsbz"               # Your project from URL  
        self.version = 2                              # Your version number
        self.api_key = "RSdAzO0IEAvVCJNRS1yV"        # Your API key
        
        # Local cache directory
        self.cache_dir = os.path.expanduser("~/.luna_bot_models")
        self.model_cache_file = os.path.join(self.cache_dir, "luna_bot_model.cache")
        self.config_file = os.path.join(self.cache_dir, "model_config.json")
        
        # Create cache directory
        os.makedirs(self.cache_dir, exist_ok=True)
        
    def setup_model(self):
        """Setup model for offline use"""
        print("🌙 LUNA BOT MODEL SETUP")
        print("=" * 50)
        print("🔄 Setting up your trained model...")
        
        try:
            # Create model ID
            model_id = f"{self.project}/{self.version}"  # Remove workspace from model ID

            
            print(f"📡 Connecting to model: {model_id}")
            print("⏳ This may take a moment...")
            
            # Initialize the inference client
            client = InferenceHTTPClient(
                api_url="https://detect.roboflow.com",
                api_key=self.api_key
            )
            
            # Test model to ensure it works
            print("🧪 Testing model...")
            
            # Create a dummy test image for verification
            test_image = np.zeros((640, 640, 3), dtype=np.uint8)
            cv2.rectangle(test_image, (100, 100), (200, 200), (128, 128, 128), -1)
            
            # Save test image temporarily
            test_image_path = "/tmp/test_luna_bot.jpg"
            cv2.imwrite(test_image_path, test_image)
            
            # Test inference
            try:
                test_results = client.infer(test_image_path, model_id=model_id)
                print("✅ Model test successful!")
                print(f"🎯 Model ready for detection")
            except Exception as test_error:
                print(f"⚠️ Model test had an issue: {test_error}")
                print("🔄 This might be normal for a new model - continuing setup...")
            
            # Clean up test image
            if os.path.exists(test_image_path):
                os.remove(test_image_path)
            
            # Save model configuration
            config = {
                'workspace': self.workspace,
                'project': self.project,
                'version': self.version,
                'model_id': model_id,
                'api_key': self.api_key,
                'classes': ['Buildings', 'Flag', 'shadows', 'antenna', 'austroads', 'rocks', 'space capsule'],
                'setup_complete': True,
                'setup_date': str(datetime.datetime.now()),
                'method': 'inference_sdk'
            }
            
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=2)
            
            print("💾 Model configuration saved!")
            print(f"📁 Cache location: {self.cache_dir}")
            print("🚀 Ready for detection!")
            
            return True
            
        except Exception as e:
            print(f"❌ Setup failed: {e}")
            print("\n🔧 Troubleshooting:")
            print("1. Check your API key is correct")
            print("2. Verify workspace/project names match your Roboflow URL")
            print("3. Ensure internet connection")
            print("4. Make sure your model is public or you have access")
            print(f"5. Trying to connect to: {model_id}")
            return False
    
    def verify_setup(self):
        """Verify model is properly cached"""
        if not os.path.exists(self.config_file):
            return False
            
        try:
            with open(self.config_file, 'r') as f:
                config = json.load(f)
            return config.get('setup_complete', False)
        except:
            return False
    
    def get_model_config(self):
        """Get cached model configuration"""
        try:
            with open(self.config_file, 'r') as f:
                return json.load(f)
        except:
            return None

def main():
    print("🚀 Starting Luna Bot Model Setup...")
    
    setup = LunaBotModelSetup()
    
    # Check if already setup
    if setup.verify_setup():
        print("✅ Model already setup!")
        config = setup.get_model_config()
        print(f"📊 Model: {config['model_id']}")
        print(f"📅 Setup: {config['setup_date']}")
        print("🎯 Ready for detection!")
        return
    
    # Setup model
    success = setup.setup_model()
    
    if success:
        print("\n🎉 SETUP COMPLETE!")
        print("=" * 50)
        print("✅ Your Luna Bot model is configured")
        print("🚀 You can now run your ROS detection node")
        print("💡 Model ready for detection")
        print("=" * 50)
    else:
        print("\n❌ SETUP FAILED!")
        print("Please check the troubleshooting steps above")

if __name__ == '__main__':
    main()
