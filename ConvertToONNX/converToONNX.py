import torch
import os
from ultralytics import YOLO

def convert_yolo_to_onnx():
    # Path to your PyTorch model
    model_path = "/home/miniman/Downloads/Models/new_arvp_front.pt"
    
    # Output path for ONNX model
    output_folder = "/home/miniman/kraken-nano/ConvertToONNX"
    onnx_filename = "new_arvp_front.onnx"
    onnx_path = os.path.join(output_folder, onnx_filename)
    
    try:
        # Load the YOLO model
        model = YOLO(model_path)
        
        # Export to ONNX format
        # The export method handles the conversion automatically
        model.export(
            format="onnx",
            imgsz=640,  # Input image size (640x640 for YOLOv8)
            optimize=True,
            half=False,  # Set to True for FP16 if your device supports it
            dynamic=False,  # Set to True for dynamic input shapes
            simplify=True,  # Simplify the ONNX model
            opset=11  # ONNX opset version
        )
        
        # The exported file will be saved in the same directory as the .pt file
        # Move it to your desired location
        source_onnx = model_path.replace('.pt', '.onnx')
        if os.path.exists(source_onnx):
            import shutil
            shutil.move(source_onnx, onnx_path)
            print(f"Model successfully converted and saved to: {onnx_path}")
        else:
            print(f"ONNX file created at: {source_onnx}")
            
    except Exception as e:
        print(f"Error converting model: {e}")
        
        # Fallback method if ultralytics doesn't work
        try:
            print("Trying alternative conversion method...")
            
            # Load model using torch.load
            model = torch.load(model_path, map_location='cpu')
            
            # If it's a state dict, you'll need the model architecture
            if isinstance(model, dict):
                print("Model appears to be a state dict. You'll need the model architecture.")
                return
            
            model.eval()
            
            # Create dummy input (YOLO typically uses 640x640)
            dummy_input = torch.randn(1, 3, 640, 640)
            
            # Export to ONNX
            torch.onnx.export(
                model,
                dummy_input,
                onnx_path,
                export_params=True,
                opset_version=11,
                do_constant_folding=True,
                input_names=['images'],
                output_names=['output'],
                dynamic_axes={
                    'images': {0: 'batch_size'},
                    'output': {0: 'batch_size'}
                }
            )
            print(f"Alternative conversion successful. Model saved to: {onnx_path}")
            
        except Exception as e2:
            print(f"Alternative conversion also failed: {e2}")

if __name__ == "__main__":
    convert_yolo_to_onnx()