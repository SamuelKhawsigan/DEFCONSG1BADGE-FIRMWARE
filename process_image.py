from PIL import Image
import sys
import os

def process_image(input_path, output_path):
    try:
        # Open the image
        img = Image.open(input_path)
        
        # Convert to RGB (in case it's PNG with alpha or grayscale)
        if img.mode != 'RGB':
            img = img.convert('RGB')
        
        # Resize to 320x240
        # Image.Resampling.LANCZOS provides high quality downsampling
        img = img.resize((320, 240), Image.Resampling.LANCZOS)
        
        # Save as Baseline JPEG
        # progressive=False ensures it is a baseline JPEG
        img.save(output_path, "JPEG", quality=90, progressive=False)
        
        print(f"Successfully processed image: {output_path}")
        print(f"Resolution: {img.size[0]}x{img.size[1]}")
        print(f"Format: Baseline JPEG")
        
    except Exception as e:
        print(f"Error processing image: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python process_image.py <input_image> [output_image]")
        sys.exit(1)
        
    input_file = sys.argv[1]
    
    # Default output to spiffs folder if no output path provided
    if len(sys.argv) > 2:
        output_file = sys.argv[2]
    else:
        filename = os.path.splitext(os.path.basename(input_file))[0]
        output_file = f"spiffs/{filename}_ready.jpg"
        
    # Ensure spiffs directory exists
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    
    process_image(input_file, output_file)
