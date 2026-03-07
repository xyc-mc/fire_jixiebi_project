from ultralytics import YOLO
from PIL import Image

# Load a pretrained YOLO11n model
model = YOLO("/home/ubuntu/fire_jixiebi_ws/best.pt")

# Define path to the image file
source = "/home/ubuntu/fire_jixiebi_ws/area_scan_data/results0.bmp"

# Run inference on the source
results = model(source)  # list of Results objects

for i, r in enumerate(results):
    # Plot results image
    im_bgr = r.plot()  # BGR-order numpy array
    im_rgb = Image.fromarray(im_bgr[..., ::-1])  # RGB-order PIL image

    # Show results to screen (in supported environments)
    r.show()

    # Save results to disk
    r.save(filename=f"results{i}.jpg")