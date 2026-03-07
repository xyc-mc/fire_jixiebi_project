import cv2
import numpy as np
from ultralytics import YOLO
import os

def crop_and_mask_object(image_path, output_path, model_path, dilation_size=5):
    
    if not os.path.exists(image_path):
        print(f"找不到文件: {image_path}")
        return

    model = YOLO(model_path) 

    original_img = cv2.imread(image_path)
    if original_img is None: return
    h_orig, w_orig = original_img.shape[:2]

    results = model(original_img, verbose=False)
    result = results[0]

    if result.masks is None:
        print("未检测到物体。无法裁剪。")
        return

    masks_data = result.masks.data.cpu().numpy()
    combined_mask = np.zeros((h_orig, w_orig), dtype=np.uint8)
    for mask_tensor in masks_data:
        mask_resized = cv2.resize(mask_tensor, (w_orig, h_orig))
        binary_mask = (mask_resized > 0.5).astype(np.uint8) * 255
        combined_mask = cv2.bitwise_or(combined_mask, binary_mask)

    if dilation_size > 0:
        k_size = dilation_size if dilation_size % 2 == 1 else dilation_size + 1
        kernel = np.ones((k_size, k_size), np.uint8)
        final_mask = cv2.dilate(combined_mask, kernel, iterations=1)
    else:
        final_mask = combined_mask

    points = cv2.findNonZero(final_mask)
    
    if points is None:
        print("掩膜处理后为空，无法裁剪。")
        return

    x, y, w, h = cv2.boundingRect(points)
    print(f"找到目标范围，进行裁剪: 左上({x},{y}) 宽高({w}x{h})")

    cropped_img = original_img[y:y+h, x:x+w]
    cropped_mask = final_mask[y:y+h, x:x+w]

    final_result = cv2.bitwise_and(cropped_img, cropped_img, mask=cropped_mask)

    cv2.imwrite(output_path, final_result)
    print(f"处理完成！输出图片尺寸: {w}x{h}，已保存至: {output_path}")

if __name__ == "__main__":
    input_img = "/home/neepu/fire_jixiebi_ws/src/point_rgb_address/config/master_aligned.jpg"
    output_img = "master_aligned.jpg" 
    
    crop_and_mask_object(input_img, output_img, '/home/neepu/fire_jixiebi_ws/src/point_rgb_address/config/best.pt', dilation_size=10)