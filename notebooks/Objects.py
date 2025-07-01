import cv2
import numpy as np
import matplotlib.pyplot as plt

mug_img = https://tr.rbxcdn.com/180DAY-e5c2ef8826d208f479a84bdff7d03ad4/150/150/Image/Webp/noFilter -O sample_objects.jpg
mug_lower = 0,165,30
mug_upper = 179,255,150

airplane_img = https://tr.rbxcdn.com/180DAY-27571e17fbd7d3522b6fe85d915790fd/150/150/Image/Webp/noFilter -O sample_objects.jpg
airplane_lower = 0,0,20
airplane_upper = 90,255,150

bleach_cleanser_img = https://tr.rbxcdn.com/180DAY-33001a6a274e5bdc18bff36dcd1dd7f5/150/150/Image/Webp/noFilter -O sample_objects.jpg
bleach_cleanser_lower = 0,0,95
bleach_cleanser_upper = 170,255,196

soccer_ball_img = https://tr.rbxcdn.com/180DAY-549ae63715bd00895fcf531371fc8bf6/150/150/Image/Webp/noFilter -O sample_objects.jpg
soccer_ball_lower = 0,0,10
soccer_ball_upper = 180,120,183


def find_contours(img,lower,upper):
    wget img
    image_path = 'sample_objects.jpg'  # ensure this file is uploaded in your notebook environment
    img = cv2.imread(image_path)
    # Check if image loaded correctly
    if img is None:
        print("Error: Could not load image. Make sure 'sample_objects.jpg' is uploaded.")
    else:
        print("Image loaded successfully! Shape:", img.shape)

    # Define HSV range for the target color (example: green color range)
    lower_color = np.array([lower])   # lower bound (H, S, V)
    upper_color = np.array([upper]) # upper bound (H, S, V)
    
    # Create a binary mask where color within range -> white (255), outside range -> black (0)
    mask = cv2.inRange(hsv_img, lower_color, upper_color)
    print("Mask created. Mask shape:", mask.shape, "| Data type:", mask.dtype)
    print("Unique values in mask:", np.unique(mask))

    # Display the mask image
    plt.imshow(mask, cmap='gray')
    plt.title("Color Mask (binary image)")
    plt.axis('off')
    plt.show()
    # Find contours in the mask
    # cv2.findContours returns a tuple: (list_of_contours, hierarchy)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    print(f"Found {len(contours)} contours in the mask.")
    # Optional: print the area of each contour
    for idx, contour in enumerate(contours):
    area = cv2.contourArea(contour)
    print(f"Contour {idx} has area {area}")
    
    # Define a minimum contour area to filter noise
    min_area = 500  # you can adjust this value based on printed areas
    # Filter contours by area
    filtered_contours = [c for c in contours if cv2.contourArea(c) >= min_area]
    print(f"{len(filtered_contours)} contours remain after filtering by area >= {min_area}.")
    return filtered_contours
