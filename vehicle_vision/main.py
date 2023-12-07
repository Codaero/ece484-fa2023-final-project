import torch
from torchvision import transforms
import numpy as np
import matplotlib.pyplot as plt
import cv2

from PIL import Image


def main():
    # Load model
    device = torch.device("cuda")

    model = torch.hub.load('hustvl/yolop', 'yolop', pretrained=True)
    model.to(device)
    model.eval()  # Set the model to evaluation mode

    # Generate or load an image tensor
    # For example, a random tensor can be used for demonstration purposes
    img = cv2.imread("/home/ece484/catkin_ws/src/ece484_final_project/lane_detection/adb4871d-4d063244.jpg")

    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Define the standard normalization transformation
    transform = transforms.Compose([
        transforms.ToPILImage(),
        transforms.Resize((736, 1280)),  # Resize the image to the expected input size
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])

    # Apply the transformation to the image
    img_tensor = transform(img).to(device)

    # Add a batch dimension
    img_tensor = img_tensor.unsqueeze(0)

    # Inference
    with torch.no_grad():
        det_out, da_seg_out, ll_seg_out = model(img_tensor)

    # Convert lane line segmentation output to binary mask
    ll_seg_mask = torch.argmax(ll_seg_out, 1)
    ll_seg_mask = ll_seg_mask.squeeze(0).cpu().numpy()

    # Assuming 'img' is normalized in the range [0, 1]
    # Convert it to a PIL image for visualization
    # If 'img' is not normalized, you should normalize it as per the model's requirements
    img_tensor = img_tensor.squeeze(0).permute(1, 2, 0)  # Change dimension from [C, H, W] to [H, W, C]
    img_tensor = (img_tensor * 255).byte().cpu().numpy()  # Denormalize and convert to numpy
    img_pil = Image.fromarray(img_tensor)

    # Create an overlay mask for the lanes
    lane_overlay = np.zeros_like(img_tensor, dtype=np.uint8)
    lane_overlay[ll_seg_mask == 1] = [255, 0, 0]  # Assuming class '1' corresponds to lane lines

    # Overlay the binary mask onto the original image
    # Adjust the alpha parameter to control the transparency of the overlay
    alpha = 0.5
    img_with_lanes = Image.blend(img_pil, Image.fromarray(lane_overlay), alpha=alpha)

    # Display the image with lanes
    plt.imshow(Image.fromarray(lane_overlay))
    # plt.imshow(img_with_lanes)
    plt.axis('off')  # Hide the axes
    plt.show()


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()
