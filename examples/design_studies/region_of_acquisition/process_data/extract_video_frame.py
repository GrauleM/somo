
# Program To Read video 
# and Extract Frames 
import cv2 
  
# Function to extract frames 
def FrameCapture(video_path, save_path=None, frame='last'): 
    # Path to video file 
    vidObj = cv2.VideoCapture(video_path) 
  
    count = 0 # Used as counter variable 
    success = 1 # checks whether frames were extracted 

  
    while success: 
        # vidObj object calls read function to extract frames 
        success, image = vidObj.read() 
  
        # Saves the frames with frame-count
        if success:
            last_image = image
            if frame == 'all' and save_path is not None:
                cv2.imwrite(save_path+"_frame%d.jpg" % count, image)
    
        count += 1
    
    if frame=='last' and save_path is not None:
        print(save_path)
        cv2.imwrite(save_path+"_last.jpg", last_image)
    
    return last_image


def crop(image, crop, save_path=None):

    h, w, channels = image.shape

    print(image.shape)
    image_cropped = image[crop[0]:-crop[1], crop[2]:-crop[3], :]

    print(image_cropped.shape)

    if save_path is not None:
        print(save_path)
        cv2.imwrite(save_path+"_last.jpg", image_cropped)