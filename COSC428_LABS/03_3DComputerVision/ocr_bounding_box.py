# ocr_bounding_box.py

import pytesseract as tess
import cv2

#image = cv2.imread("simple_text.png")
image = cv2.imread("Handwriting-test-dataset-for-OCR-operation.jpg")
#image = cv2.imread("Hard-Handwriting.png")
h = image.shape[0]  # Find the image height
boxes = tess.image_to_boxes(image)

# Show detected text
text = tess.image_to_string(image)
print(text)

# Draw the bounding boxes on the image.
for line in boxes.splitlines():
    data = line.split(' ')
    image = cv2.rectangle(image,
                          (int(data[1]), h - int(data[2])), (int(data[3]), h - int(data[4])),
                          (0, 0, 255), 1)

# Display the bounding boxes to the screen.
while cv2.waitKey(1) < 0:
    cv2.imshow("Bounding Boxes", image)
