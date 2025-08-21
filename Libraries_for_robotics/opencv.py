import cv2

image_path = "/home/luqman/Libraries_for_robotics/humanoid_robot.jpeg"

img = cv2.imread(image_path)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

cv2.imshow("Original", img)
cv2.imshow("Grayscale", gray)


cv2.waitKey(0)
cv2.destroyAllWindows()

