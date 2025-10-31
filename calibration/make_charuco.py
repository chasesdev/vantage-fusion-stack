import cv2
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(7, 10, 0.035, 0.025, aruco_dict)
img = board.draw((3508, 4961))
cv2.imwrite('charuco_A3_300dpi.png', img)
print('Saved charuco_A3_300dpi.png')
