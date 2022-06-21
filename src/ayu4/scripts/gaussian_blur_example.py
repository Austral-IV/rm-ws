import cv2


if __name__ == '__main__':

  global_map = cv2.imread( 'global_map.png', cv2.IMREAD_GRAYSCALE )
  global_map = cv2.resize( global_map, (400, 400) ) # solo para visualizacion
  cv2.imshow( "Original image", global_map )
  cv2.waitKey(0)

  blurred = cv2.GaussianBlur( global_map, (25, 25), 0 )
  cv2.imshow( "Original image + Gaussian blur", blurred )
  cv2.waitKey(0)

 




