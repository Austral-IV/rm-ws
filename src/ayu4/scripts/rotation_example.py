import numpy as np
import cv2


if __name__ == '__main__':

  local_map = cv2.imread( 'local_map.png', cv2.IMREAD_GRAYSCALE )
  for angle in [0, 90, 180, 270]:
    rows, cols = local_map.shape
    M = cv2.getRotationMatrix2D( ((cols-1)/2.0, (rows-1)/2.0), angle, 1 )
    local_map_rot = cv2.warpAffine( local_map, M, (cols,rows) )
    local_map_rot = cv2.resize( local_map_rot, (400, 400) ) # solo para visualizacion
    cv2.imshow( 'Rotation (counter clockwise): %d [deg]' % (angle), local_map_rot )
    cv2.waitKey(0)
    cv2.destroyAllWindows()


