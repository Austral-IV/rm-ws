import numpy as np
import cv2


if __name__ == '__main__':

  global_map = cv2.imread( 'global_map.png', cv2.IMREAD_GRAYSCALE )
  local_map = cv2.imread( 'local_map.png', cv2.IMREAD_GRAYSCALE )

  corr = cv2.matchTemplate( global_map, local_map, cv2.TM_CCOEFF_NORMED )
  heatmapshow = np.zeros( (corr.shape[0], corr.shape[1]) )
  heatmapshow = cv2.normalize( corr, heatmapshow, alpha = 0, beta = 255, norm_type = cv2.NORM_MINMAX, dtype = cv2.CV_8U)
  heatmapshow = cv2.applyColorMap( heatmapshow, cv2.COLORMAP_JET )
  heatmapshow = cv2.resize( heatmapshow, (400, 400) ) # solo para visualizacion
  cv2.imshow( "Correlation heatmap", heatmapshow )
  cv2.waitKey(0)


