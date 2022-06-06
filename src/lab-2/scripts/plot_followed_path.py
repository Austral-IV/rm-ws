
#!/usr/bin/env python3


from cProfile import label
from getpass import     getuser
from turtle import color
import matplotlib.pyplot as plt
import numpy as np



user = getuser()
rec_path = "sine"
hist = []
with open(f"/home/{user}/rm-ws/src/lab-2/scripts/followed_{rec_path}_path.txt", "r") as f:
    lines = f.readlines()
    for line in lines:
        pose = line.strip().split(",")
        hist.append([float(pose[0]), float(pose[1])])
t = [i for i in range(len(hist))]

designated_path =[]
designated_path_x = []
designated_path_y = []
pose_file = f"/home/{user}/rm-ws/src/lab-2/scripts/path_{rec_path}.txt"
with open(pose_file) as f:
    lines = f.readlines()
    pose_list = [pose.strip().split(",") for pose in lines]
    for pose in pose_list:
        designated_path_x.append(float(pose[0])-1)
        designated_path_y.append(float(pose[1])-1)
        designated_path.append([float(pose[0])-1, float(pose[1])-1])

hist_x = []
hist_y = []
for i in hist:
    hist_x.append(i[0])
    hist_y.append(i[1])
# plt.close("all")plt.close("all")
plt.plot(hist_x, hist_y, label=f"followed path")
plt.plot(designated_path_x, designated_path_y, label="designed path", color="red")
# plt.plot(t, hist_y, label=f"{rec_path}")
plt.legend()
plt.show()


array1 = np.array(designated_path_y)
array2 = np.array(hist_y)
array2 = np.resize(array2, (len(array1),))

difference_array = np.subtract(array1, array2)
squared_array = np.square(difference_array)
mse = squared_array.mean()


print(mse)

