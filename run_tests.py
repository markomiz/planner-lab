import os
import pandas as pd
results = []


def replace_param(name, value):
    fin = open("config.txt", "rt")
    data = fin.read()
    lines = data.splitlines()
    new_data = []
    for line in lines:
        a = line
        if name in line:
            a = name + ": " + str(value)
        new_data.append(a)
        
    new_data = "\n".join(new_data)
    fin.close()
    fin = open("config.txt", "wt")
    fin.write(new_data)
    fin.close()

def store_output(dist, points, angles):
    fin = open("results.txt", "rt")
    data = fin.read()
    ls = data.split(",")
    comp_time = ls[0]
    length = ls[1]
    connections = ls[2]
    result_entry = {"angles": angles,
                    "points": points,
                    "dist": dist,
                    "comp_time": comp_time,
                    "length": length}
    results.append(result_entry)


for dist in [ 0.7, 0.8, 0.9, 1.0, 1.1]:
    replace_param("connect_distance", dist)
    for points in [250,500,1000,2000,4000]:
        replace_param("num_points", points)
        for angles in [1,2,3,4,5,6,7,8]:
            replace_param("num_angles", angles)
            os.system("ros2 run dubin mission_planner")
            store_output(dist, points, angles)
        
df = pd.DataFrame(results)
df.to_csv("Results")
print(df)