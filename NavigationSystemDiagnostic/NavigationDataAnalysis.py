from AStar.AStar_5 import *
from matplotlib import *
import glob, os
import matplotlib.pyplot as plot

if __name__ == "__main__":
    os.chdir("NavigationSystemTestResult/")
    for file in glob.glob("*.txt"):
        current_point = []
        dislocation_vector = []
        orientation = []
        path_point = []
        computing_time = []
        integer = re.compile(r'\d+')
        dislocation_angle_to_path = []
        dislocation_pulse = []
        step = []
        sample_data = []
        try:
            file_name = file
            f = open("" + file_name)
            p = re.compile(r"[-+]?\d*\.\d+|\d+")


            for line in f:
                string = re.search("Current_Point](.*),(.*)", line)
                if string:
                    current_point.append([int(string.group(1)), int(string.group(2))])
                string_2 = re.search("Dislocation_Vector](.*),(.*)", line)
                if string_2:
                    dislocation_vector.append([float(string_2.group(1)), float(string_2.group(2))])
                string_3 = re.search("Orientation_Degree](.*),(.*)", line)
                if string_3:
                    orientation.append([float(string_3.group(1)), float(string_3.group(2))])
                string_4 = re.search("Path_point](.*),(.*),", line)
                if string_4:
                    path_point.append([int(string_4.group(1)), int(string_4.group(2))])
                string_5 = re.search("sampling_time](.*)", line)
                if string_5:
                    computing_time.append([float(string_5.group(1))])
                string_6 = re.search("Dislocation_Angle_To_Path](.*)", line)
                if string_6:
                    dislocation_angle_to_path.append([float(string_6.group(1))])

                string_7 = re.search("dislocation_pulse] (.*),(.*),(.*),(.*)", line)
                if string_7:
                    dislocation_pulse.append([float(string_7.group(1)),float(string_7.group(2)),float(string_7.group(3)),float(string_7.group(4))])

            # for dislocation_angle in dislocation_pulse:
            #     print(dislocation_angle)

            # for index, vector in enumerate(orientation):
            #     print(vector, dislocation_angle_to_path[index])

            #
            for i in range(len(dislocation_pulse)):
                step.append(i)
                sample_data.append(dislocation_pulse[i][0])
            # figure.Figure()
            plot.figure(1)
            plot.plot(step, dislocation_pulse)
            plot.savefig("../NavigationSystemPulseData/" + file_name[:-4] + '.jpg')
            # plot.show()
            plot.close()
            print("Saved ", file_name)
            time.sleep(0.1)
        except Exception as exp:
            print(exp)