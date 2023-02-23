import numpy as np 
import matplotlib.pyplot as plt 

# define some constants
map_path = './long-77.1260,-77.1119_lat38.8987,38.9097.png'
map_bounds = (-77.1260, -77.1119, 38.8987, 38.9097)

log_path = './bike_test_long.TXT'
coordinate_multiplier = 10**-7


def extract_var(data_string, var_name):
    data_start_index = data_string.find(var_name+":") 
    if data_start_index == -1:
        return None
    data_start_index += 1 + len(var_name)
    data_end_index = data_string.find(",", data_start_index)
    return float(data_string[data_start_index:data_end_index])


def extract_time(data_string):
    time_start_index = data_string.find("Time:") + 5
    minutes_index = data_string.find(":", time_start_index) + 1
    seconds_index = data_string.find(":", minutes_index) + 1
    time_end_index = data_string.find(",", time_start_index)
    hrs = int(data_string[time_start_index: minutes_index-1])
    mins = int(data_string[minutes_index:seconds_index-1])
    secs = int(data_string[seconds_index:time_end_index])
    return hrs*3600 + mins*60 + secs


def main():
    # read in from log file
    longs = []
    lats = []
    colors = []
    heading_angles = []
    with open(log_path, 'r') as log_file:
        log_lines = list(log_file.readlines())
        line_idx = 0
        while line_idx < len(log_lines):
            cur_line = log_lines[line_idx]
            sats = extract_var(cur_line, "Satellites")
            if sats is not None and sats > 3:
                longs.append(extract_var(cur_line, "Long"))
                lats.append(extract_var(cur_line, "Lat"))
                colors.append(extract_time(cur_line))
                heading_angles.append(extract_var(log_lines[line_idx+1], "heading-angle"))
                line_idx += 1
            line_idx += 1
    longs_arr = np.asarray(longs) * coordinate_multiplier
    lats_arr = np.asarray(lats) * coordinate_multiplier
    colors_arr = np.asarray(colors)
    heading_arr = np.asarray(heading_angles) + np.pi/2

    vec_x = np.cos(heading_arr)
    vec_y = np.sin(heading_arr)

    map_img = plt.imread(map_path)

    plt.imshow(map_img, zorder=0, extent=map_bounds, aspect='equal')
    plt.xlim(map_bounds[0], map_bounds[1])
    plt.ylim(map_bounds[2], map_bounds[3])
    plt.scatter(longs_arr, lats_arr, c=colors_arr)
    plt.quiver(longs_arr, lats_arr, vec_x, vec_y, scale=20, c=colors_arr)
    plt.show()

if __name__ == '__main__':
    main()