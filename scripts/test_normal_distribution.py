from pylab import *
from scipy.stats import norm

plot_polar = False

if plot_polar == True:
    plt1 = subplot(2, 1, 1, projection='polar')
    plt1.set_theta_zero_location('N')
    plt1.set_theta_direction(-1)

    plt2 = subplot(2, 1, 2, projection='polar')
    plt2.set_theta_zero_location('N')
    plt2.set_theta_direction(-1)

go_short_heading = 80

go_short_heading_weights = [1 for x in range(360)]

# Create a range for weights between -180 and 180 with integer steps.
weights_range = range(-180, 180)
std_dev = 50
#y_vals = norm.pdf(x_vals, mean, std. deviation)
normal_dist = norm.pdf(weights_range,0,std_dev)
normal_dist = normal_dist / max(normal_dist)    # Normalizes the peak to 1

subplot(2, 1, 1)
if plot_polar == True:
    plot_list = array([0.0 for i in range(360)])
    for i in range(len(weights_range)):
        plot_list[i] = float(weights_range[i]) * (pi/180)
    polar(plot_list, normal_dist)
else:
    plot(weights_range, normal_dist)
title("Unshifted normal distribution, centered around 0")

offset = go_short_heading - ((len(normal_dist)/2) + 1)  # This is the index of normal_dist where the peak occurs
for i in range(len(normal_dist)):
    go_short_heading_weights[(i + offset) % len(go_short_heading_weights)] = normal_dist[i]

subplot(2, 1, 2)
if plot_polar == True:
    for i in range(360):
        plot_list[i] = i * (pi/180)
    polar(plot_list, go_short_heading_weights)
else:
    plot(go_short_heading_weights)
title("The heading has been shifted to %i" %go_short_heading)

show()
