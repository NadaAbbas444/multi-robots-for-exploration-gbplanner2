#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Int32
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd

class TSDFObjectCounterSubscriber:
    def __init__(self, namespaces, tsdf_voxel_size=0.20):
        self.namespaces = namespaces
        self.tsdf_voxel_size = tsdf_voxel_size
        self.data = {ns: {'Volume': [], 'Count': [], 'Time': []} for ns in namespaces}
        self.total_volume = []
        self.total_count = []  # Initialize to start with a zero count
        self.total_time = []
        self.fig, self.ax1 = plt.subplots()
        self.ax2 = self.ax1.twinx()  # Create a second y-axis for the counts
        #self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=1000)

        for ns in namespaces:
            tsdf_topic = f"/{ns}/gbplanner_node/tsdf_pointcloud"
            count_topic = "/red/detected_objects_counter"
            rospy.Subscriber(tsdf_topic, PointCloud2, self.tsdf_callback, callback_args=ns)
            rospy.Subscriber(count_topic, Int32, self.count_callback, callback_args=ns)

    def tsdf_callback(self, msg, ns):
        gen = pc2.read_points(msg, skip_nans=True)
        points = list(gen)
        volume = len(points) * self.tsdf_voxel_size ** 3 if points else (self.total_volume[-1] if self.total_volume else 0)
        time_step = len(self.total_time)  # This should be incremented or properly managed based on actual time or message count

        if not self.total_time or self.total_time[-1] != time_step:
            self.total_time.append(time_step)
            self.total_volume.append(volume)
            self.total_count.append(self.total_count[-1] if self.total_count else 0)
        else:
            # If there's an existing time step, update the last volume and keep the count stable
            self.total_volume[-1] = volume

    def count_callback(self, msg, ns):
        if self.total_count:
            # Update the last count with new data if available
            self.total_count[-1] += msg.data
        else:
            self.total_count.append(msg.data)



    def update_plot(self, frame):
        self.ax1.clear()
        self.ax2.clear()
        # Plotting total volume and count
        self.ax1.plot(self.total_time, self.total_volume, 'm-', label='Total Volume (m^3)')
        self.ax2.plot(self.total_time, self.total_count, 'b-', label='Total Count')
        self.ax1.set_xlabel('Time Steps')
        self.ax1.set_ylabel('Volume (m^3)', color='magenta')
        self.ax2.set_ylabel('Count', color='blue')
        self.fig.legend(loc='upper left')
        plt.title('Live Data: Total Volume and Object Count Over Time')
        plt.draw()

    def save_data_to_excel(self):
        writer = pd.ExcelWriter('output_data.xlsx', engine='openpyxl')
        for ns in self.namespaces:
            df = pd.DataFrame(self.data[ns])
            df.to_excel(writer, sheet_name=ns, index=False)
        writer.save()

if __name__ == '__main__':
    rospy.init_node('plotting_node')
    namespaces = ['red', 'blue']
    subscriber = TSDFObjectCounterSubscriber(namespaces)
    #plt.show()
    rospy.spin()
