###############################################################################
# Imports
###############################################################################

import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField

from iii_drone_configuration.configurator import Configurator

import time
import threading

import numpy as np

from .iwr6843_interface import iwr6843_interface
from .ti import TI

###############################################################################
# Class
###############################################################################

class IWRPublisher(Node):
    def __init__(self):
        super().__init__('mmwave')
        
        self.get_logger().info("IWRPublisher.__init__(): Starting node..")

        log_level = os.environ.get('MMWAVE_LOG_LEVEL')
        
        if log_level is not None:
            log_level = log_level.upper()
            
            if log_level == 'DEBUG':
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
            elif log_level == 'INFO':
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
            elif log_level == 'WARN':
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
            elif log_level == 'ERROR':
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
            elif log_level == 'FATAL':
                self.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)
        
        self.publisher_ = self.create_publisher(PointCloud2, 'pcl', 10)
        
        self.get_logger().info("IWRPublisher.__init__(): Node started with timer period " + str(timer_period) + " s")

    def on_configure(
        self, 
        state: State
    ) -> TransitionCallbackReturn:
        self.get_logger().info("IWRPublisher.on_configure()")
        
        ret = super().on_configure(state)
        
        if ret != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error("IWRPublisher.on_configure(): Base class configuration failed")
            return ret
        
        self.configurator = Configurator(self)
        
        self.cli_port = self.configurator.get_parameter('cli_port').value
        self.data_port = self.configurator.get_parameter('data_port').value
        self.cfg_path = self.configurator.get_parameter('cfg_path').value
        self.mmwave_frame_id = self.configurator.get_parameter('mmwave_frame_id').value
        
        self.get_logger().info("IWRPublisher.__init__(): Cli port: " + str(self.cli_port))
        self.get_logger().info("IWRPublisher.__init__(): Data port: " + str(self.data_port))
        self.get_logger().info("IWRPublisher.__init__(): Cfg path: " + str(self.cfg_path))
        self.get_logger().info("IWRPublisher.__init__(): Frame id: " + str(self.mmwave_frame_id))
        
        self.ti = TI(
            cli_loc=self.cli_port,
            data_loc=self.data_port,
            config_file=self.cfg_path,
        )
        
        self.iwr6843_interface = iwr6843_interface(
            self.cli_port,
            self.data_port,
            self.ti
        )

        self.get_logger().info("IWRPublisher.on_configure(): Configuration complete")
        
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(
        self,
        state: State
    ) -> TransitionCallbackReturn:
        self.get_logger().info("IWRPublisher.on_cleanup()")
        
        ret = super().on_cleanup(state)
        
        if ret != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error("IWRPublisher.on_cleanup(): Base class cleanup failed")
            return ret
        
        del self.iwr6843_interface
        del self.ti
        del self.cli_port
        del self.data_port
        del self.cfg_path
        del self.mmwave_frame_id
        del self.configurator
        
        self.get_logger().info("IWRPublisher.on_cleanup(): Cleanup complete")
        
        return TransitionCallbackReturn.SUCCESS

    def on_activate(
        self,
        state: State
    ) -> TransitionCallbackReturn:
        self.get_logger().info("IWRPublisher.on_activate()")
        
        ret = super().on_activate(state)
        
        if ret != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error("IWRPublisher.on_activate(): Base class activation failed")
            return ret
        
        self.iwr6843_interface.start()
        
        time.sleep(1)
        
        timer_period = self.ti.ms_per_frame/1000
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("IWRPublisher.on_activate(): Activation complete")
        
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(
        self,
        state: State
    ) -> TransitionCallbackReturn:
        self.get_logger().info("IWRPublisher.on_deactivate()")
        
        ret = super().on_deactivate(state)
        
        if ret != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error("IWRPublisher.on_deactivate(): Base class deactivation failed")
            return ret
        
        self.timer.cancel()
        del self.timer
        
        self.iwr6843_interface.stop()
        
        self.get_logger().info("IWRPublisher.on_deactivate(): Deactivation complete")
        
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(
        self,
        state: State
    ) -> TransitionCallbackReturn:
        self.get_logger().info("IWRPublisher.on_shutdown()")
        
        ret = super().on_shutdown(state)
        
        if ret != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error("IWRPublisher.on_shutdown(): Base class shutdown failed")
            return ret
        
        if hasattr(self, "configurator"):
            del self.configurator

        if hasattr(self, "iwr6843_interface"):
            self.iwr6843_interface.stop()
            
        def shutdown():
            time.sleep(1)
            
            rclpy.shutdown()
            
        shutdown_thread = threading.Thread(target=shutdown)
        shutdown_thread.start()
        
        return TransitionCallbackReturn.SUCCESS
    
    def on_error(
        self,
        state: State
    ) -> TransitionCallbackReturn:
        self.get_logger().fatal("IWRPublisher.on_error(): Error occured")
        
        raise RuntimeError("IWRPublisher.on_error(): Error occured")

    def destroy_node(self):
        print("IWRPublisher.destroy_node()")
        super().destroy_node()
        
        self.iwr6843_interface.stop()
        del self.iwr6843_interface
        
        del self.configurator

    def timer_callback(self):
        self.get_logger().debug("IWRPublisher.timer_callback()")
        
        xyzdata = self.iwr6843_interface.xyz_data
        
        if not xyzdata == []: 
            self.get_logger().debug("IWRPublisher.timer_callback(): Publishing pointcloud..")
            
            cloud_arr = np.asarray(xyzdata).astype(np.float32) # on form [[x,y,z],[x,y,z],[x,y,z]..]
            
            pcl_msg = PointCloud2()
            
            pcl_msg.header = Header()
            pcl_msg.header.stamp = self.get_clock().now().to_msg()
            pcl_msg.header.frame_id = self.mmwave_frame_id
            
            pcl_msg.height = 1 # because unordered cloud
            pcl_msg.width = cloud_arr.shape[0] # number of points in cloud
            
            # define interpretation of pointcloud message (offset is in bytes, float32 is 4 bytes)
            pcl_msg.fields =   [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
            
            pcl_msg.point_step = cloud_arr.dtype.itemsize*cloud_arr.shape[1] #size of 1 point (float32 * dimensions (3 when xyz))
            pcl_msg.row_step = pcl_msg.point_step*cloud_arr.shape[0] # only 1 row because unordered
            pcl_msg.is_dense = True
            
            pcl_msg.data = cloud_arr.tostring()
            
            self.publisher_.publish(pcl_msg)
            
        else:
            self.get_logger().debug("IWRPublisher.timer_callback(): No data to publish..")

###############################################################################
# Main
###############################################################################

def main(args=None):
    rclpy.init(args=args)
    
    node = IWRPublisher()
    
    node.get_logger().info("IWRPublisher node started.")
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()
        
    except KeyboardInterrupt:
        print("IWRPublisher.main(): Keyboard interrupt, destroying node..")
        node.destroy_node()
    
    print("IWRPublisher.main(): Shutting down..")

    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
