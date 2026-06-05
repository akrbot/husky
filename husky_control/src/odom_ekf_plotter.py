#!/usr/bin/env python3
"""Real-time XY position plot: wheel odom vs. EKF vs. Gazebo ground truth.

A single top-down plot (default 10 m x 10 m, origin at the centre) showing the
robot position from three sources, updated live. Drive the robot in simulation
and watch the gap between wheel odom / EKF and the true pose grow.

Params:
  ~ekf_topic   (default: /odometry/filtered)
  ~odom_topic  (default: /x100_velocity_controller/odom)
  ~gt_topic    (default: /gazebo/model_states)
  ~robot_name  (default: x100)   model name to look up in model_states
  ~size        (default: 10.0)  plot side length in metres (centred on 0,0)
  ~rate        (default: 30.0)  refresh rate in Hz
"""

import threading

import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets


class LatestPose:
    """Thread-safe holder for the latest x, y of one odometry source."""

    def __init__(self):
        self.lock = threading.Lock()
        self.have_data = False
        self.x = 0.0
        self.y = 0.0

    def callback(self, msg):
        with self.lock:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            self.have_data = True

    def set_xy(self, x, y):
        with self.lock:
            self.x = x
            self.y = y
            self.have_data = True

    def get(self):
        with self.lock:
            return (self.x, self.y, self.have_data)


class OdomXYPlotter(QtWidgets.QMainWindow):
    EKF_COLOR = (50, 200, 90)     # green  -> EKF filtered
    ODOM_COLOR = (235, 130, 40)   # orange -> raw wheel odom
    GT_COLOR = (70, 140, 235)     # blue   -> Gazebo ground truth

    def __init__(self):
        super(OdomXYPlotter, self).__init__()

        ekf_topic = rospy.get_param("~ekf_topic", "/odometry/filtered")
        odom_topic = rospy.get_param("~odom_topic", "/x100_velocity_controller/odom")
        gt_topic = rospy.get_param("~gt_topic", "/gazebo/model_states")
        self.robot_name = rospy.get_param("~robot_name", "x100")
        self.half = float(rospy.get_param("~size", 10.0)) / 2.0
        self.rate = float(rospy.get_param("~rate", 30.0))

        self.ekf = LatestPose()
        self.odom = LatestPose()
        self.gt = LatestPose()
        rospy.Subscriber(ekf_topic, Odometry, self.ekf.callback, queue_size=20)
        rospy.Subscriber(odom_topic, Odometry, self.odom.callback, queue_size=20)
        rospy.Subscriber(gt_topic, ModelStates, self.gt_callback, queue_size=20)
        rospy.loginfo("odom_ekf_plotter: EKF='%s' (green), wheel odom='%s' (orange), "
                      "ground truth='%s' model '%s' (blue)",
                      ekf_topic, odom_topic, gt_topic, self.robot_name)

        self._build_ui()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(int(1000.0 / self.rate))

    def gt_callback(self, msg):
        try:
            i = msg.name.index(self.robot_name)
        except ValueError:
            rospy.logwarn_throttle(
                5.0, "ground truth: model '%s' not in /gazebo/model_states" % self.robot_name)
            return
        p = msg.pose[i].position
        self.gt.set_xy(p.x, p.y)

    def _build_ui(self):
        pg.setConfigOptions(antialias=True)
        self.setWindowTitle("EKF vs Wheel Odometry - XY position")
        self.resize(800, 800)

        widget = pg.GraphicsLayoutWidget()
        self.setCentralWidget(widget)

        p = widget.addPlot(title="Robot position (X-Y)")
        p.setLabel("bottom", "x", units="m")
        p.setLabel("left", "y", units="m")
        p.showGrid(x=True, y=True, alpha=0.3)
        p.addLegend(offset=(10, 10))
        p.setAspectLocked(True)
        p.setXRange(-self.half, self.half)
        p.setYRange(-self.half, self.half)
        p.disableAutoRange()
        # crosshair through the origin
        p.addLine(x=0, pen=pg.mkPen((120, 120, 120), width=1))
        p.addLine(y=0, pen=pg.mkPen((120, 120, 120), width=1))

        # trailing path lines
        self.path_gt = p.plot(pen=pg.mkPen(self.GT_COLOR, width=2), name="ground truth")
        self.path_ekf = p.plot(pen=pg.mkPen(self.EKF_COLOR, width=2), name="EKF")
        self.path_odom = p.plot(pen=pg.mkPen(self.ODOM_COLOR, width=2), name="wheel odom")
        # current-position markers (dots)
        self.dot_gt = p.plot(pen=None, symbol="o", symbolSize=12,
                             symbolBrush=self.GT_COLOR, symbolPen=None)
        self.dot_ekf = p.plot(pen=None, symbol="o", symbolSize=12,
                              symbolBrush=self.EKF_COLOR, symbolPen=None)
        self.dot_odom = p.plot(pen=None, symbol="o", symbolSize=12,
                               symbolBrush=self.ODOM_COLOR, symbolPen=None)

        self.gt_x, self.gt_y = [], []
        self.ekf_x, self.ekf_y = [], []
        self.odom_x, self.odom_y = [], []

    def update_plot(self):
        if rospy.is_shutdown():
            QtWidgets.QApplication.quit()
            return

        ex, ey, e_ok = self.ekf.get()
        ox, oy, o_ok = self.odom.get()
        gx, gy, g_ok = self.gt.get()

        if g_ok:
            self.gt_x.append(gx)
            self.gt_y.append(gy)
            self.path_gt.setData(self.gt_x, self.gt_y)
            self.dot_gt.setData([gx], [gy])
        if e_ok:
            self.ekf_x.append(ex)
            self.ekf_y.append(ey)
            self.path_ekf.setData(self.ekf_x, self.ekf_y)
            self.dot_ekf.setData([ex], [ey])
        if o_ok:
            self.odom_x.append(ox)
            self.odom_y.append(oy)
            self.path_odom.setData(self.odom_x, self.odom_y)
            self.dot_odom.setData([ox], [oy])


def main():
    rospy.init_node("odom_ekf_plotter", anonymous=True)
    app = QtWidgets.QApplication([])
    win = OdomXYPlotter()
    win.show()
    rospy.on_shutdown(lambda: QtCore.QMetaObject.invokeMethod(
        app, "quit", QtCore.Qt.QueuedConnection))
    app.exec_()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
