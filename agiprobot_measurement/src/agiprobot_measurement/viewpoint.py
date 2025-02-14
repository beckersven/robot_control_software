import numpy as np
import trimesh
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class ViewPoint:

    """
    Class for keeping track about one viewpoint.
    A viewpoint consists on the one hand of geometrical specifications (about location and length of the viewpointand execution informations), on the other
    hand of metrological specifications (e.g. what can be measured by this viewpoint).
    """


    def __init__(self, surface_point, surface_normal, angle_around_boresight, standoff_distance, tilt_theta = 0, tilt_psi = 0):
        """
        Creates viewpointanchor pose based on geometric specifications (see below). It does not assign execution- or metrology-relevant attributes to the object.
        The pose is designed by moving from surface_point in surface_normal-direction standoff_distance and generate a frame with the z-axis facing
        towards the surface point (i.e. = -1 * surface_normal). This axis is the boresight and the view's laser_emitter_frame will be turned around this angle
        the specified angle. In the end, tilting may be applied. This rotates the anchor point around the surface point according to the tilt_...-specification.
        Tilting is performed in a way that the new anchor-position is located on a sphere's surface around the surface-point with radius standoff_distance.

        :param surface_point: Point on the target-mesh surface this View's anchor is focused at
        :type surface_point: numpy.array
        :param surface_normal: Normal vector of the surface_point's face
        :type surface_normal: numpy.array
        :param angle_around_boresight: Angle this view's laser_emitter_frame is rotated around boresight at anchor-position
        :type angle_around_boresight: float
        :param standoff_distance: Distance of this view's anchor-position to the corresponding surface-point
        :type standoff_distance: float
        :param tilt_theta: Tilt-angle in theta-direction (see :ref:`sensor-model`), defaults to 0
        :type tilt_theta: float, optional
        :param tilt_psi: Tilt-angle in psi-direction (see :ref:`sensor-model`), defaults to 0
        :type tilt_psi: float, optional
        """
        self.projected_position = surface_point
        
        if not all(np.cross([1,0,0], -surface_normal) == np.array([0,0,0])):
            z_axis = -surface_normal
            y_axis = np.cross([1,0,0], z_axis) / np.linalg.norm(np.cross([1,0,0], z_axis))
            x_axis = np.cross(y_axis, z_axis)
        else:
            z_axis = -surface_normal
            y_axis = np.cross([0,0,1], z_axis) / np.linalg.norm(np.cross([0,0,1], z_axis))
            x_axis = np.cross(y_axis, z_axis)
        self.orientation_matrix = np.array([
                [x_axis[0], y_axis[0], z_axis[0], 0],
                [x_axis[1], y_axis[1], z_axis[1], 0],
                [x_axis[2], y_axis[2], z_axis[2], 0],
                [0,         0,         0,         1]
            ]).dot(np.array([
                [np.cos(angle_around_boresight),   -np.sin(angle_around_boresight),    0,  0],
                [np.sin(angle_around_boresight),   np.cos(angle_around_boresight),     0,  0],
                [0,                 0,               1,  0],
                [0,                 0,               0,  1]
            ]))
        self.orientation_matrix = self.orientation_matrix.dot(trimesh.transformations.rotation_matrix(tilt_theta, np.array([1, 0, 0])))
        self.orientation_matrix = self.orientation_matrix.dot(trimesh.transformations.rotation_matrix(tilt_psi, np.array([0, 1, 0])))
        self.anchor_position = standoff_distance * self.orientation_matrix.dot([0, 0, -1, 1])[:3] + surface_point


        self.lengths = [0,0]
        self.measurable_surface_point_indices = []
        self.measurable_surface_point_uncertainties = []
        
        self.trajectory_for_measurement = RobotTrajectory()
        self.trajectory_to_viewpoint= RobotTrajectory()
    
    def get_trajectory_to_viewpoint(self):
        """
        Returns the stored trajectory from a former state (might be robot's current_state or the end point of a previously executed trajectory) to
        the start state of this views's measurement trajectory. This is useful to connect multiple views into an execution plan.

        :return: Trajectory from certain start state to this views's measurement trajectory 
        :rtype: moveit_msgs/RobotTrajectory
        """
        return self.trajectory_to_viewpoint

    def set_trajectory_to_viewpoint(self, trajectory_to_viewpoint):
        """
        Sets a trajectory that moves the robot to the start of the view's measurement trajectory.
        The trajectory can be arbitrary but must end at this start point. This is useful to connect multiple views into an execution plan.

        :param trajectory_to_view: Trajectory from certain start state to this views's measurement trajectory 
        :type trajectory_to_view: moveit_msgs/RobotTrajectory
        """
        assert isinstance(trajectory_to_viewpoint, RobotTrajectory)
        self.trajectory_to_viewpoint = trajectory_to_viewpoint

    def reverse_trajectory_for_measurement(self):
        """
        Flips the measurement-trajectory of this viewpoint. The start-pose becomes the end-pose and vice versa. The execution time stays the same.

        :return: None
        :rtype: NoneType
        """
        reversed_trajectory = RobotTrajectory()
        reversed_trajectory.joint_trajectory.header = self.trajectory_for_measurement.joint_trajectory.header
        reversed_trajectory.joint_trajectory.joint_names = self.trajectory_for_measurement.joint_trajectory.joint_names
        
        # Make deepcopy to prevent changing the original measurement-trajectory
        temp = self.trajectory_for_measurement.joint_trajectory.points[:]
        temp.reverse()
        total_duration = self.trajectory_for_measurement.joint_trajectory.points[-1].time_from_start
        for index, point in enumerate(temp):
            new_point = JointTrajectoryPoint()
            # Positions are just reversed
            new_point.positions = point.positions
            # Dynamic values become negated (-> movement in the 'opposite direction')
            new_point.accelerations = [-1 * acc for acc in point.accelerations]
            new_point.velocities = [-1 * vel for vel in point.velocities]
            new_point.time_from_start = total_duration - self.trajectory_for_measurement.joint_trajectory.points[-(index + 1)].time_from_start
            reversed_trajectory.joint_trajectory.points.append(new_point)
        self.set_trajectory_for_measurement(reversed_trajectory)
        return True
            

    def get_surface_point(self, as_matrix=False):
        """
        Gets the position of the surface point this viewpoint is based on. This is not the same as the view_anchor-point.

        :param as_matrix: Whether to give the translation as homogeneous 4x4-matrix or vector, defaults to False
        :type as_matrix: bool, optional
        :return: Homogeneous 4x4-matrix or vector of the surface point
        :rtype: numpy.array (dimensions depend on parameter as_matrix)
        """
        if as_matrix:
            return trimesh.transformations.translation_matrix(self.projected_position)
        else:
            return self.projected_position


    def set_trajectory_for_measurement(self, trajectory_for_measurement):
        """
        Trajectory that is performed to execute the measurement corresponding to this viewpoint.
        This measurement-trajectory is a straight line in cartesian space.

        :param trajectory_for_measurement: Trajectory describing how to move the robot for this views measurement
        :type trajectory_for_measurement: moveit_msgs/RobotTrajectory
        """
        assert isinstance(trajectory_for_measurement, RobotTrajectory)
        self.trajectory_for_measurement = trajectory_for_measurement

    def get_trajectory_for_measurement(self):
        """
        Returns trajectory that is performed to execute the measurement corresponding to this viewpoint.
        This measurement-trajectory is a straight line in cartesian space.

        :return: Trajectory describing how to move the robot for this views measurement
        :rtype: moveit_msgs/RobotTrajectory
        """
        return self.trajectory_for_measurement

    def get_orientation_matrix(self):
        """
        Gets the orientation of the laser_emitter_frame of this viewpoint. The orientation (not the position) remains the same for every point on the assigned measurement-trajectory

        :return: Orientation of the laser_emitter_frame
        :rtype: numpy.array
        """
        return self.orientation_matrix

    def get_anchor_position(self, as_matrix = False):
        """
        Gets the laser_emitter_frame's position that was generated during construction. This is the point where the z-axis of the laser_emitter_frame
        hits the surface_point of the target mesh that was used to design this viewpoint. Keep in mind that 'anchor' does not necessarely mean 'anchor' of the trajectory.

        :param as_matrix: Whether to give the translation as homogeneous 4x4-matrix or vector, defaults to False
        :type as_matrix: bool, optional
        :return: Homogeneous 4x4-matrix or vector of the view's anchor position
        :rtype: np.array (dimensions depend on parameter as_matrix)
        """
        if as_matrix:
            return trimesh.transformations.translation_matrix(self.anchor_position)
        else:
            return self.anchor_position
    
    def set_measurable_surface_point_indices_and_uncertainties(self, measurable_point_indices, uncertainties):
        """
        Sets the indices of surface points measurable by this view's measurement-trajectory as well as their model-predicted uncertainties. 
        The indices are abstract and only meaningful when connected to a concrete surface-point-list which is maintained externally of the View-scope.
        Both lists must have the same length.

        :param measurable_point_indices: List of the indices of points in an external surface_point-list that can be measured by this view
        :type measurable_point_indices: list[int]
        :param uncertainties: List of uncertainty values for the measured surface_point masked through the index at the same position
        :type uncertainties: list[float]
        """
        assert len(measurable_point_indices) == len(uncertainties)
        self.measurable_surface_point_indices = measurable_point_indices
        self.measurable_surface_point_uncertainties = uncertainties
    
    def get_measurable_surface_point_indices(self):
        """
        Gets the surface_point-indices measurable by this view's measurement-trajectory. The indices are abstract and are only useful 
        when inserted into an externally maintained list of the actual surface points.

        :return: Surface_point-indices measurable by this view's measurement-trajectory of an external list
        :rtype: list[int]
        """

        return self.measurable_surface_point_indices
        
    def get_measurable_surface_point_uncertainties(self):
        """
        Gets the uncertainties of each measurable surface point. The values in the returned list must be matched with the
        actual surface points in an external viewpoint list using this views measurable_surface_point_indices: The uncertainty at index i
        is meant for the surface_point in the external list evaluated at index measurable_surface_point_indices[i].

        :return: uncertainties of each measurable surface point
        :rtype: list[float]
        """

        return self.measurable_surface_point_uncertainties
    

