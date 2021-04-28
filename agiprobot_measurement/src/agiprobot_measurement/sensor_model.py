import numpy as np
from agiprobot_measurement.viewpoint import ViewPoint
import trimesh
from imp import find_module
import array

class SensorModel:
    """
    Computional representation of an laser triangulation sensor under influence of uncertainty.
    """


    def __init__(self, parameter_map):
        """
        Fetches geometric sensor parameters in order to build a uncertainty-based model of a laser triangulation sensor.

        :param parameter_map: Geometric sensor-parameters packed as a dictionary
        :type parameter_map: dict
        :returns: None
        :rtype: NoneType
        """
        # Get sensor specifications from parameter server
        try:   
            self.optimal_standoff = parameter_map["optimal_standoff_mm"]
            self.minimum_standoff = parameter_map["minimum_standoff_mm"]
            self.maximum_standoff = parameter_map["maximum_standoff_mm"]
            self.maximum_deviation_theta = parameter_map["theta_max_rad"]
            self.maximum_deviation_psi = parameter_map["psi_max_rad"]
            self.K_u = parameter_map["K_u"]
            self.alpha = parameter_map["alpha_rad"]
            self.fan_angle = parameter_map["fan_angle_rad"]
        except KeyError as e:
            raise Exception("Provided parameter-map for sensor-model incomplete: {}".format(e.message))
        
        # Calculate useful sensor features from the specifics (so that they must not be calculated every time)
        self.xi_0 = np.sin(self.alpha) * self.optimal_standoff
        self.laser_emission_perception_distance = np.tan(self.alpha) * self.optimal_standoff
        self.m_L = np.tan(self.alpha)


        self.context = None
  
    
    def get_median_deviation_angle(self):
        """
        Get the smallest angle allowed for tilting (smallest of maximum deviations of theta and psi devided by 2).

        :returns: Smallest angle allowed for tilting
        :rtype: float
        """
        return min(self.maximum_deviation_theta / 2, self.maximum_deviation_psi / 2)

    def evaluate_score(self, z, psi):
        """
        Calculates a score of uncertainty :math:`\\in [0,1]` for the given z and psi (higher score means lower uncertainty). The score
        is linear affine in the uncertainty based on z and psi - It is 1 if the provided values match the best possible case (lowest
        possible uncertainty of the sensor model) and 0 when both values are right at the rejection limit (i.e. are equal to max_deviation_...):
        :math:`\\frac{u_{max} - u(z, \\psi)}{u_{max} - u_{min}}`

        :param z: z-coordinate of the point to evaluate in the laser_emitter_frame in mm
        :type z: float
        :param psi: Angle of the laser_emitter_frame's z-y-plane with the surface-triangle of the point to evaluate in rad
        :type psi: float
        :returns: Score :math:`\\in [0,1]` based on the parameters and the max_deviations of them
        :rtype: float
        """
        return (self.get_max_uncertainty() - self.evaluate_uncertainty(z, psi)) / (self.get_max_uncertainty() - self.get_min_uncertainty())

    def get_max_uncertainty(self):
        """
        Computes maximum possible uncertainty of a measured point that is not rejected using geometric sensor parameters. 
        The uncertainty-formula is evaluated at z- and psi-values that are at rejection-limits for a surface point, so that this value corresponds
        to the worst uncertainty assignable to a measurable point.

        :returns: Maximum possible uncertainty of a measurable point (= is within the max_deviation_...-range) in mm
        :rtype: float
        """
        return self.evaluate_uncertainty(self.maximum_standoff, self.maximum_deviation_psi + np.pi / 2)
    
    def get_min_uncertainty(self):
        """
        Computes the minimum possible uncertainty using the geometric sensor parameters. It returns the uncertainty value assignable to
        a surface point which has been measured with optimal z- and psi-values, i.e. the best-case.

        :returns: Minimum possible uncertainty in mm
        :rtype: float
        """
        return self.evaluate_uncertainty(self.minimum_standoff, np.pi / 2)

    def evaluate_uncertainty(self, z, psi):
        """
        Computes the uncertainty of a surface point with given z and psi using the specified geometric sensor parameters.

        :param z: z-coordinate of the point to evaluate in the laser_emitter_frame in mm
        :type z: float
        :param psi: Angle of the laser_emitter_frame's z-y-plane with the surface-triangle of the point to evaluate in rad
        :type psi: float
        :returns: Uncertainty based on the z, psi, and the geometric sensor parameters, or 'NaN' if z and psi are invalid
        :rtype: float
        """
        try:
            # See the corresponding Bachelors Thesis to understand the origin of this equation
            u = self.K_u * np.sqrt(z**2+self.xi_0**2-2*z*self.xi_0*np.sin(self.alpha)) / np.sin(psi) * z
        except:
            return float("NaN")
        return u

    def get_optimal_standoff(self):
        """
        Gets the optimal standoff, i.e. required the .
        The returned optimal standoff is not equal to the standoff providing the lowest uncertainty, but in the center between the z-rejection-limits
        to allow maximum flexibility in surface height deviation when moving the sensor.

        :returns: z-coordinate in the laser_emitter_frame of an surface point for measurement in mm
        :rtype: float
        
        """
        return self.optimal_standoff

    def set_processing_context(self, mesh, sampled_surface_points, sampled_face_indices):
        """
        Loads a context for metrological processing to the sensor model.
        This includes loading the target mesh into a ray-tracer as well as gaining awareness over the
        sampling results because they are directly used to assess a ViewPoint during process_viewpoint_metrologically().
        This method must be called before any metrological processing can be performed. A previously set
        context becomes overwritten completely by calling this method again.

        :param mesh: Mesh-object to load into the RayMeshIntersector
        :type mesh: trimesh.Trimesh
        :param sampled_surface_points: List of all sampled surface points that should be considered for the metrological processing
        :type sampled_surface_points: list[numpy.array]
        :param sampled_face_ids: List of face-indices, where each entry corresponds to the face of the sampled surface point at the same position
        :type sampled_face_ids: list[int]
        """
        self.context = {
            "sampled_surface_points":   np.array(sampled_surface_points),
            "sampled_face_indices":     sampled_face_indices,
            "face_normals":             [face_normal for face_normal in mesh.face_normals],
            "target_mesh":              mesh
        }
    

        # Load mesh into a RayMeshIntersector usable for ray-tracing tests and therefore metrological evaluation. 
        # It tries to use the high-performing Intel(R) Embree raytracer (works with AMD-CPUs as well), but loads a slow default
        # raytracer if 'pyembree'-python-package is not installed.
        try:
            find_module("pyembree")
            # Scale to box is set to 'False' as errors have been observed when 'True' (obviously visible surface points
            # where evaluated as invisible during raytracing)
            self.context.update({"ray_mesh_intersector": trimesh.ray.ray_pyembree.RayMeshIntersector(mesh, scale_to_box=False)})
        except:
            self.context.update({"ray_mesh_intersector": trimesh.ray.ray_triangle.RayMeshIntersector(mesh, scale_to_box=False)})



    def process_viewpoint_metrologically(self, viewpoint, uncertainty_threshold, maximum_deflection = 5e2):
        """
        Computes all measurable surface-points by a viewpoint-object as well as uncertainties and where they are measurable on the viewpoint-measurement-trajectory.
        Requires context to be set via set_processing_context(...). Checks for every sampled surface point of the given context whether it is visible and calculates the
        uncertainty for it eventually. Also, the deflection of the laser_emitter_frame along the trajectory-line from the viewpoint-anchor pose is evaluated.
        
        :param viewpoint: ViewPoint with set viewpoint-anchor-pose
        :type viewpoint: ViewPoint
        :param uncertainty_threshold: Maximum permissible uncertainty of a measured surface point, in mm
        :type uncertainty_threshold: float
        :param maximum_deflection: Maximum deflection of the trajectory to be considered for processing in mm, defaults to 5e2 
        :type maximum_deflection: float, optional
        :returns:   3 unpacked arrays of the same length in order:\n
                    - Indices of the measurable surface points in samples_surface_points_list\n
                    - Corresponding uncertainty-scores\n
                    - Metric distance in mm in trajectory-direction from the viewpoint-anchor where the corresponding surface point is measurable\n
        :rtype: array[int], array[float], array[float]
        """

        assert isinstance(viewpoint, ViewPoint)
        if self.context is None:
            raise Exception("Set the context before performing metrological processing!")    
        

        # Only consider surface_points within the scanning_frustum to reduce processing overhead:

        # Make scan-frustum
        scan_frustum = self.get_scanning_frustum(maximum_deflection)
        # Transform frustum so that it is aligned with the laser_emitter_frame at the viewpoint-anchor-pose
        scan_frustum.apply_transform(viewpoint.get_orientation_matrix())
        scan_frustum.apply_transform(viewpoint.get_anchor_position(True))

        # Use RayMeshIntersector to identify the surface_points that are potentially measurable
        try:
            find_module("pyembree")
            inliers = trimesh.ray.ray_pyembree.RayMeshIntersector(scan_frustum, scale_to_box=False).contains_points(self.context["sampled_surface_points"])
        except:
            inliers = trimesh.ray.ray_triangle.RayMeshIntersector(scan_frustum, scale_to_box=False).contains_points(self.context["sampled_surface_points"])
        # Extract the indices of the covered points in sampled_surface_points
        inlier_indices = array.array('i')
        for i in range(len(inliers)):
            if inliers[i]:
                inlier_indices.append(i)

        # Get characteristic geometrical specifications of the viewpoint-measurement-trajectory (= unit-vectors of the sensor-frame)
        trajectory_direction = viewpoint.get_orientation_matrix().dot(np.array([1, 0, 0, 1]))[0:3]
        sensor_direction = viewpoint.get_orientation_matrix().dot(np.array([0, 0, 1, 1]))[0:3]
        lateral_direction = viewpoint.get_orientation_matrix().dot(np.array([0, 1, 0, 1]))[0:3]

        # For each inlier, prepare perform 2 ray-tests (one from the laser_emitter_frame to the inlier and one from the laser_detector_frame to the inlier)
        ray_origins_emitter = np.ndarray((len(inlier_indices), 3), dtype=float)
        ray_directions_emitter = np.ndarray((len(inlier_indices), 3), dtype=float)
        ray_origins_detector = np.ndarray((len(inlier_indices), 3), dtype=float)
        ray_directions_detector = np.ndarray((len(inlier_indices), 3), dtype=float)
        trajectory_line_arguments = array.array('d', [])  
        for i, inlier_index in enumerate(inlier_indices):
            # t_0 = distance in trajectory_direction where the inlier becomes visible, i.e. the vector from inlier to trajectory becomes perpendicular to this direction
            t_0 = (self.context["sampled_surface_points"][inlier_index] - viewpoint.get_anchor_position()).dot(trajectory_direction)
            trajectory_line_arguments.append(t_0)
            ray_origins_emitter[i] = t_0 * trajectory_direction + viewpoint.get_anchor_position()
            ray_directions_emitter[i] = self.context["sampled_surface_points"][inlier_index] - ray_origins_emitter[i]
            # Detector has a fixed offset on the viewpoint-measurement-trajectory in trajectory_direction
            ray_origins_detector[i] = ray_origins_emitter[i] - trajectory_direction * self.laser_emission_perception_distance
            ray_directions_detector[i] = self.context["sampled_surface_points"][inlier_index] - ray_origins_detector[i]

        # Get the first intersections of the rays with the object mesh (only those are relevant for visibility concerns)
        rt_emitter_result = self.context["ray_mesh_intersector"].intersects_first(
            ray_origins=ray_origins_emitter,
            ray_directions=ray_directions_emitter
            )
        rt_detector_result = self.context["ray_mesh_intersector"].intersects_first(
            ray_origins=ray_origins_detector,
            ray_directions=ray_directions_detector
            )

        measurable_surface_point_indices = array.array('i', [])
        uncertainties = array.array('d', [])
        trajectory_line_arguments_selected = array.array('d', [])

        # Filter the inliers (each iteration represents the evaluation of one inlier as rt_..._results are sorted)
        for i, (rt_emitter_face_id, rt_detector_face_id) in enumerate(zip(rt_emitter_result, rt_detector_result)):
            # Reject if the rays from the detector and emitter hit different surface-triangles or if the hit surface for a surface point does not match the corresponding sample_surface_index
            if rt_emitter_face_id != rt_detector_face_id or self.context["sampled_face_indices"][inlier_indices[i]] != rt_detector_face_id:
                continue
            else:
                # Compute uncertainty-parameters z, psi, theta
                psi = np.pi / 2 + np.arctan2(trajectory_direction.dot(self.context["face_normals"][rt_detector_face_id]), -1 * sensor_direction.dot(self.context["face_normals"][rt_detector_face_id]))
                theta = np.arctan2(lateral_direction.dot(self.context["face_normals"][rt_detector_face_id]), sensor_direction.dot(self.context["face_normals"][rt_detector_face_id]))
                theta -= np.arctan2(lateral_direction.dot(-1 * ray_directions_emitter[i]), sensor_direction.dot(-1 * ray_directions_emitter[i]))
                # Wrapping into [-pi, pi]
                if theta > np.pi:
                    theta = -2 * np.pi + theta
                elif theta < -np.pi:
                    theta = 2 * np.pi + theta
                z = sensor_direction.dot(self.context["sampled_surface_points"][inlier_indices[i]] - ray_origins_emitter[i])
                # Only accept, if no violation of hard geometric limit
                
                if abs(psi - np.pi/2) < self.maximum_deviation_psi and abs(theta) < self.maximum_deviation_theta and z < self.maximum_standoff and z > self.minimum_standoff:
                    u = self.evaluate_uncertainty(z, psi) * abs(sensor_direction.dot(self.context["face_normals"][rt_detector_face_id]))
                    # Only accept, if no violation of hard uncertainty threshold
                    if u < uncertainty_threshold:
                        uncertainties.append(u)
                        measurable_surface_point_indices.append(inlier_indices[i])
                        trajectory_line_arguments_selected.append(trajectory_line_arguments[i])
        
       

        return measurable_surface_point_indices, uncertainties, trajectory_line_arguments_selected
        

    def get_scanning_frustum(self, half_length):
        """
        Generates a trimesh-mesh object representing the 'frustum of measurability' in the laser_emitter_frame. Points within this frustum have 
        the potential to be measurable, but maximum_deviation_... may still reject points within this frustum. The frustum is constructed based on
        the laser-fan-angle and the allowed z-range and the idea that the trajectory is a straight line (it can be imagined as a cut-off 'Toblerone'). 
        So points lying wthin this frustum can be touched by the laser line AND can be mapped to the optical sensor.

        :param half_length: Length of the frustum perpendicular to the fan-plane in each direction of the laser_emitter_frame in mm
        :type half_length: float
        :returns: Scan frustum represting the volume of potentially measurable points
        :rtype: trimesh.Trimesh
        """
        scanner_frustum_vertices = [
            [half_length, (self.minimum_standoff) * np.tan(self.fan_angle / 2), self.minimum_standoff],
            [- half_length, (self.minimum_standoff) * np.tan(self.fan_angle / 2), self.minimum_standoff],
            [- half_length, - (self.minimum_standoff) * np.tan(self.fan_angle / 2), self.minimum_standoff],
            [half_length, - (self.minimum_standoff) * np.tan(self.fan_angle / 2), self.minimum_standoff],
            [half_length, (self.maximum_standoff) * np.tan(self.fan_angle / 2), self.maximum_standoff],
            [- half_length, (self.maximum_standoff) * np.tan(self.fan_angle / 2), self.maximum_standoff],
            [- half_length, - (self.maximum_standoff) * np.tan(self.fan_angle / 2), self.maximum_standoff],
            [half_length, - (self.maximum_standoff) * np.tan(self.fan_angle / 2), self.maximum_standoff],
        ]
        
        return trimesh.PointCloud(scanner_frustum_vertices).convex_hull
    