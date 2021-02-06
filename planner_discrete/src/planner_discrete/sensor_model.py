import numpy as np
from planner_discrete.view import View
import trimesh
from imp import find_module
import array

class SensorModel:
    def __init__(self, parameter_map):
        # Get sensor specifications from parameter server
        try:   
            self.optimal_standoff = parameter_map["optimal_standoff_mm"]
            self.maximum_deviation_z = parameter_map["maximum_deviations"]["z_mm"]
            self.maximum_deviation_theta = parameter_map["maximum_deviations"]["theta_rad"]
            self.maximum_deviation_gamma = parameter_map["maximum_deviations"]["gamma_rad"]
            self.beam_taille_diameter = parameter_map["beam_taille_diameter_mm"]
            self.alpha = parameter_map["alpha_rad"]
            self.fan_angle = parameter_map["fan_angle_rad"]
        except KeyError as e:
            raise Exception("Provided parameter-map for sensor-model incomplete: {}".format(e.message))

        # Calculate useful sensor features from the specifics (so that they must be calculated every time)
        self.u_0 = np.sin(self.alpha) * self.optimal_standoff
        self.laser_emission_perception_distance = np.tan(self.alpha) * self.optimal_standoff
        self.m_L = np.tan(self.alpha)

        # Use ray_mesh_intersector for ray-tests as class member to prevent computional costly reinitialization
        # for each metrological evaulation
        self.ray_mesh_intersector = None
    
    def get_median_deviation_angle(self):
        return min(self.maximum_deviation_theta / 2, self.maximum_deviation_gamma / 2)

    def evaluate_score(self, z, gamma):
        return (self.get_max_uncertainty() - self.evaluate_uncertainty(z, gamma)) / (self.get_max_uncertainty() - self.get_min_uncertainty())

    def get_max_uncertainty(self):
        return self.evaluate_uncertainty(self.optimal_standoff + self.maximum_deviation_z, self.maximum_deviation_gamma + np.pi / 2)
    
    def get_min_uncertainty(self):
        return self.evaluate_uncertainty(self.optimal_standoff - self.maximum_deviation_z, np.pi / 2)

    def evaluate_uncertainty(self, z, gamma):
        try:
            beta = np.arcsin((self.u_0 * np.sin(np.pi/2 - self.alpha)) / np.sqrt((z + np.sqrt(self.u_0**2 + (self.u_0/self.m_L)**2))**2+self.u_0**2-2*(z + np.sqrt(self.u_0**2 + (self.u_0/self.m_L)**2))*self.u_0*np.cos(np.pi/2 - self.alpha)))
            d_z = np.sin(beta+gamma) / (np.sin(beta) * np.sin(gamma)) * (z + np.sqrt(self.u_0**2 + (self.u_0/self.m_L)**2)) * self.beam_taille_diameter / np.sqrt(self.u_0**2 + (self.u_0/self.m_L)**2)
            u = d_z / np.cos(gamma + beta - np.pi/2)
        except:
            return float("NaN")
        return u

    def get_optimal_standoff(self):
        return self.optimal_standoff

    def set_ray_mesh_intersector_for_mesh(self, mesh):
        try:
            find_module("pyembree")
            self.ray_mesh_intersector = trimesh.ray.ray_pyembree.RayMeshIntersector(mesh, scale_to_box=False)
        except:
            self.ray_mesh_intersector = trimesh.ray.ray_triangle.RayMeshIntersector(mesh, scale_to_box=False)

    def process_view_metrologically(self, surface_points, faces, view, face_normals, frustum_half_length_mm = 5e2):
        assert isinstance(view, View)
        if self.ray_mesh_intersector is None:
            raise Exception("Activate RayMeshIntersector before performing metrological tasks")    
            self.ray_mesh_intersector = trimesh.ray.ray_pyembree.RayMeshIntersector(target_mesh, scale_to_box = False)

        scan_frustum = self.get_scanning_frustum(frustum_half_length_mm)
        scan_frustum.apply_transform(view.get_orientation_matrix())
        scan_frustum.apply_transform(view.get_projected_position(True))
        try:
            find_module("pyembree")
            inliers = trimesh.ray.ray_pyembree.RayMeshIntersector(scan_frustum, scale_to_box=False).contains_points(surface_points)
        except:
            inliers = trimesh.ray.ray_triangle.RayMeshIntersector(scan_frustum, scale_to_box=False).contains_points(surface_points)
        inlier_indices = array.array('i')
        for i in range(len(inliers)):
            if inliers[i]:
                inlier_indices.append(i)
        trajectory_direction = view.get_orientation_matrix().dot(np.array([1, 0, 0, 1]))[0:3]
        sensor_direction = view.get_orientation_matrix().dot(np.array([0, 0, 1, 1]))[0:3]
        lateral_direction = view.get_orientation_matrix().dot(np.array([0, 1, 0, 1]))[0:3]
        ray_origins_emitter = np.ndarray((len(inlier_indices), 3), dtype=float)
        ray_directions_emitter = np.ndarray((len(inlier_indices), 3), dtype=float)
        ray_origins_detector = np.ndarray((len(inlier_indices), 3), dtype=float)
        ray_directions_detector = np.ndarray((len(inlier_indices), 3), dtype=float)
        trajectory_line_arguments = array.array('d', [])
        surface_points = np.array(surface_points)
        for i, inlier_index in enumerate(inlier_indices):
            t_0 =  (surface_points[inlier_index] - view.get_position()).dot(trajectory_direction)
            trajectory_line_arguments.append(t_0)
            
            ray_origins_emitter[i] = t_0 * trajectory_direction + view.get_position()
            ray_directions_emitter[i] = surface_points[inlier_index] - ray_origins_emitter[i]
            
            ray_origins_detector[i] = ray_origins_emitter[i] - trajectory_direction * self.laser_emission_perception_distance
            ray_directions_detector[i] = surface_points[inlier_index] - ray_origins_detector[i]
        rt_emitter_result = self.ray_mesh_intersector.intersects_first(
            ray_origins=ray_origins_emitter,
            ray_directions=ray_directions_emitter
            )
        rt_detector_result = self.ray_mesh_intersector.intersects_first(
            ray_origins=ray_origins_detector,
            ray_directions=ray_directions_detector
            )

        measurable_surface_point_indices = array.array('i', [])
        uncertainty_scores = array.array('d', [])
        trajectory_line_arguments_selected = array.array('d', [])
        for i, (rt_emitter_face_id, rt_detector_face_id) in enumerate(zip(rt_emitter_result, rt_detector_result)):
            if rt_emitter_face_id != rt_detector_face_id or faces[inlier_indices[i]] != rt_detector_face_id:
                continue
            else:
                gamma = np.pi / 2 + np.arctan2(trajectory_direction.dot(face_normals[rt_detector_face_id]), -1 * sensor_direction.dot(face_normals[rt_detector_face_id]))
                theta = np.arctan2(lateral_direction.dot(face_normals[rt_detector_face_id]), -1 * sensor_direction.dot(face_normals[rt_detector_face_id]))
                z = sensor_direction.dot(surface_points[inlier_indices[i]] - ray_origins_emitter[i])
                if gamma > np.pi / 4 and gamma < np.pi * 3 / 4 and abs(theta) < np.pi / 4 and z < 200 and z > 100:
                    uncertainty_scores.append(self.evaluate_score(z, gamma))
                    measurable_surface_point_indices.append(inlier_indices[i])
                    trajectory_line_arguments_selected.append(trajectory_line_arguments[i])
        return measurable_surface_point_indices, uncertainty_scores, trajectory_line_arguments_selected
        

    def get_scanning_frustum(self, half_length):
        scanner_frustum_vertices = [
            [half_length, (self.optimal_standoff - self.maximum_deviation_z) * np.tan(self.fan_angle / 2), - self.maximum_deviation_z],
            [- half_length, (self.optimal_standoff - self.maximum_deviation_z) * np.tan(self.fan_angle / 2), - self.maximum_deviation_z],
            [- half_length, - (self.optimal_standoff - self.maximum_deviation_z) * np.tan(self.fan_angle / 2), - self.maximum_deviation_z],
            [half_length, - (self.optimal_standoff - self.maximum_deviation_z) * np.tan(self.fan_angle / 2), - self.maximum_deviation_z],
            [half_length, (self.optimal_standoff + self.maximum_deviation_z) * np.tan(self.fan_angle / 2), self.maximum_deviation_z],
            [- half_length, (self.optimal_standoff + self.maximum_deviation_z) * np.tan(self.fan_angle / 2), self.maximum_deviation_z],
            [- half_length, - (self.optimal_standoff + self.maximum_deviation_z) * np.tan(self.fan_angle / 2), self.maximum_deviation_z],
            [half_length, - (self.optimal_standoff + self.maximum_deviation_z) * np.tan(self.fan_angle / 2), self.maximum_deviation_z],
        ]
        
        return trimesh.PointCloud(scanner_frustum_vertices).convex_hull
    