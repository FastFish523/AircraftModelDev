import tempfile
import unittest
from pathlib import Path

import numpy as np

from htv2_replay import (
    DEFAULT_STL, EARTH_RADIUS_M, MAX_MODEL_TRIANGLES, body_axes_enu, chase_view_angles,
    curved_earth_surface, earth_sphere,
    load_result_file, load_stl_triangles, plot_indices, rotation_body_to_nue,
    trajectory_frame_enu,
)


def result_row(
    time=0.0, north=1.0, up=2.0, east=3.0, yaw=0.0, pitch=0.0, roll=0.0,
    velocity_theta=4.0, velocity_psi=5.0, alpha=6.0, beta=7.0,
):
    row = np.zeros(48)
    row[[0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 42, 47]] = [
        time, north, up, east, 100.0, yaw, pitch, roll,
        velocity_theta, velocity_psi, alpha, beta, up, 2.0,
    ]
    return row


class ReplayLogicTests(unittest.TestCase):
    def write_rows(self, rows):
        handle = tempfile.NamedTemporaryFile(mode="w", suffix=".dat", delete=False)
        np.savetxt(handle, np.atleast_2d(rows))
        handle.close()
        self.addCleanup(Path(handle.name).unlink, missing_ok=True)
        return handle.name

    def test_column_mapping_and_nue_to_enu(self):
        data = load_result_file(self.write_rows(result_row()))
        np.testing.assert_allclose(data.enu[0], [3.0, 1.0, 2.0])
        self.assertEqual(data.velocity_theta[0], 4.0)
        self.assertEqual(data.velocity_psi[0], 5.0)
        self.assertEqual(data.alpha[0], 6.0)
        self.assertEqual(data.beta[0], 7.0)
        self.assertEqual(data.size, 1)
        self.assertEqual(data.phase[0], 2.0)

    def test_nan_in_non_position_field_is_allowed(self):
        row = result_row()
        row[5] = np.nan
        data = load_result_file(self.write_rows(row))
        self.assertTrue(np.isnan(data.yaw[0]))

    def test_no_valid_position_is_rejected(self):
        row = result_row()
        row[1:4] = np.nan
        with self.assertRaisesRegex(ValueError, "没有有效的 NUE"):
            load_result_file(self.write_rows(row))

    def test_short_row_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "至少 48 列"):
            load_result_file(self.write_rows(np.zeros(10)))

    def test_missing_file_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "文件不存在"):
            load_result_file("does-not-exist.dat")

    def test_time_must_be_monotonic(self):
        rows = np.vstack((result_row(time=1.0), result_row(time=0.5)))
        with self.assertRaisesRegex(ValueError, "单调递增"):
            load_result_file(self.write_rows(rows))

    def test_zero_attitude_and_enu_axis_mapping(self):
        np.testing.assert_allclose(rotation_body_to_nue(0, 0, 0), np.eye(3), atol=1e-12)
        expected = np.array(((0, 0, 1), (1, 0, 0), (0, 1, 0)))
        np.testing.assert_allclose(body_axes_enu(0, 0, 0), expected, atol=1e-12)

    def test_single_and_combined_rotations(self):
        yaw_90 = rotation_body_to_nue(90, 0, 0)
        np.testing.assert_allclose(yaw_90[:, 0], [0, 0, -1], atol=1e-12)
        combined = rotation_body_to_nue(20, -30, 40)
        np.testing.assert_allclose(combined.T @ combined, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(combined), 1.0)

    def test_chase_view_is_behind_forward_axis(self):
        axes = body_axes_enu(0.0, 0.0, 0.0)
        elevation, azimuth = chase_view_angles(axes)
        self.assertGreater(elevation, 0.0)
        self.assertGreater(azimuth, -90.0)

    def test_trajectory_frame_follows_path_not_body_attitude(self):
        points = np.array([[0.0, 0.0, 0.0], [0.0, 5.0, 1.0], [0.0, 10.0, 2.0]])
        frame = trajectory_frame_enu(points, 1)
        np.testing.assert_allclose(frame[:, 0], [0.0, 10.0, 2.0] / np.linalg.norm([0.0, 10.0, 2.0]))
        np.testing.assert_allclose(frame.T @ frame, np.eye(3), atol=1e-12)

    def test_plot_decimation_preserves_endpoints(self):
        indices = plot_indices(10001, maximum=5000)
        self.assertLessEqual(indices.size, 5000)
        self.assertEqual(indices[0], 0)
        self.assertEqual(indices[-1], 10000)

    def test_downloaded_stl_is_centered_and_decimated(self):
        mesh = load_stl_triangles(DEFAULT_STL)
        self.assertEqual(mesh.shape, (MAX_MODEL_TRIANGLES, 3, 3))
        self.assertAlmostEqual(float(np.ptp(mesh[..., 0])), 1.0)
        np.testing.assert_allclose((mesh.min((0, 1)) + mesh.max((0, 1))) / 2.0, 0.0, atol=1e-12)

    def test_curved_earth_surface_matches_radius(self):
        points = np.array([[0.0, 0.0, 10.0], [100000.0, 200000.0, 1000.0]])
        east, north, up = curved_earth_surface(points)
        valid = np.isfinite(up)
        radius = np.sqrt(east[valid] ** 2 + north[valid] ** 2 + (up[valid] + EARTH_RADIUS_M) ** 2)
        np.testing.assert_allclose(radius, EARTH_RADIUS_M, rtol=0.0, atol=1e-6)

    def test_earth_sphere_is_tangent_to_enu_origin(self):
        east, north, up = earth_sphere(24, 13)
        radius = np.sqrt(east**2 + north**2 + (up + EARTH_RADIUS_M) ** 2)
        np.testing.assert_allclose(radius, EARTH_RADIUS_M, rtol=0.0, atol=1e-6)
        self.assertAlmostEqual(float(up.max()), 0.0)


if __name__ == "__main__":
    unittest.main()
