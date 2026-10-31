"""Behaviour of the start pose derived from the navigation graph."""

import math
import sys
import tempfile
import unittest
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tb3_fleet.set_initial_pose import load_waypoints, pose_from_graph  # noqa: E402


def write_graph(vertices):
    """A nav_graph YAML holding the given (x, y, name) vertices."""
    data = {'levels': {'L1': {'vertices': [[x, y, {'name': n}] for x, y, n in vertices],
                              'lanes': []}}}
    handle = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    yaml.safe_dump(data, handle)
    handle.close()
    return handle.name


class LoadWaypointsTest(unittest.TestCase):

    def test_unnamed_vertices_are_skipped(self):
        """A graph carries unnamed vertices; only named ones are addressable."""
        path = write_graph([(1.0, 2.0, 'a')])
        with open(path) as f:
            data = yaml.safe_load(f)
        data['levels']['L1']['vertices'].append([9.0, 9.0, {}])
        with open(path, 'w') as f:
            yaml.safe_dump(data, f)
        self.addCleanup(lambda: Path(path).unlink())
        self.assertEqual(load_waypoints(path), {'a': (1.0, 2.0)})


class PoseFromGraphTest(unittest.TestCase):

    def setUp(self):
        # east: +x from origin. north: +y from origin.
        self.path = write_graph([
            (0.0, 0.0, 'origin'),
            (2.0, 0.0, 'east'),
            (0.0, 3.0, 'north'),
            (-1.0, -1.0, 'southwest'),
        ])
        self.addCleanup(lambda: Path(self.path).unlink())

    def test_heading_east(self):
        x, y, yaw = pose_from_graph(self.path, 'origin', 'east')
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(yaw, 0.0)

    def test_heading_north_is_a_quarter_turn(self):
        _, _, yaw = pose_from_graph(self.path, 'origin', 'north')
        self.assertAlmostEqual(yaw, math.pi / 2)

    def test_heading_is_the_bearing_of_the_lane(self):
        """Independently: atan2(-1 - 0, -1 - 0) = -135 degrees."""
        _, _, yaw = pose_from_graph(self.path, 'origin', 'southwest')
        self.assertAlmostEqual(math.degrees(yaw), -135.0, places=6)

    def test_two_neighbours_give_two_headings(self):
        """A waypoint with several outgoing lanes has no single heading; --toward picks."""
        _, _, east = pose_from_graph(self.path, 'origin', 'east')
        _, _, north = pose_from_graph(self.path, 'origin', 'north')
        self.assertNotAlmostEqual(east, north)

    def test_no_neighbour_leaves_the_heading_at_zero(self):
        _, _, yaw = pose_from_graph(self.path, 'origin')
        self.assertAlmostEqual(yaw, 0.0)

    def test_unknown_waypoint_is_rejected(self):
        with self.assertRaises(SystemExit):
            pose_from_graph(self.path, 'nowhere', 'east')

    def test_unknown_neighbour_is_rejected(self):
        with self.assertRaises(SystemExit):
            pose_from_graph(self.path, 'origin', 'nowhere')


if __name__ == '__main__':
    unittest.main()
