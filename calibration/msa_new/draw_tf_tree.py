#!/usr/bin/python3

import argparse

import matplotlib.pyplot as plt
import numpy as np
import yaml

"""
How to run this script:
    python3 draw_tf_tree.py <path_to_yaml>

How to install dependencies:
    pip3 install matplotlib
    pip3 install numpy
"""

"""
This script takes an MSA calibration YAML and opens a window
showing a 3D plot of sensor extrinsics.  It also prints the
extrinsics as matrices.

The input YAML will look like:
---
# <vehicle_name>
# ...
# source log: <log_uri>
camera:
  parent: "main_lidar"
  child: "camera"
  value: [0.1, 0.22, 1, -0.009, -0.3, 0.01, -0.9]
imu:
  parent: "main_lidar"
  child: "imu"
  value: [-0.15, 0.2, 0.2, 0.01, 0.2, 0.1, 0.08]
---

Interpreting the output:
Each sensor is represented by a set of axes. The axes show the orientation
of the sensor frame.  The red axis is the X axis, green is Y, and blue is Z.
(mneumonic: RGB = XYZ)

Cameras look down the Z axis, so the blue arrow will point in the look direction.

"""


def plot_axes(ax, root_from_axes_homogeneous, name):
    """
    Plots a sensor on the given axes.
    Input:
        ax: matplotlib axes object
        root_from_axes_homogeneous: 4x4 homogeneous transform from the object frame to the root frame
        name: name of the sensor
    """
    # Create a 3x3 identity matrix which represents the axes of the object in the object frame
    axes_object = np.eye(3)

    # Get the rotation matrix that takes us object_frame->root_frame (the axes_object are in the object frame)
    root_from_axes_rotation = root_from_axes_homogeneous[:3, :3]
    # Apply the rotation to the axes to put them in the root frame.
    axes_root = np.dot(root_from_axes_rotation, axes_object.T).T

    position_root = root_from_axes_homogeneous[:3, 3]

    # Draw the three object axes
    for i in range(3):
        ax.quiver(*position_root, *axes_root[i], color="rgb"[i], length=0.1)

    # Draw the name of the sensor
    # Add a small random offset to the position to avoid overlapping text when plotting sensors
    # with similar names and the same position (like ouster and outser_imu)
    small_random_offset = np.array([0, 0, np.random.rand() * 0.02])
    ax.text(*position_root + small_random_offset, name, fontsize=6)


def xyzw_quaternion_to_rotation_matrix(q):
    """
    Convert a quaternion into a full three-dimensional rotation matrix.
    Input: q - 4x1 array-like quaternion (x, y, z, w)
    Output: 3x3 rotation matrix
    """

    assert len(q) == 4, "Input quaternion must be 4 elements"

    # Raise if q is degenerate
    if np.abs(np.linalg.norm(q) - 1.0) > 1e-5:
        raise ValueError("Quaternion should be normalized")

    x, y, z, w = q[0], q[1], q[2], q[3]

    # Compute the rotation matrix components
    rxx = 1 - 2 * (y * y + z * z)
    rxy = 2 * (x * y - z * w)
    rxz = 2 * (x * z + y * w)

    ryx = 2 * (x * y + z * w)
    ryy = 1 - 2 * (x**2 + z * z)
    ryz = 2 * (y * z - x * w)

    rzx = 2 * (x * z - y * w)
    rzy = 2 * (y * z + x * w)
    rzz = 1 - 2 * (x**2 + y * y)

    return np.array([[rxx, rxy, rxz], [ryx, ryy, ryz], [rzx, rzy, rzz]])


class Edge:
    def __init__(self, parent, child, parent_from_child):
        self.parent = parent
        self.child = child
        self.parent_from_child = parent_from_child


class Graph:
    def __init__(self, edges):
        self.edges = edges

    def get_parent(self, child):
        parent_options = []
        for edge in self.edges:
            if edge.child == child:
                parent_options.append(edge.parent)
        if len(parent_options) == 1:
            return parent_options[0]
        elif len(parent_options) == 0:
            return None
        else:
            raise ValueError("Multiple parents found for child")

    def get_parent_from_node(self, child):
        for edge in self.edges:
            if edge.child == child:
                return edge.parent_from_child
        return None

    def get_children(self, parent):
        children = []
        for edge in self.edges:
            if edge.parent == parent:
                children.append(edge.child)
        return children

    def get_root(self):
        if len(self.edges) == 0:
            raise ValueError("Empty graph")
        # walk up the tree until we find a node with no parent
        cursor = self.edges[0].parent
        visited = set()
        while True:
            if cursor in visited:
                raise ValueError("Cycle detected in pose graph")
            visited.add(cursor)
            parent = self.get_parent(cursor)
            if parent is None:
                return cursor
            cursor = parent


def draw_subtree(root_from_node, node_name, graph, ax):
    """
    Recursively draw the subtree rooted at node.
    Input:
        root_from_node: 4x4 homogeneous transform from the node frame to the root frame
        node_name: name of the node
        graph: the pose graph
        ax: matplotlib axes object
    """

    if root_from_node is None:
        root_from_node = np.eye(4)

    # Print the transformation matrix
    print(f"\nMatrix taking {node_name} to root frame")
    np.set_printoptions(precision=8, suppress=True)
    print(root_from_node)

    # Plot the axes of the node
    plot_axes(ax, root_from_node, node_name)

    for child_name in graph.get_children(node_name):
        node_from_child = graph.get_parent_from_node(child_name)
        child_from_root = np.dot(root_from_node, node_from_child)

        # draw a line from this node to the child
        ax.quiver(
            *root_from_node[0:3, 3], *node_from_child[0:3, 3], color="gray", arrow_length_ratio=0.05, linewidth=0.5
        )

        # draw the child tree
        draw_subtree(child_from_root, child_name, graph, ax)


def main():
    parser = argparse.ArgumentParser(
        description="Plot 3D representation of sensor extrinsics from an MSA calibration YAML file."
    )
    parser.add_argument("yaml_file", type=str, help="Path to the input MSA calibration YAML file.")
    args = parser.parse_args()

    # Load the YAML file
    with open(args.yaml_file) as file:
        yaml_data = yaml.load(file, Loader=yaml.FullLoader)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # Equal aspect ratio so that proportions are correct
    ax.set_box_aspect([1, 1, 1])

    root_name = None

    # Draw the subtree for each sensor

    edges = []
    for item in yaml_data:
        assert "parent" in yaml_data[item], "Each sensor must have a parent frame"
        assert "child" in yaml_data[item], "Each sensor must have a child frame"
        assert "value" in yaml_data[item], "Each sensor must have a value"
        assert len(yaml_data[item]["value"]) == 3 + 4, "Position must be a 3D vector followed by a quaternion"
        parent_name = yaml_data[item]["parent"]
        child_name = yaml_data[item]["child"]
        parent_from_child = np.eye(4)
        parent_from_child[:3, 3] = np.array(yaml_data[item]["value"][:3])
        parent_from_child[:3, :3] = xyzw_quaternion_to_rotation_matrix(np.array(yaml_data[item]["value"][3:]))
        edges.append(Edge(parent_name, child_name, parent_from_child))

    if len(edges) == 0:
        raise ValueError("No sensor links found in YAML file")

    graph = Graph(edges)

    root_name = graph.get_root()
    print(f"Root node: {root_name}")

    draw_subtree(None, root_name, graph, ax)

    plt.show()


if __name__ == "__main__":
    main()
