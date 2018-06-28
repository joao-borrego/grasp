#!/usr/bin/python2

"""
All heavy lifting performed by mveres01 at
https://github.com/mveres01/multi-contact-grasping
src/collect_grasps.py
"""

# Command line args
import sys, getopt
# Triangular mesh tools
import numpy as np
import trimesh
# Plots
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
# YAML output
import yaml

def loadMesh(mesh_path):
    """Loads mesh from file
    """
    mesh = trimesh.load(mesh_path)
    return mesh

def normalizeVector(vector):
    """Normalizes a vector to have a magnitude of 1
    """
    return vector / np.sqrt(np.sum(vector ** 2))

def getRotationMat(a, b):
    """Calculates the rotation needed to bring two axes coincident.

    Uses the reflection approach from T L Davis (https://math.stackexchange.com/users/411024/t-l-davis),
    Calculate Rotation Matrix to align Vector A to Vector B in 3d?,
    URL (version: 2017-04-13): https://math.stackexchange.com/q/2161631

    Returns 4 x 4 homogenous rotation matrix
    """

    def reflection(u, n):
        """Aux reflection function"""
        if not np.dot(n.T, n).any: 
            return u
        return u - 2 * n * np.dot(n.T, u) / np.dot(n.T, n)

    u = np.atleast_2d(normalizeVector(a)).T
    v = np.atleast_2d(normalizeVector(b)).T

    S = reflection(np.eye(3), u + v)
    R = reflection(S, v)

    eye = np.eye(4)
    eye[:3, :3] = R
    return eye

def generateCandidates(mesh, samples, offset, palm_normal):
    """Generates grasp candidates via surface normals of the object
    """
    points, face_indices = trimesh.sample.sample_surface_even(mesh, samples)

    matrices = []
    for point, face_idx in zip(points, face_indices):

        # Obtain normalized surface normal
        obj_normal = normalizeVector(mesh.triangles_cross[face_idx])
        # Obtain 4x4 rotation matrix that aligns the surface normal of the
        # palm with that of the object
        rot_mat = getRotationMat(palm_normal, obj_normal)

        # Apply rotation to surface point
        rot_mat[:3, 3] = point
        # Apply offset along the outer normal direction
        offset_vec = palm_normal * offset
        offset_vec = np.append(offset_vec, [1])
        rot_mat[:3, 3] = np.dot(rot_mat, offset_vec.T)[:3]

        # Store rotation matrix
        matrices.append(rot_mat[:3].flatten())

    # Merge into single array
    matrices = np.vstack(matrices)

    return matrices

def plotEqualAspectRatio(vertices, axis):
    """Forces the plot to maintain an equal aspect ratio

    # See:
    # http://stackoverflow.com/questions/13685386/matplotlib-equal-unit-length-with-equal-aspect-ratio-z-axis-is-not-equal-to
    """

    max_dim = np.max(np.array(np.max(vertices, axis=0) - np.min(vertices, axis=0)))

    mid = 0.5 * np.max(vertices, axis=0) + np.min(vertices, axis=0)
    axis.set_xlim(mid[0] - max_dim, mid[0] + max_dim)
    axis.set_ylim(mid[1] - max_dim, mid[1] + max_dim)
    axis.set_zlim(mid[2] - max_dim, mid[2] + max_dim)

    axis.set_xlabel('X')
    axis.set_ylabel('Y')
    axis.set_zlabel('Z')

    return axis

def formatHtMatrix(matrix_in):
    """Formats a 3x3 rotation matrix into a 4x4 homogeneous matrix."""

    if isinstance(matrix_in, list):
        matrix_in = np.asarray(matrix_in).reshape(3, 4)
    elif matrix_in.ndim == 1:
        matrix_in = matrix_in.reshape(3, 4)

    ht_matrix = np.eye(4)
    ht_matrix[:3] = matrix_in
    return ht_matrix

def plotMeshWithNormals(mesh, matrices, direction_vec, axis=None):
    """Visualize where we will sample grasp candidates from
    """

    if isinstance(direction_vec, list):
        dvec = np.atleast_2d(direction_vec).T
    elif isinstance(direction_vec, np.ndarray) and direction_vec.ndim == 1:
        dvec = np.atleast_2d(direction_vec).T
    else:
        dvec = direction_vec

    if axis is None:
        figure = plt.figure()
        axis = Axes3D(figure)
        axis.autoscale(False)

    # Construct a 3D mesh via matplotlibs 'PolyCollection'
    poly = Poly3DCollection(mesh.triangles, linewidths=0.05, alpha=0.25)
    poly.set_facecolor([0.5, 0.5, 1])
    axis.add_collection3d(poly)

    axis = plotEqualAspectRatio(mesh.vertices, axis)

    for i in range(0, len(matrices), 5):

        transform = formatHtMatrix(matrices[i])

        # We'll find the direction by finding the vector between two points
        gripper_point = np.hstack([matrices[i, 3], matrices[i, 7], matrices[i, 11]])
        gripper_point = np.atleast_2d(gripper_point)

        direction = np.dot(transform[:3, :3], dvec)
        direction = np.atleast_2d(direction).T

        a = np.hstack([gripper_point, direction]).flatten()
        axis.quiver(*a, color='k', length=0.1)

        axis.scatter(*gripper_point.flatten(), c='b', marker='o', s=10)

    plt.show()

def writeOutput(file, object, robot, matrices):
    """Write output to file"""

    data = dict(
        object = dict(
            name = object,
            uri = 'model://' + object,
            grasp_candidates = dict(),
        )
    )

    data['object']['grasp_candidates'][robot] = dict()

    for i in range(0, len(matrices)):

        data['object']['grasp_candidates'][robot][i] = dict()
        data['object']['grasp_candidates'][robot][i]['tf'] = \
            matrices[i].tolist()
        # formatHtMatrix(matrices[i]).tolist()
        #data['object']['grasp_candidates'][robot][i]['joints'] = [1,1,1]

    with open(file, 'w') as outfile:
        yaml.dump(data, outfile, default_flow_style=False)

def main(argv):
    """Main method
    """
    object_name = 'Pitcher'
    file_name = 'datasets/kit/Pitcher/meshes/Pitcher.stl'
    samples = 200
    #robot = 'vizzy'
    #palm_normal = np.array([0,-1,0])
    robot = 'shadowhand'
    palm_normal = np.array([0,1,0])
    palm_offset = 0.03 # [m]
    out_file = 'grasp/config/' + object_name + '.grasp.yml'

    mesh = loadMesh(file_name)
    grasps = generateCandidates(mesh, samples, palm_offset, palm_normal)
    plotMeshWithNormals(mesh, grasps, palm_normal)
    writeOutput(out_file, object_name, robot, grasps)

if __name__ == "__main__" :
    main(sys.argv)
