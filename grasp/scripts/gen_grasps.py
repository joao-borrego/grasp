#!/usr/bin/python2

# As seen in https://github.com/mveres01/multi-contact-grasping
# src/collect_grasps.py

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

def normalizeVector(vector):
    """Normalizes a vector to have a magnitude of 1."""
    return vector / np.sqrt(np.sum(vector ** 2))

def meshCentroid(trimesh_mesh, center_type='vrep'):
    """Calculates the center of a mesh according to three different metrics.
    """
    if center_type == 'centroid':
        return trimesh_mesh.centroid
    elif center_type == 'com':
        return trimesh_mesh.center_mass
    elif center_type == 'vrep':  # How V-REP assigns object centroid
        maxv = np.max(trimesh_mesh.vertices, axis=0)
        minv = np.min(trimesh_mesh.vertices, axis=0)
        return 0.5 * (minv + maxv)

def getRotationMat(a, b):
    """Calculates the rotation needed to bring two axes coincident.

    Uses the reflection approach from:
    ----------------------------------
    T L Davis (https://math.stackexchange.com/users/411024/t-l-davis),
    Calculate Rotation Matrix to align Vector A to Vector B in 3d?,
    URL (version: 2017-04-13): https://math.stackexchange.com/q/2161631
    """

    def reflection(u, n):
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

def loadMesh(mesh_path):
    """Loads mesh from file and computes its centroid.
    """
    mesh = trimesh.load(mesh_path)
    center = meshCentroid(mesh, center_type='vrep')
    #mesh.vertices -= center
    return mesh

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

        a = np.hstack([gripper_point, -direction]).flatten()
        axis.quiver(*a, color='k', length=0.1)

        axis.scatter(*gripper_point.flatten(), c='b', marker='o', s=10)

    return axis

def generateCandidates(mesh, num_samples=1000, gripper_offset=-0.1,
        noise_level=0.05, augment=True):
    """Generates grasp candidates via surface normals of the object."""

    # Defines the up-vector for the workspace frame
    up_vector = np.asarray([0, 0, -1])

    points, face_idx = trimesh.sample.sample_surface_even(mesh, num_samples)

    matrices = []
    for p, face in zip(points, face_idx):
        normal = normalizeVector(mesh.triangles_cross[face])
        #print(normal, mesh.triangles_cross[face])

        # Add random noise to the surface normals, centered around 0
        if augment is True:
            normal += np.random.uniform(-noise_level, noise_level)
            normal = normalizeVector(normal)

        # Since we need to set a pose for the gripper, we need to calculate the
        # rotation matrix from a given surface normal
        matrix = getRotationMat(up_vector, normal)
        matrix[:3, 3] = p

        # Calculate an offset for the gripper from the object.
        matrix[:3, 3] = np.dot(matrix, np.array([0, 0, gripper_offset, 1]).T)[:3]

        matrices.append(matrix[:3].flatten())

    matrices = np.vstack(matrices)

    # Uncomment to view the generated grasp candidates
    plotMeshWithNormals(mesh, matrices, up_vector)
    plt.show()

    return matrices

def writeOutput(file, object, robot, grasp_candidates):
    """Write output to file"""

    data = dict(
        object = dict(
            name = object,
            uri = 'model://' + object,
            grasp_candidates = dict(),
        )
    )

    data['object']['grasp_candidates'][robot] = dict()
    for idx, grasp in enumerate(grasp_candidates):
        data['object']['grasp_candidates'][robot][idx] = dict()
        data['object']['grasp_candidates'][robot][idx]['pose'] = grasp[:6].tolist()
        data['object']['grasp_candidates'][robot][idx]['joints'] = [1,1,1]

    with open(file, 'w') as outfile:
        yaml.dump(data, outfile, default_flow_style=False)

def main(argv):
    """Main"""

    object_name = 'Seal'
    file_name = '/DATA/Datasets/KIT/BakingSoda_800_tex.obj'
    samples = 100
    robot = 'vizzy'
    out_file = 'data.yml'

    mesh = loadMesh(file_name)
    grasps = generateCandidates(mesh, samples)
    writeOutput(out_file, object_name, robot, grasps)


if __name__ == "__main__" :
    main(sys.argv)