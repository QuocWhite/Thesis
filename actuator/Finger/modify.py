import sys
import trimesh
import numpy as np
from sklearn.cluster import DBSCAN


def fit_circle_2d(points):
    """Fit a circle (x, y) to a set of 2D points using least squares."""
    x = points[:, 0]
    y = points[:, 1]
    A = np.c_[2*x, 2*y, np.ones_like(x)]
    b = x**2 + y**2
    c, *_ = np.linalg.lstsq(A, b, rcond=None)
    center = c[0:2]
    radius = np.sqrt(c[2] + center[0]**2 + center[1]**2)
    return center, radius


def find_holes(mesh):
    """Find circular hole-like regions on the mesh."""
    verts = mesh.vertices
    xy_points = verts[:, :2]

    # Cluster 2D projection points to detect hole centers
    db = DBSCAN(eps=2.5, min_samples=10).fit(xy_points)
    labels = db.labels_
    centers, radii = [], []

    for label in set(labels):
        if label == -1:
            continue
        cluster_points = xy_points[labels == label]
        if len(cluster_points) < 20:
            continue
        center, radius = fit_circle_2d(cluster_points)
        centers.append(center)
        radii.append(radius)
    return centers, radii


def scale_hole(mesh, hole_center, scale_factor):
    """Scale vertices around a hole center to enlarge the hole."""
    verts = mesh.vertices
    directions = verts[:, :2] - hole_center
    distances = np.linalg.norm(directions, axis=1)
    # Select vertices near hole perimeter
    mask = (distances < np.max(distances) * 0.3)
    verts[mask, :2] = hole_center + directions[mask] * scale_factor
    mesh.vertices = verts


def main(input_stl):
    mesh = trimesh.load_mesh(input_stl)
    centers, radii = find_holes(mesh)

    if len(radii) < 4:
        print("Warning: fewer than 4 circular features found.")

    sorted_radii = sorted(radii)
    largest_radius = sorted_radii[-1]
    target_radius = largest_radius - 0.25  # ~0.25 mm smaller than largest

    # Enlarge holes smaller than target
    for center, r in zip(centers, radii):
        if r < target_radius:
            scale_factor = target_radius / r
            print(f"Enlarging hole from {r:.2f} mm to {target_radius:.2f} mm")
            scale_hole(mesh, center, scale_factor)

    output_stl = input_stl.replace(".stl", "_fixed.stl")
    mesh.export(output_stl)
    print(f"Fixed STL saved as {output_stl}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python fix_holes.py <file.stl>")
        sys.exit(1)
    main(sys.argv[1])
