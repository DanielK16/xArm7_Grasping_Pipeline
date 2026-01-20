import open3d as o3d
import numpy as np
import cv2
import os

def generate_final_centered_mesh(color_path, depth_path, mask_path, intrinsics, output_filename):
    """
    Erstellt ein stabiles Mesh, rotiert es um 180 Grad, zentriert es in X/Y 
    und setzt den tiefsten Punkt (Boden) auf Z=0.
    """
    
    # 1. DATEN LADEN
    if not all(os.path.exists(p) for p in [color_path, depth_path, mask_path]):
        print("Fehler: Pfade prüfen!")
        return

    color_img = cv2.imread(color_path)
    depth_img = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
    mask_img = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)

    # 2. VORVERARBEITUNG (Stabilisierung gegen Segfaults)
    # Masken-Erosion: Schneidet 5 Pixel am Rand weg, um "schlechte" Daten zu eliminieren
    kernel = np.ones((5,5), np.uint8)
    mask_eroded = cv2.erode(mask_img, kernel, iterations=1)
    
    # Bilateraler Filter: Glättet Flächen, hält aber Kanten scharf
    depth_filtered = cv2.bilateralFilter(depth_img.astype(np.float32), 9, 75, 75).astype(np.uint16)
    depth_masked = np.where(mask_eroded > 0, depth_filtered, 0)

    # 3. OPEN3D SETUP
    pinhole_intrinsics = o3d.camera.PinholeCameraIntrinsic(
        intrinsics['width'], intrinsics['height'],
        intrinsics['fx'], intrinsics['fy'],
        intrinsics['cx'], intrinsics['cy']
    )
    o3d_color = o3d.geometry.Image(cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB))
    o3d_depth = o3d.geometry.Image(depth_masked)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d_color, o3d_depth, depth_scale=1000.0, convert_rgb_to_intensity=False
    )

    # 4. PUNKTWOLKE & FILTERUNG
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_intrinsics)
    pcd = pcd.voxel_down_sample(voxel_size=0.0015) # 1.5mm Raster für Stabilität
    pcd, ind = pcd.remove_radius_outlier(nb_points=20, radius=0.01)
    
    # Normalen berechnen
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.03, max_nn=50))
    pcd.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.]))

    # 5. POISSON REKONSTRUKTION
    print("Starte Poisson Rekonstruktion (Depth 8)...")
    try:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8)
    except Exception as e:
        print(f"Absturz bei Poisson: {e}")
        return

    # 6. MESH CLEANING
    vertices_to_remove = densities < np.quantile(densities, 0.05)
    mesh.remove_vertices_by_mask(vertices_to_remove)
    
    # 7. ROTATION & ZENTRIERUNG (Wichtig für MoveIt)
    
    # Rotate Mesh around x y z
    R = mesh.get_rotation_matrix_from_xyz((0, 0, 0))
    mesh.rotate(R, center=mesh.get_center())
    print("Mesh rotated")

    # B. X/Y auf 0 zentrieren
    center = mesh.get_center()
    mesh.translate([-center[0], -center[1], 0])
    
    # C. Den Boden exakt auf Z=0 setzen
    # min_bound[2] ist der kleinste Z-Wert (der Boden der Schale)
    min_bound = mesh.get_min_bound()
    mesh.translate([0, 0, -min_bound[2]])
    print(f"Zentrierung abgeschlossen. Boden bei Z=0.")

    # 8. VEREINFACHUNG & EXPORT
    # Für MoveIt auf 5000 Facetten reduzieren
    if len(mesh.triangles) > 5000:
        mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=5000)
    
    mesh.compute_vertex_normals()
    os.makedirs(os.path.dirname(output_filename), exist_ok=True)
    o3d.io.write_triangle_mesh(output_filename, mesh)
    print(f"Erfolg! Mesh gespeichert unter: {output_filename}")

    # 9. VIEWER
    # Das Koordinatensystem zeigt uns jetzt, ob Z nach oben zeigt
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    print("\nViewer-Steuerung: Drehen mit linker Maus, Zoom mit Rad.")
    print("Die BLAUE Achse (Z) sollte nun aus der Schale nach oben zeigen.")
    o3d.visualization.draw_geometries([mesh, coord_frame], window_name="Final Bowl Model")

# --- DEINE PARAMETER ---

d435_intrinsics = {
    'width': 1280,
    'height': 720,
    'fx': 912.0184,
    'fy': 911.9789,
    'cx': 652.22,
    'cy': 377.44
}

input_dir = "/home/rric/AI_Planner/GraspingPlanner/scene_data/"
output_dir = "/home/rric/AI_Planner/GraspingPlanner/PC_2_MESH/output_data/"

if __name__ == "__main__":
    generate_final_centered_mesh(
        color_path = input_dir + "color.png",
        depth_path = input_dir + "depth.png",
        mask_path  = input_dir + "masks/bowl_green_0_first.png",
        intrinsics = d435_intrinsics,
        output_filename = output_dir + "final_bowl_perfect.stl"
    )