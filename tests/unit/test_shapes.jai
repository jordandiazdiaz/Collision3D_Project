#import "Basic";
#load "../../src/collision_3d_system.jai";

test_sphere_sphere_collision :: () {
    print("=== Prueba: Colisión Esfera-Esfera ===\n");
    
    sphere_a := Sphere.{center = .{0, 0, 0}, radius = 1.0};
    sphere_b := Sphere.{center = .{1.5, 0, 0}, radius = 1.0};
    sphere_c := Sphere.{center = .{3.0, 0, 0}, radius = 1.0};
    
    info_ab := test_sphere_sphere(sphere_a, sphere_b);
    info_ac := test_sphere_sphere(sphere_a, sphere_c);
    
    print("Esfera A vs B (overlapping): %\n", info_ab.has_collision);
    if info_ab.has_collision {
        print("  Penetración: %.3f\n", info_ab.penetration_depth);
        print("  Normal: (%.3f, %.3f, %.3f)\n", 
              info_ab.normal.x, info_ab.normal.y, info_ab.normal.z);
    }
    
    print("Esfera A vs C (separadas): %\n", info_ac.has_collision);
    
    assert(info_ab.has_collision == true, "Esferas overlapping deberían colisionar");
    assert(info_ac.has_collision == false, "Esferas separadas no deberían colisionar");
    assert(abs(info_ab.penetration_depth - 0.5) < 0.001, "Penetración incorrecta");
    
    print("✓ Prueba esfera-esfera PASADA\n\n");
}

test_aabb_aabb_collision :: () {
    print("=== Prueba: Colisión AABB-AABB ===\n");
    
    aabb_a := AABB.{min = .{-1, -1, -1}, max = .{1, 1, 1}};
    aabb_b := AABB.{min = .{0.5, -1, -1}, max = .{2.5, 1, 1}};
    aabb_c := AABB.{min = .{2, -1, -1}, max = .{4, 1, 1}};
    
    info_ab := test_aabb_aabb(aabb_a, aabb_b);
    info_ac := test_aabb_aabb(aabb_a, aabb_c);
    
    print("AABB A vs B (overlapping): %\n", info_ab.has_collision);
    if info_ab.has_collision {
        print("  Penetración: %.3f\n", info_ab.penetration_depth);
        print("  Normal: (%.3f, %.3f, %.3f)\n",
              info_ab.normal.x, info_ab.normal.y, info_ab.normal.z);
    }
    
    print("AABB A vs C (separadas): %\n", info_ac.has_collision);
    
    assert(info_ab.has_collision == true, "AABBs overlapping deberían colisionar");
    assert(info_ac.has_collision == false, "AABBs separadas no deberían colisionar");
    assert(abs(info_ab.penetration_depth - 0.5) < 0.001, "Penetración AABB incorrecta");
    
    print("✓ Prueba AABB-AABB PASADA\n\n");
}

test_sphere_aabb_collision :: () {
    print("=== Prueba: Colisión Esfera-AABB ===\n");
    
    sphere := Sphere.{center = .{2, 0, 0}, radius = 1.0};
    aabb_a := AABB.{min = .{0, -1, -1}, max = .{2, 1, 1}};
    aabb_b := AABB.{min = .{4, -1, -1}, max = .{6, 1, 1}};
    
    info_a := test_sphere_aabb(sphere, aabb_a);
    info_b := test_sphere_aabb(sphere, aabb_b);
    
    print("Esfera vs AABB A (tocando): %\n", info_a.has_collision);
    if info_a.has_collision {
        print("  Penetración: %.3f\n", info_a.penetration_depth);
        if info_a.contact_points.count > 0 {
            print("  Punto de contacto: (%.3f, %.3f, %.3f)\n",
                  info_a.contact_points[0].x, info_a.contact_points[0].y, info_a.contact_points[0].z);
        }
    }
    
    print("Esfera vs AABB B (separadas): %\n", info_b.has_collision);
    
    assert(info_a.has_collision == true, "Esfera tocando AABB debería colisionar");
    assert(info_b.has_collision == false, "Esfera separada de AABB no debería colisionar");
    
    print("✓ Prueba Esfera-AABB PASADA\n\n");
}

test_obb_obb_collision :: () {
    print("=== Prueba: Colisión OBB-OBB ===\n");
    
    obb_a := OBB.{
        center = .{0, 0, 0},
        half_extents = .{1, 1, 1},
        orientation = .{0, 0, 0, 1}
    };
    
    obb_b := OBB.{
        center = .{1.5, 0, 0},
        half_extents = .{1, 1, 1},
        orientation = quaternion_from_axis_angle(.{0, 0, 1}, 0.785398)
    };
    
    obb_c := OBB.{
        center = .{4, 0, 0},
        half_extents = .{1, 1, 1},
        orientation = .{0, 0, 0, 1}
    };
    
    info_ab := test_obb_obb(obb_a, obb_b);
    info_ac := test_obb_obb(obb_a, obb_c);
    
    print("OBB A vs B (rotado, overlapping): %\n", info_ab.has_collision);
    if info_ab.has_collision {
        print("  Penetración: %.3f\n", info_ab.penetration_depth);
        print("  Normal: (%.3f, %.3f, %.3f)\n",
              info_ab.normal.x, info_ab.normal.y, info_ab.normal.z);
    }
    
    print("OBB A vs C (separadas): %\n", info_ac.has_collision);
    
    assert(info_ac.has_collision == false, "OBBs separadas no deberían colisionar");
    
    print("✓ Prueba OBB-OBB PASADA\n\n");
}

test_capsule_capsule_collision :: () {
    print("=== Prueba: Colisión Cápsula-Cápsula ===\n");
    
    capsule_a := Capsule.{
        point_a = .{0, -1, 0},
        point_b = .{0, 1, 0},
        radius = 0.5
    };
    
    capsule_b := Capsule.{
        point_a = .{0.8, -1, 0},
        point_b = .{0.8, 1, 0},
        radius = 0.5
    };
    
    capsule_c := Capsule.{
        point_a = .{2, -1, 0},
        point_b = .{2, 1, 0},
        radius = 0.5
    };
    
    info_ab := test_capsule_capsule(capsule_a, capsule_b);
    info_ac := test_capsule_capsule(capsule_a, capsule_c);
    
    print("Cápsula A vs B (overlapping): %\n", info_ab.has_collision);
    if info_ab.has_collision {
        print("  Penetración: %.3f\n", info_ab.penetration_depth);
        print("  Normal: (%.3f, %.3f, %.3f)\n",
              info_ab.normal.x, info_ab.normal.y, info_ab.normal.z);
    }
    
    print("Cápsula A vs C (separadas): %\n", info_ac.has_collision);
    
    assert(info_ab.has_collision == true, "Cápsulas overlapping deberían colisionar");
    assert(info_ac.has_collision == false, "Cápsulas separadas no deberían colisionar");
    assert(abs(info_ab.penetration_depth - 0.2) < 0.001, "Penetración cápsula incorrecta");
    
    print("✓ Prueba Cápsula-Cápsula PASADA\n\n");
}

test_edge_cases :: () {
    print("=== Prueba: Casos Extremos ===\n");
    
    sphere_tiny := Sphere.{center = .{0, 0, 0}, radius = 0.001};
    sphere_huge := Sphere.{center = .{0, 0, 0}, radius = 1000.0};
    sphere_normal := Sphere.{center = .{0.0005, 0, 0}, radius = 0.001};
    
    info_tiny := test_sphere_sphere(sphere_tiny, sphere_normal);
    print("Esferas muy pequeñas overlapping: %\n", info_tiny.has_collision);
    
    info_huge := test_sphere_sphere(sphere_huge, sphere_normal);
    print("Esfera huge vs normal: %\n", info_huge.has_collision);
    
    aabb_degenerate := AABB.{min = .{0, 0, 0}, max = .{0, 0, 0}};
    aabb_point := AABB.{min = .{0, 0, 0}, max = .{0.001, 0.001, 0.001}};
    
    info_degenerate := test_aabb_aabb(aabb_degenerate, aabb_point);
    print("AABB degenerada vs punto: %\n", info_degenerate.has_collision);
    
    print("✓ Prueba Casos Extremos PASADA\n\n");
}

test_transform_consistency :: () {
    print("=== Prueba: Consistencia de Transformaciones ===\n");
    
    original_sphere := Sphere.{center = .{0, 0, 0}, radius = 1.0};
    
    transform: Transform;
    transform.position = .{5, 3, -2};
    transform.rotation = quaternion_from_axis_angle(.{0, 1, 0}, 1.57079632);
    transform.scale = .{2, 1, 1};
    
    transformed := transform_sphere(original_sphere, transform);
    
    print("Esfera original: centro=(%.1f,%.1f,%.1f), radio=%.1f\n",
          original_sphere.center.x, original_sphere.center.y, original_sphere.center.z,
          original_sphere.radius);
    
    print("Esfera transformada: centro=(%.1f,%.1f,%.1f), radio=%.1f\n",
          transformed.center.x, transformed.center.y, transformed.center.z,
          transformed.radius);
    
    expected_center := Vector3.{5, 3, -2};
    expected_radius := 2.0;
    
    assert(length(transformed.center - expected_center) < 0.001, "Centro transformado incorrecto");
    assert(abs(transformed.radius - expected_radius) < 0.001, "Radio transformado incorrecto");
    
    print("✓ Prueba Transformaciones PASADA\n\n");
}

main :: () {
    print("======================================\n");
    print("  PRUEBAS DEL SISTEMA DE COLISIONES  \n");
    print("======================================\n\n");
    
    test_sphere_sphere_collision();
    test_aabb_aabb_collision();
    test_sphere_aabb_collision();
    test_obb_obb_collision();
    test_capsule_capsule_collision();
    test_edge_cases();
    test_transform_consistency();
    
    print("======================================\n");
    print("  ✓ TODAS LAS PRUEBAS PASARON!       \n");
    print("======================================\n");
}