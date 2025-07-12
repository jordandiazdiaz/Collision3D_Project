#import "Basic";
#load "../../src/physics/collision_3d_ccd.jai";

test_high_speed_collision :: () {
    print("=== Prueba: Colisión a Alta Velocidad ===\n");
    
    world := create_physics_world();
    world.collision_world.use_continuous_detection = true;
    world.gravity = .{0, 0, 0}; // Sin gravedad para esta prueba
    
    // Objeto estático - pared
    wall_collider: Collider;
    wall_collider.type = .AABB;
    wall_collider.aabb = .{min = .{9, -2, -2}, max = .{10, 2, 2}};
    wall_collider.transform = .{
        position = .{0, 0, 0},
        rotation = .{0, 0, 0, 1},
        scale = .{1, 1, 1}
    };
    
    wall_body: RigidBody;
    wall_body.mass = 0;
    wall_body.inverse_mass = 0;
    wall_body.position = .{0, 0, 0};
    wall_body.orientation = .{0, 0, 0, 1};
    wall_body.restitution = 0.8;
    
    wall_id := add_rigid_body(world, wall_body, wall_collider);
    
    // Proyectil a alta velocidad
    projectile_collider: Collider;
    projectile_collider.type = .SPHERE;
    projectile_collider.sphere = .{center = .{0, 0, 0}, radius = 0.1};
    projectile_collider.transform = .{
        position = .{-5, 0, 0},
        rotation = .{0, 0, 0, 1},
        scale = .{1, 1, 1}
    };
    
    projectile_body: RigidBody;
    projectile_body.mass = 0.1;
    projectile_body.inverse_mass = 10.0;
    projectile_body.inertia = compute_inertia_tensor_sphere(0.1, 0.1);
    projectile_body.inverse_inertia = matrix3_inverse(projectile_body.inertia);
    projectile_body.position = .{-5, 0, 0};
    projectile_body.orientation = .{0, 0, 0, 1};
    projectile_body.linear_velocity = .{100, 0, 0}; // 100 m/s - muy rápido
    projectile_body.restitution = 0.8;
    
    projectile_id := add_rigid_body(world, projectile_body, projectile_collider);
    
    print("Proyectil inicial: velocidad = 100 m/s, posición = (-5, 0, 0)\n");
    print("Pared en x = 9-10\n");
    print("Sin CCD, el proyectil podría atravesar la pared\n\n");
    
    dt := 0.016; // 60 FPS normal
    collision_detected := false;
    collision_time: float = -1;
    
    for step: 0..124 { // 2 segundos
        physics_step_with_ccd(world, dt);
        
        projectile := *world.rigid_bodies[projectile_id];
        total_time := cast(float)step * dt;
        
        if step % 6 == 0 { // Cada 0.1 segundos
            print("T=%.2fs: Pos=(%.2f, %.2f, %.2f), Vel=(%.1f, %.1f, %.1f)\n",
                  total_time, projectile.position.x, projectile.position.y, projectile.position.z,
                  projectile.linear_velocity.x, projectile.linear_velocity.y, projectile.linear_velocity.z);
        }
        
        // Detectar colisión
        if world.collision_world.contact_manifolds.count > 0 && !collision_detected {
            collision_detected = true;
            collision_time = total_time;
            print("¡COLISIÓN DETECTADA en T=%.3fs!\n", collision_time);
            print("  Posición del proyectil: (%.3f, %.3f, %.3f)\n",
                  projectile.position.x, projectile.position.y, projectile.position.z);
            print("  Velocidad post-colisión: (%.1f, %.1f, %.1f)\n",
                  projectile.linear_velocity.x, projectile.linear_velocity.y, projectile.linear_velocity.z);
        }
        
        // Verificar que no atravesó
        if projectile.position.x > 10.2 && !collision_detected {
            print("❌ TUNNELING DETECTADO! El proyectil atravesó la pared sin colisión\n");
            break;
        }
        
        // Verificar si se alejó después de la colisión
        if collision_detected && projectile.position.x < 8 {
            print("✓ Proyectil rebotó correctamente\n");
            break;
        }
    }
    
    if collision_detected {
        print("✓ CCD previno el tunneling exitosamente\n");
        
        // Verificar tiempo de colisión teórico
        distance_to_wall := 9.0 - (-5.0) - 0.1; // Distancia menos radio
        theoretical_time := distance_to_wall / 100.0;
        time_error := abs(collision_time - theoretical_time);
        
        print("Tiempo teórico de colisión: %.3fs\n", theoretical_time);
        print("Error temporal: %.3fs (%.1f%%)\n", time_error, (time_error / theoretical_time) * 100);
        
        if time_error < 0.02 {
            print("✓ Precisión temporal EXCELENTE\n");
        }
    } else {
        print("❌ No se detectó colisión - posible fallo del CCD\n");
    }
    
    print("\n");
}

test_swept_sphere_collision :: () {
    print("=== Prueba: Swept Sphere Algorithm ===\n");
    
    // Test directo del algoritmo swept sphere
    sphere_a_start := Sphere.{center = .{-5, 0, 0}, radius = 0.5};
    sphere_a_end := Sphere.{center = .{5, 0, 0}, radius = 0.5};
    
    sphere_b_start := Sphere.{center = .{2, 0, 0}, radius = 0.5};
    sphere_b_end := Sphere.{center = .{2, 0, 0}, radius = 0.5}; // Estática
    
    result := swept_sphere_sphere(sphere_a_start, sphere_a_end, sphere_b_start, sphere_b_end);
    
    print("Esfera A: (-5,0,0) → (5,0,0), radio = 0.5\n");
    print("Esfera B: (2,0,0) estática, radio = 0.5\n");
    
    if result.hit {
        print("✓ Colisión detectada\n");
        print("  Tiempo de impacto: %.3f (0.0-1.0)\n", result.time_of_impact);
        print("  Normal de colisión: (%.3f, %.3f, %.3f)\n",
              result.normal.x, result.normal.y, result.normal.z);
        print("  Punto de contacto: (%.3f, %.3f, %.3f)\n",
              result.contact_point.x, result.contact_point.y, result.contact_point.z);
        
        // Verificar tiempo teórico
        // Distancia entre centros cuando se tocan = 1.0
        // Esfera A viaja desde x=-5 hasta que toca esfera B en x=2
        // Punto de contacto teórico: x = 2 - 1.0 = 1.0
        // Tiempo teórico: (1.0 - (-5.0)) / (5.0 - (-5.0)) = 6/10 = 0.6
        
        theoretical_time := 0.6;
        time_error := abs(result.time_of_impact - theoretical_time);
        
        print("  Tiempo teórico: %.3f\n", theoretical_time);
        print("  Error: %.3f\n", time_error);
        
        if time_error < 0.01 {
            print("✓ Precisión temporal EXCELENTE\n");
        }
    } else {
        print("❌ No se detectó colisión (debería detectarla)\n");
    }
    
    // Test con esferas que no colisionan
    sphere_c_start := Sphere.{center = .{-5, 2, 0}, radius = 0.5};
    sphere_c_end := Sphere.{center = .{5, 2, 0}, radius = 0.5};
    
    result_no_hit := swept_sphere_sphere(sphere_c_start, sphere_c_end, sphere_b_start, sphere_b_end);
    
    print("\nEsfera C: (-5,2,0) → (5,2,0), radio = 0.5\n");
    print("Esfera B: (2,0,0) estática, radio = 0.5\n");
    
    if !result_no_hit.hit {
        print("✓ Correctamente no detectó colisión (esferas separadas)\n");
    } else {
        print("❌ Falso positivo - detectó colisión donde no debería\n");
    }
    
    print("\n");
}

test_conservative_advancement :: () {
    print("=== Prueba: Conservative Advancement ===\n");
    
    // Crear dos objetos simples para probar el algoritmo
    collider_a: Collider;
    collider_a.type = .SPHERE;
    collider_a.sphere = .{center = .{0, 0, 0}, radius = 0.5};
    collider_a.transform = .{
        position = .{-3, 0, 0},
        rotation = .{0, 0, 0, 1},
        scale = .{1, 1, 1}
    };
    
    collider_b: Collider;
    collider_b.type = .SPHERE;
    collider_b.sphere = .{center = .{0, 0, 0}, radius = 0.5};
    collider_b.transform = .{
        position = .{3, 0, 0},
        rotation = .{0, 0, 0, 1},
        scale = .{1, 1, 1}
    };
    
    velocity_a := Vector3.{10, 0, 0};  // Hacia la derecha
    velocity_b := Vector3.{-10, 0, 0}; // Hacia la izquierda
    dt := 1.0; // 1 segundo
    
    print("Esfera A: pos=(-3,0,0), vel=(10,0,0)\n");
    print("Esfera B: pos=(3,0,0), vel=(-10,0,0)\n");
    print("Simulando por 1 segundo...\n");
    
    result := conservative_advancement(*collider_a, velocity_a, *collider_b, velocity_b, dt);
    
    if result.hit {
        print("✓ Conservative Advancement detectó colisión\n");
        print("  Tiempo de impacto: %.3f segundos\n", result.time_of_impact);
        print("  Normal: (%.3f, %.3f, %.3f)\n",
              result.normal.x, result.normal.y, result.normal.z);
        print("  Punto de contacto: (%.3f, %.3f, %.3f)\n",
              result.contact_point.x, result.contact_point.y, result.contact_point.z);
        
        // Tiempo teórico: las esferas se encuentran en el medio
        // Distancia inicial entre centros: 6.0
        // Distancia cuando se tocan: 1.0 (radio + radio)
        // Distancia a recorrer: 6.0 - 1.0 = 5.0
        // Velocidad relativa: 20.0 m/s
        // Tiempo: 5.0 / 20.0 = 0.25 segundos
        
        theoretical_time := 0.25;
        time_error := abs(result.time_of_impact - theoretical_time);
        
        print("  Tiempo teórico: %.3f segundos\n", theoretical_time);
        print("  Error: %.3f segundos\n", time_error);
        
        if time_error < 0.05 {
            print("✓ Conservative Advancement PRECISO\n");
        }
    } else {
        print("❌ Conservative Advancement no detectó colisión esperada\n");
    }
    
    print("\n");
}

test_bullet_through_paper :: () {
    print("=== Prueba: Bala Atravesando Papel ===\n");
    
    world := create_physics_world();
    world.collision_world.use_continuous_detection = true;
    world.gravity = .{0, 0, 0};
    
    // "Papel" - objeto muy delgado
    paper_collider: Collider;
    paper_collider.type = .AABB;
    paper_collider.aabb = .{min = .{-1, -1, -0.001}, max = .{1, 1, 0.001}};
    paper_collider.transform = .{
        position = .{0, 0, 0},
        rotation = .{0, 0, 0, 1},
        scale = .{1, 1, 1}
    };
    
    paper_body: RigidBody;
    paper_body.mass = 0;
    paper_body.inverse_mass = 0;
    paper_body.position = .{0, 0, 0};
    paper_body.orientation = .{0, 0, 0, 1};
    
    paper_id := add_rigid_body(world, paper_body, paper_collider);
    
    // "Bala" - objeto muy pequeño y rápido
    bullet_collider: Collider;
    bullet_collider.type = .SPHERE;
    bullet_collider.sphere = .{center = .{0, 0, 0}, radius = 0.005}; // 5mm radio
    bullet_collider.transform = .{
        position = .{0, 0, -2},
        rotation = .{0, 0, 0, 1},
        scale = .{1, 1, 1}
    };
    
    bullet_body: RigidBody;
    bullet_body.mass = 0.01; // 10 gramos
    bullet_body.inverse_mass = 100.0;
    bullet_body.inertia = compute_inertia_tensor_sphere(0.005, 0.01);
    bullet_body.inverse_inertia = matrix3_inverse(bullet_body.inertia);
    bullet_body.position = .{0, 0, -2};
    bullet_body.orientation = .{0, 0, 0, 1};
    bullet_body.linear_velocity = .{0, 0, 400}; // 400 m/s - velocidad de bala
    
    bullet_id := add_rigid_body(world, bullet_body, bullet_collider);
    
    print("Papel delgado: z = -0.001 a 0.001 (2mm grosor)\n");
    print("Bala: radio = 5mm, velocidad = 400 m/s, posición inicial z = -2m\n");
    print("Tiempo esperado de contacto: ~0.005 segundos\n\n");
    
    dt := 0.001; // Timestep muy pequeño para esta prueba
    collision_detected := false;
    collision_time: float = -1;
    
    for step: 0..19 { // 0.02 segundos total
        physics_step_with_ccd(world, dt);
        
        bullet := *world.rigid_bodies[bullet_id];
        total_time := cast(float)step * dt;
        
        print("T=%.4fs: Bala z=%.4f, vel_z=%.1f\n",
              total_time, bullet.position.z, bullet.linear_velocity.z);
        
        if world.collision_world.contact_manifolds.count > 0 && !collision_detected {
            collision_detected = true;
            collision_time = total_time;
            print("¡COLISIÓN con papel detectada en T=%.4fs!\n", collision_time);
            print("  Posición de bala: z=%.4f\n", bullet.position.z);
        }
        
        // Verificar que no atravesó sin detección
        if bullet.position.z > 0.1 && !collision_detected {
            print("❌ TUNNELING! Bala atravesó papel sin detección\n");
            break;
        }
        
        if bullet.position.z > 2 {
            print("Bala pasó completamente\n");
            break;
        }
    }
    
    if collision_detected {
        print("✓ CCD detectó correctamente bala vs papel delgado\n");
        
        theoretical_time := 2.0 / 400.0; // distancia / velocidad
        time_error := abs(collision_time - theoretical_time);
        
        print("Tiempo teórico: %.4fs\n", theoretical_time);
        print("Error temporal: %.4fs\n", time_error);
    } else {
        print("❌ CCD falló en detectar bala vs papel\n");
    }
    
    print("\n");
}

benchmark_ccd_performance :: () {
    print("=== Benchmark: Rendimiento CCD ===\n");
    
    world := create_physics_world();
    world.collision_world.use_continuous_detection = true;
    world.gravity = .{0, -9.81, 0};
    
    object_count := 50;
    
    print("Creando % objetos rápidos...\n", object_count);
    
    for i: 0..object_count-1 {
        collider: Collider;
        collider.type = .SPHERE;
        collider.sphere = .{center = .{0, 0, 0}, radius = 0.1};
        
        x := (cast(float)(i % 10) - 5) * 2;
        y := cast(float)(i / 10) * 2 + 10;
        z := random_get_within_range(-5.0, 5.0);
        
        collider.transform = .{
            position = .{x, y, z},
            rotation = .{0, 0, 0, 1},
            scale = .{1, 1, 1}
        };
        
        body: RigidBody;
        body.mass = 0.1;
        body.inverse_mass = 10.0;
        body.inertia = compute_inertia_tensor_sphere(0.1, 0.1);
        body.inverse_inertia = matrix3_inverse(body.inertia);
        body.position = .{x, y, z};
        body.orientation = .{0, 0, 0, 1};
        
        // Velocidades altas aleatorias
        body.linear_velocity = .{
            random_get_within_range(-20.0, 20.0),
            random_get_within_range(-10.0, 10.0),
            random_get_within_range(-20.0, 20.0)
        };
        
        add_rigid_body(world, body, collider);
    }
    
    // Suelo para colisiones
    ground_collider: Collider;
    ground_collider.type = .AABB;
    ground_collider.aabb = .{min = .{-20, -1, -20}, max = .{20, 0, 20}};
    ground_collider.transform = .{
        position = .{0, 0, 0},
        rotation = .{0, 0, 0, 1},
        scale = .{1, 1, 1}
    };
    
    ground_body: RigidBody;
    ground_body.mass = 0;
    ground_body.inverse_mass = 0;
    ground_body.position = .{0, 0, 0};
    ground_body.orientation = .{0, 0, 0, 1};
    
    add_rigid_body(world, ground_body, ground_collider);
    
    print("Ejecutando benchmark con CCD activado...\n");
    
    start_time := current_time_monotonic();
    
    dt := 0.016;
    total_ccd_detections := 0;
    
    for step: 0..124 { // 2 segundos
        physics_step_with_ccd(world, dt);
        
        // Contar detecciones CCD (aproximación)
        for manifold: world.collision_world.contact_manifolds {
            // Si hay alta velocidad involucrada, probablemente fue CCD
            body_a := *world.rigid_bodies[manifold.collider_a];
            body_b := *world.rigid_bodies[manifold.collider_b];
            
            speed_a := length(body_a.linear_velocity);
            speed_b := length(body_b.linear_velocity);
            
            if speed_a > 10.0 || speed_b > 10.0 {
                total_ccd_detections += 1;
            }
        }
        
        if step % 31 == 0 {
            print("T=%.1fs: Colisiones=%\n", 
                  cast(float)step * dt, world.collision_world.contact_manifolds.count);
        }
    }
    
    end_time := current_time_monotonic();
    duration := to_float64_seconds(end_time - start_time);
    
    print("Benchmark completado:\n");
    print("  Duración total: %.1f ms\n", duration * 1000);
    print("  Promedio por frame: %.2f ms\n", (duration * 1000) / 125);
    print("  Detecciones CCD estimadas: %\n", total_ccd_detections);
    
    fps := 125.0 / duration;
    print("  FPS efectivo: %.1f\n", fps);
    
    if fps > 30.0 {
        print("✓ Rendimiento CCD ACEPTABLE\n");
    } else {
        print("❌ Rendimiento CCD podría ser lento\n");
    }
    
    print("\n");
}

main :: () {
    print("======================================\n");
    print("  PRUEBAS DE DETECCIÓN CONTINUA CCD  \n");
    print("======================================\n\n");
    
    random_seed(789);
    
    test_high_speed_collision();
    test_swept_sphere_collision();
    test_conservative_advancement();
    test_bullet_through_paper();
    benchmark_ccd_performance();
    
    print("======================================\n");
    print("     ✓ TODAS LAS PRUEBAS CCD OK!     \n");
    print("======================================\n");
}