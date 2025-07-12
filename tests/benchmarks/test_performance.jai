#import "Basic";
#load "../../src/collision_3d_system.jai";

Timer :: struct {
    start_time: Apollo_Time;
    name: string;
}

start_timer :: (name: string) -> Timer {
    timer: Timer;
    timer.name = name;
    timer.start_time = current_time_monotonic();
    return timer;
}

end_timer :: (timer: Timer) -> float64 {
    end_time := current_time_monotonic();
    duration := to_float64_seconds(end_time - timer.start_time);
    print("⏱️  %: %.3f ms\n", timer.name, duration * 1000.0);
    return duration;
}

benchmark_sphere_sphere :: (iterations: int) {
    print("=== Benchmark: Detección Esfera-Esfera ===\n");
    
    sphere_a := Sphere.{center = .{0, 0, 0}, radius = 1.0};
    sphere_b := Sphere.{center = .{1.5, 0, 0}, radius = 1.0};
    
    timer := start_timer("Esfera-Esfera x" + tprint("%", iterations));
    
    collision_count := 0;
    for i: 0..iterations-1 {
        info := test_sphere_sphere(sphere_a, sphere_b);
        if info.has_collision collision_count += 1;
    }
    
    duration := end_timer(timer);
    
    print("Colisiones detectadas: %/%\n", collision_count, iterations);
    print("Promedio por colisión: %.3f μs\n\n", (duration * 1000000.0) / cast(float64)iterations);
}

benchmark_aabb_aabb :: (iterations: int) {
    print("=== Benchmark: Detección AABB-AABB ===\n");
    
    aabb_a := AABB.{min = .{-1, -1, -1}, max = .{1, 1, 1}};
    aabb_b := AABB.{min = .{0.5, -1, -1}, max = .{2.5, 1, 1}};
    
    timer := start_timer("AABB-AABB x" + tprint("%", iterations));
    
    collision_count := 0;
    for i: 0..iterations-1 {
        info := test_aabb_aabb(aabb_a, aabb_b);
        if info.has_collision collision_count += 1;
    }
    
    duration := end_timer(timer);
    
    print("Colisiones detectadas: %/%\n", collision_count, iterations);
    print("Promedio por colisión: %.3f μs\n\n", (duration * 1000000.0) / cast(float64)iterations);
}

benchmark_obb_obb :: (iterations: int) {
    print("=== Benchmark: Detección OBB-OBB ===\n");
    
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
    
    timer := start_timer("OBB-OBB x" + tprint("%", iterations));
    
    collision_count := 0;
    for i: 0..iterations-1 {
        info := test_obb_obb(obb_a, obb_b);
        if info.has_collision collision_count += 1;
    }
    
    duration := end_timer(timer);
    
    print("Colisiones detectadas: %/%\n", collision_count, iterations);
    print("Promedio por colisión: %.3f μs\n\n", (duration * 1000000.0) / cast(float64)iterations);
}

stress_test_many_objects :: (object_count: int) {
    print("=== Stress Test: % Objetos ===\n", object_count);
    
    world := create_physics_world();
    
    timer := start_timer("Creación de objetos");
    
    for i: 0..object_count-1 {
        x := (cast(float)(i % 20) - 10) * 2;
        y := (cast(float)(i / 400) + 1) * 3;
        z := (cast(float)((i / 20) % 20) - 10) * 2;
        
        collider: Collider;
        collider.type = .SPHERE;
        collider.sphere = .{center = .{0, 0, 0}, radius = 0.5};
        collider.transform = .{
            position = .{x, y, z},
            rotation = .{0, 0, 0, 1},
            scale = .{1, 1, 1}
        };
        
        body: RigidBody;
        body.mass = 1.0;
        body.inverse_mass = 1.0;
        body.position = .{x, y, z};
        body.orientation = .{0, 0, 0, 1};
        
        add_rigid_body(world, body, collider);
    }
    
    end_timer(timer);
    
    timer = start_timer("Broadphase (BVH)");
    update_broadphase(world.collision_world);
    end_timer(timer);
    
    print("Pares de colisión potenciales: %\n", world.collision_world.collision_pairs.count);
    
    timer = start_timer("Detección de colisiones");
    detect_collisions(world.collision_world);
    end_timer(timer);
    
    print("Colisiones reales detectadas: %\n", world.collision_world.contact_manifolds.count);
    
    timer = start_timer("Paso de física completo");
    physics_step(world, 0.016);
    end_timer(timer);
    
    print("\n");
}

benchmark_raycast :: (ray_count: int, target_count: int) {
    print("=== Benchmark: Raycast ===\n");
    
    world := create_collision_world();
    
    for i: 0..target_count-1 {
        x := (cast(float)(i % 10) - 5) * 4;
        y := (cast(float)((i / 10) % 10)) * 2;
        z := (cast(float)(i / 100) - 5) * 4;
        
        collider: Collider;
        collider.type = .SPHERE;
        collider.sphere = .{center = .{0, 0, 0}, radius = 1.0};
        collider.transform = .{
            position = .{x, y, z},
            rotation = .{0, 0, 0, 1},
            scale = .{1, 1, 1}
        };
        
        add_collider(world, collider);
    }
    
    rays: [..]Ray;
    array_reserve(*rays, ray_count);
    
    for i: 0..ray_count-1 {
        ray: Ray;
        ray.origin = .{
            (cast(float)(i % 20) - 10) * 2,
            20,
            (cast(float)(i / 20) - 10) * 2
        };
        ray.direction = normalize(.{
            sin(cast(float)i * 0.1) * 0.2,
            -1,
            cos(cast(float)i * 0.1) * 0.2
        });
        array_add(*rays, ray);
    }
    
    timer := start_timer(tprint("Raycast % rayos vs % objetos", ray_count, target_count));
    
    hit_count := 0;
    for ray: rays {
        hit := ray_cast(world, ray, 100.0);
        if hit.hit hit_count += 1;
    }
    
    duration := end_timer(timer);
    
    print("Hits detectados: %/%\n", hit_count, ray_count);
    print("Promedio por rayo: %.3f μs\n\n", (duration * 1000000.0) / cast(float64)ray_count);
    
    array_free(rays);
}

memory_usage_test :: (object_count: int) {
    print("=== Test: Uso de Memoria ===\n");
    
    // Nota: En Jai no hay una forma estándar de medir memoria como en C/C++
    // Este test muestra el patrón para implementar medición de memoria
    
    world := create_physics_world();
    
    print("Creando % objetos...\n", object_count);
    
    for i: 0..object_count-1 {
        x := random_get_within_range(-50.0, 50.0);
        y := random_get_within_range(1.0, 20.0);
        z := random_get_within_range(-50.0, 50.0);
        
        collider: Collider;
        collider.type = .SPHERE;
        collider.sphere = .{center = .{0, 0, 0}, radius = 0.5};
        collider.transform = .{
            position = .{x, y, z},
            rotation = .{0, 0, 0, 1},
            scale = .{1, 1, 1}
        };
        
        body: RigidBody;
        body.mass = 1.0;
        body.inverse_mass = 1.0;
        body.position = .{x, y, z};
        body.orientation = .{0, 0, 0, 1};
        
        add_rigid_body(world, body, collider);
    }
    
    print("Objetos en CollisionWorld: %\n", world.collision_world.colliders.count);
    print("Nodos BVH: %\n", world.collision_world.broadphase_bvh.nodes.count);
    print("Nodos BVH libres: %\n", world.collision_world.broadphase_bvh.free_nodes.count);
    
    print("Memoria estimada por objeto: ~% bytes\n", 
          size_of(RigidBody) + size_of(Collider) + size_of(BVHNode));
    
    print("\n");
}

comparative_broadphase_benchmark :: (object_count: int) {
    print("=== Benchmark Comparativo: Broadphase Algorithms ===\n");
    
    positions: [..]Vector3;
    array_reserve(*positions, object_count);
    
    for i: 0..object_count-1 {
        pos := Vector3.{
            random_get_within_range(-100.0, 100.0),
            random_get_within_range(-100.0, 100.0),
            random_get_within_range(-100.0, 100.0)
        };
        array_add(*positions, pos);
    }
    
    // Test BVH
    {
        bvh := create_bvh();
        timer := start_timer("BVH - Inserción");
        
        for i: 0..positions.count-1 {
            aabb := AABB.{
                min = positions[i] - Vector3.{1, 1, 1},
                max = positions[i] + Vector3.{1, 1, 1}
            };
            bvh_insert(bvh, aabb, i);
        }
        
        end_timer(timer);
        
        timer = start_timer("BVH - Query de pares");
        pairs: [..]BroadphasePair;
        bvh_query_pairs(bvh, *pairs);
        end_timer(timer);
        
        print("BVH - Pares encontrados: %\n", pairs.count);
        array_free(pairs);
    }
    
    // Test Octree
    {
        bounds := AABB.{min = .{-150, -150, -150}, max = .{150, 150, 150}};
        octree := create_octree(bounds, 6);
        
        timer := start_timer("Octree - Inserción");
        
        for i: 0..positions.count-1 {
            octree_insert(octree, octree.root, i, positions[i]);
        }
        
        end_timer(timer);
        
        timer = start_timer("Octree - Query de región");
        results: [..]int;
        query_region := AABB.{min = .{-50, -50, -50}, max = .{50, 50, 50}};
        octree_query(octree.root, query_region, *results);
        end_timer(timer);
        
        print("Octree - Objetos en región: %\n", results.count);
        array_free(results);
    }
    
    // Test Spatial Hash
    {
        grid := create_spatial_hash(10.0);
        
        timer := start_timer("Spatial Hash - Inserción");
        
        for i: 0..positions.count-1 {
            aabb := AABB.{
                min = positions[i] - Vector3.{1, 1, 1},
                max = positions[i] + Vector3.{1, 1, 1}
            };
            spatial_hash_insert(grid, aabb, i);
        }
        
        end_timer(timer);
        
        timer = start_timer("Spatial Hash - Query de región");
        results: [..]int;
        query_region := AABB.{min = .{-25, -25, -25}, max = .{25, 25, 25}};
        spatial_hash_query(grid, query_region, *results);
        end_timer(timer);
        
        print("Spatial Hash - Objetos en región: %\n", results.count);
        array_free(results);
    }
    
    array_free(positions);
    print("\n");
}

main :: () {
    print("==========================================\n");
    print("     BENCHMARKS DE RENDIMIENTO 3D        \n");
    print("==========================================\n\n");
    
    // Configurar random seed para resultados consistentes
    random_seed(12345);
    
    benchmark_sphere_sphere(100000);
    benchmark_aabb_aabb(100000);
    benchmark_obb_obb(50000);
    
    stress_test_many_objects(100);
    stress_test_many_objects(500);
    stress_test_many_objects(1000);
    
    benchmark_raycast(1000, 100);
    benchmark_raycast(100, 1000);
    
    memory_usage_test(2000);
    
    comparative_broadphase_benchmark(1000);
    comparative_broadphase_benchmark(5000);
    
    print("==========================================\n");
    print("      BENCHMARKS COMPLETADOS!            \n");
    print("==========================================\n");
}