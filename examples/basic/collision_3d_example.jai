#import "Basic";
#load "../../src/physics/collision_3d_ccd.jai";

main :: () {
    print("=== Sistema de Detección de Colisiones 3D ===\n\n");
    
    physics_world := create_physics_world();
    physics_world.collision_world.use_continuous_detection = true;
    
    ground_collider: Collider;
    ground_collider.type = .AABB;
    ground_collider.aabb = .{min = .{-50, -1, -50}, max = .{50, 0, 50}};
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
    
    ground_id := add_rigid_body(physics_world, ground_body, ground_collider);
    print("Suelo creado (ID: %)\n", ground_id);
    
    sphere_collider: Collider;
    sphere_collider.type = .SPHERE;
    sphere_collider.sphere = .{center = .{0, 0, 0}, radius = 1.0};
    sphere_collider.transform = .{
        position = .{0, 10, 0},
        rotation = .{0, 0, 0, 1},
        scale = .{1, 1, 1}
    };
    
    sphere_body: RigidBody;
    sphere_body.mass = 1.0;
    sphere_body.inverse_mass = 1.0;
    sphere_body.inertia = compute_inertia_tensor_sphere(1.0, 1.0);
    sphere_body.inverse_inertia = matrix3_inverse(sphere_body.inertia);
    sphere_body.position = .{0, 10, 0};
    sphere_body.orientation = .{0, 0, 0, 1};
    sphere_body.restitution = 0.8;
    sphere_body.friction = 0.3;
    
    sphere_id := add_rigid_body(physics_world, sphere_body, sphere_collider);
    print("Esfera creada (ID: %) en posición (0, 10, 0)\n", sphere_id);
    
    box_collider: Collider;
    box_collider.type = .OBB;
    box_collider.obb = .{
        center = .{0, 0, 0},
        half_extents = .{1, 1, 1},
        orientation = .{0, 0, 0, 1}
    };
    box_collider.transform = .{
        position = .{3, 5, 0},
        rotation = quaternion_from_axis_angle(.{0, 1, 0}, 0.785398),
        scale = .{1, 1, 1}
    };
    
    box_body: RigidBody;
    box_body.mass = 2.0;
    box_body.inverse_mass = 0.5;
    box_body.inertia = compute_inertia_tensor_box(.{1, 1, 1}, 2.0);
    box_body.inverse_inertia = matrix3_inverse(box_body.inertia);
    box_body.position = .{3, 5, 0};
    box_body.orientation = quaternion_from_axis_angle(.{0, 1, 0}, 0.785398);
    box_body.restitution = 0.5;
    box_body.friction = 0.4;
    
    box_id := add_rigid_body(physics_world, box_body, box_collider);
    print("Caja rotada creada (ID: %) en posición (3, 5, 0)\n", box_id);
    
    capsule_collider: Collider;
    capsule_collider.type = .CAPSULE;
    capsule_collider.capsule = .{
        point_a = .{0, -1, 0},
        point_b = .{0, 1, 0},
        radius = 0.5
    };
    capsule_collider.transform = .{
        position = .{-3, 3, 0},
        rotation = .{0, 0, 0, 1},
        scale = .{1, 1, 1}
    };
    
    capsule_body: RigidBody;
    capsule_body.mass = 1.5;
    capsule_body.inverse_mass = 1.0 / 1.5;
    capsule_body.inertia = compute_inertia_tensor_box(.{0.5, 1, 0.5}, 1.5);
    capsule_body.inverse_inertia = matrix3_inverse(capsule_body.inertia);
    capsule_body.position = .{-3, 3, 0};
    capsule_body.orientation = .{0, 0, 0, 1};
    capsule_body.restitution = 0.6;
    
    capsule_id := add_rigid_body(physics_world, capsule_body, capsule_collider);
    print("Cápsula creada (ID: %) en posición (-3, 3, 0)\n", capsule_id);
    
    print("\n=== Aplicando fuerzas iniciales ===\n");
    
    apply_impulse(*physics_world.rigid_bodies[sphere_id], .{5, 0, 0}, 
                  physics_world.rigid_bodies[sphere_id].position);
    print("Impulso aplicado a la esfera: (5, 0, 0)\n");
    
    apply_impulse(*physics_world.rigid_bodies[box_id], .{-3, 2, 0}, 
                  physics_world.rigid_bodies[box_id].position + .{1, 1, 0});
    print("Impulso aplicado a la caja: (-3, 2, 0) con torque\n");
    
    print("\n=== Simulación física (100 pasos) ===\n");
    
    dt: float = 0.016;
    total_time: float = 0;
    
    for step: 0..99 {
        physics_step_with_ccd(physics_world, dt);
        total_time += dt;
        
        if step % 10 == 0 {
            print("\nTiempo: %.2f s\n", total_time);
            print("  Esfera - Pos: (%.2f, %.2f, %.2f), Vel: (%.2f, %.2f, %.2f)\n",
                  physics_world.rigid_bodies[sphere_id].position.x,
                  physics_world.rigid_bodies[sphere_id].position.y,
                  physics_world.rigid_bodies[sphere_id].position.z,
                  physics_world.rigid_bodies[sphere_id].linear_velocity.x,
                  physics_world.rigid_bodies[sphere_id].linear_velocity.y,
                  physics_world.rigid_bodies[sphere_id].linear_velocity.z);
            
            print("  Caja   - Pos: (%.2f, %.2f, %.2f), Vel angular: (%.2f, %.2f, %.2f)\n",
                  physics_world.rigid_bodies[box_id].position.x,
                  physics_world.rigid_bodies[box_id].position.y,
                  physics_world.rigid_bodies[box_id].position.z,
                  physics_world.rigid_bodies[box_id].angular_velocity.x,
                  physics_world.rigid_bodies[box_id].angular_velocity.y,
                  physics_world.rigid_bodies[box_id].angular_velocity.z);
            
            if physics_world.collision_world.contact_manifolds.count > 0 {
                print("  Colisiones detectadas: %\n", 
                      physics_world.collision_world.contact_manifolds.count);
            }
        }
    }
    
    print("\n=== Prueba de Raycast ===\n");
    
    ray: Ray;
    ray.origin = .{0, 15, 0};
    ray.direction = normalize(.{0.2, -1, 0});
    
    hit := ray_cast(physics_world.collision_world, ray, 100.0);
    
    if hit.hit {
        print("Rayo impactó en punto: (%.2f, %.2f, %.2f)\n", 
              hit.point.x, hit.point.y, hit.point.z);
        print("Normal de impacto: (%.2f, %.2f, %.2f)\n",
              hit.normal.x, hit.normal.y, hit.normal.z);
        print("Distancia: %.2f\n", hit.distance);
    } else {
        print("El rayo no impactó ningún objeto\n");
    }
    
    print("\n=== Prueba de volumen (query espacial) ===\n");
    
    query_aabb := AABB.{min = .{-2, 0, -2}, max = .{2, 4, 2}};
    query_results: [..]int;
    
    if physics_world.collision_world.broadphase_octree {
        octree_query(physics_world.collision_world.broadphase_octree.root, 
                     query_aabb, *query_results);
        print("Objetos en región (%.1f,%.1f,%.1f) a (%.1f,%.1f,%.1f): ",
              query_aabb.min.x, query_aabb.min.y, query_aabb.min.z,
              query_aabb.max.x, query_aabb.max.y, query_aabb.max.z);
        for id: query_results {
            print("% ", id);
        }
        print("\n");
    }
    
    print("\n=== Sistema de colisiones 3D completado ===\n");
}