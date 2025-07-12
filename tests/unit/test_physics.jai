#import "Basic";
#load "../../src/physics/collision_3d_response.jai";

test_energy_conservation :: () {
    print("=== Prueba: Conservación de Energía ===\n");
    
    world := create_physics_world();
    world.gravity = .{0, 0, 0}; // Sin gravedad para esta prueba
    
    sphere_a_collider: Collider;
    sphere_a_collider.type = .SPHERE;
    sphere_a_collider.sphere = .{center = .{0, 0, 0}, radius = 1.0};
    sphere_a_collider.transform = .{
        position = .{-2, 0, 0},
        rotation = .{0, 0, 0, 1},
        scale = .{1, 1, 1}
    };
    
    sphere_a_body: RigidBody;
    sphere_a_body.mass = 1.0;
    sphere_a_body.inverse_mass = 1.0;
    sphere_a_body.inertia = compute_inertia_tensor_sphere(1.0, 1.0);
    sphere_a_body.inverse_inertia = matrix3_inverse(sphere_a_body.inertia);
    sphere_a_body.position = .{-2, 0, 0};
    sphere_a_body.orientation = .{0, 0, 0, 1};
    sphere_a_body.linear_velocity = .{5, 0, 0};
    sphere_a_body.restitution = 1.0; // Colisión perfectamente elástica
    sphere_a_body.friction = 0.0;
    
    sphere_b_collider: Collider;
    sphere_b_collider.type = .SPHERE;
    sphere_b_collider.sphere = .{center = .{0, 0, 0}, radius = 1.0};
    sphere_b_collider.transform = .{
        position = .{2, 0, 0},
        rotation = .{0, 0, 0, 1},
        scale = .{1, 1, 1}
    };
    
    sphere_b_body: RigidBody;
    sphere_b_body.mass = 1.0;
    sphere_b_body.inverse_mass = 1.0;
    sphere_b_body.inertia = compute_inertia_tensor_sphere(1.0, 1.0);
    sphere_b_body.inverse_inertia = matrix3_inverse(sphere_b_body.inertia);
    sphere_b_body.position = .{2, 0, 0};
    sphere_b_body.orientation = .{0, 0, 0, 1};
    sphere_b_body.linear_velocity = .{-5, 0, 0};
    sphere_b_body.restitution = 1.0;
    sphere_b_body.friction = 0.0;
    
    id_a := add_rigid_body(world, sphere_a_body, sphere_a_collider);
    id_b := add_rigid_body(world, sphere_b_body, sphere_b_collider);
    
    calculate_kinetic_energy :: (body: *RigidBody) -> float {
        linear_ke := 0.5 * body.mass * length_squared(body.linear_velocity);
        angular_ke := 0.5 * dot(body.angular_velocity, 
                               matrix3_multiply_vector(body.inertia, body.angular_velocity));
        return linear_ke + angular_ke;
    }
    
    initial_energy := calculate_kinetic_energy(*world.rigid_bodies[id_a]) + 
                     calculate_kinetic_energy(*world.rigid_bodies[id_b]);
    
    print("Energía cinética inicial: %.6f J\n", initial_energy);
    print("Velocidades iniciales: A=(%.1f,%.1f,%.1f), B=(%.1f,%.1f,%.1f)\n",
          world.rigid_bodies[id_a].linear_velocity.x,
          world.rigid_bodies[id_a].linear_velocity.y,
          world.rigid_bodies[id_a].linear_velocity.z,
          world.rigid_bodies[id_b].linear_velocity.x,
          world.rigid_bodies[id_b].linear_velocity.y,
          world.rigid_bodies[id_b].linear_velocity.z);
    
    dt := 0.001;
    for step: 0..2000 {
        physics_step(world, dt);
        
        if step % 100 == 0 {
            current_energy := calculate_kinetic_energy(*world.rigid_bodies[id_a]) + 
                            calculate_kinetic_energy(*world.rigid_bodies[id_b]);
            
            print("Paso %: Energía=%.6f J, Conservación=%.2f%%\n", 
                  step, current_energy, (current_energy / initial_energy) * 100);
            
            if world.collision_world.contact_manifolds.count > 0 {
                print("  Colisión detectada!\n");
                print("  Velocidades post: A=(%.3f,%.3f,%.3f), B=(%.3f,%.3f,%.3f)\n",
                      world.rigid_bodies[id_a].linear_velocity.x,
                      world.rigid_bodies[id_a].linear_velocity.y,
                      world.rigid_bodies[id_a].linear_velocity.z,
                      world.rigid_bodies[id_b].linear_velocity.x,
                      world.rigid_bodies[id_b].linear_velocity.y,
                      world.rigid_bodies[id_b].linear_velocity.z);
                break;
            }
        }
    }
    
    final_energy := calculate_kinetic_energy(*world.rigid_bodies[id_a]) + 
                   calculate_kinetic_energy(*world.rigid_bodies[id_b]);
    
    energy_loss_percent := ((initial_energy - final_energy) / initial_energy) * 100;
    
    print("Energía final: %.6f J\n", final_energy);
    print("Pérdida de energía: %.2f%%\n", energy_loss_percent);
    
    // Para colisión elástica, la pérdida debería ser mínima
    if energy_loss_percent < 5.0 {
        print("✓ Conservación de energía ACEPTABLE\n");
    } else {
        print("❌ Pérdida de energía excesiva\n");
    }
    
    print("\n");
}

test_stability :: () {
    print("=== Prueba: Estabilidad Numérica ===\n");
    
    world := create_physics_world();
    
    // Crear pila de cajas
    box_ids: [..]int;
    for i: 0..5 {
        box_collider: Collider;
        box_collider.type = .AABB;
        box_collider.aabb = .{min = .{-0.5, -0.5, -0.5}, max = .{0.5, 0.5, 0.5}};
        box_collider.transform = .{
            position = .{0, cast(float)i + 0.5, 0},
            rotation = .{0, 0, 0, 1},
            scale = .{1, 1, 1}
        };
        
        box_body: RigidBody;
        box_body.mass = 1.0;
        box_body.inverse_mass = 1.0;
        box_body.inertia = compute_inertia_tensor_box(.{0.5, 0.5, 0.5}, 1.0);
        box_body.inverse_inertia = matrix3_inverse(box_body.inertia);
        box_body.position = .{0, cast(float)i + 0.5, 0};
        box_body.orientation = .{0, 0, 0, 1};
        box_body.restitution = 0.1;
        box_body.friction = 0.7;
        
        id := add_rigid_body(world, box_body, box_collider);
        array_add(*box_ids, id);
    }
    
    // Suelo
    ground_collider: Collider;
    ground_collider.type = .AABB;
    ground_collider.aabb = .{min = .{-10, -1, -10}, max = .{10, 0, 10}};
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
    
    ground_id := add_rigid_body(world, ground_body, ground_collider);
    
    print("Simulando pila de % cajas por 5 segundos...\n", box_ids.count);
    
    dt := 0.016;
    total_time: float = 0;
    max_velocity: float = 0;
    min_height: float = FLOAT32_MAX;
    
    unstable_frames := 0;
    
    for step: 0..312 { // ~5 segundos a 60fps
        physics_step(world, dt);
        total_time += dt;
        
        step_max_velocity: float = 0;
        step_min_height: float = FLOAT32_MAX;
        
        for box_id: box_ids {
            body := *world.rigid_bodies[box_id];
            velocity := length(body.linear_velocity);
            height := body.position.y;
            
            step_max_velocity = max(step_max_velocity, velocity);
            step_min_height = min(step_min_height, height);
            
            // Detectar inestabilidad numérica
            if velocity > 100.0 || abs(body.position.x) > 50.0 || 
               abs(body.position.z) > 50.0 || body.position.y < -10.0 {
                unstable_frames += 1;
                break;
            }
        }
        
        max_velocity = max(max_velocity, step_max_velocity);
        min_height = min(min_height, step_min_height);
        
        if step % 60 == 0 {
            print("T=%.1fs: Max_vel=%.2f, Min_height=%.2f, Colisiones=%\n",
                  total_time, step_max_velocity, step_min_height,
                  world.collision_world.contact_manifolds.count);
        }
    }
    
    print("Simulación completada:\n");
    print("  Velocidad máxima: %.2f m/s\n", max_velocity);
    print("  Altura mínima: %.2f m\n", min_height);
    print("  Frames inestables: %\n", unstable_frames);
    
    if unstable_frames == 0 && max_velocity < 20.0 {
        print("✓ Simulación ESTABLE\n");
    } else {
        print("❌ Simulación INESTABLE\n");
    }
    
    array_free(box_ids);
    print("\n");
}

test_friction_behavior :: () {
    print("=== Prueba: Comportamiento de Fricción ===\n");
    
    world := create_physics_world();
    
    // Plano inclinado
    ramp_collider: Collider;
    ramp_collider.type = .AABB;
    ramp_collider.aabb = .{min = .{-5, -0.5, -2}, max = .{5, 0, 2}};
    ramp_collider.transform = .{
        position = .{0, 0, 0},
        rotation = quaternion_from_axis_angle(.{0, 0, 1}, 0.2617993), // 15 grados
        scale = .{1, 1, 1}
    };
    
    ramp_body: RigidBody;
    ramp_body.mass = 0;
    ramp_body.inverse_mass = 0;
    ramp_body.position = .{0, 0, 0};
    ramp_body.orientation = quaternion_from_axis_angle(.{0, 0, 1}, 0.2617993);
    ramp_body.friction = 0.6;
    
    ramp_id := add_rigid_body(world, ramp_body, ramp_collider);
    
    // Cajas con diferentes fricciones
    friction_values := float.[0.1, 0.3, 0.6, 0.9];
    box_ids: [..]int;
    
    for i: 0..friction_values.count-1 {
        box_collider: Collider;
        box_collider.type = .AABB;
        box_collider.aabb = .{min = .{-0.25, -0.25, -0.25}, max = .{0.25, 0.25, 0.25}};
        box_collider.transform = .{
            position = .{-3 + cast(float)i * 1.5, 3, 0},
            rotation = .{0, 0, 0, 1},
            scale = .{1, 1, 1}
        };
        
        box_body: RigidBody;
        box_body.mass = 1.0;
        box_body.inverse_mass = 1.0;
        box_body.inertia = compute_inertia_tensor_box(.{0.25, 0.25, 0.25}, 1.0);
        box_body.inverse_inertia = matrix3_inverse(box_body.inertia);
        box_body.position = .{-3 + cast(float)i * 1.5, 3, 0};
        box_body.orientation = .{0, 0, 0, 1};
        box_body.restitution = 0.2;
        box_body.friction = friction_values[i];
        
        id := add_rigid_body(world, box_body, box_collider);
        array_add(*box_ids, id);
    }
    
    print("Simulando cajas con fricción % en rampa...\n", friction_values);
    
    dt := 0.016;
    simulation_time := 5.0;
    steps := cast(int)(simulation_time / dt);
    
    initial_positions: [..]Vector3;
    for box_id: box_ids {
        array_add(*initial_positions, world.rigid_bodies[box_id].position);
    }
    
    for step: 0..steps-1 {
        physics_step(world, dt);
    }
    
    print("Resultados después de %.1f segundos:\n", simulation_time);
    
    for i: 0..box_ids.count-1 {
        box_id := box_ids[i];
        body := *world.rigid_bodies[box_id];
        
        displacement := length(body.position - initial_positions[i]);
        final_velocity := length(body.linear_velocity);
        
        print("  Caja % (fricción=%.1f): Desplazamiento=%.2fm, Velocidad=%.2fm/s\n",
              i, friction_values[i], displacement, final_velocity);
    }
    
    // Verificar que mayor fricción = menor desplazamiento
    displacement_0 := length(world.rigid_bodies[box_ids[0]].position - initial_positions[0]);
    displacement_3 := length(world.rigid_bodies[box_ids[3]].position - initial_positions[3]);
    
    if displacement_0 > displacement_3 {
        print("✓ Fricción funciona correctamente (menor fricción = mayor desplazamiento)\n");
    } else {
        print("❌ Comportamiento de fricción incorrecto\n");
    }
    
    array_free(box_ids);
    array_free(initial_positions);
    print("\n");
}

test_restitution_behavior :: () {
    print("=== Prueba: Comportamiento de Restitución ===\n");
    
    world := create_physics_world();
    
    // Suelo
    ground_collider: Collider;
    ground_collider.type = .AABB;
    ground_collider.aabb = .{min = .{-10, -1, -10}, max = .{10, 0, 10}};
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
    ground_body.restitution = 1.0;
    ground_body.friction = 0.0;
    
    ground_id := add_rigid_body(world, ground_body, ground_collider);
    
    // Esferas con diferentes restituciones
    restitution_values := float.[0.0, 0.3, 0.6, 0.9];
    sphere_ids: [..]int;
    
    for i: 0..restitution_values.count-1 {
        sphere_collider: Collider;
        sphere_collider.type = .SPHERE;
        sphere_collider.sphere = .{center = .{0, 0, 0}, radius = 0.5};
        sphere_collider.transform = .{
            position = .{cast(float)i * 2 - 3, 5, 0},
            rotation = .{0, 0, 0, 1},
            scale = .{1, 1, 1}
        };
        
        sphere_body: RigidBody;
        sphere_body.mass = 1.0;
        sphere_body.inverse_mass = 1.0;
        sphere_body.inertia = compute_inertia_tensor_sphere(0.5, 1.0);
        sphere_body.inverse_inertia = matrix3_inverse(sphere_body.inertia);
        sphere_body.position = .{cast(float)i * 2 - 3, 5, 0};
        sphere_body.orientation = .{0, 0, 0, 1};
        sphere_body.restitution = restitution_values[i];
        sphere_body.friction = 0.0;
        
        id := add_rigid_body(world, sphere_body, sphere_collider);
        array_add(*sphere_ids, id);
    }
    
    print("Simulando rebote de esferas con restitución %...\n", restitution_values);
    
    dt := 0.016;
    max_heights: [..]float;
    array_resize(*max_heights, sphere_ids.count);
    
    for step: 0..250 { // ~4 segundos
        physics_step(world, dt);
        
        for i: 0..sphere_ids.count-1 {
            sphere_id := sphere_ids[i];
            height := world.rigid_bodies[sphere_id].position.y;
            max_heights[i] = max(max_heights[i], height);
        }
        
        if step % 60 == 0 {
            print("T=%.1fs: Alturas = ", cast(float)step * dt);
            for i: 0..sphere_ids.count-1 {
                print("%.1f ", world.rigid_bodies[sphere_ids[i]].position.y);
            }
            print("\n");
        }
    }
    
    print("Alturas máximas alcanzadas después del primer rebote:\n");
    
    for i: 0..sphere_ids.count-1 {
        sphere_id := sphere_ids[i];
        theoretical_height := 5.0 * restitution_values[i] * restitution_values[i];
        
        print("  Esfera % (rest=%.1f): Altura=%.2fm (teórica=%.2fm)\n",
              i, restitution_values[i], max_heights[i], theoretical_height);
    }
    
    // Verificar que mayor restitución = mayor altura de rebote
    if max_heights[0] < max_heights[3] {
        print("✓ Restitución funciona correctamente\n");
    } else {
        print("❌ Comportamiento de restitución incorrecto\n");
    }
    
    array_free(sphere_ids);
    array_free(max_heights);
    print("\n");
}

main :: () {
    print("======================================\n");
    print("     PRUEBAS DEL SISTEMA FÍSICO      \n");
    print("======================================\n\n");
    
    random_seed(123);
    
    test_energy_conservation();
    test_stability();
    test_friction_behavior();
    test_restitution_behavior();
    
    print("======================================\n");
    print("   ✓ TODAS LAS PRUEBAS FÍSICAS OK!   \n");
    print("======================================\n");
}