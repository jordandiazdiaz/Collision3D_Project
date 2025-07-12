#import "Basic";
#load "../../src/physics/collision_3d_response.jai";

BilliardBall :: struct {
    id: int;
    number: int;
    color: Vector3;
    is_moving: bool;
}

create_billiard_table :: (world: *PhysicsWorld) -> int {
    // Mesa de billar (superficie de juego)
    table_collider: Collider;
    table_collider.type = .AABB;
    table_collider.aabb = .{min = .{-1.27, -0.05, -0.635}, max = .{1.27, 0, 0.635}};
    table_collider.transform = .{
        position = .{0, 0, 0},
        rotation = .{0, 0, 0, 1},
        scale = .{1, 1, 1}
    };
    
    table_body: RigidBody;
    table_body.mass = 0;
    table_body.inverse_mass = 0;
    table_body.position = .{0, 0, 0};
    table_body.orientation = .{0, 0, 0, 1};
    table_body.restitution = 0.9;
    table_body.friction = 0.02; // Mesa muy lisa
    
    return add_rigid_body(world, table_body, table_collider);
}

create_table_rails :: (world: *PhysicsWorld) {
    // Bandas de la mesa
    rail_height := 0.1;
    
    // Banda superior
    rail_collider: Collider;
    rail_collider.type = .AABB;
    rail_collider.aabb = .{min = .{-1.3, 0, 0.635}, max = .{1.3, rail_height, 0.685}};
    rail_collider.transform = .{
        position = .{0, 0, 0},
        rotation = .{0, 0, 0, 1},
        scale = .{1, 1, 1}
    };
    
    rail_body: RigidBody;
    rail_body.mass = 0;
    rail_body.inverse_mass = 0;
    rail_body.position = .{0, 0, 0};
    rail_body.orientation = .{0, 0, 0, 1};
    rail_body.restitution = 0.95;
    rail_body.friction = 0.1;
    
    add_rigid_body(world, rail_body, rail_collider);
    
    // Banda inferior
    rail_collider.aabb = .{min = .{-1.3, 0, -0.685}, max = .{1.3, rail_height, -0.635}};
    add_rigid_body(world, rail_body, rail_collider);
    
    // Banda izquierda
    rail_collider.aabb = .{min = .{-1.3, 0, -0.685}, max = .{-1.27, rail_height, 0.685}};
    add_rigid_body(world, rail_body, rail_collider);
    
    // Banda derecha
    rail_collider.aabb = .{min = .{1.27, 0, -0.685}, max = .{1.3, rail_height, 0.685}};
    add_rigid_body(world, rail_body, rail_collider);
}

create_billiard_ball :: (world: *PhysicsWorld, position: Vector3, number: int) -> BilliardBall {
    ball_radius := 0.028; // Radio estándar de bola de billar
    
    ball_collider: Collider;
    ball_collider.type = .SPHERE;
    ball_collider.sphere = .{center = .{0, 0, 0}, radius = ball_radius};
    ball_collider.transform = .{
        position = position,
        rotation = .{0, 0, 0, 1},
        scale = .{1, 1, 1}
    };
    
    ball_body: RigidBody;
    ball_body.mass = 0.17; // Masa estándar de bola de billar en kg
    ball_body.inverse_mass = 1.0 / 0.17;
    ball_body.inertia = compute_inertia_tensor_sphere(ball_radius, 0.17);
    ball_body.inverse_inertia = matrix3_inverse(ball_body.inertia);
    ball_body.position = position;
    ball_body.orientation = .{0, 0, 0, 1};
    ball_body.restitution = 0.95;
    ball_body.friction = 0.05;
    
    id := add_rigid_body(world, ball_body, ball_collider);
    
    // Colores de las bolas
    colors := Vector3.[
        .{1, 1, 1},     // 0 - Blanca
        .{1, 1, 0},     // 1 - Amarilla
        .{0, 0, 1},     // 2 - Azul
        .{1, 0, 0},     // 3 - Roja
        .{0.5, 0, 0.5}, // 4 - Púrpura
        .{1, 0.5, 0},   // 5 - Naranja
        .{0, 0.5, 0},   // 6 - Verde
        .{0.5, 0.2, 0}, // 7 - Marrón
        .{0, 0, 0}      // 8 - Negra
    ];
    
    ball: BilliardBall;
    ball.id = id;
    ball.number = number;
    ball.color = colors[min(number, 8)];
    ball.is_moving = false;
    
    return ball;
}

setup_triangle_rack :: (world: *PhysicsWorld, balls: *[..]BilliardBall) {
    ball_radius := 0.028;
    spacing := ball_radius * 2.01; // Pequeño espacio entre bolas
    
    // Posición inicial del triángulo
    start_x := 0.5;
    start_z := 0.0;
    
    // Configuración del triángulo (5 filas)
    row_counts := int.[1, 2, 3, 4, 5];
    ball_number := 1;
    
    for row: 0..4 {
        balls_in_row := row_counts[row];
        row_start_z := start_z - (cast(float)(balls_in_row - 1) * spacing * 0.5);
        
        for col: 0..balls_in_row-1 {
            position := Vector3.{
                start_x + cast(float)row * spacing * 0.866, // Distancia entre filas (sqrt(3)/2)
                ball_radius + 0.001,
                row_start_z + cast(float)col * spacing
            };
            
            ball := create_billiard_ball(world, position, ball_number);
            array_add(balls, ball);
            
            ball_number += 1;
            if ball_number > 15 break;
        }
        if ball_number > 15 break;
    }
}

calculate_total_momentum :: (world: *PhysicsWorld, balls: []BilliardBall) -> Vector3 {
    total_momentum := Vector3.{0, 0, 0};
    
    for ball: balls {
        body := *world.rigid_bodies[ball.id];
        momentum := body.linear_velocity * body.mass;
        total_momentum = total_momentum + momentum;
    }
    
    return total_momentum;
}

calculate_total_energy :: (world: *PhysicsWorld, balls: []BilliardBall) -> float {
    total_energy: float = 0;
    
    for ball: balls {
        body := *world.rigid_bodies[ball.id];
        
        // Energía cinética lineal
        linear_ke := 0.5 * body.mass * length_squared(body.linear_velocity);
        
        // Energía cinética rotacional
        angular_ke := 0.5 * dot(body.angular_velocity, 
                               matrix3_multiply_vector(body.inertia, body.angular_velocity));
        
        total_energy += linear_ke + angular_ke;
    }
    
    return total_energy;
}

count_moving_balls :: (world: *PhysicsWorld, balls: []BilliardBall, threshold: float = 0.01) -> int {
    moving_count := 0;
    
    for ball: balls {
        body := *world.rigid_bodies[ball.id];
        speed := length(body.linear_velocity);
        
        if speed > threshold {
            moving_count += 1;
        }
    }
    
    return moving_count;
}

simulate_break_shot :: (world: *PhysicsWorld, cue_ball: *BilliardBall, power: float, angle: float) {
    body := *world.rigid_bodies[cue_ball.id];
    
    // Calcular velocidad inicial basada en potencia y ángulo
    velocity := Vector3.{
        cos(angle) * power,
        0,
        sin(angle) * power
    };
    
    body.linear_velocity = velocity;
    
    // Aplicar algo de spin hacia atrás (efecto común en el break)
    body.angular_velocity = .{0, 0, -power * 0.5};
    
    print("Break shot! Velocidad: (%.1f, %.1f, %.1f) m/s\n",
          velocity.x, velocity.y, velocity.z);
}

main :: () {
    print("======================================\n");
    print("      SIMULACIÓN DE BILLAR 3D        \n");
    print("======================================\n\n");
    
    world := create_physics_world();
    world.gravity = .{0, -9.81, 0};
    world.constraint_iterations = 8; // Mayor precisión para billar
    
    print("Creando mesa de billar...\n");
    table_id := create_billiard_table(world);
    create_table_rails(world);
    
    print("Configurando bolas en triángulo...\n");
    balls: [..]BilliardBall;
    setup_triangle_rack(world, *balls);
    
    print("Creando bola blanca (cue ball)...\n");
    cue_position := Vector3.{-0.5, 0.029, 0};
    cue_ball := create_billiard_ball(world, cue_position, 0);
    
    print("Total de bolas en mesa: %\n", balls.count + 1);
    
    // Configuración del tiro de break
    break_power := 8.0; // m/s
    break_angle := 0.0;  // radianes (tiro recto)
    
    print("\nEjecutando break shot...\n");
    simulate_break_shot(world, *cue_ball, break_power, break_angle);
    
    // Mediciones iniciales
    initial_momentum := calculate_total_momentum(world, balls);
    initial_energy := calculate_total_energy(world, balls);
    cue_body := *world.rigid_bodies[cue_ball.id];
    initial_energy += 0.5 * cue_body.mass * length_squared(cue_body.linear_velocity);
    
    print("Momento inicial: (%.3f, %.3f, %.3f) kg⋅m/s\n",
          initial_momentum.x, initial_momentum.y, initial_momentum.z);
    print("Energía inicial: %.3f J\n", initial_energy);
    
    print("\n=== Iniciando simulación ===\n");
    
    dt := 0.008; // Timestep pequeño para mayor precisión
    total_time: float = 0;
    collision_count := 0;
    last_collision_count := 0;
    
    for step: 0..1249 { // 10 segundos de simulación
        physics_step_with_ccd(world, dt);
        total_time += dt;
        
        collision_count = world.collision_world.contact_manifolds.count;
        
        if step % 62 == 0 { // Cada 0.5 segundos
            moving_balls := count_moving_balls(world, balls);
            
            current_momentum := calculate_total_momentum(world, balls);
            cue_body = *world.rigid_bodies[cue_ball.id];
            current_momentum = current_momentum + cue_body.linear_velocity * cue_body.mass;
            
            current_energy := calculate_total_energy(world, balls);
            cue_body = *world.rigid_bodies[cue_ball.id];
            current_energy += 0.5 * cue_body.mass * length_squared(cue_body.linear_velocity);
            
            energy_retention := (current_energy / initial_energy) * 100;
            
            print("T=%.1fs: Bolas moviendo=%/%, Energía=%.1f%%, Colisiones=%\n",
                  total_time, moving_balls, balls.count + 1, energy_retention, collision_count);
            
            if collision_count > last_collision_count {
                print("  ¡Nuevas colisiones detectadas!\n");
            }
            last_collision_count = collision_count;
            
            // Mostrar posición de algunas bolas importantes
            print("  Bola blanca: (%.2f, %.2f, %.2f)\n",
                  world.rigid_bodies[cue_ball.id].position.x,
                  world.rigid_bodies[cue_ball.id].position.y,
                  world.rigid_bodies[cue_ball.id].position.z);
            
            if balls.count > 0 {
                print("  Bola 1: (%.2f, %.2f, %.2f)\n",
                      world.rigid_bodies[balls[0].id].position.x,
                      world.rigid_bodies[balls[0].id].position.y,
                      world.rigid_bodies[balls[0].id].position.z);
            }
            
            // Verificar si la simulación se ha estabilizado
            if moving_balls == 0 {
                print("\n¡Todas las bolas se han detenido!\n");
                break;
            }
        }
    }
    
    print("\n=== Resultados finales ===\n");
    
    final_momentum := calculate_total_momentum(world, balls);
    cue_body = *world.rigid_bodies[cue_ball.id];
    final_momentum = final_momentum + cue_body.linear_velocity * cue_body.mass;
    
    final_energy := calculate_total_energy(world, balls);
    cue_body = *world.rigid_bodies[cue_ball.id];
    final_energy += 0.5 * cue_body.mass * length_squared(cue_body.linear_velocity);
    
    momentum_change := length(final_momentum - initial_momentum);
    energy_loss := ((initial_energy - final_energy) / initial_energy) * 100;
    
    print("Momento final: (%.3f, %.3f, %.3f) kg⋅m/s\n",
          final_momentum.x, final_momentum.y, final_momentum.z);
    print("Cambio en momento: %.3f kg⋅m/s\n", momentum_change);
    print("Energía final: %.3f J\n", final_energy);
    print("Pérdida de energía: %.1f%%\n", energy_loss);
    print("Total de contactos detectados: %\n", collision_count);
    
    // Análisis de distribución final
    print("\nPosiciones finales de las bolas:\n");
    print("Bola blanca (0): (%.2f, %.2f, %.2f)\n",
          world.rigid_bodies[cue_ball.id].position.x,
          world.rigid_bodies[cue_ball.id].position.y,
          world.rigid_bodies[cue_ball.id].position.z);
    
    for i: 0..min(balls.count-1, 7) {
        ball := balls[i];
        pos := world.rigid_bodies[ball.id].position;
        print("Bola %: (%.2f, %.2f, %.2f)\n", ball.number, pos.x, pos.y, pos.z);
    }
    
    // Verificar realismo físico
    if energy_loss > 10.0 && energy_loss < 90.0 && momentum_change < 1.0 {
        print("\n✓ Simulación de billar REALISTA\n");
    } else {
        print("\n❌ Simulación podría no ser realista\n");
    }
    
    array_free(balls);
    
    print("\n======================================\n");
    print("    SIMULACIÓN DE BILLAR COMPLETA    \n");
    print("======================================\n");
}