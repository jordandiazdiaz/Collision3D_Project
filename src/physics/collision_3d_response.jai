#import "Basic";
#load "../collision_3d_system.jai";

PhysicsWorld :: struct {
    collision_world: *CollisionWorld;
    rigid_bodies: [..]RigidBody;
    gravity: Vector3 = .{0, -9.81, 0};
    
    constraint_iterations: int = 4;
    penetration_slop: float = 0.01;
    penetration_correction_percent: float = 0.8;
}

create_physics_world :: () -> *PhysicsWorld {
    world := New(PhysicsWorld);
    world.collision_world = create_collision_world();
    return world;
}

add_rigid_body :: (world: *PhysicsWorld, body: RigidBody, collider: Collider) -> int {
    body_id := world.rigid_bodies.count;
    array_add(*world.rigid_bodies, body);
    
    collider_id := add_collider(world.collision_world, collider, body.inverse_mass == 0);
    assert(body_id == collider_id);
    
    return body_id;
}

compute_inertia_tensor_box :: (half_extents: Vector3, mass: float) -> Matrix3 {
    inertia: Matrix3;
    
    xx := half_extents.x * half_extents.x * 4;
    yy := half_extents.y * half_extents.y * 4;
    zz := half_extents.z * half_extents.z * 4;
    
    inertia.m[0][0] = mass * (yy + zz) / 12.0;
    inertia.m[1][1] = mass * (xx + zz) / 12.0;
    inertia.m[2][2] = mass * (xx + yy) / 12.0;
    
    return inertia;
}

compute_inertia_tensor_sphere :: (radius: float, mass: float) -> Matrix3 {
    inertia: Matrix3;
    
    i := 0.4 * mass * radius * radius;
    inertia.m[0][0] = i;
    inertia.m[1][1] = i;
    inertia.m[2][2] = i;
    
    return inertia;
}

matrix3_inverse :: (m: Matrix3) -> Matrix3 {
    det := m.m[0][0] * (m.m[1][1] * m.m[2][2] - m.m[1][2] * m.m[2][1]) -
           m.m[0][1] * (m.m[1][0] * m.m[2][2] - m.m[1][2] * m.m[2][0]) +
           m.m[0][2] * (m.m[1][0] * m.m[2][1] - m.m[1][1] * m.m[2][0]);
    
    inv_det := 1.0 / det;
    result: Matrix3;
    
    result.m[0][0] = (m.m[1][1] * m.m[2][2] - m.m[1][2] * m.m[2][1]) * inv_det;
    result.m[0][1] = (m.m[0][2] * m.m[2][1] - m.m[0][1] * m.m[2][2]) * inv_det;
    result.m[0][2] = (m.m[0][1] * m.m[1][2] - m.m[0][2] * m.m[1][1]) * inv_det;
    
    result.m[1][0] = (m.m[1][2] * m.m[2][0] - m.m[1][0] * m.m[2][2]) * inv_det;
    result.m[1][1] = (m.m[0][0] * m.m[2][2] - m.m[0][2] * m.m[2][0]) * inv_det;
    result.m[1][2] = (m.m[0][2] * m.m[1][0] - m.m[0][0] * m.m[1][2]) * inv_det;
    
    result.m[2][0] = (m.m[1][0] * m.m[2][1] - m.m[1][1] * m.m[2][0]) * inv_det;
    result.m[2][1] = (m.m[0][1] * m.m[2][0] - m.m[0][0] * m.m[2][1]) * inv_det;
    result.m[2][2] = (m.m[0][0] * m.m[1][1] - m.m[0][1] * m.m[1][0]) * inv_det;
    
    return result;
}

matrix3_multiply_vector :: (m: Matrix3, v: Vector3) -> Vector3 {
    return .{
        m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z,
        m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z,
        m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z
    };
}

integrate_forces :: (world: *PhysicsWorld, dt: float) {
    for i: 0..world.rigid_bodies.count-1 {
        body := *world.rigid_bodies[i];
        
        if body.inverse_mass == 0 continue;
        
        body.linear_velocity = body.linear_velocity + (world.gravity + body.force * body.inverse_mass) * dt;
        body.angular_velocity = body.angular_velocity + matrix3_multiply_vector(body.inverse_inertia, body.torque) * dt;
        
        body.force = .{0, 0, 0};
        body.torque = .{0, 0, 0};
    }
}

integrate_velocities :: (world: *PhysicsWorld, dt: float) {
    for i: 0..world.rigid_bodies.count-1 {
        body := *world.rigid_bodies[i];
        collider := *world.collision_world.colliders[i];
        
        if body.inverse_mass == 0 continue;
        
        body.position = body.position + body.linear_velocity * dt;
        
        angular_velocity_quat := Quaternion.{
            body.angular_velocity.x * 0.5,
            body.angular_velocity.y * 0.5,
            body.angular_velocity.z * 0.5,
            0
        };
        
        body.orientation = quaternion_multiply(body.orientation, angular_velocity_quat);
        body.orientation = normalize_quaternion(body.orientation);
        
        collider.transform.position = body.position;
        collider.transform.rotation = body.orientation;
    }
}

normalize_quaternion :: (q: Quaternion) -> Quaternion {
    len := sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if len < EPSILON return .{0, 0, 0, 1};
    return .{q.x / len, q.y / len, q.z / len, q.w / len};
}

resolve_collision :: (world: *PhysicsWorld, manifold: *ContactManifold) {
    body_a := *world.rigid_bodies[manifold.collider_a];
    body_b := *world.rigid_bodies[manifold.collider_b];
    
    if body_a.inverse_mass == 0 && body_b.inverse_mass == 0 return;
    
    for i: 0..manifold.contact_count-1 {
        contact := *manifold.contacts[i];
        
        ra := contact.position - body_a.position;
        rb := contact.position - body_b.position;
        
        relative_velocity := body_b.linear_velocity + cross(body_b.angular_velocity, rb) -
                           body_a.linear_velocity - cross(body_a.angular_velocity, ra);
        
        velocity_along_normal := dot(relative_velocity, manifold.normal);
        
        if velocity_along_normal > 0 continue;
        
        e := min(body_a.restitution, body_b.restitution);
        
        ra_cross_n := cross(ra, manifold.normal);
        rb_cross_n := cross(rb, manifold.normal);
        
        inverse_mass_sum := body_a.inverse_mass + body_b.inverse_mass +
            dot(matrix3_multiply_vector(body_a.inverse_inertia, ra_cross_n), ra_cross_n) +
            dot(matrix3_multiply_vector(body_b.inverse_inertia, rb_cross_n), rb_cross_n);
        
        j := -(1.0 + e) * velocity_along_normal / inverse_mass_sum;
        j /= cast(float)manifold.contact_count;
        
        impulse := manifold.normal * j;
        
        body_a.linear_velocity = body_a.linear_velocity - impulse * body_a.inverse_mass;
        body_a.angular_velocity = body_a.angular_velocity - 
            matrix3_multiply_vector(body_a.inverse_inertia, cross(ra, impulse));
        
        body_b.linear_velocity = body_b.linear_velocity + impulse * body_b.inverse_mass;
        body_b.angular_velocity = body_b.angular_velocity + 
            matrix3_multiply_vector(body_b.inverse_inertia, cross(rb, impulse));
        
        tangent := relative_velocity - manifold.normal * velocity_along_normal;
        tangent_length := length(tangent);
        
        if tangent_length > EPSILON {
            tangent = tangent * (1.0 / tangent_length);
            
            velocity_along_tangent := dot(relative_velocity, tangent);
            
            ra_cross_t := cross(ra, tangent);
            rb_cross_t := cross(rb, tangent);
            
            inverse_mass_sum_tangent := body_a.inverse_mass + body_b.inverse_mass +
                dot(matrix3_multiply_vector(body_a.inverse_inertia, ra_cross_t), ra_cross_t) +
                dot(matrix3_multiply_vector(body_b.inverse_inertia, rb_cross_t), rb_cross_t);
            
            jt := -velocity_along_tangent / inverse_mass_sum_tangent;
            jt /= cast(float)manifold.contact_count;
            
            mu := sqrt(body_a.friction * body_b.friction);
            
            friction_impulse: Vector3;
            if abs(jt) < j * mu {
                friction_impulse = tangent * jt;
            } else {
                friction_impulse = tangent * (-j * mu);
            }
            
            body_a.linear_velocity = body_a.linear_velocity - friction_impulse * body_a.inverse_mass;
            body_a.angular_velocity = body_a.angular_velocity - 
                matrix3_multiply_vector(body_a.inverse_inertia, cross(ra, friction_impulse));
            
            body_b.linear_velocity = body_b.linear_velocity + friction_impulse * body_b.inverse_mass;
            body_b.angular_velocity = body_b.angular_velocity + 
                matrix3_multiply_vector(body_b.inverse_inertia, cross(rb, friction_impulse));
        }
    }
}

resolve_penetration :: (world: *PhysicsWorld, manifold: *ContactManifold) {
    body_a := *world.rigid_bodies[manifold.collider_a];
    body_b := *world.rigid_bodies[manifold.collider_b];
    
    if body_a.inverse_mass == 0 && body_b.inverse_mass == 0 return;
    
    max_penetration: float = 0;
    for i: 0..manifold.contact_count-1 {
        max_penetration = max(max_penetration, manifold.contacts[i].penetration);
    }
    
    correction_amount := max(max_penetration - world.penetration_slop, 0.0) * 
                       world.penetration_correction_percent;
    
    total_inverse_mass := body_a.inverse_mass + body_b.inverse_mass;
    correction := manifold.normal * (correction_amount / total_inverse_mass);
    
    body_a.position = body_a.position - correction * body_a.inverse_mass;
    body_b.position = body_b.position + correction * body_b.inverse_mass;
    
    world.collision_world.colliders[manifold.collider_a].transform.position = body_a.position;
    world.collision_world.colliders[manifold.collider_b].transform.position = body_b.position;
}

physics_step :: (world: *PhysicsWorld, dt: float) {
    integrate_forces(world, dt);
    
    detect_collisions(world.collision_world);
    
    for i: 0..world.constraint_iterations-1 {
        for manifold: *world.collision_world.contact_manifolds {
            resolve_collision(world, manifold);
        }
    }
    
    for manifold: *world.collision_world.contact_manifolds {
        resolve_penetration(world, manifold);
    }
    
    integrate_velocities(world, dt);
}

apply_force :: (body: *RigidBody, force: Vector3, world_point: Vector3) {
    body.force = body.force + force;
    body.torque = body.torque + cross(world_point - body.position, force);
}

apply_impulse :: (body: *RigidBody, impulse: Vector3, world_point: Vector3) {
    body.linear_velocity = body.linear_velocity + impulse * body.inverse_mass;
    body.angular_velocity = body.angular_velocity + 
        matrix3_multiply_vector(body.inverse_inertia, cross(world_point - body.position, impulse));
}