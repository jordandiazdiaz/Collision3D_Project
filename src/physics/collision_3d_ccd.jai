#import "Basic";
#load "../physics/collision_3d_response.jai";

CCDResult :: struct {
    hit: bool;
    time_of_impact: float;
    normal: Vector3;
    contact_point: Vector3;
}

swept_sphere_sphere :: (a_start: Sphere, a_end: Sphere, b_start: Sphere, b_end: Sphere) -> CCDResult {
    result: CCDResult;
    
    va := a_end.center - a_start.center;
    vb := b_end.center - b_start.center;
    
    relative_velocity := va - vb;
    relative_position := a_start.center - b_start.center;
    
    radius_sum := a_start.radius + b_start.radius;
    
    a := dot(relative_velocity, relative_velocity);
    b := 2.0 * dot(relative_position, relative_velocity);
    c := dot(relative_position, relative_position) - radius_sum * radius_sum;
    
    discriminant := b * b - 4 * a * c;
    
    if discriminant < 0 || a < EPSILON {
        return result;
    }
    
    sqrt_discriminant := sqrt(discriminant);
    t1 := (-b - sqrt_discriminant) / (2 * a);
    t2 := (-b + sqrt_discriminant) / (2 * a);
    
    t := t1;
    if t < 0 || t > 1 {
        t = t2;
        if t < 0 || t > 1 {
            return result;
        }
    }
    
    result.hit = true;
    result.time_of_impact = t;
    
    a_pos := a_start.center + va * t;
    b_pos := b_start.center + vb * t;
    
    result.normal = normalize(a_pos - b_pos);
    result.contact_point = b_pos + result.normal * b_start.radius;
    
    return result;
}

ray_vs_moving_sphere :: (ray: Ray, sphere_start: Sphere, sphere_velocity: Vector3, max_time: float) -> CCDResult {
    result: CCDResult;
    
    relative_velocity := ray.direction * length(ray.direction) - sphere_velocity;
    relative_position := ray.origin - sphere_start.center;
    
    a := dot(relative_velocity, relative_velocity);
    b := 2.0 * dot(relative_position, relative_velocity);
    c := dot(relative_position, relative_position) - sphere_start.radius * sphere_start.radius;
    
    discriminant := b * b - 4 * a * c;
    
    if discriminant < 0 || a < EPSILON {
        return result;
    }
    
    sqrt_discriminant := sqrt(discriminant);
    t1 := (-b - sqrt_discriminant) / (2 * a);
    t2 := (-b + sqrt_discriminant) / (2 * a);
    
    t := t1;
    if t < 0 || t > max_time {
        t = t2;
        if t < 0 || t > max_time {
            return result;
        }
    }
    
    result.hit = true;
    result.time_of_impact = t;
    
    sphere_pos := sphere_start.center + sphere_velocity * t;
    ray_pos := ray.origin + ray.direction * t;
    
    result.normal = normalize(ray_pos - sphere_pos);
    result.contact_point = sphere_pos + result.normal * sphere_start.radius;
    
    return result;
}

conservative_advancement :: (collider_a: *Collider, vel_a: Vector3, collider_b: *Collider, vel_b: Vector3, dt: float) -> CCDResult {
    result: CCDResult;
    
    time_left := dt;
    accumulated_time: float = 0;
    
    max_iterations := 32;
    tolerance := 0.001;
    
    for i: 0..max_iterations-1 {
        aabb_a := get_collider_aabb(collider_a);
        aabb_b := get_collider_aabb(collider_b);
        
        expanded_aabb_a := expand_aabb_by_velocity(aabb_a, vel_a, time_left);
        expanded_aabb_b := expand_aabb_by_velocity(aabb_b, vel_b, time_left);
        
        if !test_aabb_aabb(expanded_aabb_a, expanded_aabb_b).has_collision {
            return result;
        }
        
        info := test_collider_pair(collider_a, collider_b);
        
        if info.has_collision {
            result.hit = true;
            result.time_of_impact = accumulated_time / dt;
            result.normal = info.normal;
            if info.contact_points.count > 0 {
                result.contact_point = info.contact_points[0];
            }
            return result;
        }
        
        closest_distance := compute_closest_distance(collider_a, collider_b);
        relative_velocity := vel_a - vel_b;
        relative_speed := length(relative_velocity);
        
        if relative_speed < EPSILON {
            return result;
        }
        
        step_time := closest_distance / relative_speed;
        step_time = min(step_time, time_left);
        
        if step_time < tolerance {
            result.hit = true;
            result.time_of_impact = (accumulated_time + step_time) / dt;
            
            advance_collider(collider_a, vel_a, step_time);
            advance_collider(collider_b, vel_b, step_time);
            
            final_info := test_collider_pair(collider_a, collider_b);
            result.normal = final_info.normal;
            if final_info.contact_points.count > 0 {
                result.contact_point = final_info.contact_points[0];
            }
            
            advance_collider(collider_a, vel_a, -step_time);
            advance_collider(collider_b, vel_b, -step_time);
            
            return result;
        }
        
        advance_collider(collider_a, vel_a, step_time);
        advance_collider(collider_b, vel_b, step_time);
        
        accumulated_time += step_time;
        time_left -= step_time;
    }
    
    return result;
}

expand_aabb_by_velocity :: (aabb: AABB, velocity: Vector3, dt: float) -> AABB {
    displacement := velocity * dt;
    result := aabb;
    
    if displacement.x > 0 {
        result.max.x += displacement.x;
    } else {
        result.min.x += displacement.x;
    }
    
    if displacement.y > 0 {
        result.max.y += displacement.y;
    } else {
        result.min.y += displacement.y;
    }
    
    if displacement.z > 0 {
        result.max.z += displacement.z;
    } else {
        result.min.z += displacement.z;
    }
    
    return result;
}

compute_closest_distance :: (collider_a: *Collider, collider_b: *Collider) -> float {
    if collider_a.type == .SPHERE && collider_b.type == .SPHERE {
        sphere_a := transform_sphere(collider_a.sphere, collider_a.transform);
        sphere_b := transform_sphere(collider_b.sphere, collider_b.transform);
        
        distance := length(sphere_b.center - sphere_a.center) - sphere_a.radius - sphere_b.radius;
        return max(0, distance);
    }
    
    return 0.1;
}

advance_collider :: (collider: *Collider, velocity: Vector3, dt: float) {
    collider.transform.position = collider.transform.position + velocity * dt;
}

physics_step_with_ccd :: (world: *PhysicsWorld, dt: float) {
    integrate_forces(world, dt);
    
    if world.collision_world.use_continuous_detection {
        ccd_pairs: [..]struct {a: int; b: int; toi: float;};
        
        for i: 0..world.rigid_bodies.count-1 {
            for j: i+1..world.rigid_bodies.count-1 {
                body_a := *world.rigid_bodies[i];
                body_b := *world.rigid_bodies[j];
                
                if body_a.inverse_mass == 0 && body_b.inverse_mass == 0 continue;
                
                speed_a := length(body_a.linear_velocity);
                speed_b := length(body_b.linear_velocity);
                
                collider_a := *world.collision_world.colliders[i];
                collider_b := *world.collision_world.colliders[j];
                
                aabb_a := get_collider_aabb(collider_a);
                aabb_b := get_collider_aabb(collider_b);
                
                size_a := length(aabb_a.max - aabb_a.min);
                size_b := length(aabb_b.max - aabb_b.min);
                
                threshold_a := size_a * 0.5;
                threshold_b := size_b * 0.5;
                
                if speed_a * dt > threshold_a || speed_b * dt > threshold_b {
                    ccd_result := conservative_advancement(
                        collider_a, body_a.linear_velocity,
                        collider_b, body_b.linear_velocity,
                        dt
                    );
                    
                    if ccd_result.hit {
                        array_add(*ccd_pairs, .{a = i, b = j, toi = ccd_result.time_of_impact});
                    }
                }
            }
        }
        
        sort_ccd_pairs :: (a: type_of(ccd_pairs[0]), b: type_of(ccd_pairs[0])) -> bool {
            return a.toi < b.toi;
        }
        
        if ccd_pairs.count > 0 {
            for pair: ccd_pairs {
                body_a := *world.rigid_bodies[pair.a];
                body_b := *world.rigid_bodies[pair.b];
                collider_a := *world.collision_world.colliders[pair.a];
                collider_b := *world.collision_world.colliders[pair.b];
                
                body_a.position = body_a.position + body_a.linear_velocity * (dt * pair.toi);
                body_b.position = body_b.position + body_b.linear_velocity * (dt * pair.toi);
                
                collider_a.transform.position = body_a.position;
                collider_b.transform.position = body_b.position;
                
                manifold: ContactManifold;
                manifold.collider_a = pair.a;
                manifold.collider_b = pair.b;
                
                info := test_collider_pair(collider_a, collider_b);
                if info.has_collision {
                    manifold.normal = info.normal;
                    manifold.contact_count = min(info.contact_points.count, 4);
                    for i: 0..manifold.contact_count-1 {
                        manifold.contacts[i].position = info.contact_points[i];
                        manifold.contacts[i].penetration = info.penetration_depth;
                    }
                    
                    resolve_collision(world, *manifold);
                }
                
                body_a.position = body_a.position - body_a.linear_velocity * (dt * pair.toi);
                body_b.position = body_b.position - body_b.linear_velocity * (dt * pair.toi);
            }
        }
        
        array_free(ccd_pairs);
    }
    
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