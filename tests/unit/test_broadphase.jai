#import "Basic";
#load "../../src/broadphase/collision_3d_broadphase.jai";

test_bvh_integrity :: () {
    print("=== Prueba: Integridad del BVH ===\n");
    
    bvh := create_bvh();
    nodes: [..]BVHNode;
    
    object_count := 20;
    
    for i: 0..object_count-1 {
        aabb := AABB.{
            min = .{cast(float)i * 2, 0, 0},
            max = .{cast(float)i * 2 + 1, 1, 1}
        };
        
        node := bvh_insert(bvh, aabb, i);
        array_add(*nodes, <<node);
    }
    
    print("Objetos insertados en BVH: %\n", object_count);
    print("Nodos totales: %\n", bvh.nodes.count);
    
    validate_bvh_node :: (node: *BVHNode, depth: int = 0) -> bool {
        if !node return true;
        
        if node.is_leaf {
            return node.left == null && node.right == null && node.collider_id >= 0;
        } else {
            if !node.left || !node.right {
                print("❌ Nodo no-hoja sin hijos en profundidad %\n", depth);
                return false;
            }
            
            if node.left.parent != node || node.right.parent != node {
                print("❌ Enlaces de padre incorrectos en profundidad %\n", depth);
                return false;
            }
            
            merged := merge_aabb(node.left.aabb, node.right.aabb);
            if abs(merged.min.x - node.aabb.min.x) > 0.001 ||
               abs(merged.min.y - node.aabb.min.y) > 0.001 ||
               abs(merged.min.z - node.aabb.min.z) > 0.001 ||
               abs(merged.max.x - node.aabb.max.x) > 0.001 ||
               abs(merged.max.y - node.aabb.max.y) > 0.001 ||
               abs(merged.max.z - node.aabb.max.z) > 0.001 {
                print("❌ AABB del padre no coincide con merged de hijos en profundidad %\n", depth);
                return false;
            }
            
            return validate_bvh_node(node.left, depth + 1) && 
                   validate_bvh_node(node.right, depth + 1);
        }
    }
    
    is_valid := validate_bvh_node(bvh.root);
    if is_valid {
        print("✓ BVH es válido\n");
    } else {
        print("❌ BVH tiene errores de integridad\n");
    }
    
    print("Probando queries de pares...\n");
    pairs: [..]BroadphasePair;
    bvh_query_pairs(bvh, *pairs);
    print("Pares de colisión potenciales: %\n", pairs.count);
    
    expected_pairs := 0;
    for i: 0..object_count-1 {
        for j: i+1..object_count-1 {
            aabb_i := AABB.{
                min = .{cast(float)i * 2, 0, 0},
                max = .{cast(float)i * 2 + 1, 1, 1}
            };
            aabb_j := AABB.{
                min = .{cast(float)j * 2, 0, 0},
                max = .{cast(float)j * 2 + 1, 1, 1}
            };
            
            if test_aabb_aabb(aabb_i, aabb_j).has_collision {
                expected_pairs += 1;
            }
        }
    }
    
    print("Pares esperados (fuerza bruta): %\n", expected_pairs);
    
    assert(pairs.count == expected_pairs, "BVH no encontró todos los pares");
    
    print("Probando remoción de nodos...\n");
    for i: 0..5 {
        bvh_remove(bvh, *nodes[i]);
    }
    
    is_valid_after_removal := validate_bvh_node(bvh.root);
    if is_valid_after_removal {
        print("✓ BVH sigue válido después de remociones\n");
    } else {
        print("❌ BVH se corrompió después de remociones\n");
    }
    
    array_free(pairs);
    array_free(nodes);
    
    print("✓ Prueba de integridad BVH PASADA\n\n");
}

test_octree_subdivision :: () {
    print("=== Prueba: Subdivisión de Octree ===\n");
    
    bounds := AABB.{min = .{-16, -16, -16}, max = .{16, 16, 16}};
    octree := create_octree(bounds, 3);
    
    object_positions: [..]Vector3;
    
    for i: 0..20 {
        pos := Vector3.{
            random_get_within_range(-15.0, 15.0),
            random_get_within_range(-15.0, 15.0),
            random_get_within_range(-15.0, 15.0)
        };
        array_add(*object_positions, pos);
        octree_insert(octree, octree.root, i, pos);
    }
    
    print("Objetos insertados: %\n", object_positions.count);
    
    count_nodes :: (node: *OctreeNode) -> int {
        if !node return 0;
        
        count := 1;
        for i: 0..7 {
            if node.children[i] {
                count += count_nodes(node.children[i]);
            }
        }
        return count;
    }
    
    total_nodes := count_nodes(octree.root);
    print("Nodos totales del Octree: %\n", total_nodes);
    
    check_objects_in_bounds :: (node: *OctreeNode, global_positions: []Vector3) -> bool {
        for obj_id: node.objects {
            pos := global_positions[obj_id];
            if pos.x < node.bounds.min.x || pos.x > node.bounds.max.x ||
               pos.y < node.bounds.min.y || pos.y > node.bounds.max.y ||
               pos.z < node.bounds.min.z || pos.z > node.bounds.max.z {
                print("❌ Objeto % fuera de bounds del nodo\n", obj_id);
                return false;
            }
        }
        
        for i: 0..7 {
            if node.children[i] {
                if !check_objects_in_bounds(node.children[i], global_positions) {
                    return false;
                }
            }
        }
        
        return true;
    }
    
    bounds_valid := check_objects_in_bounds(octree.root, object_positions);
    if bounds_valid {
        print("✓ Todos los objetos están en los bounds correctos\n");
    } else {
        print("❌ Algunos objetos están mal ubicados\n");
    }
    
    print("Probando query en región...\n");
    query_region := AABB.{min = .{-5, -5, -5}, max = .{5, 5, 5}};
    results: [..]int;
    octree_query(octree.root, query_region, *results);
    
    print("Objetos en región central: %\n", results.count);
    
    expected_in_region := 0;
    for i: 0..object_positions.count-1 {
        pos := object_positions[i];
        if pos.x >= query_region.min.x && pos.x <= query_region.max.x &&
           pos.y >= query_region.min.y && pos.y <= query_region.max.y &&
           pos.z >= query_region.min.z && pos.z <= query_region.max.z {
            expected_in_region += 1;
        }
    }
    
    print("Objetos esperados en región: %\n", expected_in_region);
    
    found_all := true;
    for i: 0..object_positions.count-1 {
        pos := object_positions[i];
        if pos.x >= query_region.min.x && pos.x <= query_region.max.x &&
           pos.y >= query_region.min.y && pos.y <= query_region.max.y &&
           pos.z >= query_region.min.z && pos.z <= query_region.max.z {
            
            found := false;
            for result_id: results {
                if result_id == i {
                    found = true;
                    break;
                }
            }
            
            if !found {
                print("❌ Objeto % debería estar en results pero no está\n", i);
                found_all = false;
            }
        }
    }
    
    if found_all {
        print("✓ Octree query encontró todos los objetos correctos\n");
    }
    
    array_free(object_positions);
    array_free(results);
    
    print("✓ Prueba de Octree PASADA\n\n");
}

test_spatial_hash_grid :: () {
    print("=== Prueba: Spatial Hash Grid ===\n");
    
    grid := create_spatial_hash(5.0);
    
    objects: [..]AABB;
    
    for i: 0..30 {
        pos := Vector3.{
            random_get_within_range(-50.0, 50.0),
            random_get_within_range(-50.0, 50.0),
            random_get_within_range(-50.0, 50.0)
        };
        
        aabb := AABB.{
            min = pos - Vector3.{1, 1, 1},
            max = pos + Vector3.{1, 1, 1}
        };
        
        array_add(*objects, aabb);
        spatial_hash_insert(grid, aabb, i);
    }
    
    print("Objetos insertados en grid: %\n", objects.count);
    print("Celdas ocupadas: %\n", grid.cells.count);
    
    print("Probando query en región...\n");
    query_region := AABB.{min = .{-10, -10, -10}, max = .{10, 10, 10}};
    results: [..]int;
    spatial_hash_query(grid, query_region, *results);
    
    print("Objetos encontrados en región: %\n", results.count);
    
    expected_overlaps := 0;
    for i: 0..objects.count-1 {
        if test_aabb_aabb(objects[i], query_region).has_collision {
            expected_overlaps += 1;
        }
    }
    
    print("Overlaps esperados: %\n", expected_overlaps);
    
    all_overlaps_found := true;
    for i: 0..objects.count-1 {
        if test_aabb_aabb(objects[i], query_region).has_collision {
            found := false;
            for result_id: results {
                if result_id == i {
                    found = true;
                    break;
                }
            }
            
            if !found {
                print("❌ Overlap no detectado para objeto %\n", i);
                all_overlaps_found = false;
            }
        }
    }
    
    if all_overlaps_found {
        print("✓ Spatial Hash Grid encontró todos los overlaps\n");
    }
    
    array_free(objects);
    array_free(results);
    
    print("✓ Prueba de Spatial Hash Grid PASADA\n\n");
}

benchmark_broadphase_scaling :: () {
    print("=== Benchmark: Escalabilidad de Broadphase ===\n");
    
    test_sizes := int.[100, 500, 1000, 2000];
    
    for size: test_sizes {
        print("\n--- Prueba con % objetos ---\n", size);
        
        positions: [..]Vector3;
        array_reserve(*positions, size);
        
        for i: 0..size-1 {
            pos := Vector3.{
                random_get_within_range(-100.0, 100.0),
                random_get_within_range(-100.0, 100.0),
                random_get_within_range(-100.0, 100.0)
            };
            array_add(*positions, pos);
        }
        
        // BVH Test
        {
            bvh := create_bvh();
            start_time := current_time_monotonic();
            
            for i: 0..positions.count-1 {
                aabb := AABB.{
                    min = positions[i] - Vector3.{1, 1, 1},
                    max = positions[i] + Vector3.{1, 1, 1}
                };
                bvh_insert(bvh, aabb, i);
            }
            
            insert_time := current_time_monotonic();
            
            pairs: [..]BroadphasePair;
            bvh_query_pairs(bvh, *pairs);
            
            query_time := current_time_monotonic();
            
            insert_duration := to_float64_seconds(insert_time - start_time);
            query_duration := to_float64_seconds(query_time - insert_time);
            
            print("BVH - Inserción: %.2f ms, Query: %.2f ms, Pares: %\n",
                  insert_duration * 1000, query_duration * 1000, pairs.count);
            
            array_free(pairs);
        }
        
        // Octree Test
        {
            bounds := AABB.{min = .{-120, -120, -120}, max = .{120, 120, 120}};
            octree := create_octree(bounds, 6);
            
            start_time := current_time_monotonic();
            
            for i: 0..positions.count-1 {
                octree_insert(octree, octree.root, i, positions[i]);
            }
            
            insert_time := current_time_monotonic();
            
            results: [..]int;
            query_region := AABB.{min = .{-50, -50, -50}, max = .{50, 50, 50}};
            octree_query(octree.root, query_region, *results);
            
            query_time := current_time_monotonic();
            
            insert_duration := to_float64_seconds(insert_time - start_time);
            query_duration := to_float64_seconds(query_time - insert_time);
            
            print("Octree - Inserción: %.2f ms, Query: %.2f ms, Objetos: %\n",
                  insert_duration * 1000, query_duration * 1000, results.count);
            
            array_free(results);
        }
        
        array_free(positions);
    }
    
    print("\n✓ Benchmark de escalabilidad COMPLETADO\n\n");
}

main :: () {
    print("========================================\n");
    print("  PRUEBAS DE ALGORITMOS BROADPHASE     \n");
    print("========================================\n\n");
    
    random_seed(42);
    
    test_bvh_integrity();
    test_octree_subdivision();
    test_spatial_hash_grid();
    benchmark_broadphase_scaling();
    
    print("========================================\n");
    print("  ✓ TODAS LAS PRUEBAS BROADPHASE OK!   \n");
    print("========================================\n");
}