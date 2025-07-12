#import "Basic";
#load "../core/collision_3d_shapes.jai";

BVHNode :: struct {
    aabb: AABB;
    left: *BVHNode;
    right: *BVHNode;
    parent: *BVHNode;
    collider_id: int = -1;
    is_leaf: bool;
}

BVH :: struct {
    root: *BVHNode;
    nodes: [..]*BVHNode;
    free_nodes: [..]*BVHNode;
}

OctreeNode :: struct {
    bounds: AABB;
    children: [8]*OctreeNode;
    objects: [..]int;
    depth: int;
    max_depth: int = 5;
    max_objects_per_node: int = 8;
}

Octree :: struct {
    root: *OctreeNode;
    all_nodes: [..]*OctreeNode;
}

SpatialHashGrid :: struct {
    cell_size: float;
    cells: Table(u64, [..]int);
}

BroadphasePair :: struct {
    a: int;
    b: int;
}

create_bvh :: () -> *BVH {
    bvh := New(BVH);
    return bvh;
}

allocate_bvh_node :: (bvh: *BVH) -> *BVHNode {
    node: *BVHNode;
    if bvh.free_nodes.count > 0 {
        node = array_pop(*bvh.free_nodes);
    } else {
        node = New(BVHNode);
        array_add(*bvh.nodes, node);
    }
    return node;
}

free_bvh_node :: (bvh: *BVH, node: *BVHNode) {
    array_add(*bvh.free_nodes, node);
}

bvh_insert :: (bvh: *BVH, aabb: AABB, collider_id: int) -> *BVHNode {
    leaf := allocate_bvh_node(bvh);
    leaf.aabb = aabb;
    leaf.collider_id = collider_id;
    leaf.is_leaf = true;
    
    if !bvh.root {
        bvh.root = leaf;
        return leaf;
    }
    
    best_sibling := bvh_find_best_sibling(bvh.root, aabb);
    
    old_parent := best_sibling.parent;
    new_parent := allocate_bvh_node(bvh);
    new_parent.parent = old_parent;
    new_parent.aabb = merge_aabb(aabb, best_sibling.aabb);
    new_parent.is_leaf = false;
    
    if old_parent {
        if old_parent.left == best_sibling {
            old_parent.left = new_parent;
        } else {
            old_parent.right = new_parent;
        }
    } else {
        bvh.root = new_parent;
    }
    
    new_parent.left = best_sibling;
    new_parent.right = leaf;
    best_sibling.parent = new_parent;
    leaf.parent = new_parent;
    
    bvh_refit(new_parent.parent);
    
    return leaf;
}

bvh_find_best_sibling :: (node: *BVHNode, aabb: AABB) -> *BVHNode {
    if node.is_leaf return node;
    
    combined_aabb := merge_aabb(node.aabb, aabb);
    cost := aabb_surface_area(combined_aabb);
    inherited_cost := cost - aabb_surface_area(node.aabb);
    
    cost_left: float;
    if node.left.is_leaf {
        cost_left = aabb_surface_area(merge_aabb(node.left.aabb, aabb)) + inherited_cost;
    } else {
        old_area := aabb_surface_area(node.left.aabb);
        new_area := aabb_surface_area(merge_aabb(node.left.aabb, aabb));
        cost_left = new_area - old_area + inherited_cost;
    }
    
    cost_right: float;
    if node.right.is_leaf {
        cost_right = aabb_surface_area(merge_aabb(node.right.aabb, aabb)) + inherited_cost;
    } else {
        old_area := aabb_surface_area(node.right.aabb);
        new_area := aabb_surface_area(merge_aabb(node.right.aabb, aabb));
        cost_right = new_area - old_area + inherited_cost;
    }
    
    if cost < cost_left && cost < cost_right {
        return node;
    }
    
    if cost_left < cost_right {
        return bvh_find_best_sibling(node.left, aabb);
    } else {
        return bvh_find_best_sibling(node.right, aabb);
    }
}

bvh_remove :: (bvh: *BVH, leaf: *BVHNode) {
    if leaf == bvh.root {
        bvh.root = null;
        free_bvh_node(bvh, leaf);
        return;
    }
    
    parent := leaf.parent;
    grandparent := parent.parent;
    sibling := ifx parent.left == leaf then parent.right else parent.left;
    
    if grandparent {
        if grandparent.left == parent {
            grandparent.left = sibling;
        } else {
            grandparent.right = sibling;
        }
        sibling.parent = grandparent;
        free_bvh_node(bvh, parent);
        free_bvh_node(bvh, leaf);
        bvh_refit(grandparent);
    } else {
        bvh.root = sibling;
        sibling.parent = null;
        free_bvh_node(bvh, parent);
        free_bvh_node(bvh, leaf);
    }
}

bvh_refit :: (node: *BVHNode) {
    while node {
        node.aabb = merge_aabb(node.left.aabb, node.right.aabb);
        node = node.parent;
    }
}

bvh_query_pairs :: (bvh: *BVH, pairs: *[..]BroadphasePair) {
    if !bvh.root return;
    array_clear(pairs);
    bvh_query_node_pairs(bvh.root, bvh.root, pairs);
}

bvh_query_node_pairs :: (a: *BVHNode, b: *BVHNode, pairs: *[..]BroadphasePair) {
    if a == b {
        if !a.is_leaf {
            bvh_query_node_pairs(a.left, a.left, pairs);
            bvh_query_node_pairs(a.left, a.right, pairs);
            bvh_query_node_pairs(a.right, a.right, pairs);
        }
        return;
    }
    
    if !test_aabb_aabb(a.aabb, b.aabb).has_collision return;
    
    if a.is_leaf && b.is_leaf {
        pair: BroadphasePair;
        pair.a = min(a.collider_id, b.collider_id);
        pair.b = max(a.collider_id, b.collider_id);
        array_add(pairs, pair);
    } else if a.is_leaf {
        bvh_query_node_pairs(a, b.left, pairs);
        bvh_query_node_pairs(a, b.right, pairs);
    } else if b.is_leaf {
        bvh_query_node_pairs(a.left, b, pairs);
        bvh_query_node_pairs(a.right, b, pairs);
    } else {
        bvh_query_node_pairs(a.left, b.left, pairs);
        bvh_query_node_pairs(a.left, b.right, pairs);
        bvh_query_node_pairs(a.right, b.left, pairs);
        bvh_query_node_pairs(a.right, b.right, pairs);
    }
}

create_octree :: (bounds: AABB, max_depth: int = 5) -> *Octree {
    octree := New(Octree);
    octree.root = New(OctreeNode);
    octree.root.bounds = bounds;
    octree.root.depth = 0;
    octree.root.max_depth = max_depth;
    array_add(*octree.all_nodes, octree.root);
    return octree;
}

octree_get_child_index :: (node: *OctreeNode, position: Vector3) -> int {
    center := (node.bounds.min + node.bounds.max) * 0.5;
    index := 0;
    if position.x > center.x index += 1;
    if position.y > center.y index += 2;
    if position.z > center.z index += 4;
    return index;
}

octree_subdivide :: (octree: *Octree, node: *OctreeNode) {
    center := (node.bounds.min + node.bounds.max) * 0.5;
    size := (node.bounds.max - node.bounds.min) * 0.5;
    
    for i: 0..7 {
        child := New(OctreeNode);
        child.depth = node.depth + 1;
        child.max_depth = node.max_depth;
        child.max_objects_per_node = node.max_objects_per_node;
        
        offset: Vector3;
        offset.x = ifx (i & 1) then size.x else 0;
        offset.y = ifx (i & 2) then size.y else 0;
        offset.z = ifx (i & 4) then size.z else 0;
        
        child.bounds.min = node.bounds.min + offset;
        child.bounds.max = child.bounds.min + size;
        
        node.children[i] = child;
        array_add(*octree.all_nodes, child);
    }
    
    objects_to_redistribute := node.objects;
    array_clear(*node.objects);
    
    for obj_id: objects_to_redistribute {
        octree_insert(octree, node, obj_id, .{0, 0, 0});
    }
}

octree_insert :: (octree: *Octree, node: *OctreeNode, object_id: int, position: Vector3) {
    if node.depth < node.max_depth && node.objects.count >= node.max_objects_per_node {
        if !node.children[0] {
            octree_subdivide(octree, node);
        }
        
        child_index := octree_get_child_index(node, position);
        octree_insert(octree, node.children[child_index], object_id, position);
    } else {
        array_add(*node.objects, object_id);
    }
}

octree_query :: (node: *OctreeNode, aabb: AABB, results: *[..]int) {
    if !test_aabb_aabb(node.bounds, aabb).has_collision return;
    
    for obj_id: node.objects {
        array_add(results, obj_id);
    }
    
    if node.children[0] {
        for i: 0..7 {
            octree_query(node.children[i], aabb, results);
        }
    }
}

create_spatial_hash :: (cell_size: float) -> *SpatialHashGrid {
    grid := New(SpatialHashGrid);
    grid.cell_size = cell_size;
    init(*grid.cells);
    return grid;
}

hash_position :: (grid: *SpatialHashGrid, position: Vector3) -> u64 {
    x := cast(s64)(position.x / grid.cell_size);
    y := cast(s64)(position.y / grid.cell_size);
    z := cast(s64)(position.z / grid.cell_size);
    
    hash: u64 = cast(u64)x;
    hash = (hash << 20) | cast(u64)y;
    hash = (hash << 20) | cast(u64)z;
    
    return hash;
}

spatial_hash_insert :: (grid: *SpatialHashGrid, aabb: AABB, object_id: int) {
    min_cell: Vector3;
    min_cell.x = floor(aabb.min.x / grid.cell_size) * grid.cell_size;
    min_cell.y = floor(aabb.min.y / grid.cell_size) * grid.cell_size;
    min_cell.z = floor(aabb.min.z / grid.cell_size) * grid.cell_size;
    
    max_cell: Vector3;
    max_cell.x = floor(aabb.max.x / grid.cell_size) * grid.cell_size;
    max_cell.y = floor(aabb.max.y / grid.cell_size) * grid.cell_size;
    max_cell.z = floor(aabb.max.z / grid.cell_size) * grid.cell_size;
    
    pos: Vector3;
    pos.z = min_cell.z;
    while pos.z <= max_cell.z {
        pos.y = min_cell.y;
        while pos.y <= max_cell.y {
            pos.x = min_cell.x;
            while pos.x <= max_cell.x {
                hash := hash_position(grid, pos);
                cell, found := table_find(*grid.cells, hash);
                if !found {
                    new_cell: [..]int;
                    array_add(*new_cell, object_id);
                    table_add(*grid.cells, hash, new_cell);
                } else {
                    array_add(cell, object_id);
                }
                pos.x += grid.cell_size;
            }
            pos.y += grid.cell_size;
        }
        pos.z += grid.cell_size;
    }
}

spatial_hash_query :: (grid: *SpatialHashGrid, aabb: AABB, results: *[..]int) {
    array_clear(results);
    seen: Table(int, bool);
    init(*seen);
    defer deinit(*seen);
    
    min_cell: Vector3;
    min_cell.x = floor(aabb.min.x / grid.cell_size) * grid.cell_size;
    min_cell.y = floor(aabb.min.y / grid.cell_size) * grid.cell_size;
    min_cell.z = floor(aabb.min.z / grid.cell_size) * grid.cell_size;
    
    max_cell: Vector3;
    max_cell.x = floor(aabb.max.x / grid.cell_size) * grid.cell_size;
    max_cell.y = floor(aabb.max.y / grid.cell_size) * grid.cell_size;
    max_cell.z = floor(aabb.max.z / grid.cell_size) * grid.cell_size;
    
    pos: Vector3;
    pos.z = min_cell.z;
    while pos.z <= max_cell.z {
        pos.y = min_cell.y;
        while pos.y <= max_cell.y {
            pos.x = min_cell.x;
            while pos.x <= max_cell.x {
                hash := hash_position(grid, pos);
                cell, found := table_find(*grid.cells, hash);
                if found {
                    for obj_id: <<cell {
                        if !table_find(*seen, obj_id) {
                            table_add(*seen, obj_id, true);
                            array_add(results, obj_id);
                        }
                    }
                }
                pos.x += grid.cell_size;
            }
            pos.y += grid.cell_size;
        }
        pos.z += grid.cell_size;
    }
}