# Guía de Inicio Rápido

## 🚀 Instalación

### Requisitos
- Compilador Jai (versión beta o superior)
- Sistema operativo: Windows, Linux, macOS

### Descarga
```bash
git clone <url-del-repositorio> Collision3D_Project
cd Collision3D_Project
```

## 🏃‍♂️ Primer Ejemplo

### Compilación Básica
```bash
# Compilar ejemplo principal
jai examples/basic/collision_3d_example.jai

# Ejecutar
./collision_3d_example
```

### Código Mínimo
```jai
#import "Basic";
#load "../../src/collision_3d_system.jai";

main :: () {
    // Crear mundo de física
    world := create_physics_world();
    
    // Crear suelo
    ground := create_aabb_collider(.{-10, -1, -10}, .{10, 0, 10});
    ground_body := create_static_body(.{0, 0, 0});
    add_rigid_body(world, ground_body, ground);
    
    // Crear esfera que cae
    sphere := create_sphere_collider(.{0, 0, 0}, 1.0);
    sphere_body := create_dynamic_body(.{0, 5, 0}, 1.0);
    add_rigid_body(world, sphere_body, sphere);
    
    // Simular 60 pasos (1 segundo a 60fps)
    for i: 0..59 {
        physics_step(world, 0.016);
        print("Paso %: Esfera Y = %.2f\n", i, sphere_body.position.y);
    }
}
```

## 📋 Ejemplos por Complejidad

### 1. Básico - Esfera Rebotando
```bash
jai examples/basic/collision_3d_example.jai
```
**Características:**
- Esfera, caja y cápsula cayendo
- Física básica con gravedad
- Colisiones simples

### 2. Intermedio - Simulación de Billar
```bash
jai examples/advanced/test_billiards.jai
```
**Características:**
- 16 bolas de billar
- Mesa con bandas
- Fricción y restitución realistas
- Tiro de break automático

### 3. Avanzado - Stress Test
```bash
jai tests/benchmarks/test_performance.jai
```
**Características:**
- Miles de objetos simultáneos
- Benchmarks de rendimiento
- Comparación de algoritmos

## 🔧 Configuración Típica

### Para Juegos
```jai
world := create_physics_world();
world.gravity = .{0, -9.81, 0};
world.collision_world.use_bvh = true;
world.collision_world.use_continuous_detection = true;
world.constraint_iterations = 4;
```

### Para Simulaciones Precisas
```jai
world := create_physics_world();
world.gravity = .{0, -9.81, 0};
world.constraint_iterations = 8;
world.penetration_slop = 0.001;
world.penetration_correction_percent = 0.95;
```

### Para Objetos Rápidos
```jai
world := create_physics_world();
world.collision_world.use_continuous_detection = true;
// CCD se activa automáticamente cuando velocidad es alta
```

## 🎯 Tipos de Colisionadores

### Esfera
```jai
collider: Collider;
collider.type = .SPHERE;
collider.sphere = .{center = .{0, 0, 0}, radius = 1.0};
```

### Caja (AABB)
```jai
collider: Collider;
collider.type = .AABB;
collider.aabb = .{min = .{-1, -1, -1}, max = .{1, 1, 1}};
```

### Caja Orientada (OBB)
```jai
collider: Collider;
collider.type = .OBB;
collider.obb = .{
    center = .{0, 0, 0},
    half_extents = .{1, 1, 1},
    orientation = quaternion_from_axis_angle(.{0, 1, 0}, 0.785398)
};
```

### Cápsula
```jai
collider: Collider;
collider.type = .CAPSULE;
collider.capsule = .{
    point_a = .{0, -1, 0},
    point_b = .{0, 1, 0},
    radius = 0.5
};
```

## 🏋️‍♂️ Propiedades de Materiales

### Rebote Perfecto
```jai
body.restitution = 1.0;  // Sin pérdida de energía
body.friction = 0.0;     // Sin fricción
```

### Material Pegajoso
```jai
body.restitution = 0.0;  // Sin rebote
body.friction = 1.0;     // Fricción máxima
```

### Material Realista (madera)
```jai
body.restitution = 0.3;  // Rebote moderado
body.friction = 0.6;     // Fricción media
```

## 🎮 Interacción con Raycast

### Selección de Objetos
```jai
ray: Ray;
ray.origin = camera_position;
ray.direction = normalize(mouse_world_direction);

hit := ray_cast(world.collision_world, ray, 100.0);

if hit.hit {
    print("Objeto seleccionado en: (%.2f, %.2f, %.2f)\n",
          hit.point.x, hit.point.y, hit.point.z);
}
```

### Disparo de Proyectil
```jai
ray: Ray;
ray.origin = gun_position;
ray.direction = gun_direction;

hit := ray_cast(world.collision_world, ray, 1000.0);

if hit.hit {
    // Crear efecto de impacto
    create_impact_effect(hit.point, hit.normal);
}
```

## 🔍 Debugging Común

### Problema: Objetos Atraviesan
**Solución:**
```jai
// Activar CCD
world.collision_world.use_continuous_detection = true;

// O reducir timestep
physics_step(world, 0.008); // En lugar de 0.016
```

### Problema: Simulación Inestable
**Solución:**
```jai
// Aumentar iteraciones
world.constraint_iterations = 8;

// Reducir tolerancia
world.penetration_slop = 0.001;
```

### Problema: Rendimiento Lento
**Solución:**
```jai
// Verificar broadphase apropiado
world.collision_world.use_bvh = true; // Para objetos dinámicos

// O usar spatial hash para objetos estáticos
grid := create_spatial_hash(5.0);
```

## 📊 Monitoreo de Rendimiento

### Contadores Básicos
```jai
print("Colisiones detectadas: %\n", 
      world.collision_world.contact_manifolds.count);
print("Pares broadphase: %\n", 
      world.collision_world.collision_pairs.count);
```

### Timing Manual
```jai
start_time := current_time_monotonic();
physics_step(world, dt);
end_time := current_time_monotonic();

duration := to_float64_seconds(end_time - start_time);
print("Tiempo de física: %.3f ms\n", duration * 1000);
```

## 📚 Próximos Pasos

1. **Experimentar** con `examples/basic/collision_3d_example.jai`
2. **Estudiar** la simulación de billar en `examples/advanced/test_billiards.jai`
3. **Ejecutar pruebas** con `tests/build_and_test.jai`
4. **Leer arquitectura** en `docs/architecture.md`
5. **Crear tu propia simulación** basada en los ejemplos

## 🆘 Ayuda y Soporte

- **Documentación completa**: `README.md`
- **Arquitectura detallada**: `docs/architecture.md`
- **Ejemplos funcionales**: `examples/`
- **Pruebas exhaustivas**: `tests/`

¡Disfruta construyendo mundos físicos realistas! 🎯