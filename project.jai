#import "Basic";

// Archivo principal del proyecto de Colisiones 3D
// Este archivo sirve como punto de entrada y coordinador del sistema

#load "src/core/collision_3d_math.jai";
#load "src/core/collision_3d_shapes.jai";
#load "src/core/collision_3d_detection.jai";
#load "src/broadphase/collision_3d_broadphase.jai";
#load "src/collision_3d_system.jai";
#load "src/physics/collision_3d_response.jai";
#load "src/physics/collision_3d_ccd.jai";

COLLISION3D_VERSION :: "1.0.0";

print_banner :: () {
    print("========================================\n");
    print("  SISTEMA DE COLISIONES 3D EN JAI     \n");
    print("         Versión %                     \n", COLLISION3D_VERSION);
    print("========================================\n\n");
}

main :: () {
    print_banner();
    
    print("Proyecto organizado en:\n");
    print("  src/core/          - Matemáticas y formas básicas\n");
    print("  src/broadphase/    - Algoritmos de optimización espacial\n");
    print("  src/physics/       - Sistema físico y CCD\n");
    print("  tests/             - Pruebas y benchmarks\n");
    print("  examples/          - Ejemplos de uso\n");
    print("  docs/              - Documentación\n\n");
    
    print("Para compilar ejemplos:\n");
    print("  jai examples/basic/collision_3d_example.jai\n");
    print("  jai examples/advanced/test_billiards.jai\n\n");
    
    print("Para ejecutar pruebas:\n");
    print("  jai tests/build_and_test.jai\n\n");
    
    print("Sistema listo para usar!\n");
}