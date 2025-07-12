# Makefile para el Sistema de Colisiones 3D en Jai

# Variables
JAI_COMPILER = jai
SRC_DIR = src
TESTS_DIR = tests
EXAMPLES_DIR = examples
BUILD_DIR = build

# Crear directorio de build
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Target principal
all: $(BUILD_DIR) examples tests

# Compilar ejemplos
examples: $(BUILD_DIR)
	@echo "=== Compilando Ejemplos ==="
	$(JAI_COMPILER) $(EXAMPLES_DIR)/basic/collision_3d_example.jai -output-executable $(BUILD_DIR)/collision_example
	$(JAI_COMPILER) $(EXAMPLES_DIR)/advanced/test_billiards.jai -output-executable $(BUILD_DIR)/billiards_sim
	@echo "✓ Ejemplos compilados en $(BUILD_DIR)/"

# Compilar y ejecutar pruebas
tests: $(BUILD_DIR)
	@echo "=== Compilando Pruebas ==="
	$(JAI_COMPILER) $(TESTS_DIR)/unit/test_shapes.jai -output-executable $(BUILD_DIR)/test_shapes
	$(JAI_COMPILER) $(TESTS_DIR)/unit/test_broadphase.jai -output-executable $(BUILD_DIR)/test_broadphase
	$(JAI_COMPILER) $(TESTS_DIR)/unit/test_physics.jai -output-executable $(BUILD_DIR)/test_physics
	$(JAI_COMPILER) $(TESTS_DIR)/unit/test_ccd.jai -output-executable $(BUILD_DIR)/test_ccd
	$(JAI_COMPILER) $(TESTS_DIR)/benchmarks/test_performance.jai -output-executable $(BUILD_DIR)/test_performance
	@echo "✓ Pruebas compiladas en $(BUILD_DIR)/"

# Ejecutar todas las pruebas
run-tests: tests
	@echo "=== Ejecutando Pruebas ==="
	./$(BUILD_DIR)/test_shapes
	./$(BUILD_DIR)/test_broadphase
	./$(BUILD_DIR)/test_physics
	./$(BUILD_DIR)/test_ccd
	@echo "✓ Todas las pruebas completadas"

# Ejecutar benchmarks
run-benchmarks: tests
	@echo "=== Ejecutando Benchmarks ==="
	./$(BUILD_DIR)/test_performance
	@echo "✓ Benchmarks completados"

# Ejecutar ejemplos
run-examples: examples
	@echo "=== Ejecutando Ejemplo Principal ==="
	./$(BUILD_DIR)/collision_example
	@echo "=== Ejecutando Simulación de Billar ==="
	./$(BUILD_DIR)/billiards_sim

# Compilar proyecto principal
project: $(BUILD_DIR)
	$(JAI_COMPILER) project.jai -output-executable $(BUILD_DIR)/collision3d_project

# Limpiar archivos generados
clean:
	rm -rf $(BUILD_DIR)
	@echo "✓ Archivos de build eliminados"

# Verificar estructura del proyecto
check-structure:
	@echo "=== Verificando Estructura ==="
	@echo "Archivos core:"
	@ls -la $(SRC_DIR)/core/
	@echo "Archivos broadphase:"
	@ls -la $(SRC_DIR)/broadphase/
	@echo "Archivos physics:"
	@ls -la $(SRC_DIR)/physics/
	@echo "Ejemplos:"
	@ls -la $(EXAMPLES_DIR)/*/
	@echo "Pruebas:"
	@ls -la $(TESTS_DIR)/*/

# Contar líneas de código
count-lines:
	@echo "=== Estadísticas de Código ==="
	@echo "Líneas en src/:"
	@find $(SRC_DIR) -name "*.jai" -exec wc -l {} + | tail -1
	@echo "Líneas en tests/:"
	@find $(TESTS_DIR) -name "*.jai" -exec wc -l {} + | tail -1
	@echo "Líneas en examples/:"
	@find $(EXAMPLES_DIR) -name "*.jai" -exec wc -l {} + | tail -1
	@echo "Total:"
	@find . -name "*.jai" -exec wc -l {} + | tail -1

# Ayuda
help:
	@echo "Targets disponibles:"
	@echo "  all           - Compilar todo (ejemplos + pruebas)"
	@echo "  examples      - Compilar solo ejemplos"
	@echo "  tests         - Compilar solo pruebas"
	@echo "  run-tests     - Compilar y ejecutar todas las pruebas"
	@echo "  run-benchmarks- Compilar y ejecutar benchmarks"
	@echo "  run-examples  - Compilar y ejecutar ejemplos"
	@echo "  project       - Compilar proyecto principal"
	@echo "  clean         - Limpiar archivos generados"
	@echo "  check-structure - Verificar estructura del proyecto"
	@echo "  count-lines   - Contar líneas de código"
	@echo "  help          - Mostrar esta ayuda"

# Targets que no generan archivos
.PHONY: all examples tests run-tests run-benchmarks run-examples project clean check-structure count-lines help