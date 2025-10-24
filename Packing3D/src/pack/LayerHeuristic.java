package pack;

import ilog.concert.IloException;

/**
 * LayerHeuristic - versión revisada evitando StringBuilder y minimizando imports
 * Mantiene la lógica:
 *  - Encuentra mejor orientación (CompleteModel2D)
 *  - Guarda posiciones colocadas en la capa base
 *  - Replica posiciones por cada capa y marca solo voxeles ocupados
 *  - Llama a CompleteModel para rellenar con restricciones (usar cualquier orientación si CompleteModel lo soporta)
 */
public class LayerHeuristic {

    public static void run(Instance instance) {
        try {
            Discretization disc = new Discretization(instance);
            System.out.printf("Discretización: I=%d, J=%d, K=%d%n",
                    disc.sizeI(), disc.sizeJ(), disc.sizeK());

            int bestTotal = 0;
            Box.Orientation bestOrientation = null;
            int bestBoxesPerLayer = 0;
            int bestLayers = 0;
            double bestTime = 0.0;
            int best3DFillBoxes = 0;
            CompleteModel2D bestModel = null;

            System.out.println("=== Heurística por Capas (PLE con relleno 3D) ===");
            System.out.printf("Contenedor: L=%d, W=%d, H=%d%n",
                    instance.getL(), instance.getW(), instance.getH());

            // ==== FASE 1: capas idénticas con modelo 2D ====
            for (Box.Orientation o : Box.Orientation.values()) {
                Box sample = new Box(0, 0, 0, o);
                int layerHeight = sample.getHeight();

                // Saltar orientaciones inválidas
                if (layerHeight > instance.getH()) continue;

                int layers = instance.getH() / layerHeight;
                if (layers == 0) continue;

                CompleteModel2D model = new CompleteModel2D(instance, disc, o, 0);
                try {
                    model.solve();
                } catch (IloException e) {
                    System.out.println("Error resolviendo CompleteModel2D para orientación " + o);
                    e.printStackTrace();
                    continue;
                }

                int boxesInLayer = model.getSolutionCount();
                double solveTime = model.getSolveTime();
                int totalBoxes = boxesInLayer * layers;

                System.out.print("Orientación " + o.toString() + " -> ");
                System.out.printf("%3d cajas/capa × %2d capas = %3d cajas totales%n",
                        boxesInLayer, layers, totalBoxes);

                if (totalBoxes > bestTotal) {
                    bestTotal = totalBoxes;
                    bestOrientation = o;
                    bestBoxesPerLayer = boxesInLayer;
                    bestLayers = layers;
                    bestTime = solveTime;
                    bestModel = model; // guardamos modelo ganador con posiciones
                }
            }

            // === Resultado Fase 1 ===
            System.out.println();
            System.out.println("=== Resultado Fase 1 (capas 2D) ===");
            System.out.println("Mejor orientación: " + bestOrientation);
            System.out.println("Cajas por capa: " + bestBoxesPerLayer);
            System.out.println("Cantidad de capas: " + bestLayers);
            System.out.println("Total de cajas (capas): " + bestTotal);
            System.out.printf("Tiempo total (1 capa): %.2f segundos%n", bestTime / 1000.0);

            if (bestModel == null) {
                System.out.println("No se encontró modelo 2D válido. Termina heurística.");
                return;
            }

            // ==== FASE 2: Construcción de matriz de ocupación (solo voxeles realmente ocupados) ====
            System.out.println();
            System.out.println("=== Fase 2: Construyendo matriz de ocupación ===");

            boolean[][][] occupied = buildOccupiedMatrix(bestOrientation, bestLayers, disc, instance, bestModel);

            double freeRatio = computeFreeRatio(occupied, disc);
            int totalCells = disc.sizeI() * disc.sizeJ() * disc.sizeK();
            int occupiedCells = totalCells - (int)(freeRatio * totalCells);

            System.out.printf("Celdas ocupadas: %d / %d (%.2f%%)%n",
                    occupiedCells, totalCells, (100.0 * occupiedCells / totalCells));
            System.out.printf("Volumen libre restante: %.2f%% del contenedor%n", freeRatio * 100.0);

            // Visualización ASCII simple de la última capa (si existe)
            printTopLayerMap(occupied, disc);

            // Si hay espacio libre, lanzar CompleteModel de relleno
            if (freeRatio < 0.01) {
                System.out.println("No queda volumen libre significativo para relleno.");
            } else {
                System.out.println("Resolviendo modelo de relleno 3D...");
                CompleteModel fill3D = new CompleteModel(instance, disc, occupied);
                try {
                    fill3D.solve();
                } catch (IloException e) {
                    System.out.println("Error resolviendo CompleteModel (relleno).");
                    e.printStackTrace();
                }

                // Lee objetivo de forma segura usando getter si existe
                try {
                    // Si CompleteModel implementó getObjectiveValue() usarlo; sino leer de CPLEX con cuidado
                    double added = 0.0;
                    try {
                        added = fill3D.getObjectiveValue();
                    } catch (NoSuchMethodError | AbstractMethodError ex) {
                        // fallback: intentar leer desde _cplex si está disponible
                        if (fill3D._cplex != null) {
                            added = fill3D._cplex.getObjValue();
                        } else {
                            added = 0.0;
                        }
                    }
                    best3DFillBoxes = (int) Math.round(added);
                    System.out.println("Cajas adicionales (relleno 3D): " + best3DFillBoxes);
                    System.out.println("Total final de cajas: " + (bestTotal + best3DFillBoxes));
                } catch (Exception e) {
                    System.out.println("No se pudo leer el resultado del relleno correctamente.");
                    e.printStackTrace();
                }
            }

            // ==== RESUMEN FINAL ====
            System.out.println();
            System.out.println("=== RESUMEN FINAL ===");
            System.out.println("Mejor orientación: " + bestOrientation);
            System.out.println("Cajas por capa: " + bestBoxesPerLayer);
            System.out.println("Cantidad de capas: " + bestLayers);
            System.out.println("Cajas 3D adicionales: " + best3DFillBoxes);
            System.out.println("Total final de cajas: " + (bestTotal + best3DFillBoxes));

        } catch (Exception e) {
            System.err.println("❌ Error durante la ejecución de la heurística:");
            e.printStackTrace();
        }
    }

    /**
     * Marca solo las posiciones realmente ocupadas por las cajas en cada capa.
     * Usa model2D.getPlacedBoxes() que devuelve java.util.List<int[]> con pares {i,j}.
     */
    private static boolean[][][] buildOccupiedMatrix(Box.Orientation orientation, int layers,
                                                     Discretization disc, Instance instance,
                                                     CompleteModel2D model2D) {
        int I = disc.sizeI();
        int J = disc.sizeJ();
        int K = disc.sizeK();
        boolean[][][] occupied = new boolean[I][J][K];

        // obtener la lista de posiciones usando el tipo totalmente calificado para evitar imports
        java.util.List<int[]> placedBoxes = model2D.getPlacedBoxes();

        Box sample = new Box(0, 0, 0, orientation);
        int boxLength = sample.getLength();
        int boxWidth = sample.getWidth();
        int boxHeight = sample.getHeight();

        // Inicializar a false (ya está por defecto)
        for (int layer = 0; layer < layers; layer++) {
            int zStart = layer * boxHeight;
            int zEnd = zStart + boxHeight; // exclusive upper bound in coordenadas físicas

            for (int idx = 0; idx < placedBoxes.size(); idx++) {
                int[] pos = placedBoxes.get(idx);
                int i = pos[0];
                int j = pos[1];

                // crear una caja a partir de (i,j) en esta orientación
                Box b = new Box(i, j, 0, orientation);
                if (!b.fits()) continue;

                // marcar solo voxeles realmente ocupados por esta caja replica en la capa
                for (int ip = i; ip < i + boxLength && ip < I; ip++) {
                    for (int jp = j; jp < j + boxWidth && jp < J; jp++) {
                        for (int kp = 0; kp < K; kp++) {
                            int z = disc.getz(kp);
                            if (z >= zStart && z < zEnd) {
                                occupied[ip][jp][kp] = true;
                            }
                        }
                    }
                }
            }
        }
        return occupied;
    }

    /** Calcula el porcentaje de volumen libre. */
    private static double computeFreeRatio(boolean[][][] occupied, Discretization disc) {
        int total = disc.sizeI() * disc.sizeJ() * disc.sizeK();
        int occ = 0;
        for (int i = 0; i < disc.sizeI(); i++)
            for (int j = 0; j < disc.sizeJ(); j++)
                for (int k = 0; k < disc.sizeK(); k++)
                    if (occupied[i][j][k]) occ++;
        return (double) (total - occ) / (double) total;
    }

    /** Imprime un mapa simple del plano superior (última capa discretizada no necesariamente la superior física). */
    private static void printTopLayerMap(boolean[][][] occupied, Discretization disc) {
        int kIndex = disc.sizeK() - 1;
        System.out.println("Mapa de ocupación (Z = top index " + kIndex + "):");
        for (int i = 0; i < disc.sizeI(); i++) {
            // imprimimos cada columna sin StringBuilder, con concatenación simple (evita uso explícito)
            String row = "";
            for (int j = 0; j < disc.sizeJ(); j++) {
                if (occupied[i][j][kIndex]) row = row + "█";
                else row = row + " ";
            }
            System.out.println(row);
        }
    }
}
