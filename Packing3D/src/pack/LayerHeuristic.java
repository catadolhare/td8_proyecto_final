package pack;

import ilog.concert.IloException;

public class LayerHeuristic {

    public static void run(Instance instance) {
        try {
            Discretization disc = new Discretization(instance);

            int bestTotal = 0;
            Box.Orientation bestOrientation = null;
            int bestBoxesPerLayer = 0;
            int bestLayers = 0;
            double bestTime = 0.0;
            int best3DFillBoxes = 0;

            System.out.println("=== Heurística por Capas (PLE con relleno 3D) ===");
            System.out.printf("Contenedor: L=%d, W=%d, H=%d%n", instance.getL(), instance.getW(), instance.getH());
            System.out.printf("Discretización: I=%d, J=%d, K=%d%n", disc.sizeI(), disc.sizeJ(), disc.sizeK());

            // ==== FASE 1: capas 2D ====
            for (Box.Orientation o : Box.Orientation.values()) {
                Box sample = new Box(0, 0, 0, o);
                int layerHeight = sample.getHeight();
                if (layerHeight > instance.getH()) continue;

                int layers = instance.getH() / layerHeight;
                if (layers == 0) continue;

                CompleteModel2D model2D = new CompleteModel2D(instance, disc, o, 0);
                model2D.solve();

                int boxesInLayer = model2D.getSolutionCount();
                double solveTime = model2D.getSolveTime();
                int totalBoxes = boxesInLayer * layers;

                System.out.printf("Orientación %-10s → %3d cajas/capa × %2d capas = %3d cajas totales%n",
                        o, boxesInLayer, layers, totalBoxes);

                if (totalBoxes > bestTotal) {
                    bestTotal = totalBoxes;
                    bestOrientation = o;
                    bestBoxesPerLayer = boxesInLayer;
                    bestLayers = layers;
                    bestTime = solveTime;
                }
            }

            System.out.println("\n=== Resultado Fase 1 (capas 2D) ===");
            System.out.println("Mejor orientación: " + bestOrientation);
            System.out.println("Cajas por capa: " + bestBoxesPerLayer);
            System.out.println("Cantidad de capas: " + bestLayers);
            System.out.println("Total de cajas (capas): " + bestTotal);
            System.out.printf("Tiempo total (1 capa): %.2f segundos%n", bestTime / 1000.0);

            // ==== FASE 2: Relleno 3D ====
            System.out.println("\n=== Fase 2: Construyendo matriz de ocupación ===");

            boolean[][][] occupied = buildOccupiedMatrix(bestOrientation, bestLayers, disc, instance);
            double freeRatio = computeFreeRatio(occupied, disc);
            System.out.printf("Celdas ocupadas: %d / %d (%.2f%%)%n",
                    countOccupied(occupied), disc.sizeI() * disc.sizeJ() * disc.sizeK(),
                    (1 - freeRatio) * 100);
            System.out.printf("Volumen libre restante: %.2f%% del contenedor%n", freeRatio * 100);

            System.out.println("Resolviendo modelo de relleno 3D...");
            CompleteModel fill3D = new CompleteModel(instance, disc);
            fill3D.solve();

            int fillBoxes = 0;
            try {
                if (fill3D._cplex != null) {
                    fillBoxes = (int) Math.round(fill3D._cplex.getObjValue());
                    System.out.println("Cajas adicionales (relleno 3D): " + fillBoxes);
                    System.out.println("Total final de cajas: " + (bestTotal + fillBoxes));
                }
            } catch (IloException e) {
                System.out.println("⚠️ Error al leer resultado del modelo de relleno.");
            }

            bestTotal += fillBoxes;
            best3DFillBoxes = fillBoxes;

            // ==== RESUMEN ====
            System.out.println("\n=== RESUMEN FINAL ===");
            System.out.println("Mejor orientación: " + bestOrientation);
            System.out.println("Cajas por capa: " + bestBoxesPerLayer);
            System.out.println("Cantidad de capas: " + bestLayers);
            System.out.println("Cajas 3D adicionales: " + best3DFillBoxes);
            System.out.println("Total final de cajas: " + bestTotal);

        } catch (Exception e) {
            System.err.println("❌ Error durante la ejecución de la heurística:");
            e.printStackTrace();
        }
    }

    // ======== helpers ========
    private static boolean[][][] buildOccupiedMatrix(Box.Orientation orientation, int layers,
                                                     Discretization disc, Instance instance) {
        boolean[][][] occupied = new boolean[disc.sizeI()][disc.sizeJ()][disc.sizeK()];
        Box sample = new Box(0, 0, 0, orientation);
        int layerHeight = sample.getHeight();
        int usedHeight = layers * layerHeight;

        for (int i = 0; i < disc.sizeI(); i++)
        for (int j = 0; j < disc.sizeJ(); j++)
        for (int k = 0; k < disc.sizeK(); k++) {
            int z = disc.getz(k);
            if (z < usedHeight) occupied[i][j][k] = true;
        }
        return occupied;
    }

    private static double computeFreeRatio(boolean[][][] occ, Discretization disc) {
        int total = disc.sizeI() * disc.sizeJ() * disc.sizeK();
        int used = countOccupied(occ);
        return (double) (total - used) / total;
    }

    private static int countOccupied(boolean[][][] occ) {
        int count = 0;
        for (int i = 0; i < occ.length; i++)
            for (int j = 0; j < occ[0].length; j++)
                for (int k = 0; k < occ[0][0].length; k++)
                    if (occ[i][j][k]) count++;
        return count;
    }
}
