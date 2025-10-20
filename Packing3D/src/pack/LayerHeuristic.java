package pack;

import ilog.cplex.IloCplex;

public class LayerHeuristic {
	
	private IloCplex _cplex;

    public static HeuristicResult run(Instance instance) {
        Discretization disc = new Discretization(instance);
        HeuristicResult result = new HeuristicResult();

        int bestTotal = 0;
        Box.Orientation bestOrientation = null;
        int bestBoxesPerLayer = 0;
        double bestTime = 0.0;
        int bestLayers = 0;

        System.out.println("=== Heurística por Capas (versión completa) ===");
        System.out.println("Contenedor: L=" + instance.getL() + " W=" + instance.getW() + " H=" + instance.getH());
        System.out.println();

        // ==== FASE 1: Capas idénticas ====
        for (Box.Orientation o : Box.Orientation.values()) {
            Box sample = new Box(0, 0, 0, o);
            int layerHeight = sample.getHeight();
            int layers = instance.getH() / layerHeight;

            CompleteModel model = new CompleteModel(instance, disc);
            model.solve();

            int boxesInLayer = (int) Math.round(model._cplex.getObjValue());
            double solveTime = model._cplex.getCplexTime();
            int totalBoxes = boxesInLayer * layers;

            System.out.printf("Orientación %-10s → %3d cajas por capa × %2d capas = %3d cajas totales\n",
                    o.toString(), boxesInLayer, layers, totalBoxes);

            if (totalBoxes > bestTotal) {
                bestTotal = totalBoxes;
                bestOrientation = o;
                bestBoxesPerLayer = boxesInLayer;
                bestTime = solveTime;
                bestLayers = layers;
            }
        }

        System.out.println("\n=== Resultado Fase 1 ===");
        System.out.println("Mejor orientación: " + bestOrientation);
        System.out.println("Cajas por capa: " + bestBoxesPerLayer);
        System.out.println("Cantidad de capas: " + bestLayers);
        System.out.println("Total de cajas (capas idénticas): " + bestTotal);
        System.out.printf("Tiempo total de resolución (1 capa): %.2f segundos\n", bestTime / 1000);

        // Guardar resultados Fase 1
        result.bestOrientation = bestOrientation;
        result.boxesPerLayer = bestBoxesPerLayer;
        result.layers = bestLayers;
        result.totalBoxes = bestTotal;
        result.solveTime = bestTime;

        // ==== FASE 2: Relleno vertical ====
        Box sample = new Box(0, 0, 0, bestOrientation);
        int usedHeight = bestLayers * sample.getHeight();
        int remainingHeight = instance.getH() - usedHeight;

        if (remainingHeight > 0) {
            System.out.println("\n=== Fase 2: Relleno vertical ===");
            System.out.println("Altura restante disponible: " + remainingHeight);

            int bestFillBoxes = 0;
            Box.Orientation bestFillOrientation = null;
            double bestFillTime = 0.0;

            for (Box.Orientation o2 : Box.Orientation.values()) {
                Box testBox = new Box(0, 0, 0, o2);
                if (testBox.getHeight() <= remainingHeight) {
                    CompleteModel fillModel = new CompleteModel(instance, disc);
                    fillModel.solve();
                    int fillCount = (int) Math.round(fillModel._cplex.getObjValue());
                    if (fillCount > bestFillBoxes) {
                        bestFillBoxes = fillCount;
                        bestFillOrientation = o2;
                        bestFillTime = fillModel._cplex.getCplexTime();
                    }
                }
            }

            System.out.println("Mejor orientación de relleno vertical: " + bestFillOrientation);
            System.out.println("Cajas adicionales: " + bestFillBoxes);
            System.out.println("Total con relleno vertical: " + (bestTotal + bestFillBoxes));
            System.out.printf("Tiempo adicional de relleno: %.2f segundos\n", bestFillTime / 1000);

            result.verticalFillBoxes = bestFillBoxes;
            result.verticalFillOrientation = bestFillOrientation;
            result.fillTime = bestFillTime;

        } else {
            System.out.println("\nNo queda espacio vertical para relleno.");
            result.verticalFillBoxes = 0;
        }

        // ==== FASE 3: Relleno 3D residual ====
        System.out.println("\n=== Fase 3: Relleno 3D residual ===");
        boolean[][][] occupied = buildOccupiedMatrix(bestOrientation, bestLayers, disc, instance);
        ResidualCompleteModel residual = new ResidualCompleteModel(instance, disc, occupied);
        residual.solve();

        // obtener cantidad de cajas nuevas del modelo residual
        int residualBoxes = (int) Math.round(residual._cplex.getObjValue());
        double residualTime = residual._cplex.getCplexTime();
        System.out.println("Cajas adicionales del modelo 3D residual: " + residualBoxes);
        System.out.printf("Tiempo adicional (3D residual): %.2f segundos\n", residualTime / 1000);

        result.residualBoxes = residualBoxes;
        result.residualTime = residualTime;

        // ==== RESULTADO FINAL ====
        System.out.println("\n=== Resultado Final ===");
        System.out.println("Mejor orientación: " + result.bestOrientation);
        System.out.println("Cajas por capa: " + result.boxesPerLayer);
        System.out.println("Cantidad de capas: " + result.layers);
        System.out.println("Total (capas + relleno + residual): " + result.totalFinal());
        System.out.printf("Tiempo total: %.2f segundos\n",
                (result.solveTime + result.fillTime + result.residualTime) / 1000);

        return result;
    }

    // Construye la matriz de posiciones ocupadas
    private static boolean[][][] buildOccupiedMatrix(Box.Orientation orientation, int layers,
                                                     Discretization disc, Instance instance) {
        boolean[][][] occupied = new boolean[disc.sizeI()][disc.sizeJ()][disc.sizeK()];

        Box bSample = new Box(0, 0, 0, orientation);
        int layerHeight = bSample.getHeight();

        for (int k = 0; k < layers; k++) {
            for (int i = 0; i < disc.sizeI(); i++) {
                for (int j = 0; j < disc.sizeJ(); j++) {
                    Box b = new Box(i, j, k, orientation);
                    if (b.fits()) {
                        int kStart = k * layerHeight;
                        int kEnd = Math.min(kStart + b.getHeight(), disc.sizeK() - 1);
                        for (int ip = i; ip < i + b.getLength() && ip < disc.sizeI(); ip++) {
                            for (int jp = j; jp < j + b.getWidth() && jp < disc.sizeJ(); jp++) {
                                for (int kp = kStart; kp <= kEnd; kp++) {
                                    occupied[ip][jp][kp] = true;
                                }
                            }
                        }
                    }
                }
            }
        }
        return occupied;
    }
}
