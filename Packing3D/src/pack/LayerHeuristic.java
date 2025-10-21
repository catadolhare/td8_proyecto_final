package pack;

import ilog.concert.IloException;

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

            System.out.println("=== Heurística por Capas (PLE con relleno 3D) ===");
            System.out.printf("Contenedor: L=%d, W=%d, H=%d%n", instance.getL(), instance.getW(), instance.getH());

            // ==== FASE 1: capas idénticas con modelo 2D ====
            for (Box.Orientation o : Box.Orientation.values()) {
                Box sample = new Box(0, 0, 0, o);
                int layerHeight = sample.getHeight();

                // Si la caja es más alta que el contenedor, descartar orientación
                if (layerHeight > instance.getH()) continue;

                int layers = instance.getH() / layerHeight; // cantidad de capas completas
                if (layers == 0) continue;

                // Resolver una sola capa con CompleteModel2D
                CompleteModel2D model = new CompleteModel2D(instance, disc, o, 0);
                model.solve();

                int boxesInLayer = model.getSolutionCount();
                double solveTime = model.getSolveTime();
                int totalBoxes = boxesInLayer * layers;

                System.out.printf("Orientación %-10s → %3d cajas/capa × %2d capas = %3d cajas totales%n",
                        o.toString(), boxesInLayer, layers, totalBoxes);

                if (totalBoxes > bestTotal) {
                    bestTotal = totalBoxes;
                    bestOrientation = o;
                    bestBoxesPerLayer = boxesInLayer;
                    bestLayers = layers;
                    bestTime = solveTime;
                }
            }
            
            

            // === Resultado Fase 1 ===
            System.out.println("\n=== Resultado Fase 1 (capas 2D) ===");
            System.out.println("Mejor orientación: " + bestOrientation);
            System.out.println("Cajas por capa: " + bestBoxesPerLayer);
            System.out.println("Cantidad de capas: " + bestLayers);
            System.out.println("Total de cajas (capas): " + bestTotal);
            System.out.printf("Tiempo total (1 capa): %.2f segundos%n", bestTime / 1000.0);
            

            // ==== FASE 2: Relleno 3D ====
            System.out.println("\n=== Fase 2: Construyendo matriz de ocupación ===");

            boolean[][][] occupied = buildOccupiedMatrix(bestOrientation, bestLayers, disc, instance);
            int occupiedCount = 0;
            for (int i = 0; i < disc.sizeI(); i++)
              for (int j = 0; j < disc.sizeJ(); j++)
                for (int k = 0; k < disc.sizeK(); k++)
                  if (occupied[i][j][k]) occupiedCount++;

            System.out.printf("Celdas ocupadas: %d / %d (%.2f%%)%n",
                occupiedCount, disc.sizeI() * disc.sizeJ() * disc.sizeK(),
                100.0 * occupiedCount / (disc.sizeI() * disc.sizeJ() * disc.sizeK()));


            double freeRatio = computeFreeRatio(occupied, disc);
            System.out.printf("Volumen libre restante: %.2f%% del contenedor%n", freeRatio * 100);
            
            System.out.println("Mapa de ocupación (Z = última capa usada):");
            int topLayerIndex = bestLayers - 1;
            for (int i = 0; i < disc.sizeI(); i++) {
                for (int j = 0; j < disc.sizeJ(); j++) {
                    System.out.print(occupied[i][j][topLayerIndex] ? "█" : ".");
                }
                System.out.println();
            }


            if (freeRatio < 0.01) {
                System.out.println("No queda volumen libre significativo para relleno.");
            } else {
                // Crear modelo 3D para relleno
            	// Crear y resolver modelo 3D para relleno
            	CompleteModel fill3D = new CompleteModel(instance, disc, occupied);
            	System.out.println("Resolviendo modelo de relleno 3D...");
            	fill3D.solve();
            	System.out.println("Estado CPLEX (relleno): " + fill3D.getStatus());
            	System.out.println("Valor objetivo CPLEX (relleno): " + fill3D.getObjectiveValue());


            	// Leer objetivo y estado con getters (ya que CompleteModel guarda antes de cerrar CPLEX)
            	int fillBoxes = 0;
            	ilog.cplex.IloCplex.Status fillStatus = fill3D.getStatus();
            	if (fillStatus == ilog.cplex.IloCplex.Status.Optimal || fillStatus == ilog.cplex.IloCplex.Status.Feasible) {
            	    double objVal = fill3D.getObjectiveValue(); // seguro, no usa CPLEX interno
            	    fillBoxes = (int) Math.round(objVal);
            	    System.out.println("Cajas adicionales (relleno 3D): " + fillBoxes);
            	    System.out.println("Total final de cajas: " + (bestTotal + fillBoxes));
            	} else {
            	    System.out.println("⚠️ Modelo de relleno no retorna solución útil. Estado: " + fillStatus);
            	}

                bestTotal += fillBoxes;
                best3DFillBoxes = fillBoxes;
            }


            // ==== RESUMEN FINAL ====
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

    /**
     * Marca las posiciones ocupadas por las capas apiladas en la matriz 3D.
     */
    	private static boolean[][][] buildOccupiedMatrix(Box.Orientation orientation, int layers,
            Discretization disc, Instance instance) {
    		boolean[][][] occupied = new boolean[disc.sizeI()][disc.sizeJ()][disc.sizeK()];

    		Box box = new Box(0, 0, 0, orientation);
    		int boxHeight = box.getHeight();
    		int usedHeight = layers * boxHeight;

    		for (int i = 0; i < disc.sizeI(); i++) {
    			for (int j = 0; j < disc.sizeJ(); j++) {
    				for (int k = 0; k < disc.sizeK(); k++) {
    					int z = disc.getz(k);

    					// ✅ CORREGIDO: ocupa todas las celdas debajo de la altura usada
    					if (z + boxHeight <= usedHeight) {
    						occupied[i][j][k] = true;
    					}
    				}
    			}
    		}

    		return occupied;
    	}

    /**
     * Calcula el porcentaje de volumen libre.
     */
    private static double computeFreeRatio(boolean[][][] occupied, Discretization disc) {
        int total = disc.sizeI() * disc.sizeJ() * disc.sizeK();
        int occ = 0;

        for (int i = 0; i < disc.sizeI(); i++)
            for (int j = 0; j < disc.sizeJ(); j++)
                for (int k = 0; k < disc.sizeK(); k++)
                    if (occupied[i][j][k]) occ++;

        return (double) (total - occ) / total;
    }
}
