package console;

import pack.*;

public class EntryPoint {
    public static void main(String[] args) {
        ArgMap argmap = new ArgMap(args);
        if (argmap.containsArg("-help")) {
            showParameters();
            return;
        }

        int L = argmap.intArg("-L", 20);
        int W = argmap.intArg("-W", 10);
        int H = argmap.intArg("-H", 10);
        int l = argmap.intArg("-l", 5);
        int w = argmap.intArg("-w", 5);
        int h = argmap.intArg("-h", 7);

        Instance instance = new Instance(L, W, H, l, w, h);
        Discretization discretization = new Discretization(instance);
        Box.initialize(instance, discretization);

        // Ejecutar la heurística completa
        HeuristicResult result = LayerHeuristic.run(instance);

        // Mostrar resumen final (legible por tu script Python)
        System.out.println("\n=== RESUMEN FINAL (para script) ===");
        System.out.println("Mejor orientación: " + result.bestOrientation);
        System.out.println("Cajas por capa: " + result.boxesPerLayer);
        System.out.println("Cantidad de capas: " + result.layers);
        System.out.println("Cajas relleno vertical: " + result.verticalFillBoxes);
        System.out.println("Orientación relleno vertical: " + result.verticalFillOrientation);
        System.out.println("Cajas relleno residual: " + result.residualBoxes);
        System.out.println("Total final de cajas: " + result.totalFinal());
        System.out.printf("Tiempo total de resolución: %.2f\n",
                (result.solveTime + result.fillTime + result.residualTime) / 1000);
    }

    private static void showParameters() {
        System.out.println("  -L [n]     Length of container");
        System.out.println("  -W [n]     Width of container");
        System.out.println("  -H [n]     Height of container");
        System.out.println("  -l [n]     Length of each box");
        System.out.println("  -w [n]     Width of each box");
        System.out.println("  -h [n]     Height of each box");
    }
}
