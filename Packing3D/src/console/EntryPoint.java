package console;

import pack.*;

public class EntryPoint {

    public static void main(String[] args) {
        ArgMap argmap = new ArgMap(args);

        if (argmap.containsArg("-help")) {
            showParameters();
            return;
        }

        // === Lectura de parámetros del contenedor y la caja ===
        int L = argmap.intArg("-L", 20);  // largo contenedor
        int W = argmap.intArg("-W", 10);  // ancho contenedor
        int H = argmap.intArg("-H", 10);  // alto contenedor
        int l = argmap.intArg("-l", 5);   // largo caja
        int w = argmap.intArg("-w", 5);   // ancho caja
        int h = argmap.intArg("-h", 7);   // alto caja

        System.out.println("=====================================");
        System.out.println("🚀 Iniciando ejecución de la heurística por capas (con relleno 3D)");
        System.out.println("Contenedor: " + L + " × " + W + " × " + H);
        System.out.println("Caja: " + l + " × " + w + " × " + h);
        System.out.println("=====================================\n");

        // === Inicialización de la instancia ===
        Instance instance = new Instance(L, W, H, l, w, h);
        Discretization discretization = new Discretization(instance);
        Box.initialize(instance, discretization);

        // === Ejecutar la heurística completa (ella mide su propio tiempo) ===
        LayerHeuristic.run(instance);

        System.out.println("\n=== EJECUCIÓN FINALIZADA ===");
    }

    private static void showParameters() {
        System.out.println("Uso:");
        System.out.println("  java -jar programa.jar -L [n] -W [n] -H [n] -l [n] -w [n] -h [n]");
        System.out.println();
        System.out.println("Parámetros:");
        System.out.println("  -L [n]   Largo del contenedor");
        System.out.println("  -W [n]   Ancho del contenedor");
        System.out.println("  -H [n]   Alto del contenedor");
        System.out.println("  -l [n]   Largo de la caja");
        System.out.println("  -w [n]   Ancho de la caja");
        System.out.println("  -h [n]   Alto de la caja");
        System.out.println();
        System.out.println("Ejemplo:");
        System.out.println("  java -jar programa.jar -L 20 -W 10 -H 10 -l 5 -w 5 -h 7");
    }
}