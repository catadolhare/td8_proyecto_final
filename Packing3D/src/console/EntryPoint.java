package console;

import pack.*;

public class EntryPoint
{
	public static void main(String[] args)
	{
		ArgMap argmap = new ArgMap(args);
		if( argmap.containsArg("-help") )
		{
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

		// CAMBIADO: ejecutar LayerHeuristic en lugar de CompleteModel
		LayerHeuristic.run(instance);
	}

	private static void showParameters()
	{
		System.out.println("  -L [n]     Length of container");
		System.out.println("  -W [n]     Width of container");
		System.out.println("  -H [n]     Height of container");
		System.out.println("  -l [n]     Length of each box");
		System.out.println("  -w [n]     Width of each box");
		System.out.println("  -h [n]     Height of each box");
	}
}