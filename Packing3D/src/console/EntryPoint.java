package console;

import pack.*;

public class EntryPoint
{
	public static void main(String[] args)
	{
		Instance instance = new Instance(40, 30, 20, 8, 5, 9);
		Discretization discretization = new Discretization(instance);
		Box.initialize(instance, discretization);
		
		CompleteModel model = new CompleteModel(instance, discretization);
		model.solve();
	}
}
