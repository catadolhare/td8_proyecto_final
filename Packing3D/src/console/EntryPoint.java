package console;

import pack.*;

public class EntryPoint
{
	public static void main(String[] args)
	{
		Instance instance = new Instance(20, 10, 10, 10, 7, 5);
		Discretization discretization = new Discretization(instance);
		Box.initialize(instance, discretization);
		
		CompleteModel model = new CompleteModel(instance, discretization);
		model.solve();
	}
}
