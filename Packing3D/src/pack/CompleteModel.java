package pack;

import ilog.concert.IloException;
import ilog.cplex.IloCplex;

public class CompleteModel
{
	private Instance _instance;
	private Discretization _discretization;
	private IloCplex _cplex;
	
	private boolean _verbose = false;

	
	public CompleteModel(Instance instance)
	{
		_instance = instance;
		_discretization = new Discretization(instance);
	}
	
	public void solve()
	{
		try
		{
			_cplex = new IloCplex();
			
			if( _verbose == false )
				_cplex.setOut(null);
			
			_cplex.end();
		}
		catch (IloException e)
		{
			e.printStackTrace();
		}
	}
}
