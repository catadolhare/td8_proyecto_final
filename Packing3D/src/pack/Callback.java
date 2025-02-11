package pack;

public interface Callback
{
	public enum Type { VariablesCreated, ObjectiveCreated, ConstraintsCreated, ModelSolved };
	public void notify(Type type);
}
