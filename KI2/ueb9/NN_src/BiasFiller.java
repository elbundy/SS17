/// ----------------------------------------------------------------------------------------
/// This interface describes methods which fill the initial bias of each neuron
/// ----------------------------------------------------------------------------------------

public interface BiasFiller 
{
	public float compute(int currWeight, int in, int out);
}
