/// ----------------------------------------------------------------------------------------
/// This class implements the linear activation per neuron
/// ----------------------------------------------------------------------------------------


/* ToDo: This class should implement the Linear-Activation-Function used for Output-Layers */
public class LinearActivation implements ActivationFunction 
{
	// This function computes linear(x) and returns it
	public float compute(float x)
	{
		return x;
	}

	// This function computes linear'(x) and returns it
	public float derivative(float x) 
	{
		return (float)1.0;
	}

}
