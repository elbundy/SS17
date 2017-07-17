/// ----------------------------------------------------------------------------------------
/// This interfaces is used for Neuron-Activation functions
/// ----------------------------------------------------------------------------------------
public interface ActivationFunction 
{
	// computes activationFunction(x)
	float compute(float x);
	// computes activationFunction'(x)
	float derivative(float x);
}
