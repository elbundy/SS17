import java.lang.Math;

/* ToDo: This class should implement tanh as activation function */
public class TanhActivation implements ActivationFunction 
{

	// This method implements tanh(x)
	public float compute(float x) 
	{
		return (float) Math.tanh(x);
	}

	// This method implements tanh'(x)
	public float derivative(float x) 
	{
		return (float) (1.0 - Math.pow(Math.tanh(x), 2));
	}

}
