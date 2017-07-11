/// ----------------------------------------------------------------------------------------
/// This class implements a constant learning rate while training
/// ----------------------------------------------------------------------------------------

/* ToDo: This class implements a Constant-Learning-Rate */
public class ConstantLearningRate implements LearningRate 
{
	
	float rate;
	public ConstantLearningRate(float rate)
	{
		
	}
	
	public float getLearningRate() 
	{
		return (float)0.1337;
	}

}
