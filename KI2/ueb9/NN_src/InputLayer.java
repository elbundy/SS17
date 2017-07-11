/// ----------------------------------------------------------------------------------------
/// This class implements the InputLayer for our NeuralNetwork
/// ----------------------------------------------------------------------------------------

public class InputLayer implements Layer 
{
		int inputLength;
	
		// Forwards the inputBlob without changing it
		public Blob forward(Blob inputBlob)
		{
			return inputBlob;
		}
	   
	   // Backwards the inputBlob without changing it
		public Blob backward (Blob deltaBefore, Blob weightsBefore)
		{
			return deltaBefore;
		}

		public void updateWeightsAndBias(Blob inputBlob, float learningRate)
		{
			// Nothing to do
		}

		public InputLayer(int in)
		{
			inputLength=in;
		}
		
		public Blob getWeights() 
		{
			return null;
		}

		@Override
		public void setWeights(Blob weights) 
		{
			
		}

		@Override
		public void setBias(Blob bias) 
		{

		}
}
