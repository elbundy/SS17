/// ----------------------------------------------------------------------------------------
/// This interfaces describes the general structure of a layer in our Neural Network
/// ----------------------------------------------------------------------------------------
public interface Layer 
{
	// This method returns a Blob which is the processed version (after applying activation function etc.) of inputBlob
	public Blob forward(Blob inputBlob);
	// This method returns a Blob which is the new gradient/neuronDelta at the current layer
	public Blob backward (Blob expectedOutput, Blob weightsBefore);
	// This method updates the Weights and Bias values of each neuron
	public void updateWeightsAndBias(Blob inputBlob, float learningRate);
	// Simple getter
	public Blob getWeights();
	// Simple setter
	public void setWeights(Blob weights);
	public void setBias(Blob bias);
}
