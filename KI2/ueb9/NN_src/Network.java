import java.util.ArrayList;

/// ----------------------------------------------------------------------------------------
/// This class represents the whole network. It also offers functions to train layers
/// and propagate forward a given input.
/// ----------------------------------------------------------------------------------------

public class Network
{
	ArrayList<Layer> layers;
	LearningRate learningRate;
	
   public Network(LearningRate learningRate)
   {
	   this.learningRate=learningRate;
	   layers=new ArrayList<Layer>();   	
   }
	   
   // Adds one layer to network
   public void add(Layer l)
   {
	   layers.add(l);
   }
   
   // Forwards an input through the network
   public Blob[] forward(Blob in)
   {
	   Blob[] forward=new Blob[layers.size()+1];
	   forward[0]=in;
	   for(int i=0;i<layers.size();i++)
	   {
		   forward[i+1]=layers.get(i).forward(forward[i]);
	   }
	   return forward;
   }
   
   // Trains the network using basic stochastic gradient descent (SGD)
   // Returns the output of the last layer, which can be used to calculate loss.
   public Blob trainSimpleSGD(Blob in, Blob out)
   {
	   Blob[] forward=forward(in);
	   
	   Blob backward=out;
	   for(int i=layers.size()-1;i>=0;i--)
	   {
		   if(i==layers.size()-1)
			   backward=layers.get(i).backward(backward,null);
		   else
			   backward=layers.get(i).backward(backward,layers.get(i+1).getWeights());
	   }
	   
	   float currRate=learningRate.getLearningRate();
	   for(int i=0;i<layers.size();i++)
	   {
		   if(i==0)
			   layers.get(i).updateWeightsAndBias(in, currRate);
		   else
			   layers.get(i).updateWeightsAndBias(forward[i], currRate);
	   }
	   
	   return forward[forward.length-1];
   }
   
}