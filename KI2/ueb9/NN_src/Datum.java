/// ----------------------------------------------------------------------------------------
/// This class implements one datum which is described by the inputs and the expected (=correct) outputs (=labels)
/// ----------------------------------------------------------------------------------------

public class Datum {
	public Blob data;
	public Blob label;
	
	public Datum()
	{
		
	}
	
	public Datum(int sizeData,int sizeLabel)
	{
		data=new Blob(sizeData);
		label=new Blob(sizeLabel);
	}
	
	public Datum(int width,int height,int channels ,int sizeLabel)
	{
		data=new Blob(width,height,channels);
		label=new Blob(sizeLabel);
	}
}
