import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.Files;

/// ----------------------------------------------------------------------------------------
/// This class implements a few functions to do tests with, later on 
/// this class will also implement methods to read competition datasets
/// ----------------------------------------------------------------------------------------


public class DataReader 
{
	// A very simple XOr Dataset with 2 inputs and one output
	public static Datum[] getXORData() 
	{
		Blob[] in=new Blob[4];
		
		for(int i=0;i<in.length;i++)
		{
			in[i]=new Blob(2);
		}
		
		in[0].setValue(0,1);
		in[0].setValue(1,1);
		in[1].setValue(0,0);
		in[1].setValue(1,1);
		in[2].setValue(0,1);
		in[2].setValue(1,0);
		in[3].setValue(0,0);
		in[3].setValue(1,0);
		
		Blob[] out=new Blob[4];
		
		for(int i=0;i<out.length;i++)
		{
			out[i]=new Blob(1);
		}
		
		out[0].setValue(0,0);
		out[1].setValue(0,1);
		out[2].setValue(0,1);
		out[3].setValue(0,0);
		
		Datum[] xorDataset=new Datum[4];
		
		for(int i=0;i<4;i++)
		{
			xorDataset[i]=new Datum();
			xorDataset[i].data=in[i];
			xorDataset[i].label=out[i];
		}
		
		return xorDataset;
	}
	
	// A very simple And Dataset with 2 inputs and one output
	public static Datum[] getAndData() 
	{
		Blob[] in=new Blob[4];
		
		for(int i=0;i<in.length;i++)
		{
			in[i]=new Blob(2);
		}
		
		in[0].setValue(0,1);
		in[0].setValue(1,1);
		in[1].setValue(0,0);
		in[1].setValue(1,1);
		in[2].setValue(0,1);
		in[2].setValue(1,0);
		in[3].setValue(0,0);
		in[3].setValue(1,0);
		
		Blob[] out=new Blob[4];
		
		for(int i=0;i<out.length;i++)
		{
			out[i]=new Blob(1);
		}
		
		out[0].setValue(0,1);
		out[1].setValue(0,0);
		out[2].setValue(0,0);
		out[3].setValue(0,0);
		
		Datum[] andDataset=new Datum[4];
		
		for(int i=0;i<4;i++)
		{
			andDataset[i]=new Datum();
			andDataset[i].data=in[i];
			andDataset[i].label=out[i];
		}
		
		return andDataset;
	}
	
	// A more advanced Dataset with 2 inputs and 4 outputs
	public static Datum[] get2In4OutDataset() 
	{
		Blob[] in=new Blob[4];
		Blob[] out=new Blob[4];
		
		for(int i=0;i<in.length;i++)
		{
			in[i]=new Blob(2);
			out[i]=new Blob(4);
		}
		
		int count=0;
		for(int i=0;i<2;i++)
		{
			for(int j=0;j<2;j++)
			{
						in[count].setValue(0,i);
						in[count].setValue(1,j);
						
						if(i==1 && j==1)
						{
							out[count].setValue(0,1);
							out[count].setValue(1,0);
							out[count].setValue(2,0);
							out[count].setValue(3,0);
						}
						else if(i==0 && j==1)
						{
							out[count].setValue(0,0);
							out[count].setValue(1,1);
							out[count].setValue(2,0);
							out[count].setValue(3,0);
						}
						else if(i==0 && j==0)
						{
							out[count].setValue(0,0);
							out[count].setValue(1,0);
							out[count].setValue(2,1);
							out[count].setValue(3,0);
						}
						else
						{
							out[count].setValue(0,0);
							out[count].setValue(1,0);
							out[count].setValue(2,0);
							out[count].setValue(3,1);
						}

						count++;
			}
		}
		
		Datum[] dataset=new Datum[4];
		
		for(int i=0;i<dataset.length;i++)
		{
			dataset[i]=new Datum();
			dataset[i].data=in[i];
			dataset[i].label=out[i];
		}
		
		return dataset;
	}
}