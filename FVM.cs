using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM : MonoBehaviour
{
	float dt 			= 0.003f;
    float mass 			= 1;
	float stiffness_0	= 2000.0f;
    float stiffness_1 	= 2000.0f;
    float damp			= 0.999f;

	int[] 		Tet;
	int tet_number;			//The number of tetrahedra

	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;
	int number;				//The number of vertices

	Matrix4x4[] inv_Dm;

	//For Laplacian smoothing.
	Vector3[]   V_sum;
	int[]		V_num;

	SVD svd = new SVD();

    // Start is called before the first frame update
    void Start()
    {
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
    	{
    		string fileContent = File.ReadAllText("Assets/house2.ele");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		
    		tet_number=int.Parse(Strings[0]);
        	Tet = new int[tet_number*4];

    		for(int tet=0; tet<tet_number; tet++)
    		{
				Tet[tet*4+0]=int.Parse(Strings[tet*5+4])-1;
				Tet[tet*4+1]=int.Parse(Strings[tet*5+5])-1;
				Tet[tet*4+2]=int.Parse(Strings[tet*5+6])-1;
				Tet[tet*4+3]=int.Parse(Strings[tet*5+7])-1;
			}
    	}
    	{
			string fileContent = File.ReadAllText("Assets/house2.node");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		number = int.Parse(Strings[0]);
    		X = new Vector3[number];
       		for(int i=0; i<number; i++)
       		{
       			X[i].x=float.Parse(Strings[i*5+5])*0.4f;
       			X[i].y=float.Parse(Strings[i*5+6])*0.4f;
       			X[i].z=float.Parse(Strings[i*5+7])*0.4f;
       		}
    		//Centralize the model.
	    	Vector3 center=Vector3.zero;
	    	for(int i=0; i<number; i++)		center+=X[i];
	    	center=center/number;
	    	for(int i=0; i<number; i++)
	    	{
	    		X[i]-=center;
	    		float temp=X[i].y;
	    		X[i].y=X[i].z;
	    		X[i].z=temp;
	    	}
		}
        /*tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);*/


        //Create triangle mesh.
       	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];

        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }

        int[] triangles = new int[tet_number*12];
        for(int t=0; t<tet_number*4; t++)
        {
        	triangles[t*3+0]=t*3+0;
        	triangles[t*3+1]=t*3+1;
        	triangles[t*3+2]=t*3+2;
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals ();


		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

		//TODO: Need to allocate and assign inv_Dm
		inv_Dm = new Matrix4x4[tet_number];
		Matrix4x4 Dm = Matrix4x4.zero;
		for(int tet=0; tet<tet_number; tet++){
			
			Vector3 X10 = X[Tet[tet*4+1]]-X[Tet[tet*4+0]];
			Vector3 X20 = X[Tet[tet*4+2]]-X[Tet[tet*4+0]];
			Vector3 X30 = X[Tet[tet*4+3]]-X[Tet[tet*4+0]];
			for(int i = 0;i<3;i++){
				Dm[i,0] = X10[i]; 
				Dm[i,1] = X20[i]; 
				Dm[i,2] = X30[i]; 
			}
			Dm[3,3] = 1;
			inv_Dm[tet] = Dm.inverse;
		}
		
    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	Matrix4x4 ret=Matrix4x4.zero;
    	//TODO: Need to build edge matrix here.

		return ret;
    }

	Matrix4x4 halfMinues(Matrix4x4 a,Matrix4x4 b){
		Matrix4x4 ret = new Matrix4x4();
		for(int i =0;i<4;i++){
			for(int j=0;j<4;j++){
				ret[i,j] = (a[i,j]-b[i,j])*0.5f;
			}
		}
		return ret;
	}
	Matrix4x4 matrixMultiplyFloat(Matrix4x4 a,float b){
		Matrix4x4 ret = new Matrix4x4();
		for(int i =0;i<4;i++){
			for(int j=0;j<4;j++){
				ret[i,j] = a[i,j]*b;
			}
		}
		return ret;
	}

    void _Update()
    {
    	// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i=0; i<number; i++)
    			V[i].y+=0.2f;
    	}
 
    	for(int i=0 ;i<number; i++)
    	{
    		//TODO: Add gravity to Force.
			Force[i] = new Vector3(0,-9.8f,0);
    	}

    	for(int tet=0; tet<tet_number; tet++)
    	{
    		//TODO: Deformation Gradient
    		Vector3 x10 = X[Tet[tet*4+1]]-X[Tet[tet*4+0]];
			Vector3 x20 = X[Tet[tet*4+2]]-X[Tet[tet*4+0]];
			Vector3 x30 = X[Tet[tet*4+3]]-X[Tet[tet*4+0]];
			Matrix4x4 D = Matrix4x4.zero;
			for(int i = 0;i<3;i++){
				D[i,0] = x10[i]; 
				D[i,1] = x20[i]; 
				D[i,2] = x30[i]; 
			}
			D[3,3] = 1;
			Matrix4x4 F = D*inv_Dm[tet];
    		//TODO: Green Strain
			Matrix4x4 G = halfMinues(F.transpose*F , Matrix4x4.identity);
    		//TODO: Second PK Stress
			float trace = G[0,0]+G[1,1]+G[2,2]+G[3,3];
			Matrix4x4 S = new Matrix4x4();
			Matrix4x4 I = Matrix4x4.identity;
			for(int i =0;i<4;i++){
				for(int j=0;j<4;j++){
					S[i,j] = 2.0f*stiffness_1*G[i,j]+stiffness_0*trace*I[i,j];
				}
			}
    		//TODO: Elastic Force
			
			Matrix4x4 force = matrixMultiplyFloat(F*S*inv_Dm[tet].transpose,-1/(inv_Dm[tet].determinant*6));
			Vector3 f10 = new Vector3();
			Vector3 f20 = new Vector3();
			Vector3 f30 = new Vector3();
			Vector3 f0 = new Vector3();
			for(int i = 0;i<3;i++){
				f10[i] = force[i,0];
				f20[i] = force[i,1];
				f30[i] = force[i,2];
			}
			f0 = -f10-f20-f30;
			Force[Tet[tet*4+0]] += f0;
			Force[Tet[tet*4+1]] += f10;
			Force[Tet[tet*4+2]] += f20;
			Force[Tet[tet*4+3]] += f30;
			
    	}

    	for(int i=0; i<number; i++)
    	{
    		//TODO: Update X and V here.
			V[i] += Force[i]*dt/mass;
			V[i] *= damp;
			X[i] += V[i]*dt;
    		//TODO: (Particle) collision with floor.
			if(X[i].y<-3.0f){
				X[i].y = -3.0f;
				Vector3 vn = Vector3.zero;
				vn.y = V[i].y;
				Vector3 vt = V[i]-vn;
				float a = Math.Max(1 - 0.8f*(1+0.8f)*vn.magnitude/vt.magnitude,0);
				V[i] = -0.8f*vn + a*vt;
			}
    	}
    }

    // Update is called once per frame
    void Update()
    {
    	for(int l=0; l<10; l++)
    		 _Update();

    	// Dump the vertex array for rendering.
    	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.RecalculateNormals ();
    }
}
