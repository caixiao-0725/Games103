using UnityEngine;
using System.Collections;

using System;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.0015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;					// for collision


	// Use this for initialization
	void Start () 
	{		
		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref[3, 3] = 1;
	}
	//这里为了省事直接除以mass了，并不是真正的矩阵减法
	Matrix4x4 Minues(Matrix4x4 a,Matrix4x4 b){
		Matrix4x4 ret= Matrix4x4.identity;
		for(int i =0;i<4;i++){
			for(int j =0;j<4;j++){
				ret[i,j] = a[i,j]/mass - b[i,j];
			}
		}
		return ret;
	}

	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}


	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		float u = 0.8f;
		float a = 0.5f;
		Vector3 origin = transform.position;
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		//求平均碰撞点的准备
		int count = 0;
		Vector3 x_sum = new Vector3(0, 0, 0);
		
		for (int i=0; i<vertices.Length; i++) {
			//计算当前点的位置
			Vector3 x_i = origin + R.MultiplyVector(vertices[i]);
			Vector3 px = x_i-P;
			if(Vector3.Dot(px,N)<0){
				//计算当前点的速度 v+wx(R.r)
				Vector3 v_i = v + Vector3.Cross(w,R.MultiplyVector(vertices[i]));
				if(Vector3.Dot(v_i,N)<0){
					count+=1;
					x_sum += x_i;
				}
			}
		}

		if(count>0){
			Matrix4x4 I = R*I_ref*R.transpose;
			//根据Implus method 算出新的速度，通过速度推导冲量
			Vector3 x_average = x_sum/count;
			Vector3 Rri = x_average-origin;

			Vector3 v_average = v + Vector3.Cross(w,Rri);
			Vector3 v_n = Vector3.Dot(v_average,N)*N;
			Vector3 v_t = v_average-v_n;
			Vector3 new_v_n = (-1)*u*v_n;
			Vector3 new_v_t = a*v_t;
			Vector3 new_v = new_v_n+new_v_t;


			//先算K
			
			Matrix4x4 K = Minues(Matrix4x4.identity,Get_Cross_Matrix(Rri)*I.inverse*Get_Cross_Matrix(Rri));
			//计算冲量
			Vector3 j = K.inverse.MultiplyVector(new_v-v_average);

			//有了j以后，回推出兔子模型质心的v和w
 			v += 1.0f / mass * j;
        	w += I.inverse.MultiplyVector(Vector3.Cross(Rri, j));

		}

	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			launched=true;
		}

		// Part I: Update velocities
		if(launched){
			v += new Vector3(0,-9.8f,0)*dt;
			v *= linear_decay;
			w *= angular_decay;

			// Part II: Collision Impulse
			Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
			Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

			// Part III: Update position & orientation
			//Update linear status
			Vector3 x    = transform.position;
			x = x + v*dt;
			//Update angular status
			Quaternion q = transform.rotation;
			Vector3 wt = 0.5f * dt * w;
			Quaternion dq = new Quaternion(wt.x, wt.y, wt.z, 0.0f) * q;
			q.Set(q.x + dq.x, q.y + dq.y, q.z + dq.z, q.w + dq.w);



			// Part IV: Assign to the object
			transform.position = x;
			transform.rotation = q;
		}

	}
}
