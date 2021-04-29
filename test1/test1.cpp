#include<iostream> 
#include<vector>
#include<cstdlib>     
#include<math.h>
using namespace std;

#define N 9999    //Random number precision    
#define eps  0.000001  
#define FREQ_CAM 30
#define FREQ_IMU 100
#define T0_CAM 0.0
#define T0_IMU 0.42
#define DURATION 5

// Quaternion
struct Q{
    double x;
    double y;
    double z;
    double w;
};


void Normalize(const Q& q1);
Q NLerp(const Q& q1, const Q& q2, const double& t);
void GenTimestampQuaternion(std::vector<double>& cam_t, std::vector<double>& imu_t, std::vector<Q>& imu_q);
bool CalImuPose(const double & tc, const std::vector<double>& imu_t, const std::vector<Q>& imu_q, Q& qc);

int main() {
    	// Generate timestamp and quaternion
    	std::vector<double> cam_t;
    	std::vector<double> imu_t;
    	std::vector<Q> imu_q;
    	cam_t.reserve(200);
    	imu_t.reserve(600);
    	imu_q.reserve(600);
    	GenTimestampQuaternion(cam_t,imu_t,imu_q);
	//std::cout << cam_t.size() << imu_t.size() << imu_q.size()<< endl;
    	//  Caculate imu's pose (qc) for each timestamp (tc) in cam_t
    	Q qc;
    	for(const auto& tc : cam_t){
        	if( CalImuPose(tc, imu_t, imu_q, qc)){
            		cout << qc.x << qc.y << qc.z << qc.w << endl;
        	}
    	}
    	return 0;
}

void Normalize(Q& q) {
	double dieL = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
	q.x = q.x / dieL;
	q.y = q.y / dieL;
	q.z = q.z / dieL;
	q.w = q.w / dieL;
}

Q NLerp(const Q& q1, const Q& q2, const double& t) {
	Q tempQ;
	tempQ.x = (1-t) * q1.x + t * q2.x;
	tempQ.y = (1-t) * q1.y + t * q2.y;
	tempQ.z = (1-t) * q1.z + t * q2.z;
	tempQ.w = (1-t) * q1.w + t * q2.w;
	Normalize(tempQ);
	return tempQ;
}

void GenTimestampQuaternion(std::vector<double>& cam_t, std::vector<double>& imu_t, std::vector<Q>& imu_q) {
	int camIte = FREQ_CAM * DURATION;
	int imuIte = FREQ_IMU * DURATION;
	Q T0_q = {0,0,0,1};
	Q tempQ;
	for(int i = 0; i < camIte; i++)	{
		cam_t.push_back(T0_CAM + (double)i/FREQ_CAM);
		//std::cout << cam_t[i] << endl;
	}

	for(int i = 0; i < imuIte; i++)	{

		if(i==0) {
			imu_t.push_back(T0_IMU);
			imu_q.push_back(T0_q);
			//std::cout << imu_t[i] << endl;
			//std::cout << imu_q[i].x << imu_q[i].y <<imu_q[i].z <<imu_q[i].w << endl;
		}
		else {
			double temp = T0_IMU + (double)i * (1.0 / FREQ_IMU) + 0.5 * (1.0 / FREQ_IMU) * (rand() % (N + 1) / (float)(N + 1));
			imu_t.push_back(temp);
			//std::cout << imu_t[i] << endl;
			tempQ = imu_q[i-1];
			tempQ.x = tempQ.x + 0.1 * (rand() % (N + 1) / (float)(N + 1));	
			tempQ.y = tempQ.y + 0.1 * (rand() % (N + 1) / (float)(N + 1));
			tempQ.z = tempQ.z + 0.1 * (rand() % (N + 1) / (float)(N + 1));
			tempQ.w = tempQ.w + 0.1 * (rand() % (N + 1) / (float)(N + 1));
			Normalize(tempQ);
			//std::cout << tempQ.x << tempQ.y <<tempQ.z <<tempQ.w <<endl;
			imu_q.push_back(tempQ);
		}
	}

}

bool CalImuPose(const double & tc, const std::vector<double>& imu_t, const std::vector<Q>& imu_q, Q& qc)
{
	int left = 0, right = imu_t.size() - 1;

    	while(1) {
        	int mid = (right + left) / 2;
        	if ((imu_t[mid]-tc) > eps) {
			if(mid > 0) {
		    		if((tc-imu_t[mid-1]) > eps) {
					qc = NLerp(imu_q[mid],imu_q[mid-1],(tc-imu_t[mid-1])/(imu_t[mid]-imu_t[mid-1]));
					return true;
				}
				else {
					right = mid;
					if(left == right) {
						return false;
					}
					continue;
				}
			}
			else	
				return false;	
        	} 

		if ((tc-imu_t[mid]) > eps) {
			if(mid < imu_t.size() - 1) {
		    		if((imu_t[mid+1]-tc) > eps) {
					qc = NLerp(imu_q[mid],imu_q[mid+1],(tc-imu_t[mid])/(imu_t[mid+1]-imu_t[mid]));
					return true;
				}
				else {
					left = mid;
					if(left == right) {
						return false;
					}
					continue;
				}
			}
			else
				return false;
        	} 
    	}
}
